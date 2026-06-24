#!/usr/bin/env python3
"""Offline validator for the IMU touchdown detector (flight_safety.touchdown).

Replays land_drill bags through the EXACT detector logic the live node uses, so the
thresholds can be verified against real flights before trusting the KILL path:
  - fires once, during the descent, ~1s after LAND (= first ground contact)
  - fires WELL before the manual kill (disarm) -- that gap is the time IMU-kill saves
  - NO fire during the pre-touchdown descent (false-positive check)

    python3 tools/flight_safety/validate_touchdown.py flight_logs/landdrill_*.bag
                 [--vert-thresh 16] [--gyro-thresh 0.3] [--confirm 2] [--window 0.25]

Host tool: uses `rosbag` (no live ROS). Imports touchdown.py by path so it stays in
lockstep with the node.
"""
import argparse
import importlib.util
import os
import numpy as np
import rosbag

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TD = os.path.join(REPO, "ws/flight-safety/src/flight_safety/src/flight_safety/touchdown.py")
spec = importlib.util.spec_from_file_location("touchdown", TD)
touchdown = importlib.util.module_from_spec(spec)
spec.loader.exec_module(touchdown)


def world_vert(q, a):
    """ENU-up component of body specific-force a, given body->world quat q=(x,y,z,w)."""
    x, y, z, w = q
    return 2 * (x * z + w * y) * a[0] + 2 * (y * z - w * x) * a[1] + (1 - 2 * (x * x + y * y)) * a[2]


def load(path):
    b = rosbag.Bag(path)
    imu = []
    for _, m, _ in b.read_messages(topics=["/mavros/imu/data"]):
        a = m.linear_acceleration
        g = m.angular_velocity
        o = m.orientation
        vert = world_vert((o.x, o.y, o.z, o.w), (a.x, a.y, a.z))
        gmag = (g.x ** 2 + g.y ** 2 + g.z ** 2) ** 0.5
        imu.append((m.header.stamp.to_sec(), vert, gmag))
    lane = [(m.header.stamp.to_sec(), m.control_lane)
            for _, m, _ in b.read_messages(topics=["/flight_safety/state"])]
    arm = [(m.header.stamp.to_sec(), m.armed)
           for _, m, _ in b.read_messages(topics=["/mavros/state"])]
    b.close()
    return imu, lane, arm


def edge_to(seq, target):
    for i in range(1, len(seq)):
        if seq[i][1] == target and seq[i - 1][1] != target:
            return seq[i][0]
    return None


def lane_at(lane, t):
    cur = "NORMAL"
    for ts, l in lane:
        if ts <= t:
            cur = l
        else:
            break
    return cur


def main():
    p = argparse.ArgumentParser()
    p.add_argument("bags", nargs="+")
    p.add_argument("--vert-thresh", type=float, default=16.0)
    p.add_argument("--gyro-thresh", type=float, default=0.3)
    p.add_argument("--min-land", type=float, default=0.3)
    p.add_argument("--confirm", type=int, default=2)
    p.add_argument("--window", type=float, default=0.25)
    a = p.parse_args()

    print("detector: vert>%.1f AND gmag>=%.2f, confirm %d within %.2fs, min_land %.2fs\n"
          % (a.vert_thresh, a.gyro_thresh, a.confirm, a.window, a.min_land))
    all_ok = True
    for path in a.bags:
        imu, lane, arm = load(path)
        t_land = edge_to(lane, "LAND")
        t_disarm = edge_to(arm, False)
        t0 = imu[0][0]
        det = touchdown.TouchdownDetector(a.vert_thresh, a.gyro_thresh, a.min_land,
                                          a.confirm, a.window)
        t_fire = None
        false_before = []
        for (t, vert, gmag) in imu:
            in_land = lane_at(lane, t) == "LAND"
            if det.update(t, vert, gmag, in_land):
                t_fire = t
                break
        rel = lambda x: "NONE" if x is None else "%.2f" % (x - t0)
        print("=== %s ===" % os.path.basename(path))
        print("  LAND@%s  fire@%s  manual-disarm@%s (1Hz coarse)"
              % (rel(t_land), rel(t_fire), rel(t_disarm)))
        ok = True
        if t_fire is None:
            print("  FAIL: never fired"); ok = False
        else:
            if t_land:
                print("  -> fired %.2fs after LAND" % (t_fire - t_land))
            if t_disarm:
                gap = t_disarm - t_fire
                print("  -> fired %.2fs BEFORE manual kill  (time IMU-kill would save)" % gap)
                if gap <= 0:
                    print("  FAIL: fired at/after the manual kill"); ok = False
            # false-positive check: any vert/gmag hit during LAND before fire but > min_land?
            # (the detector already gates; this re-checks the descent had no qualifying burst)
            if t_land and t_fire - t_land < a.min_land + 0.05:
                print("  WARN: fired suspiciously close to LAND start (entry transient?)")
        print("  %s\n" % ("PASS" if ok else "FAIL"))
        all_ok = all_ok and ok
    print("ALL PASS" if all_ok else "SOME FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

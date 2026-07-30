#!/usr/bin/env python3
"""Rewrite /visualizer/trajectory in a flight bag into the FOV-frustum view.

Bags recorded before the LA-style frustum change carry /visualizer/trajectory as
a visualization_msgs/Marker (SPHERE_LIST, jet-coloured by speed). This replaces
it with the current MarkerArray form -- red LINE_STRIP path plus green LINE_LIST
frustums that vanish as the *planned* time advances -- so an old bag can be
viewed with the normal real.rviz layout.

The frustums are NOT recomputed by re-running the planner. They are rebuilt from
what the bag already recorded:

    /planning/trajectory      traj_utils/PolyTraj   position MINCO coefficients
    /planning/yaw_trajectory  traj_utils/PolyTraj   yaw MINCO coefficients

which are exactly the two inputs Visualizer::visualize() takes at runtime
(local_data_.minco_traj_ / minco_yaw_traj_ / start_time_). So the output shows
the trajectory that actually flew, not a replay-dependent approximation.

Geometry, namespaces, ids, colours, sampling and the DELETE sweep all mirror
minco_planner/include/misc/visualizer.hpp. Frames are emitted the same way the
live 20 Hz timer does: once per new plan, then only when the set of surviving
frustums actually changes -- which keeps the rewritten bag small instead of
20 Hz x N markers.

Usage (inside the epic container, so rosbag/genpy are importable):
    rosrun_env python3 tools/bag_rewrite_traj_viz.py <in.bag> [out.bag]
Default output is <in>.frustum.bag next to the input.
"""

import math
import os
import sys

import rosbag
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

TOPIC_VIZ = "/visualizer/trajectory"
TOPIC_POS = "/planning/trajectory"
TOPIC_YAW = "/planning/yaw_trajectory"

# Defaults mirror visualizer.hpp's ROS params (visualizer/*).
FRAME_ID = "odom"
FOV_DEPTH = 0.3
FOV_H_RAD = math.radians(79.1396)
FOV_V_RAD = math.radians(63.5803)
FOV_SAMPLE_DT = 0.3
FOV_MAX_SLOTS = 50
LINE_SAMPLE_DT = 0.05
VIZ_PERIOD = 1.0 / 20.0

NS_PATH = "optimal_trajectory"
NS_FOV = "optimal_trajectory_fov"


class PolyTrajEval(object):
    """Piecewise quintic, decoded exactly as traj_server rebuilds PolyTraj.

    coef_* is row-major over pieces: coef_x[i*6 + j] == coeffMat(0, j) of piece
    i. gcopter's Piece::getPos walks columns from D down to 0 multiplying by an
    increasing power of t, so column j carries t**(order - j) -- column 0 is the
    HIGHEST order term and column 5 is the constant.
    """

    def __init__(self, msg):
        self.order = msg.order
        self.n = msg.order + 1
        self.durations = list(msg.duration)
        self.pieces = []
        for i in range(len(self.durations)):
            o = i * self.n
            self.pieces.append((
                list(msg.coef_x[o:o + self.n]),
                list(msg.coef_y[o:o + self.n]),
                list(msg.coef_z[o:o + self.n]),
            ))
        self.total = sum(self.durations)

    def _locate(self, t):
        """Port of Trajectory::locatePieceIdx -- returns (idx, piece-local t)."""
        idx = 0
        n = len(self.durations)
        while idx < n and t > self.durations[idx]:
            t -= self.durations[idx]
            idx += 1
        if idx == n:
            idx -= 1
            t += self.durations[idx]
        return idx, t

    def pos(self, t):
        idx, tl = self._locate(t)
        cx, cy, cz = self.pieces[idx]
        x = y = z = 0.0
        tn = 1.0
        for j in range(self.order, -1, -1):
            x += tn * cx[j]
            y += tn * cy[j]
            z += tn * cz[j]
            tn *= tl
        return x, y, z

    def vel(self, t):
        idx, tl = self._locate(t)
        cx, cy, cz = self.pieces[idx]
        x = y = z = 0.0
        tn = 1.0
        k = 1
        for j in range(self.order - 1, -1, -1):
            x += k * tn * cx[j]
            y += k * tn * cy[j]
            z += k * tn * cz[j]
            tn *= tl
            k += 1
        return x, y, z

    def scalar(self, t):
        """Yaw trajectories carry the angle in the x row (matches .x() in C++)."""
        return self.pos(t)[0]


def build_fov_marker(px, py, pz, yaw, marker_id, stamp):
    mk = Marker()
    mk.header.frame_id = FRAME_ID
    mk.header.stamp = stamp
    mk.ns = NS_FOV
    mk.id = marker_id
    mk.type = Marker.LINE_LIST
    mk.action = Marker.ADD
    mk.pose.orientation.w = 1.0
    mk.scale.x = FOV_DEPTH * 0.1
    mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 1.0, 0.0, 1.0

    half_w = math.tan(FOV_H_RAD / 2.0) * FOV_DEPTH
    half_h = math.tan(FOV_V_RAD / 2.0) * FOV_DEPTH
    corners = [
        (FOV_DEPTH, -half_w, -half_h),
        (FOV_DEPTH, half_w, -half_h),
        (FOV_DEPTH, half_w, half_h),
        (FOV_DEPTH, -half_w, half_h),
    ]
    c, s = math.cos(yaw), math.sin(yaw)
    world = []
    for cx, cy, cz in corners:
        world.append(Point(x=c * cx - s * cy + px,
                           y=s * cx + c * cy + py,
                           z=cz + pz))
    apex = Point(x=px, y=py, z=pz)
    for i in range(4):                      # far-plane rectangle
        mk.points.append(world[i])
        mk.points.append(world[(i + 1) % 4])
    for i in range(4):                      # apex -> corner rays
        mk.points.append(apex)
        mk.points.append(world[i])
    return mk


def stale_fov_marker(marker_id, stamp):
    mk = Marker()
    mk.header.frame_id = FRAME_ID
    mk.header.stamp = stamp
    mk.ns = NS_FOV
    mk.id = marker_id
    mk.action = Marker.DELETE
    return mk


class Plan(object):
    """One plan's cached samples, mirroring Visualizer::visualize()."""

    def __init__(self, pos_traj, yaw_traj, start_time):
        self.start_time = start_time
        dur = pos_traj.total
        self.line_pts = []
        t = 0.0
        while t < dur:
            x, y, z = pos_traj.pos(t)
            self.line_pts.append(Point(x=x, y=y, z=z))
            t += LINE_SAMPLE_DT
        x, y, z = pos_traj.pos(dur)
        self.line_pts.append(Point(x=x, y=y, z=z))

        yaw_dur = yaw_traj.total if yaw_traj is not None else 0.0
        yaw_ok = yaw_traj is not None and yaw_dur > 0.0

        def yaw_at(tt):
            if yaw_ok:
                return yaw_traj.scalar(min(tt, yaw_dur))
            vx, vy, _ = pos_traj.vel(tt)          # fallback: velocity heading
            if math.hypot(vx, vy) > 1e-6:
                return math.atan2(vy, vx)
            return 0.0

        fov_dt = max(FOV_SAMPLE_DT, dur / FOV_MAX_SLOTS)
        self.fov = []
        t = 0.0
        while t < dur and len(self.fov) < FOV_MAX_SLOTS:
            px, py, pz = pos_traj.pos(t)
            self.fov.append((px, py, pz, yaw_at(t), t))
            t += fov_dt

    def skip_at(self, elapsed):
        skip = 0
        while skip < len(self.fov) and self.fov[skip][4] < elapsed:
            skip += 1
        return skip

    def frame(self, skip, stamp):
        arr = MarkerArray()
        line = Marker()
        line.header.frame_id = FRAME_ID
        line.header.stamp = stamp
        line.ns = NS_PATH
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.08
        line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.0, 0.0, 1.0
        line.points = self.line_pts
        arr.markers.append(line)

        n = len(self.fov)
        for i in range(skip, n):
            px, py, pz, yaw, _ = self.fov[i]
            arr.markers.append(build_fov_marker(px, py, pz, yaw, i + 1, stamp))
        for i in range(skip):                       # already flown past
            arr.markers.append(stale_fov_marker(i + 1, stamp))
        for i in range(n, FOV_MAX_SLOTS):           # horizon shrank
            arr.markers.append(stale_fov_marker(i + 1, stamp))
        return arr


def collect_plans(path):
    """Pass 1 -- read only the two PolyTraj topics (cheap) and pair by traj_id."""
    pos_msgs, yaw_by_id = [], {}
    with rosbag.Bag(path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[TOPIC_POS, TOPIC_YAW]):
            if topic == TOPIC_POS:
                pos_msgs.append((t, msg))
            else:
                yaw_by_id[msg.traj_id] = msg

    plans = []
    for t, msg in pos_msgs:
        if not msg.duration:
            continue
        yaw_msg = yaw_by_id.get(msg.traj_id)
        yaw_traj = PolyTrajEval(yaw_msg) if (yaw_msg and yaw_msg.duration) else None
        plans.append((t, Plan(PolyTrajEval(msg), yaw_traj, msg.start_time)))
    plans.sort(key=lambda p: p[0].to_sec())
    return plans, len(yaw_by_id)


def build_frames(plans):
    """Emit once per plan, then only when the surviving frustum set changes.

    Same rule as publishTrajViz(force) plus the 20 Hz timer, so playback looks
    identical without writing a frame every 50 ms.
    """
    frames = []
    for k, (t_pub, plan) in enumerate(plans):
        t_end = plans[k + 1][0] if k + 1 < len(plans) else \
            rospy.Time.from_sec(t_pub.to_sec() + plan.fov[-1][4] + 1.0 if plan.fov else t_pub.to_sec() + 1.0)
        last_skip = None
        t = t_pub
        while t.to_sec() < t_end.to_sec():
            elapsed = t.to_sec() - plan.start_time.to_sec()
            skip = plan.skip_at(elapsed)
            if skip != last_skip:
                frames.append((t, plan.frame(skip, t)))
                last_skip = skip
            t = rospy.Time.from_sec(t.to_sec() + VIZ_PERIOD)
    return frames


def _mounts():
    """(mountpoint, host_root, is_ro) for every mount, longest mountpoint first."""
    out = []
    try:
        with open("/proc/self/mountinfo") as f:
            for line in f:
                p = line.split()
                if len(p) < 6:
                    continue
                unesc = lambda s: s.replace("\\040", " ").replace("\\011", "\t")
                root, mp, opts = unesc(p[3]), unesc(p[4]), p[5]
                out.append((mp, root, "ro" in opts.split(",")))
    except OSError:
        return []
    out.sort(key=lambda m: len(m[0]), reverse=True)
    return out


def writable_container_path(path):
    """Redirect a write that lands on a read-only bind mount to a writable alias.

    $EPIC_BAGS_DIR is mounted twice in the epic container: at /bags read-only
    (module.yml pins :ro so replay can never clobber a flight recording) and
    again under /work, which is the whole repo read-write. Same directory, same
    inode — only the mount flag differs, so "/bags/out.bag" fails with EROFS
    while "/work/flight_logs/epic_bags/out.bag" succeeds. Nobody should have to
    know that, so translate it and say so.

    Generic on purpose: resolve the path to its host path via
    /proc/self/mountinfo, then look for another mount that is rw and whose host
    root contains it. Returns the path unchanged when it is already writable or
    no alias exists.
    """
    ap = os.path.abspath(path)
    d = os.path.dirname(ap) or "/"
    if os.access(d, os.W_OK):
        return ap
    mounts = _mounts()
    src = None
    for mp, root, _ro in mounts:
        if ap == mp or ap.startswith(mp.rstrip("/") + "/"):
            src = root.rstrip("/") + ap[len(mp.rstrip("/")):]
            break
    if src is None:
        return ap
    for mp, root, ro in mounts:
        if ro:
            continue
        r = root.rstrip("/")
        if src == r or src.startswith(r + "/"):
            alt = mp.rstrip("/") + src[len(r):]
            if os.access(os.path.dirname(alt) or "/", os.W_OK):
                print("note: %s is on a read-only mount; writing via %s"
                      % (os.path.dirname(ap), os.path.dirname(alt)))
                return alt
    return ap


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    inp = argv[1]
    out = argv[2] if len(argv) > 2 else \
        os.path.join(os.path.dirname(os.path.abspath(inp)),
                     os.path.basename(inp).rsplit(".bag", 1)[0] + ".frustum.bag")
    out = writable_container_path(out)
    if not os.path.isfile(inp):
        print("no such bag: %s" % inp, file=sys.stderr)
        return 1
    # Compare identity, not strings. $EPIC_BAGS_DIR is mounted at BOTH /bags and
    # /work/flight_logs/epic_bags, so "/bags/x.bag" and
    # "/work/flight_logs/epic_bags/x.bag" are different paths naming the same
    # inode — a string compare happily lets you shred the input, and the write
    # is streamed so the damage is done before anything notices.
    if os.path.exists(out) and os.path.samefile(inp, out):
        print("refusing: %s and %s are the same file" % (inp, out), file=sys.stderr)
        return 1
    if os.path.exists(out):
        print("refusing: %s already exists (delete it or pick another name)" % out,
              file=sys.stderr)
        return 1

    plans, n_yaw = collect_plans(inp)
    if not plans:
        print("no %s messages in %s -- nothing to rebuild from" % (TOPIC_POS, inp),
              file=sys.stderr)
        return 1
    frames = build_frames(plans)
    print("plans=%d yaw_trajs=%d -> %d MarkerArray frames" %
          (len(plans), n_yaw, len(frames)))

    # Pass 2 -- stream the input through, dropping the old topic and splicing the
    # generated frames in timestamp order. Streaming keeps a multi-GB bag off the
    # heap; only the (small) frame list is held in memory.
    fi = 0
    dropped = 0
    with rosbag.Bag(inp, "r") as bin_, rosbag.Bag(out, "w") as bout:
        for topic, msg, t in bin_.read_messages():
            while fi < len(frames) and frames[fi][0].to_sec() <= t.to_sec():
                bout.write(TOPIC_VIZ, frames[fi][1], frames[fi][0])
                fi += 1
            if topic == TOPIC_VIZ:
                dropped += 1
                continue
            bout.write(topic, msg, t)
        while fi < len(frames):
            bout.write(TOPIC_VIZ, frames[fi][1], frames[fi][0])
            fi += 1

    print("dropped %d old Marker msgs, wrote %d MarkerArray msgs" % (dropped, len(frames)))
    print("out: %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

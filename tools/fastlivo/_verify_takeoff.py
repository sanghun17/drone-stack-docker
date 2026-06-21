#!/usr/bin/env python3
# one-off verification: per-axis (x/y/z) translation error during takeoff, /aft_mapped_to_odom vs vrpn
import sys, copy
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync

BAG = sys.argv[1] if len(sys.argv) > 1 else "/work/tools/fastlivo/2026-06-19-17-17-03_livo.bag"
EST = "/aft_mapped_to_odom"
GT = "/vrpn_client_node/pure/pose"

with Rosbag1Reader(Path(BAG)) as r:
    est = file_interface.read_bag_trajectory(r, EST)
    gt = file_interface.read_bag_trajectory(r, GT)
gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=0.02)
t = gt_s.timestamps - gt_s.timestamps[0]


def axis_stats(g, e, tag):
    d = e.positions_xyz - g.positions_xyz           # per-axis signed error (m)
    n = np.linalg.norm(d, axis=1)
    rmse = lambda v: float(np.sqrt(np.mean(v ** 2)))
    print("  %-12s  ex=%5.2f ey=%5.2f ez=%5.2f  |  |e|rmse=%5.2f  max=%5.2f m"
          % (tag, rmse(d[:, 0]), rmse(d[:, 1]), rmse(d[:, 2]), rmse(n), float(n.max())))


# takeoff window from GT z: from t0 until z first plateaus near its early max
gz = gt_s.positions_xyz[:, 2]
z0 = gz[0]
early = t < 60
zmax = gz[early].max()
thr = z0 + 0.90 * (zmax - z0)
idx = np.argmax(gz >= thr)                          # first index reaching 90% of climb
t_to = t[idx]
print("BAG:", Path(BAG).name)
print("GT z: start=%.2f  early-max(<60s)=%.2f  climb=%.2f m  -> takeoff ends ~%.1fs (idx %d)"
      % (z0, zmax, zmax - z0, t_to, idx))
print("pairs total=%d  duration=%.1fs\n" % (gt_s.num_poses, t[-1]))

for label, mask in [("FULL FLIGHT", np.ones_like(t, bool)),
                    ("TAKEOFF win", t <= t_to + 1.0)]:
    g = copy.deepcopy(gt_s); g.reduce_to_ids(np.where(mask)[0])
    e = copy.deepcopy(est_s); e.reduce_to_ids(np.where(mask)[0])
    eo = copy.deepcopy(e); eo.align_origin(g)
    ea = copy.deepcopy(e); ea.align(g, correct_scale=False)
    print("[%s]  (%d pairs, %.1fs)" % (label, g.num_poses, (t[mask][-1] - t[mask][0])))
    axis_stats(g, e, "RAW")
    axis_stats(g, eo, "align_origin")
    axis_stats(g, ea, "SE3 best-fit")
    print()

# coarse time table over takeoff (origin-aligned) so we SEE where error grows
eo = copy.deepcopy(est_s); eo.align_origin(gt_s)
d = eo.positions_xyz - gt_s.positions_xyz
print("t[s]  gt_z   ex     ey     ez    |e|   (origin-aligned, every ~2s through takeoff)")
last = -10
for i in range(len(t)):
    if t[i] - last < 2.0 or t[i] > t_to + 6:
        if t[i] <= t_to + 6:
            continue
    last = t[i]
    print("%4.1f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f"
          % (t[i], gt_s.positions_xyz[i, 2], d[i, 0], d[i, 1], d[i, 2], np.linalg.norm(d[i])))
    if t[i] > t_to + 6:
        break

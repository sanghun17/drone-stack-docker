#!/usr/bin/env python3
# Validate OFFLINE (no rebuild) the proposed fix: the gt_odom->odom path is missing the
# optical->ROS axis flip Q(0.5,-0.5,0.5,-0.5) that the fallback path applies.
#   current(buggy): odom_pos = R_vrpn0 @ pos_end            + t_vrpn0
#   fixed:          odom_pos = R_vrpn0 @ R_o2r @ pos_end    + t_vrpn0
# pos_end is recorded as /aft_mapped_to_init (camera_init optical frame, gravity_align off).
import copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync, metrics, trajectory

BAG = "/work/tools/fastlivo/flight_clean_livo.bag"
with Rosbag1Reader(Path(BAG)) as r:
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    ainit = file_interface.read_bag_trajectory(r, "/aft_mapped_to_init")

R_o2r = Rot.from_quat([0.5, -0.5, 0.5, -0.5]).as_matrix()   # exactly the code's fallback flip
print("R_o2r maps optical down[0,1,0]->[%+.2f %+.2f %+.2f] (want world-down [0,0,-1])" % tuple(R_o2r @ [0, 1, 0]))

gt_s, ai_s = sync.associate_trajectories(gt, ainit, max_diff=0.02)
t = gt_s.timestamps - gt_s.timestamps[0]
R_v0 = Rot.from_quat([*gt_s.orientations_quat_wxyz[0][[1, 2, 3]], gt_s.orientations_quat_wxyz[0][0]]).as_matrix()
t_v0 = gt_s.positions_xyz[0]
P = ai_s.positions_xyz                                       # pos_end, optical camera_init


def recon(flip):
    R = R_v0 @ R_o2r if flip else R_v0
    return (R @ P.T).T + t_v0


def ape(pos, mask, align):
    g = copy.deepcopy(gt_s); g.reduce_to_ids(np.where(mask)[0])
    e = trajectory.PoseTrajectory3D(positions_xyz=pos[mask],
                                    orientations_quat_wxyz=ai_s.orientations_quat_wxyz[mask],
                                    timestamps=ai_s.timestamps[mask])
    if align:
        e.align_origin(g)
    m = metrics.APE(metrics.PoseRelation.translation_part)
    m.process_data((g, e))
    return m.get_statistic(metrics.StatisticsType.rmse)


gz = gt_s.positions_xyz[:, 2]; z0 = gz[0]
zmax = gz[t < 60].max(); thr = z0 + 0.9 * (zmax - z0)
idx = int(np.argmax(gz >= thr))
to = t <= t[idx] + 1.0           # takeoff window
pre = t <= 160.0                 # pre-divergence (whole flight diverges ~240s)

buggy, fixed = recon(False), recon(True)
print("\nclimb (start->top): GT %.2f | buggy z %.2f | FIXED z %.2f m" %
      (gz[idx] - z0, buggy[idx, 2] - buggy[0, 2], fixed[idx, 2] - fixed[0, 2]))

for name, mask in [("TAKEOFF (<=%.0fs)" % (t[idx] + 1), to), ("PRE-DIVERGENCE (<=160s)", pre)]:
    print("\n[%s]  %d pairs" % (name, mask.sum()))
    print("  buggy  APE  raw %.2f  origin %.2f m" % (ape(buggy, mask, False), ape(buggy, mask, True)))
    print("  FIXED  APE  raw %.2f  origin %.2f m" % (ape(fixed, mask, False), ape(fixed, mask, True)))

# cross-check: does buggy recon match the recorded /aft_mapped_to_odom?
with Rosbag1Reader(Path(BAG)) as r:
    aodom = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")
go, ao = sync.associate_trajectories(gt, aodom, max_diff=0.02)
# recompute buggy on the aodom-associated init set isn't trivial; instead compare end magnitudes
print("\nsanity: recorded aft_odom last|p|=%.1f  buggy-recon last|p|=%.1f (should be close)" %
      (np.linalg.norm(ao.positions_xyz[-1]), np.linalg.norm(buggy[-1])))

#!/usr/bin/env python3
# verify BOTH position and attitude of /aft_mapped_to_odom vs GT (after the full optical->ROS fix).
# expectation: RAW ~= align_origin (no more attitude-offset artifact), init attitude err ~0.
import sys, copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync, metrics

BAG = sys.argv[1] if len(sys.argv) > 1 else "/work/tools/fastlivo/flight_fixed90b_livo.bag"
with Rosbag1Reader(Path(BAG)) as r:
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")
WIN = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
g, e = sync.associate_trajectories(gt, od, max_diff=0.02)
ids = np.where((g.timestamps - g.timestamps[0]) <= WIN)[0]
g.reduce_to_ids(ids); e.reduce_to_ids(ids)
t = g.timestamps - g.timestamps[0]


def ape_pos(est, align):
    ee = copy.deepcopy(est)
    if align:
        ee.align_origin(g)
    mt = metrics.APE(metrics.PoseRelation.translation_part)
    mt.process_data((g, ee))
    return mt.get_statistic(metrics.StatisticsType.rmse)


# attitude: relative rotation error est vs gt, per pose (geodesic deg)
def rot_err_deg(qg, qe):
    Rg = Rot.from_quat([qg[1], qg[2], qg[3], qg[0]])
    Re = Rot.from_quat([qe[1], qe[2], qe[3], qe[0]])
    return (Rg.inv() * Re).magnitude() * 180 / np.pi


errs = np.array([rot_err_deg(g.orientations_quat_wxyz[i], e.orientations_quat_wxyz[i]) for i in range(g.num_poses)])
print("BAG:", Path(BAG).name, " pairs:", g.num_poses, " dur %.1fs" % t[-1])
print("\nPOSITION (translation APE, m):")
print("  RAW          %.3f" % ape_pos(e, False))
print("  align_origin %.3f   (should ~= RAW now; big gap = leftover attitude offset)" % ape_pos(e, True))
print("\nATTITUDE (est vs GT body orientation, geodesic deg):")
print("  at init (t=0)   %.1f deg   (should be ~0 if attitude frame correct)" % errs[0])
print("  mean over 90s   %.1f deg" % errs.mean())
print("  median          %.1f deg" % np.median(errs))
# yaw specifically
yaw = lambda q: np.degrees(np.arctan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2)))
print("  init yaw: GT %.1f  est %.1f deg" % (yaw(g.orientations_quat_wxyz[0]), yaw(e.orientations_quat_wxyz[0])))

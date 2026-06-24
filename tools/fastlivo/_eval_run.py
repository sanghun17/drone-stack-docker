#!/usr/bin/env python3
# unified per-run metrics for the overnight sweep. Works for any config:
#  - aft_odom RAW/att: valid when the gt_odom mapping matches the build (fixed binary, gravity_align off)
#  - aft_init Umeyama (pos RMSE + scale): FRAME-INVARIANT estimator quality (use to compare gravity_align on/off)
#  - climb scale: |aft_init takeoff disp| / |GT takeoff disp| (frame-invariant magnitude ratio)
import sys, copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync, metrics

BAG = sys.argv[1]
with Rosbag1Reader(Path(BAG)) as r:
    have = set(r.topics)
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")
    ai = file_interface.read_bag_trajectory(r, "/aft_mapped_to_init")
from _eval_offset import correct_est_offset  # est stamp=ros::Time::now() lags GT by processing latency
od, _o = correct_est_offset(gt, od); ai, _a = correct_est_offset(gt, ai)


def att_mean(g, e):
    return float(np.mean([(Rot.from_quat([*g.orientations_quat_wxyz[i][[1, 2, 3]], g.orientations_quat_wxyz[i][0]]).inv()
                           * Rot.from_quat([*e.orientations_quat_wxyz[i][[1, 2, 3]], e.orientations_quat_wxyz[i][0]])).magnitude()
                          for i in range(g.num_poses)])) * 180 / np.pi


def w(g, e, lo, hi):
    t = g.timestamps - g.timestamps[0]; ids = np.where((t >= lo) & (t < hi))[0]
    gg = copy.deepcopy(g); gg.reduce_to_ids(ids); ee = copy.deepcopy(e); ee.reduce_to_ids(ids)
    return gg, ee


def odom_raw(lo, hi):
    g, e = sync.associate_trajectories(gt, od, max_diff=0.02)
    gg, ee = w(g, e, lo, hi)
    pos = np.sqrt(np.mean(np.linalg.norm(ee.positions_xyz - gg.positions_xyz, axis=1) ** 2))
    return pos, np.max(np.linalg.norm(ee.positions_xyz - gg.positions_xyz, axis=1)), att_mean(gg, ee)


def init_umeyama(lo, hi):
    g, e = sync.associate_trajectories(gt, ai, max_diff=0.02)
    gg, ee = w(g, e, lo, hi)
    ea = copy.deepcopy(ee); ea.align(gg, correct_scale=False)
    m = metrics.APE(metrics.PoseRelation.translation_part); m.process_data((gg, ea))
    return m.get_statistic(metrics.StatisticsType.rmse)


def climb_scale():
    g, e = sync.associate_trajectories(gt, ai, max_diff=0.02)
    t = g.timestamps - g.timestamps[0]; gz = g.positions_xyz[:, 2]
    zmax = gz[t < 60].max(); idx = int(np.argmax(gz >= gz[0] + 0.9 * (zmax - gz[0])))
    dgt = np.linalg.norm(g.positions_xyz[idx] - g.positions_xyz[0])
    dai = np.linalg.norm(e.positions_xyz[idx] - e.positions_xyz[0])
    return dai / (dgt + 1e-9)


pre = odom_raw(0, 190); yaw = odom_raw(200, 240)
print("%-26s | pre odomRAW=%.3f att=%4.1f initUmey=%.3f climbScale=%.2f | yaw odomPos=%5.2f/max%6.2f att=%5.1f initUmey=%.3f"
      % (Path(BAG).stem, pre[0], pre[2], init_umeyama(0, 190), climb_scale(),
         yaw[0], yaw[1], yaw[2], init_umeyama(200, 240)))

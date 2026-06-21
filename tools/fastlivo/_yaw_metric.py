#!/usr/bin/env python3
# yaw-window attitude/position error of /aft_mapped_to_odom vs GT, for the imu_time_offset A/B.
import sys, copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync

BAG = sys.argv[1]
LO = float(sys.argv[2]) if len(sys.argv) > 2 else 200.0
HI = float(sys.argv[3]) if len(sys.argv) > 3 else 240.0
with Rosbag1Reader(Path(BAG)) as r:
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")
g, e = sync.associate_trajectories(gt, od, max_diff=0.02)
t = g.timestamps - g.timestamps[0]


def att(gg, ee):
    return np.array([(Rot.from_quat([*gg.orientations_quat_wxyz[i][[1, 2, 3]], gg.orientations_quat_wxyz[i][0]]).inv()
                      * Rot.from_quat([*ee.orientations_quat_wxyz[i][[1, 2, 3]], ee.orientations_quat_wxyz[i][0]])).magnitude() * 180 / np.pi
                     for i in range(gg.num_poses)])


def win(lo, hi):
    ids = np.where((t >= lo) & (t < hi))[0]
    gg = copy.deepcopy(g); gg.reduce_to_ids(ids)
    ee = copy.deepcopy(e); ee.reduce_to_ids(ids)
    ae = att(gg, ee); pe = np.linalg.norm(ee.positions_xyz - gg.positions_xyz, axis=1)
    return ae.mean(), pe.mean(), pe.max()


pre = win(0, 190)
early = win(200, 220)   # early yaw (growth, more reproducible than saturated chaos)
full = win(LO, HI)
print("%-28s pre(0-190): att %4.1f pos %.2f | earlyYaw(200-220): att %5.1f pos %4.2f | yaw(%g-%g): att %5.1f pos %5.2f/max%5.2f"
      % (Path(BAG).stem, pre[0], pre[1], early[0], early[1], LO, HI, full[0], full[1], full[2]))

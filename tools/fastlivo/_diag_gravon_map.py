#!/usr/bin/env python3
# For gravity_align=ON the internal camera_init is z-up (gravity-aligned), so the published
# /aft_mapped_to_odom needs a DIFFERENT gt_odom mapping than the optical-flip one (which is for
# gravity_align off). Find the rotation that aligns gravon aft_init to GT (Umeyama over pre-yaw)
# and decompose it: if it's ~pure yaw, the correct mapping for gravity_align=on is Rz(yaw) only.
import sys, copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync

BAG = sys.argv[1] if len(sys.argv) > 1 else "/work/tools/fastlivo/wv1_gravon1_livo.bag"
with Rosbag1Reader(Path(BAG)) as r:
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    ai = file_interface.read_bag_trajectory(r, "/aft_mapped_to_init")
g, e = sync.associate_trajectories(gt, ai, max_diff=0.02)
t = g.timestamps - g.timestamps[0]
ids = np.where(t <= 190)[0]
gg = copy.deepcopy(g); gg.reduce_to_ids(ids); ee = copy.deepcopy(e); ee.reduce_to_ids(ids)
R, tr, s = ee.align(gg, correct_scale=True)
print("BAG:", Path(BAG).stem)
print("Umeyama R (aft_init->GT) euler ZYX [yaw,pitch,roll] deg:", np.round(Rot.from_matrix(R).as_euler('ZYX', degrees=True), 1))
print("  scale=%.3f  |t|=%.2f m" % (s, np.linalg.norm(tr)))
# vrpn init orientation yaw, for comparison
q0 = g.orientations_quat_wxyz[0]
print("vrpn_init euler ZYX [yaw,pitch,roll] deg:", np.round(Rot.from_quat([q0[1], q0[2], q0[3], q0[0]]).as_euler('ZYX', degrees=True), 1))
print("=> if Umeyama R is ~pure yaw (pitch,roll~0) and matches vrpn yaw, then gravity_align=on")
print("   mapping is Rz(vrpn_yaw_init) only (NO optical flip). Internal frame is already z-up.")

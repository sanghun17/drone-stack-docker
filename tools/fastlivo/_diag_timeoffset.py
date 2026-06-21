#!/usr/bin/env python3
# Is the yaw error partly a TIME-OFFSET (improvable) or pure divergence (not)?
# Scan a time shift on the estimate; find the dt minimizing attitude error pre-divergence,
# then check whether the yaw-phase error survives optimal time-alignment.
import copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync, trajectory

BAG = "/work/tools/fastlivo/flight_fixed_full_livo.bag"
with Rosbag1Reader(Path(BAG)) as r:
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")


def att_err(g, e):
    return np.array([(Rot.from_quat([*g.orientations_quat_wxyz[i][[1, 2, 3]], g.orientations_quat_wxyz[i][0]]).inv()
                      * Rot.from_quat([*e.orientations_quat_wxyz[i][[1, 2, 3]], e.orientations_quat_wxyz[i][0]])).magnitude() * 180 / np.pi
                     for i in range(g.num_poses)])


def windowed(g, e, lo, hi):
    t = g.timestamps - g.timestamps[0]
    ids = np.where((t >= lo) & (t < hi))[0]
    gg = copy.deepcopy(g); gg.reduce_to_ids(ids)
    ee = copy.deepcopy(e); ee.reduce_to_ids(ids)
    return gg, ee


print("dt[ms]  preYaw_attErr(0-190s)  yawPhase_attErr(200-240s)   posRMSE_pre")
best = (1e9, 0)
for dt in np.arange(-0.10, 0.101, 0.02):
    es = trajectory.PoseTrajectory3D(positions_xyz=od.positions_xyz.copy(),
                                     orientations_quat_wxyz=od.orientations_quat_wxyz.copy(),
                                     timestamps=od.timestamps + dt)
    g, e = sync.associate_trajectories(gt, es, max_diff=0.012)
    gp, ep = windowed(g, e, 0, 190)
    gy, ey = windowed(g, e, 200, 240)
    ae_pre = att_err(gp, ep).mean(); ae_yaw = att_err(gy, ey).mean()
    pos_pre = np.sqrt(np.mean(np.linalg.norm(ep.positions_xyz - gp.positions_xyz, axis=1) ** 2))
    print("  %+4.0f      %6.1f                 %6.1f                  %.3f" % (dt * 1000, ae_pre, ae_yaw, pos_pre))
    if ae_pre < best[0]:
        best = (ae_pre, dt)
print("\nbest pre-yaw attitude at dt=%+.0fms (att %.1f deg)" % (best[1] * 1000, best[0]))
print("=> if yawPhase_attErr stays ~10-30deg at EVERY dt, the yaw blow-up is genuine divergence,")
print("   not a time offset (a constant shift can't undo an unbounded/diverging error).")

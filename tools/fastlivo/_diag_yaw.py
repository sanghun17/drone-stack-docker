#!/usr/bin/env python3
# Is the yaw-phase divergence REAL (internal observability) or another artifact (mapping / GT dropout / CPU stall)?
import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync

BAG = sys.argv[1] if len(sys.argv) > 1 else "/work/tools/fastlivo/flight_fixed_full_livo.bag"
with Rosbag1Reader(Path(BAG)) as r:
    gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
    od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")
    ai = file_interface.read_bag_trajectory(r, "/aft_mapped_to_init")


def yaw(q):  # wxyz
    return np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]),
                                 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2)))


# 1) GT continuity (dropout check) — raw vrpn timestamps
tg = gt.timestamps - gt.timestamps[0]
dtg = np.diff(tg)
print("[GT vrpn continuity] rate~%.0fHz  max gap %.3fs @ t=%.1fs   (big gap => dropout could fake divergence)"
      % (1 / np.median(dtg), dtg.max(), tg[1:][np.argmax(dtg)]))

# 2) odom publish rate over time (CPU-stall / frame-drop check)
to = od.timestamps - od.timestamps[0]
print("\n[aft_odom rate per 20s bin] (drops far below 10Hz during yaw => real-time stall, not observability)")
for lo in range(0, int(to[-1]), 20):
    seg = (to >= lo) & (to < lo + 20)
    if seg.sum() > 1:
        print("  t=%3d-%3ds  %4.1f Hz" % (lo, lo + 20, seg.sum() / 20.0), end="")
        if (lo // 20) % 4 == 3:
            print()
print()

# 3) internal vs mapped divergence (takeoff bug was mapping-only; if aft_init also blows up => internal)
print("\n[internal vs mapped] last |pos|:  aft_odom %.1f m   aft_init %.1f m" %
      (np.linalg.norm(od.positions_xyz[-1]), np.linalg.norm(ai.positions_xyz[-1])))
print("  (aft_init = raw internal estimate, NO vrpn mapping. both huge => TRUE internal divergence, not a frame artifact)")

# 4) yaw tracking + attitude/position error through the divergence
gs, es = sync.associate_trajectories(gt, od, max_diff=0.02)
t = gs.timestamps - gs.timestamps[0]
ygt = np.unwrap(np.radians(yaw(gs.orientations_quat_wxyz)))
yaw_rate = np.gradient(np.degrees(ygt), t)                     # deg/s
yg = yaw(gs.orientations_quat_wxyz); ye = yaw(es.orientations_quat_wxyz)
atterr = np.array([(Rot.from_quat([gs.orientations_quat_wxyz[i][[1, 2, 3]].tolist() + [gs.orientations_quat_wxyz[i][0]]][0]).inv()
                    * Rot.from_quat(es.orientations_quat_wxyz[i][[1, 2, 3]].tolist() + [es.orientations_quat_wxyz[i][0]])).magnitude() * 180 / np.pi
                   for i in range(gs.num_poses)])
poserr = np.linalg.norm(es.positions_xyz - gs.positions_xyz, axis=1)
print("\n[through divergence]  t   GTyaw  estYaw  yawRate  attErr  posErr   (does attitude break BEFORE position?)")
last = -1e9
for i in range(len(t)):
    if t[i] >= 150 and (t[i] - last >= 5 or t[i] > 250):
        last = t[i]
        print("  %5.1fs  %6.1f %6.1f   %6.1f   %6.1f  %7.2f" % (t[i], yg[i], ye[i], yaw_rate[i], atterr[i], poserr[i]))
    if t[i] > 250:
        break
print("\nmax |GT yaw-rate| over flight: %.0f deg/s @ t=%.1fs" % (np.abs(yaw_rate).max(), t[np.argmax(np.abs(yaw_rate))]))

#!/usr/bin/env python3
# Decisive diagnosis of the takeoff residual & whole-flight divergence:
#  - is the vertical climb seen by the INTERNAL estimate (/aft_mapped_to_init, camera_init z-up)
#    but mis-mapped by the vrpn-init multiply (/aft_mapped_to_odom)?  -> frame bug, fixable
#  - or does the internal estimate itself miss the climb / diverge?    -> sensor/estimator limit
import sys, copy
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync

BAG = sys.argv[1] if len(sys.argv) > 1 else "/work/tools/fastlivo/flight_clean_livo.bag"
GT = "/vrpn_client_node/pure/pose"
AODOM = "/aft_mapped_to_odom"
AINIT = "/aft_mapped_to_init"

with Rosbag1Reader(Path(BAG)) as r:
    have = set(r.topics)
    gt = file_interface.read_bag_trajectory(r, GT)
    aodom = file_interface.read_bag_trajectory(r, AODOM)
    ainit = file_interface.read_bag_trajectory(r, AINIT) if AINIT in have else None
print("BAG:", Path(BAG).name, "| topics ok:", {AODOM, AINIT, GT} <= have)

# associate GT with each estimate independently
gt_o, est_o = sync.associate_trajectories(gt, aodom, max_diff=0.02)
t = gt_o.timestamps - gt_o.timestamps[0]
gz = gt_o.positions_xyz[:, 2]; z0 = gz[0]
early = t < 60
zmax = gz[early].max(); thr = z0 + 0.90 * (zmax - z0)
idx = int(np.argmax(gz >= thr))
print("takeoff: GT z %.2f->%.2f (climb %.2f m), ends ~%.1fs (idx %d); total %.1fs, %d pairs\n"
      % (z0, zmax, zmax - z0, t[idx], idx, t[-1], gt_o.num_poses))

# ---- R(vrpn_init): where does the latched init orientation send world-up/forward? ----
q0 = gt_o.orientations_quat_wxyz[0]              # wxyz
R0 = Rot.from_quat([q0[1], q0[2], q0[3], q0[0]]).as_matrix()
print("R(vrpn_init) maps:  up[0,0,1]->[%+.2f %+.2f %+.2f]   fwd[1,0,0]->[%+.2f %+.2f %+.2f]"
      % (*(R0 @ [0, 0, 1]), *(R0 @ [1, 0, 0])))
print("  (if up-> ~[0,0,1] vrpn is Z-up/clean; if it tilts into x/y the mocap convention leaks)\n")

# ---- takeoff displacement vectors (RAW, no alignment) ----
def disp(traj_assoc, lo, hi):
    p = traj_assoc.positions_xyz
    return p[hi] - p[lo]
lo, hi = 0, idx
d_gt = disp(gt_o, lo, hi)
d_od = disp(est_o, lo, hi)
print("RAW takeoff displacement (start->top of climb):")
print("  GT       d=[%+.2f %+.2f %+.2f]  |d|=%.2f" % (*d_gt, np.linalg.norm(d_gt)))
print("  aft_odom d=[%+.2f %+.2f %+.2f]  |d|=%.2f" % (*d_od, np.linalg.norm(d_od)))
cosang = np.dot(d_gt, d_od) / (np.linalg.norm(d_gt) * np.linalg.norm(d_od) + 1e-9)
print("  angle(GT,aft_odom)=%.1f deg   |aft_odom|/|GT|=%.2f" %
      (np.degrees(np.arccos(np.clip(cosang, -1, 1))), np.linalg.norm(d_od) / (np.linalg.norm(d_gt) + 1e-9)))
if ainit is not None:
    gt_i, est_i = sync.associate_trajectories(gt, ainit, max_diff=0.02)
    ti = gt_i.timestamps - gt_i.timestamps[0]
    idxi = int(np.argmax((gt_i.positions_xyz[:, 2]) >= thr))
    d_in = est_i.positions_xyz[idxi] - est_i.positions_xyz[0]
    print("  aft_init d=[%+.2f %+.2f %+.2f]  |d|=%.2f   (camera_init z=up: z-component IS the climb)"
          % (*d_in, np.linalg.norm(d_in)))
    print("  >>> climb seen:  GT %.2f | aft_init z %.2f | aft_odom z %.2f m" % (d_gt[2], d_in[2], d_od[2]))

# ---- Umeyama residual rotation over takeoff (aft_odom vs GT) ----
g = copy.deepcopy(gt_o); g.reduce_to_ids(list(range(0, idx + 1)))
e = copy.deepcopy(est_o); e.reduce_to_ids(list(range(0, idx + 1)))
r_, t_, s_ = e.align(g, correct_scale=True)   # returns R, t, scale
ang = np.degrees(np.arccos(np.clip((np.trace(r_) - 1) / 2, -1, 1)))
print("\nUmeyama(aft_odom->GT) over takeoff: residual rotation %.1f deg, scale %.3f" % (ang, s_))

# ---- whole-flight: internal vs mapped divergence + onset ----
print("\nwhole-flight magnitude (raw, last pose):")
print("  GT       last |p|=%.1f m" % np.linalg.norm(gt_o.positions_xyz[-1]))
print("  aft_odom last |p|=%.1f m" % np.linalg.norm(est_o.positions_xyz[-1]))
if ainit is not None:
    print("  aft_init last |p|=%.1f m  (if this is also huge: TRUE LIVO divergence, not a mapping artifact)"
          % np.linalg.norm(est_i.positions_xyz[-1]))
eo = copy.deepcopy(est_o); eo.align_origin(gt_o)
err = np.linalg.norm(eo.positions_xyz - gt_o.positions_xyz, axis=1)
print("\nerror-over-time (aft_odom origin-aligned), coarse — find divergence onset:")
last = -1e9
for i in range(len(t)):
    if t[i] - last >= 10.0 or i == len(t) - 1:
        last = t[i]
        print("  t=%5.1fs  err=%8.2f m   gt_z=%.2f" % (t[i], err[i], gz[i]))

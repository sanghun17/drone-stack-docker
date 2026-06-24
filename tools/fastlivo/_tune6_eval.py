#!/usr/bin/env python3
# Parallel NO-ALIGN position-RMSE eval of FAST-LIVO2 tuning replays. Reads _t6o_*.bag directly via
# `rosbags` (NO ROS needed) -> runs anywhere with: numpy scipy evo rosbags. Same metric as the
# sequential grid: /aft_mapped_to_optitrack vs /vrpn_client_node/pure/pose, correct_est_offset (time
# only), NO Umeyama align, rank by position RMSE + handle marginals.
#   usage: _tune5_eval.py <dir_with_t5o_bags> <out_rank.txt>
import sys, os, glob, numpy as np
from pathlib import Path
from multiprocessing import Pool
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rosbags.rosbag1 import Reader
from evo.tools import file_interface
from evo.core import sync
from scipy.spatial.transform import Rotation as Rot
from _eval_offset import correct_est_offset

def ev(b):
    name = os.path.basename(b)[5:-4]   # strip "_t5o_" .. ".bag"
    try:
        with Reader(Path(b)) as r:
            gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
            od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_optitrack")
        od, off = correct_est_offset(gt, od)
        ref, es = sync.associate_trajectories(gt, od, max_diff=0.05)
        pe = np.linalg.norm(es.positions_xyz - ref.positions_xyz, axis=1)
        Re = Rot.from_quat(np.c_[es.orientations_quat_wxyz[:, 1:4], es.orientations_quat_wxyz[:, 0]])
        Rg = Rot.from_quat(np.c_[ref.orientations_quat_wxyz[:, 1:4], ref.orientations_quat_wxyz[:, 0]])
        dR = Re * Rg.inv(); eu = (dR.mean().inv() * dR).as_euler('xyz', degrees=True)
        return (name, float(np.sqrt((pe**2).mean())), float(pe.max()),
                float(np.sqrt((eu[:, 0]**2).mean())), float(np.sqrt((eu[:, 1]**2).mean())),
                float(np.sqrt((eu[:, 2]**2).mean())), len(pe))
    except Exception as e:
        return (name, None, None, None, None, None, str(e)[:40])

def main():
    d, out = sys.argv[1], sys.argv[2]
    bags = sorted(glob.glob(os.path.join(d, "_t6o_*.bag")))
    with Pool() as p:
        res = p.map(ev, bags)
    ok = sorted([r for r in res if r[1] is not None], key=lambda x: x[1])
    bad = [r for r in res if r[1] is None]
    o = ["FAST-LIVO2 tune5 — NO-ALIGN position RMSE [m] of /aft_mapped_to_optitrack vs pure (time-corrected)",
         "handles: im{imu}_v{img}_l{lto}_d{dep}_vox{vs}_fs{fsurf}_ml{maxlayer}   (%d/%d evaluated)" % (len(ok), len(bags)), "",
         "rank config                      posRMSE posMax | att r/p/y [deg]  n"]
    for i, (n, rm, mx, rr, pp, yy, k) in enumerate(ok):
        o.append("%3d  %-26s %.3f  %.3f | %.2f/%.2f/%.2f  %d" % (i + 1, n, rm, mx, rr, pp, yy, k))
    o.append("\n=== handle marginals (mean posRMSE per level, lower=better) ===")
    def lvl(pos, val):
        s = [r[1] for r in ok if r[0].split('_')[pos] == val]
        return np.mean(s) if s else float('nan')
    for lab, pos, vals in [("IMU_trust", 0, ["imlo", "imhi"]), ("img_point_cov", 1, ["v100", "v1000"]),
                           ("lidar_off", 2, ["l005", "l010"]), ("dept_err_rel", 3, ["d00", "d01"]),
                           ("voxel_size", 4, ["vox03", "vox05", "vox08"]), ("filter_surf", 5, ["fs005", "fs020"]),
                           ("max_layer", 6, ["ml1", "ml2", "ml3"])]:
        o.append("  %-14s " % lab + "  ".join("%s=%.3f" % (v, lvl(pos, v)) for v in vals))
    if bad:
        o.append("\nFAILED(%d): " % len(bad) + " ".join(r[0] for r in bad))
    txt = "\n".join(o) + "\n"
    print(txt)
    open(out, "w").write(txt)

if __name__ == "__main__":
    main()

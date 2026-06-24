#!/usr/bin/env python3
# Master tuning table builder. Scans EVERY tuning replay bag (_t<N>o_*.bag) on disk, joins each with
# its generated config yaml (_t<N>_<name>.yaml) to recover the FULL lever vector (incl. levers a given
# round held fixed), recomputes the NO-ALIGN position-RMSE metrics from the bag, and writes one tidy
# CSV: tune_master.csv. One row per replay = {round, config, metrics..., every lever...}. Rebuildable
# from scratch anytime (bags + configs persist) -> the growing high-dim "lever values -> performance"
# table across all rounds (v5, v6, v7, ...).
#   usage: build_master.py <dir_with_bags_and_configs> <out_master.csv>
import sys, os, glob, csv, re, numpy as np, yaml
from pathlib import Path
from multiprocessing import Pool
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rosbags.rosbag1 import Reader
from evo.tools import file_interface
from evo.core import sync
from scipy.spatial.transform import Rotation as Rot
from _eval_offset import correct_est_offset

# (section, key, column) — the lever schema. Section None = top level. Duplicate keys (max_iterations)
# disambiguated by section. Missing key -> blank cell (e.g. an old round predating a lever).
LEVERS = [
    ("imu","acc_cov","acc_cov"), ("imu","gyr_cov","gyr_cov"),
    ("imu","b_acc_cov","b_acc_cov"), ("imu","b_gyr_cov","b_gyr_cov"), ("imu","imu_int_frame","imu_int_frame"),
    ("vio","img_point_cov","img_point_cov"), ("vio","patch_size","patch_size"),
    ("vio","patch_pyrimid_level","patch_pyr_level"), ("vio","max_iterations","vio_max_iter"),
    ("vio","inv_expo_cov","inv_expo_cov"), ("vio","exposure_estimate_en","exposure_est_en"),
    ("time_offset","imu_time_offset","imu_time_offset"), ("time_offset","img_time_offset","img_time_offset"),
    ("time_offset","lidar_time_offset","lidar_time_offset"),
    ("lio","dept_err","dept_err"), ("lio","beam_err","beam_err"), ("lio","dept_err_rel","dept_err_rel"),
    ("lio","min_eigen_value","min_eigen_value"), ("lio","voxel_size","voxel_size"),
    ("lio","max_layer","max_layer"), ("lio","max_points_num","max_points_num"), ("lio","max_iterations","lio_max_iter"),
    ("preprocess","point_filter_num","point_filter_num"), ("preprocess","filter_size_surf","filter_size_surf"),
    ("preprocess","blind","blind"),
    ("uav","gravity_align_en","gravity_align_en"),
]
METRIC_COLS = ["posRMSE","posMax","att_r","att_p","att_y","n","coverage","gt_dist","valid","converged"]
COLS = ["round","config"] + METRIC_COLS + [c for _,_,c in LEVERS]

def levers_from_cfg(cfg_path):
    out = {c: "" for _,_,c in LEVERS}
    try:
        with open(cfg_path) as fh: y = yaml.safe_load(fh)
    except Exception:
        return out
    for sec,key,col in LEVERS:
        node = y.get(sec) if sec else y
        if isinstance(node, dict) and key in node:
            out[col] = node[key]
    return out

def metrics_from_bag(b):
    try:
        with Reader(Path(b)) as r:
            gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
            od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_optitrack")
        # coverage: did the estimate span the whole GT flight, or terminate early? A config whose
        # node dies/stops mid-flight only reports the easy early poses -> artificially low RMSE.
        # coverage<1 must disqualify it, else "die early" games the no-align metric (see v8 pl=2).
        gt_span = float(gt.timestamps.max() - gt.timestamps.min())
        est_span = float(od.timestamps.max() - od.timestamps.min())
        cov = round(est_span / gt_span, 3) if gt_span > 0 else 0.0
        od,_ = correct_est_offset(gt, od)
        ref, es = sync.associate_trajectories(gt, od, max_diff=0.05)
        gt_dist = round(float(np.sum(np.linalg.norm(np.diff(ref.positions_xyz, axis=0), axis=1))), 3)
        pe = np.linalg.norm(es.positions_xyz - ref.positions_xyz, axis=1)
        Re = Rot.from_quat(np.c_[es.orientations_quat_wxyz[:,1:4], es.orientations_quat_wxyz[:,0]])
        Rg = Rot.from_quat(np.c_[ref.orientations_quat_wxyz[:,1:4], ref.orientations_quat_wxyz[:,0]])
        dR = Re*Rg.inv(); eu = (dR.mean().inv()*dR).as_euler('xyz', degrees=True)
        rmse = float(np.sqrt((pe**2).mean()))
        # valid = tracked the whole flight (cov>=0.9) AND stayed bounded (rmse<0.5)
        return dict(posRMSE=round(rmse,4), posMax=round(float(pe.max()),4),
                    att_r=round(float(np.sqrt((eu[:,0]**2).mean())),3),
                    att_p=round(float(np.sqrt((eu[:,1]**2).mean())),3),
                    att_y=round(float(np.sqrt((eu[:,2]**2).mean())),3),
                    n=len(pe), coverage=cov, gt_dist=gt_dist,
                    valid=int(rmse < 0.5 and cov >= 0.9), converged=int(rmse < 0.5))
    except Exception as e:
        return dict(posRMSE="", posMax="", att_r="", att_p="", att_y="", n="",
                    coverage="", gt_dist="", valid=0, converged=0)

def one(b):
    base = os.path.basename(b)
    m = re.match(r"_t(\d+)o_(.+)\.bag$", base)
    if not m: return None
    rnd, name = "v"+m.group(1), m.group(2)
    d = os.path.dirname(b)
    cfg = os.path.join(d, f"_t{m.group(1)}_{name}.yaml")
    row = {"round": rnd, "config": name}
    row.update(metrics_from_bag(b))
    row.update(levers_from_cfg(cfg))
    return row

def main():
    d, out = sys.argv[1], sys.argv[2]
    bags = sorted(glob.glob(os.path.join(d, "_t*o_*.bag")))
    with Pool() as p: rows = [r for r in p.map(one, bags) if r]
    # sort by converged desc then posRMSE asc (blanks last)
    rows.sort(key=lambda r: (0 if r["converged"] else 1,
                             r["posRMSE"] if isinstance(r["posRMSE"],float) else 1e9))
    with open(out,"w",newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS); w.writeheader()
        for r in rows: w.writerow(r)
    # also emit a typed/compressed parquet (the real source-of-truth for analysis); CSV stays for
    # human/git readability. Optional: skip silently if pandas/pyarrow not installed.
    try:
        import pandas as pd
        pq = os.path.splitext(out)[0] + ".parquet"
        df = pd.DataFrame(rows, columns=COLS)
        for c in df.columns:
            if c not in ("round","config"): df[c] = pd.to_numeric(df[c], errors="ignore")
        df.to_parquet(pq, index=False)
        print(f"wrote parquet -> {pq}")
    except Exception as e:
        print(f"(parquet skipped: {e})")
    rounds = {}
    for r in rows: rounds[r["round"]] = rounds.get(r["round"],0)+1
    print(f"wrote {len(rows)} rows -> {out}")
    print("per round:", dict(sorted(rounds.items())))
    print("columns:", len(COLS), "=", ", ".join(COLS))

if __name__ == "__main__":
    main()

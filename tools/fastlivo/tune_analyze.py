#!/usr/bin/env python3
# Analysis layer over the tidy master table. The CSV/parquet is the source of truth (one row per
# replay, every lever recorded incl. the ones a round fixed). This script materializes the
# "high-dim lever -> performance" labeled array ON DEMAND with pandas+xarray: pick any set of levers
# and get an N-D xarray.DataArray (cell = mean metric over replicates) plus per-axis marginals.
#
#   tune_analyze.py [master.parquet]                       -> overview (per-round, top configs, 1-D marginals)
#   tune_analyze.py [master.parquet] lever1 lever2 ...     -> N-D array over those levers + marginals
#       e.g. tune_analyze.py tune_master.parquet filter_size_surf voxel_size img_point_cov
import sys, os
import pandas as pd

DEF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tune_master.parquet")

def load(path):
    if path.endswith(".csv") or not os.path.exists(path):
        csvp = path if path.endswith(".csv") else os.path.splitext(path)[0] + ".csv"
        df = pd.read_csv(csvp)
    else:
        df = pd.read_parquet(path)
    df["converged"] = df["converged"].astype(int)
    return df

def overview(df):
    print(f"=== master table: {len(df)} rows, {df.shape[1]} cols ===")
    print("rounds:", df.groupby("round").size().to_dict())
    print("\n=== per-round (converged = posRMSE<0.5m) ===")
    g = df.groupby("round").agg(n=("config","size"),
                                converged=("converged","sum"),
                                best=("posRMSE","min"),
                                median_conv=("posRMSE", lambda s: s[df.loc[s.index,"converged"]==1].median()))
    print(g.to_string())
    print("\n=== top 8 configs overall (by posRMSE) ===")
    cols = ["round","config","posRMSE","posMax","att_y","voxel_size","filter_size_surf",
            "img_point_cov","beam_err","point_filter_num","max_points_num","gravity_align_en"]
    cols = [c for c in cols if c in df.columns]
    print(df.nsmallest(8,"posRMSE")[cols].to_string(index=False))
    # 1-D marginals (converged only) for every lever that actually varies
    print("\n=== 1-D marginals: mean posRMSE per level (converged only; only varying levers) ===")
    conv = df[df.converged==1]
    meta = {"round","config","posRMSE","posMax","att_r","att_p","att_y","n","converged"}
    for c in df.columns:
        if c in meta: continue
        if df[c].nunique(dropna=True) < 2: continue
        m = conv.groupby(c)["posRMSE"].agg(["mean","size"])
        cells = "  ".join(f"{k}={r['mean']:.3f}(n{int(r['size'])})" for k,r in m.iterrows())
        print(f"  {c:18s} {cells}")

def ndview(df, levers):
    miss = [l for l in levers if l not in df.columns]
    if miss: sys.exit(f"unknown levers: {miss}")
    conv = df[df.converged==1].dropna(subset=levers)
    # THE high-dim labeled array: group by the chosen lever axes -> N-D xarray.DataArray
    mean_da = conv.groupby(levers)["posRMSE"].mean().to_xarray()
    cnt_da  = conv.groupby(levers)["posRMSE"].size().to_xarray()
    print(f"=== N-D array  mean posRMSE  over axes {levers}  (converged rows: {len(conv)}) ===")
    print("shape:", dict(zip(mean_da.dims, mean_da.shape)))
    print(mean_da.to_series().dropna().to_string())
    print(f"\nbest cell: {mean_da.where(mean_da==mean_da.min(), drop=True).coords}")
    print("\n=== per-axis marginals (mean over the other axes) ===")
    for ax in levers:
        others = [d for d in mean_da.dims if d != ax]
        marg = mean_da.mean(dim=others)
        print(f"  {ax}: " + "  ".join(f"{float(v):g}->{float(marg.sel({ax:v})):.3f}" for v in marg[ax].values))
    print("\n(cell counts:)")
    print(cnt_da.to_series().dropna().astype(int).to_string())

def main():
    args = sys.argv[1:]
    path = DEF
    if args and (args[0].endswith(".parquet") or args[0].endswith(".csv")):
        path = args.pop(0)
    df = load(path)
    if args: ndview(df, args)
    else: overview(df)

if __name__ == "__main__":
    main()

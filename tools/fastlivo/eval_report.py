#!/usr/bin/env python3
"""
FAST-LIVO2 open-loop accuracy report vs OptiTrack GT, built on the `evo` library.

For each replay `*_livo.bag` (holding the estimate /aft_mapped_to_init AND the GT
/vrpn_client_node/pure/pose on the same sim clock), this:
  - reads both trajectories with evo, time-associates them,
  - aligns two ways: ORIGIN (compensate only the initial constant SE3 offset = honest
    drift) and SE3 Umeyama (conventional best-fit APE),
  - computes APE RMSE (both) + the final-pose error (end-point drift),
  - and assembles ONE multi-page PDF (summary table + per-bag x/y/z/yaw vs GT, error
    over time, top-down XY) via matplotlib PdfPages.

It composes evo's metrics/plots — it does not re-implement them.

    python3 tools/fastlivo/eval_report.py --out report.pdf [opts] BAG [BAG ...]
      BAG            one or more replay *_livo.bag (label = filename stem)
      --out PDF      output report (default: fastlivo_eval_report.pdf)
      --est TOPIC    estimate topic   (default /aft_mapped_to_init)
      --gt  TOPIC    ground-truth topic (default /vrpn_client_node/pure/pose)
      --t-max-diff S association window (default 0.05)
      --label L=BAG  optional explicit label for a bag (repeatable)
"""
import argparse, copy, os, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

from rosbags.rosbag1 import Reader as Rosbag1Reader   # evo>=1.31 reads bags via rosbags
from evo.tools import file_interface
from evo.core import sync, metrics


def yaw_deg(quat_wxyz):
    w, x, y, z = quat_wxyz.T
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _ape(gt_s, est_s):
    m = metrics.APE(metrics.PoseRelation.translation_part)
    m.process_data((gt_s, est_s))
    return m.get_statistic(metrics.StatisticsType.rmse), np.asarray(m.error)


def evaluate(bag_path, est_topic, gt_topic, max_diff):
    with Rosbag1Reader(Path(bag_path)) as reader:
        have = set(reader.topics)
        if est_topic not in have or gt_topic not in have:
            raise RuntimeError("missing %s" % (
                [t for t in (est_topic, gt_topic) if t not in have]))
        est = file_interface.read_bag_trajectory(reader, est_topic)
        gt = file_interface.read_bag_trajectory(reader, gt_topic)
    gt_s, est_s = sync.associate_trajectories(gt, est, max_diff=max_diff)
    est_o = copy.deepcopy(est_s); est_o.align_origin(gt_s)        # initial-offset only
    est_e = copy.deepcopy(est_s); est_e.align(gt_s, correct_scale=False)  # SE3 Umeyama
    rmse_o, err_o = _ape(gt_s, est_o)
    rmse_e, err_e = _ape(gt_s, est_e)
    t = gt_s.timestamps - gt_s.timestamps[0]
    return dict(bag=os.path.basename(bag_path), n=est_s.num_poses, t=t, gt=gt_s,
                est_o=est_o, est_e=est_e, rmse_o=rmse_o, rmse_e=rmse_e,
                err_o=err_o, err_e=err_e, final_o=float(err_o[-1]),
                final_e=float(err_e[-1]), dur=float(t[-1] - t[0]))


def page_for_bag(pdf, label, r):
    fig, ax = plt.subplots(2, 3, figsize=(11.7, 8.3))
    fig.suptitle("%s   |   APE RMSE: origin %.1f cm,  SE3 %.1f cm   |   final drift %.1f cm   "
                 "|   %d pairs, %.1fs" % (label, r["rmse_o"] * 100, r["rmse_e"] * 100,
                                          r["final_o"] * 100, r["n"], r["dur"]), fontsize=11)
    t = r["t"]
    gxyz = r["gt"].positions_xyz
    exyz = r["est_o"].positions_xyz          # origin-aligned -> overlaid in GT frame
    for i, axis in enumerate("xyz"):
        a = ax[0, i]
        a.plot(t, gxyz[:, i], lw=1.2, color="green", label="GT")
        a.plot(t, exyz[:, i], lw=1.0, color="tab:blue", label="est (origin-aln)")
        a.set_title(axis); a.set_xlabel("t [s]"); a.set_ylabel("%s [m]" % axis); a.grid(alpha=.3)
        if i == 0: a.legend(fontsize=7, loc="best")
    # yaw
    ax[1, 0].plot(t, yaw_deg(r["gt"].orientations_quat_wxyz), lw=1.2, color="green", label="GT")
    ax[1, 0].plot(t, yaw_deg(r["est_o"].orientations_quat_wxyz), lw=1.0, color="tab:blue", label="est")
    ax[1, 0].set_title("yaw"); ax[1, 0].set_xlabel("t [s]"); ax[1, 0].set_ylabel("deg"); ax[1, 0].grid(alpha=.3)
    # error over time
    ax[1, 1].plot(t, r["err_o"] * 100, lw=1.0, color="tab:red", label="origin-aln")
    ax[1, 1].plot(t, r["err_e"] * 100, lw=1.0, color="tab:orange", alpha=.8, label="SE3-aln")
    ax[1, 1].set_title("APE (translation) over time"); ax[1, 1].set_xlabel("t [s]")
    ax[1, 1].set_ylabel("error [cm]"); ax[1, 1].grid(alpha=.3); ax[1, 1].legend(fontsize=7)
    # top-down XY
    a = ax[1, 2]
    a.plot(gxyz[:, 0], gxyz[:, 1], lw=1.2, color="green", label="GT")
    a.plot(exyz[:, 0], exyz[:, 1], lw=1.0, color="tab:blue", label="est")
    a.scatter([gxyz[0, 0]], [gxyz[0, 1]], c="k", s=20, zorder=5, label="start")
    a.set_title("top-down XY"); a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    a.axis("equal"); a.grid(alpha=.3); a.legend(fontsize=7)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    pdf.savefig(fig); plt.close(fig)


def summary_page(pdf, results):
    fig = plt.figure(figsize=(11.7, 8.3))
    fig.suptitle("FAST-LIVO2 open-loop vs OptiTrack GT — summary", fontsize=13)
    ax = fig.add_subplot(111); ax.axis("off")
    cols = ["bag", "dur [s]", "pairs", "RMSE origin [cm]", "RMSE SE3 [cm]", "final drift [cm]"]
    rows = [[r["bag"], "%.1f" % r["dur"], str(r["n"]), "%.1f" % (r["rmse_o"] * 100),
             "%.1f" % (r["rmse_e"] * 100), "%.1f" % (r["final_o"] * 100)] for r in results]
    tbl = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center")
    tbl.auto_set_font_size(False); tbl.set_fontsize(10); tbl.scale(1, 2.0)
    note = ("origin = align first pose only (compensate the initial constant GT->local offset, "
            "honest drift)\nSE3 = Umeyama best-fit (conventional APE).  final drift = APE of the "
            "last associated pose.")
    fig.text(0.5, 0.18, note, ha="center", fontsize=9, color="0.3")
    pdf.savefig(fig); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--out", default="fastlivo_eval_report.pdf")
    ap.add_argument("--est", default="/aft_mapped_to_init")
    ap.add_argument("--gt", default="/vrpn_client_node/pure/pose")
    ap.add_argument("--t-max-diff", type=float, default=0.05)
    ap.add_argument("--label", action="append", default=[], help="LABEL=BAG override")
    a = ap.parse_args()
    labels = dict(x.split("=", 1) for x in a.label)

    results = []
    for b in a.bags:
        if not os.path.isfile(b):
            print("  SKIP (no file): %s" % b); continue
        try:
            r = evaluate(b, a.est, a.gt, a.t_max_diff)
        except Exception as e:
            print("  FAIL %s: %s" % (b, e)); continue
        r["label"] = labels.get(b, os.path.splitext(os.path.basename(b))[0])
        if r["n"] < 10:
            print("  WARN %s: only %d associated pairs (check clock/topics/--t-max-diff)"
                  % (r["label"], r["n"]))
        print("  %-34s RMSE origin %6.1f cm | SE3 %6.1f cm | final %6.1f cm | %d pairs"
              % (r["label"], r["rmse_o"] * 100, r["rmse_e"] * 100, r["final_o"] * 100, r["n"]))
        results.append(r)
    if not results:
        sys.exit("no evaluable bags")
    with PdfPages(a.out) as pdf:
        summary_page(pdf, results)
        for r in results:
            page_for_bag(pdf, r["label"], r)
        d = pdf.infodict(); d["Title"] = "FAST-LIVO2 open-loop evaluation"
    print("[report] wrote %s (%d bags)" % (a.out, len(results)))


if __name__ == "__main__":
    main()

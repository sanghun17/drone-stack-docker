#!/usr/bin/env python3
# full-flight before/after of the gt_odom optical->ROS fix: error-over-time, z(t), top-down.
import copy
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from rosbags.rosbag1 import Reader as Rosbag1Reader
from evo.tools import file_interface
from evo.core import sync

CASES = [("BEFORE (buggy)", "/work/tools/fastlivo/flight_clean_livo.bag", "tab:red"),
         ("AFTER  (full fix)", "/work/tools/fastlivo/flight_fixed_full_livo.bag", "tab:blue")]
OUT = "/work/tools/fastlivo/fix_full_before_after.pdf"


def load(bag):
    with Rosbag1Reader(Path(bag)) as r:
        gt = file_interface.read_bag_trajectory(r, "/vrpn_client_node/pure/pose")
        od = file_interface.read_bag_trajectory(r, "/aft_mapped_to_odom")
    g, e = sync.associate_trajectories(gt, od, max_diff=0.02)
    t = g.timestamps - g.timestamps[0]
    err = np.linalg.norm(e.positions_xyz - g.positions_xyz, axis=1)   # RAW (pre-aligned topic)
    return g, e, t, err


with PdfPages(OUT) as pdf:
    fig = plt.figure(figsize=(11.7, 8.3))
    fig.suptitle("FAST-LIVO2 /aft_mapped_to_odom vs OptiTrack — gt_odom optical->ROS fix (full 316s flight)", fontsize=12)
    gs = fig.add_gridspec(2, 2)
    ax_e = fig.add_subplot(gs[0, :])      # error-over-time, both
    ax_z = fig.add_subplot(gs[1, 0])      # z(t) takeoff
    ax_xy = fig.add_subplot(gs[1, 1])     # top-down pre-divergence
    data = {}
    for name, bag, c in CASES:
        g, e, t, err = load(bag); data[name] = (g, e, t, err, c)
        ax_e.plot(t, err, color=c, lw=1.1, label="%s  (RAW APE 0-195s: %.2f m)" %
                  (name, np.sqrt(np.mean(err[t <= 195] ** 2))))
        m60 = t <= 60
        ax_z.plot(t[m60], e.positions_xyz[m60, 2], color=c, lw=1.1, label=name)
        m195 = t <= 195
        ax_xy.plot(e.positions_xyz[m195, 0], e.positions_xyz[m195, 1], color=c, lw=1.0, label=name)
    # GT overlays
    g0, _, t0, _, _ = data["AFTER  (full fix)"]
    ax_z.plot(t0[t0 <= 60], g0.positions_xyz[t0 <= 60, 2], color="green", lw=1.5, label="GT z")
    ax_xy.plot(g0.positions_xyz[t0 <= 195, 0], g0.positions_xyz[t0 <= 195, 1], color="green", lw=1.5, label="GT")
    ax_e.axvline(200, color="0.5", ls="--", lw=0.8); ax_e.text(202, 4.2, "yaw maneuver\n-> divergence", fontsize=8, color="0.4")
    ax_e.set_ylim(0, 5); ax_e.set_xlabel("t [s]"); ax_e.set_ylabel("RAW position error [m]")
    ax_e.set_title("position error over time (capped 5m; both diverge in yaw, but pre-yaw: ~1.0m vs ~0.19m)")
    ax_e.grid(alpha=.3); ax_e.legend(fontsize=8, loc="upper left")
    ax_z.set_title("z(t) takeoff (first 60s)"); ax_z.set_xlabel("t [s]"); ax_z.set_ylabel("z [m]")
    ax_z.grid(alpha=.3); ax_z.legend(fontsize=7)
    ax_xy.set_title("top-down XY (pre-yaw 0-195s)"); ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
    ax_xy.axis("equal"); ax_xy.grid(alpha=.3); ax_xy.legend(fontsize=7)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    pdf.savefig(fig); plt.close(fig)
print("wrote", OUT)

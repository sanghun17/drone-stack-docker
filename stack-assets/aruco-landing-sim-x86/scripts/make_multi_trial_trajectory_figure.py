#!/usr/bin/env python3
"""Create Matplotlib 3-D trajectory figures for multiple landing bags."""

import argparse
import glob
import os
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from make_multi_trial_environment_figure import (
    load_yaml,
    read_trajectory,
    trial_number,
)


COLORS = [
    "#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00",
    "#56B4E9", "#F0E442", "#332288", "#44AA99", "#AA4499",
]


def configure_axis(axis, limits, pad_size, elevation, azimuth):
    half = pad_size / 2.0
    xx, yy = np.meshgrid([-half, half], [-half, half])
    axis.plot_surface(
        xx, yy, np.zeros_like(xx), color="#dddddd", alpha=0.30,
        shade=False, edgecolor="#777777", linewidth=0.45,
    )
    axis.set_xlim(*limits[0])
    axis.set_ylim(*limits[1])
    axis.set_zlim(*limits[2])
    axis.set_xlabel(r"$x_P$ (m)", labelpad=2)
    axis.set_ylabel(r"$y_P$ (m)", labelpad=2)
    axis.set_zlabel(r"$z_P$ (m)", labelpad=2)
    axis.view_init(elev=elevation, azim=azimuth)
    axis.grid(True, alpha=0.28)
    if hasattr(axis, "set_box_aspect"):
        spans = [maximum - minimum for minimum, maximum in limits]
        axis.set_box_aspect(spans)


def common_limits(trials, pad_size):
    points = np.vstack([array for _, gt, estimate in trials for array in (gt, estimate)])
    xy_extent = max(
        pad_size / 2.0,
        float(np.max(np.abs(points[:, :2]))) + 0.04,
    )
    z_min = min(0.0, float(points[:, 2].min()) - 0.02)
    z_max = max(0.1, float(points[:, 2].max()) + 0.05)
    return ((-xy_extent, xy_extent), (-xy_extent, xy_extent), (z_min, z_max))


def draw_trial(axis, gt, estimate, color, label=None, thin=False):
    gt_width = 1.1 if thin else 2.0
    estimate_width = 0.9 if thin else 1.45
    axis.plot(
        gt[:, 0], gt[:, 1], gt[:, 2], color=color,
        linewidth=gt_width, alpha=0.92, label=label,
    )
    axis.plot(
        estimate[:, 0], estimate[:, 1], estimate[:, 2], color=color,
        linewidth=estimate_width, linestyle="--", alpha=0.66,
    )
    axis.scatter(*gt[0], color=color, marker="o", s=14 if thin else 28,
                 depthshade=False)
    axis.scatter(*gt[-1], color=color, marker="x", s=18 if thin else 34,
                 depthshade=False)


def make_combined(trials, output, limits, pad_size, args):
    figure = plt.figure(figsize=(7.2, 5.4))
    axis = figure.add_subplot(111, projection="3d")
    configure_axis(axis, limits, pad_size, args.elevation, args.azimuth)
    for index, (number, gt, estimate) in enumerate(trials):
        draw_trial(axis, gt, estimate, COLORS[index % len(COLORS)], "Trial %d" % number)
    style_handles = [
        Line2D([0], [0], color="#333333", lw=1.8, linestyle="-", label="Ground truth"),
        Line2D([0], [0], color="#333333", lw=1.4, linestyle="--", label="Marker estimate"),
    ]
    trial_handles, trial_labels = axis.get_legend_handles_labels()
    axis.legend(
        style_handles + trial_handles,
        [item.get_label() for item in style_handles] + trial_labels,
        loc="upper left", bbox_to_anchor=(0.0, 1.0), ncol=2,
        framealpha=0.94, fontsize=7.5,
    )
    figure.tight_layout()
    figure.savefig(output, dpi=args.dpi, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def make_panels(trials, output, limits, pad_size, args):
    figure = plt.figure(figsize=(15.0, 7.4))
    for index, (number, gt, estimate) in enumerate(trials):
        axis = figure.add_subplot(2, 5, index + 1, projection="3d")
        configure_axis(axis, limits, pad_size, args.elevation, args.azimuth)
        draw_trial(axis, gt, estimate, COLORS[index % len(COLORS)], thin=True)
        axis.set_title("Trial %d" % number, pad=0, fontsize=10)
        axis.tick_params(axis="both", which="major", labelsize=6, pad=0)
        axis.zaxis.set_tick_params(labelsize=6, pad=0)
    figure.legend(
        handles=[
            Line2D([0], [0], color="#333333", lw=1.8, linestyle="-", label="Ground truth"),
            Line2D([0], [0], color="#333333", lw=1.4, linestyle="--", label="Marker estimate"),
            Line2D([0], [0], color="#333333", marker="o", linestyle="None", label="Start"),
            Line2D([0], [0], color="#333333", marker="x", linestyle="None", label="End"),
        ],
        loc="lower center", ncol=4, bbox_to_anchor=(0.5, 0.005), framealpha=0.96,
    )
    figure.subplots_adjust(left=0.015, right=0.985, top=0.96, bottom=0.10,
                           wspace=0.02, hspace=0.18)
    figure.savefig(output, dpi=args.dpi, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    module_dir = os.path.dirname(script_dir)
    root = os.path.abspath(os.path.join(module_dir, "../../.."))
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", help="bag paths or glob expressions")
    parser.add_argument(
        "--output-dir",
        default=os.path.join(root, "experiments/aruco-landing/paper"),
    )
    parser.add_argument(
        "--environment-config",
        default=os.path.join(module_dir, "config/baseline_environment.yaml"),
    )
    parser.add_argument("--state-topic", default="/landing/controller/state")
    parser.add_argument("--ground-truth-topic", default="/landing/ground_truth/airsim_local_ned")
    parser.add_argument("--estimate-topic", default="/landing/vehicle_pose_pad")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--elevation", type=float, default=24.0)
    parser.add_argument("--azimuth", type=float, default=-55.0)
    parser.add_argument(
        "--output-prefix", default=None,
        help="filename prefix; defaults to the text before _trial_ in the first bag",
    )
    args = parser.parse_args()

    paths = []
    for expression in args.bags:
        matches = glob.glob(expression)
        paths.extend(matches if matches else [expression])
    paths = sorted({os.path.abspath(path) for path in paths}, key=trial_number)
    if not paths:
        raise RuntimeError("no bags selected")
    if len(paths) != 10:
        raise ValueError("expected exactly 10 trial bags, got %d" % len(paths))

    environment = load_yaml(args.environment_config)
    topics = {
        "state": args.state_topic,
        "ground_truth": args.ground_truth_topic,
        "estimate": args.estimate_topic,
    }
    trials = []
    for index, path in enumerate(paths):
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        gt, estimate = read_trajectory(path, environment, topics)
        number = trial_number(path) or index + 1
        trials.append((number, gt, estimate))
        print("trial %d: GT=%d estimate=%d" % (number, len(gt), len(estimate)))

    pad_size = float(environment["environment"]["pad_size_m"])
    limits = common_limits(trials, pad_size)
    os.makedirs(args.output_dir, exist_ok=True)
    inferred_prefix = re.sub(
        r"_trial_\d+.*$", "", os.path.splitext(os.path.basename(paths[0]))[0]
    )
    output_prefix = args.output_prefix or inferred_prefix or "landing"
    combined = os.path.join(
        args.output_dir, output_prefix + "_10_trials_trajectory_3d.png"
    )
    panels = os.path.join(
        args.output_dir, output_prefix + "_10_trials_trajectory_panels.png"
    )
    make_combined(trials, combined, limits, pad_size, args)
    make_panels(trials, panels, limits, pad_size, args)
    print(combined)
    print(panels)


if __name__ == "__main__":
    main()

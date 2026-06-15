#!/usr/bin/env python3
"""
Pose-bag analyzer: pull x/y/z/roll/pitch/yaw from every pose-like topic in a rosbag and
overlay them (time series + XY trajectory) for cross-source comparison.

    python3 scripts/analyze_pose_bag.py [BAG] [--only k1,k2] [--add LABEL=/topic]
                                        [--out PNG] [--rad] [--show]

Auto-discovers any geometry_msgs/PoseStamped|TransformStamped|PoseWithCovarianceStamped
and nav_msgs/Odometry topic, so adding a new source (e.g. fast-livo) needs no code
change — drop it in the bag and it shows up. KNOWN just maps topics to nice labels.
"""
import argparse
import glob
import math
import os
import sys

import numpy as np
import rosbag

KNOWN = {
    "/mavros/local_position/pose": "mavros (EKF2 local)",
    "/mavros/vision_pose/pose": "vision_pose (in)",
    "/vrpn_client_node/pure/pose": "vrpn (OptiTrack)",
    "/aft_mapped_to_init": "fast-livo",
    "/Odometry": "fast-livo (odom)",
}

# (translation path, rotation path) attribute chains we know how to read.
POSE_ACCESSORS = {
    "geometry_msgs/PoseStamped": ("pose.position", "pose.orientation"),
    "geometry_msgs/PoseWithCovarianceStamped": ("pose.pose.position", "pose.pose.orientation"),
    "geometry_msgs/TransformStamped": ("transform.translation", "transform.rotation"),
    "nav_msgs/Odometry": ("pose.pose.position", "pose.pose.orientation"),
}


def _get(obj, path):
    for attr in path.split("."):
        obj = getattr(obj, attr)
    return obj


def quat_to_rpy(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return roll, pitch, yaw


def extract(bag, topic, msgtype):
    pos_path, rot_path = POSE_ACCESSORS[msgtype]
    c = dict(t=[], x=[], y=[], z=[], roll=[], pitch=[], yaw=[])
    for _, msg, bagt in bag.read_messages(topics=[topic]):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0 else bagt.to_sec()
        p, q = _get(msg, pos_path), _get(msg, rot_path)
        roll, pitch, yaw = quat_to_rpy(q)
        c["t"].append(stamp); c["x"].append(p.x); c["y"].append(p.y); c["z"].append(p.z)
        c["roll"].append(roll); c["pitch"].append(pitch); c["yaw"].append(yaw)
    return {k: np.asarray(v) for k, v in c.items()}


def find_gaps(t, factor):
    if len(t) < 2:
        return np.array([], dtype=int), []
    dt = np.diff(t)
    idx = np.where(dt > factor * np.median(dt))[0]
    return idx, [(float(t[i]), float(t[i + 1])) for i in idx]


def with_breaks(arr, idx):
    return np.insert(arr, idx + 1, np.nan) if len(idx) else arr


def discover(bag, only, extra):
    info = bag.get_type_and_topic_info().topics
    sources = []
    for topic, meta in sorted(info.items()):
        if meta.msg_type not in POSE_ACCESSORS:
            continue
        label = KNOWN.get(topic, topic.strip("/").replace("/", "."))
        if only and not any(k in topic or k in label for k in only):
            continue
        sources.append((topic, label, meta.msg_type, meta.message_count))
    for label, topic in extra:
        info_t = info.get(topic)
        if info_t is None:
            print(f"  ! --add {topic} not in bag, skipping", file=sys.stderr); continue
        sources.append((topic, label, info_t.msg_type, info_t.message_count))
    return sources


def main():
    ap = argparse.ArgumentParser(description="Analyze x/y/z/roll/pitch/yaw of pose topics in a rosbag.")
    ap.add_argument("bag", nargs="?", help="bag file (default: the single *.bag in CWD)")
    ap.add_argument("--only", help="comma-separated substrings; keep only matching topics/labels")
    ap.add_argument("--add", action="append", default=[], metavar="LABEL=/topic",
                    help="force-add a topic not in KNOWN (repeatable)")
    ap.add_argument("--out", help="output PNG path (default: <bag>_pose.png)")
    ap.add_argument("--gap-factor", type=float, default=4.0,
                    help="flag a dropout when a sample gap exceeds this x median interval (default 4)")
    ap.add_argument("--rad", action="store_true", help="angles in radians (default: degrees)")
    ap.add_argument("--show", action="store_true", help="also open an interactive window")
    args = ap.parse_args()

    bagpath = args.bag
    if not bagpath:
        bags = glob.glob("*.bag")
        if len(bags) != 1:
            ap.error(f"specify a bag (found {len(bags)} *.bag in CWD)")
        bagpath = bags[0]

    only = [s for s in (args.only or "").split(",") if s]
    extra = [tuple(a.split("=", 1)) for a in args.add if "=" in a]

    import matplotlib
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with rosbag.Bag(bagpath) as bag:
        t0 = bag.get_start_time()
        sources = discover(bag, only, extra)
        if not sources:
            ap.error("no pose-like topics found")
        data = []
        colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
        print(f"\n{bagpath}  ({bag.get_end_time() - t0:.1f}s)\n")
        hdr = f"{'label':<22}{'topic':<34}{'msgs':>6}{'Hz':>7}"
        print(hdr); print("-" * len(hdr))
        for i, (topic, label, msgtype, count) in enumerate(sources):
            d = extract(bag, topic, msgtype)
            d["label"] = label
            d["color"] = colors[i % len(colors)]
            for ang in ("roll", "pitch", "yaw"):
                d[ang] = np.unwrap(d[ang]) if args.rad else np.degrees(np.unwrap(d[ang]))
            d["t"] = d["t"] - t0
            d["gap_idx"], d["gaps"] = find_gaps(d["t"], args.gap_factor)
            data.append(d)
            dur = d["t"][-1] - d["t"][0] if len(d["t"]) > 1 else 0
            rate = (len(d["t"]) - 1) / dur if dur > 0 else 0
            print(f"{label:<22}{topic:<34}{count:>6}{rate:>7.1f}")

    ang_unit = "rad" if args.rad else "deg"
    print()
    for d in data:
        rng = lambda k: f"[{d[k].min():+.2f},{d[k].max():+.2f}]"
        print(f"{d['label']:<22} x{rng('x')} y{rng('y')} z{rng('z')}")
        print(f"{'':<22} roll{rng('roll')} pitch{rng('pitch')} yaw{rng('yaw')} [{ang_unit}]")
    print()
    for d in data:
        if len(d["gaps"]):
            ls, le = max(d["gaps"], key=lambda se: se[1] - se[0])
            print(f"{d['label']:<22} DROPOUT {len(d['gaps'])}x, "
                  f"longest {(le - ls) * 1000:.0f}ms @ t={ls:.2f}s  (line broken in plot)")
        else:
            print(f"{d['label']:<22} no dropouts")

    fig = plt.figure(figsize=(15, 12))
    gs = fig.add_gridspec(6, 2, width_ratios=[2, 1.4])
    axes = [fig.add_subplot(gs[i, 0]) for i in range(6)]
    keys = ("x", "y", "z", "roll", "pitch", "yaw")
    ylabels = ("x [m]", "y [m]", "z [m]",
               f"roll [{ang_unit}]", f"pitch [{ang_unit}]", f"yaw [{ang_unit}]")
    for ax, key, ylabel in zip(axes, keys, ylabels):
        for d in data:
            ax.plot(with_breaks(d["t"], d["gap_idx"]), with_breaks(d[key], d["gap_idx"]),
                    lw=1.0, color=d["color"], label=d["label"])
            for s, e in d["gaps"]:
                ax.axvspan(s, e, color=d["color"], alpha=0.12, lw=0)
        ax.set_ylabel(ylabel); ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time [s]")

    ax_xy = fig.add_subplot(gs[:, 1])
    for d in data:
        ax_xy.plot(with_breaks(d["x"], d["gap_idx"]), with_breaks(d["y"], d["gap_idx"]),
                   lw=1.0, color=d["color"], label=d["label"])
        if len(d["x"]):
            ax_xy.plot(d["x"][0], d["y"][0], "o", ms=5, color=d["color"])
    ax_xy.set_xlabel("x [m]"); ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("XY trajectory (o = start)"); ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3); ax_xy.legend(fontsize=8)

    fig.suptitle(os.path.basename(bagpath))
    fig.tight_layout()
    out = args.out or os.path.splitext(bagpath)[0] + "_pose.png"
    fig.savefig(out, dpi=120)
    print(f"\nsaved {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()

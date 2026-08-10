#!/usr/bin/env python3
"""Build a traceable, Windows-offline paper/PPT asset pack for p1 and pm4.

This tool intentionally creates *case-study assets*, not an aggregate benchmark.
It reads the two source-preserving spliced bags, extracts the recorded onboard RGB
frames and GT path, and compares GT with the post-hoc tuned FAST-LIVO trajectory.
No spatial trajectory alignment is applied.  The same association implementation
used by ``campaign_20260803.py`` is reused here.

The generated ``index.html`` has no server or network dependency.  Open it directly
from Windows Explorer after copying or unzipping the complete output directory.
Source bags are not copied because they are hundreds of MB; their absolute paths,
hashes, and adjacent splice provenance are frozen in ``manifest.json``.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import html
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
import rosbag

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_fastlivo import (  # noqa: E402
    associate,
    estimate_offset,
    geodesic_deg,
    q_to_R,
    read_traj,
    slerp,
)


REPO = Path(__file__).resolve().parents[2]
CAMPAIGN_ROOT = REPO / "tools/fastlivo/_campaign_20260805"
WINDOW_CACHE_INDEX = (
    CAMPAIGN_ROOT / "timeseries/production_primary/cache/index.json"
)
DEFAULT_OUTPUT = CAMPAIGN_ROOT / "paper_assets/realworld_p1_pm4_v1"
WINDOWS_PORTABLE_FILES = (
    REPO / "tools/fastlivo/windows_bag_tools.py",
    REPO / "tools/fastlivo/README_WINDOWS.md",
    REPO / "tools/fastlivo/requirements-windows.txt",
    REPO / "tools/fastlivo/transfer_manifest.json",
)

GT_TOPIC = "/vrpn_client_node/pure/pose"
EST_TOPIC = "/aft_mapped_to_optitrack"
RGB_TOPIC = "/camera/color/image_raw/compressed"
MAX_ASSOCIATION_DIFF_S = 0.05
PROGRESS_FRACTIONS = tuple(np.linspace(0.0, 1.0, 11))
PHYSICAL_CLOTH_AABB_MIN_M = np.asarray([-0.5, -0.5, 0.0], dtype=float)
PHYSICAL_CLOTH_AABB_MAX_M = np.asarray([0.5, 0.5, 2.5], dtype=float)

SESSIONS = (
    {
        "id": "p1_20260804_212926",
        "short_id": "p1",
        "actual_condition": "pure",
        "display_label": "PURE",
        "color": "#59A14F",
        "source_bag": Path(
            "/home/ml/webcam_recorder/recordings/"
            "pure_flight_2026-08-04_21-29-26_1/"
            "flight_2026-08-04_21-29-26.bag"
        ),
        "tuned_spliced_bag": Path(
            "/home/ml/webcam_recorder/recordings/"
            "pure_flight_2026-08-04_21-29-26_1/"
            "flight_2026-08-04_21-29-26_fastlivo_hybrid_imu_acc10.bag"
        ),
    },
    {
        "id": "pm4_20260805_020904",
        "short_id": "pm4",
        "actual_condition": "pure_mean",
        "display_label": "PURE-Mean",
        "color": "#F28E2B",
        "source_bag": Path(
            "/home/ml/webcam_recorder/recordings/"
            "pure_mean_flight_2026-08-05_02-09-04_4/"
            "flight_2026-08-05_02-09-04.bag"
        ),
        "tuned_spliced_bag": Path(
            "/home/ml/webcam_recorder/recordings/"
            "pure_mean_flight_2026-08-05_02-09-04_4/"
            "flight_2026-08-05_02-09-04_fastlivo_hybrid_imu_acc10.bag"
        ),
    },
)

GLOBAL_CAVEATS = (
    "The labels are the actual recorded conditions: p1 is PURE and pm4 is "
    "PURE-Mean. Neither session is nominal, and no relabelling is permitted.",
    "These two sessions were selected post hoc for a qualitative case study. "
    "They must not be presented as an unbiased two-sample method comparison.",
    "The tuned FAST-LIVO trajectories were generated after flight and the two "
    "selected sessions participated in tuning. They are not locked validation data.",
    "Planner, Voxblox, controller, RGB, and GT records are the historical flight "
    "records. Replacing the VIO trajectory is not a counterfactual planner rerun.",
    "Trajectory error uses time association only (maximum 50 ms) and no SE(3), "
    "Sim(3), origin, yaw, or scale alignment.",
    "RGB frames come from the onboard recorded RGB topic. Webcam-perspective "
    "projected overlays are intentionally excluded from this version.",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="move an existing output aside before installing the new pack",
    )
    return parser.parse_args()


def sha256(path: Path, block_size: int = 8 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path: Path) -> Dict[str, object]:
    stat = path.stat()
    return {
        "path": str(path.resolve()),
        "size_bytes": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
        "sha256": sha256(path),
    }


def message_stamp(msg, bag_time) -> float:
    header = getattr(msg, "header", None)
    if header is not None and header.stamp.to_sec() > 0:
        return float(header.stamp.to_sec())
    return float(bag_time.to_sec())


def yaw_from_quaternion(q: Sequence[float]) -> float:
    x, y, z, w = (float(value) for value in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_windows() -> Dict[str, Mapping[str, object]]:
    document = json.loads(WINDOW_CACHE_INDEX.read_text())
    wanted = {str(row["id"]) for row in SESSIONS}
    rows = {
        str(row["flight_id"]): row
        for row in document["sessions"]
        if str(row["flight_id"]) in wanted
    }
    if set(rows) != wanted:
        raise RuntimeError(
            f"window cache missing sessions: {sorted(wanted - set(rows))}"
        )
    return rows


def deduplicate_pose(
    times: np.ndarray, xyz: np.ndarray, quaternion: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    order = np.argsort(times, kind="stable")
    times = times[order]
    xyz = xyz[order]
    quaternion = quaternion[order]
    keep = np.r_[True, np.diff(times) > 1e-9]
    return times[keep], xyz[keep], quaternion[keep]


def within_window(
    times: np.ndarray,
    xyz: np.ndarray,
    quaternion: np.ndarray,
    start: float,
    end: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    mask = (times >= start) & (times <= end)
    if int(mask.sum()) < 2:
        raise RuntimeError(f"pose window has only {int(mask.sum())} samples")
    return times[mask], xyz[mask], quaternion[mask]


def interpolate_pose(
    times: np.ndarray,
    xyz: np.ndarray,
    quaternion: np.ndarray,
    query: float,
) -> Tuple[np.ndarray, np.ndarray]:
    upper = int(np.searchsorted(times, query, side="left"))
    if upper <= 0:
        return xyz[0].copy(), quaternion[0].copy()
    if upper >= len(times):
        return xyz[-1].copy(), quaternion[-1].copy()
    lower = upper - 1
    span = times[upper] - times[lower]
    weight = 0.0 if span <= 1e-12 else (query - times[lower]) / span
    point = xyz[lower] + weight * (xyz[upper] - xyz[lower])
    quat = slerp(quaternion[lower], quaternion[upper], float(weight))
    return point, quat


def uniform_path_length(times: np.ndarray, xyz: np.ndarray, hz: float = 10.0) -> float:
    """Match the campaign's jitter-robust 10 Hz GT path-length definition."""
    if len(times) < 2:
        return 0.0
    query = np.arange(times[0], times[-1] + 1e-9, 1.0 / hz)
    points = np.column_stack(
        [np.interp(query, times, xyz[:, axis]) for axis in range(3)]
    )
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def compute_spatial_events(
    times: np.ndarray, xyz: np.ndarray, quaternion: np.ndarray
) -> List[Dict[str, object]]:
    event_specs: List[Tuple[str, str, int, str]] = []
    x_indices = np.flatnonzero(xyz[:, 0] >= 0.0)
    y_indices = np.flatnonzero(xyz[:, 1] <= 0.0)
    if len(x_indices):
        event_specs.append(
            (
                "first_x_nonnegative",
                "First GT sample with x >= 0 m",
                int(x_indices[0]),
                "first_sample_in_hover_with_gt_x_ge_0",
            )
        )
    if len(y_indices):
        event_specs.append(
            (
                "first_y_nonpositive",
                "First GT sample with y <= 0 m",
                int(y_indices[0]),
                "first_sample_in_hover_with_gt_y_le_0",
            )
        )

    inside = np.all(
        (xyz >= PHYSICAL_CLOTH_AABB_MIN_M[None, :])
        & (xyz <= PHYSICAL_CLOTH_AABB_MAX_M[None, :]),
        axis=1,
    )
    delta = np.maximum(
        np.maximum(PHYSICAL_CLOTH_AABB_MIN_M[None, :] - xyz,
                   xyz - PHYSICAL_CLOTH_AABB_MAX_M[None, :]),
        0.0,
    )
    distance = np.linalg.norm(delta, axis=1)
    distance[inside] = np.inf
    if np.any(np.isfinite(distance)):
        index = int(np.argmin(distance))
        event_specs.append(
            (
                "closest_outside_cloth_region",
                "Closest GT pose outside physical cloth region",
                index,
                "minimum_3d_distance_to_cloth_aabb_excluding_inside_samples",
            )
        )

    events = []
    for event_id, label, index, rule in event_specs:
        events.append(
            {
                "event_id": event_id,
                "label": label,
                "absolute_time_s": float(times[index]),
                "gt_xyz_m": xyz[index].tolist(),
                "gt_quaternion_xyzw": quaternion[index].tolist(),
                "gt_yaw_rad": yaw_from_quaternion(quaternion[index]),
                "selection_rule": rule,
                "cloth_distance_m": (
                    float(distance[index])
                    if event_id == "closest_outside_cloth_region"
                    else None
                ),
            }
        )
    return events


def extract_nearest_rgb(
    bag_path: Path,
    targets: Sequence[Mapping[str, object]],
) -> Dict[str, Dict[str, object]]:
    nearest: Dict[str, Dict[str, object]] = {
        str(target["asset_key"]): {"difference_s": math.inf}
        for target in targets
    }
    with rosbag.Bag(str(bag_path), "r") as bag:
        available = bag.get_type_and_topic_info().topics
        if RGB_TOPIC not in available:
            raise RuntimeError(f"{bag_path}: missing {RGB_TOPIC}")
        for _, message, bag_time in bag.read_messages(topics=[RGB_TOPIC]):
            stamp = message_stamp(message, bag_time)
            for target in targets:
                key = str(target["asset_key"])
                difference = abs(stamp - float(target["absolute_time_s"]))
                if difference < float(nearest[key]["difference_s"]):
                    nearest[key] = {
                        "difference_s": float(difference),
                        "message_time_s": float(stamp),
                        "bag_time_s": float(bag_time.to_sec()),
                        "format": str(message.format),
                        "data": bytes(message.data),
                    }
    missing = [key for key, row in nearest.items() if "data" not in row]
    if missing:
        raise RuntimeError(f"no RGB frame for targets: {missing}")
    return nearest


def decode_compressed(data: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("OpenCV failed to decode a CompressedImage")
    return image


def annotate_thumbnail(image: np.ndarray, lines: Sequence[str]) -> np.ndarray:
    target_width = 480
    scale = target_width / float(image.shape[1])
    resized = cv2.resize(
        image,
        (target_width, int(round(image.shape[0] * scale))),
        interpolation=cv2.INTER_AREA,
    )
    header_height = 30 + 25 * len(lines)
    canvas = np.full(
        (resized.shape[0] + header_height, resized.shape[1], 3), 248, dtype=np.uint8
    )
    canvas[header_height:, :] = resized
    for index, line in enumerate(lines):
        cv2.putText(
            canvas,
            str(line),
            (12, 28 + 25 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (30, 30, 30),
            1,
            cv2.LINE_AA,
        )
    return canvas


def write_contact_sheet(
    records: Sequence[Mapping[str, object]], output: Path, columns: int
) -> None:
    thumbnails = []
    for record in records:
        image = cv2.imread(str(record["absolute_path"]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"failed to read {record['absolute_path']}")
        thumbnails.append(annotate_thumbnail(image, record["contact_lines"]))
    if not thumbnails:
        raise RuntimeError(f"empty contact sheet: {output}")
    cell_width = max(image.shape[1] for image in thumbnails)
    cell_height = max(image.shape[0] for image in thumbnails)
    rows = int(math.ceil(len(thumbnails) / columns))
    canvas = np.full((rows * cell_height, columns * cell_width, 3), 255, dtype=np.uint8)
    for index, image in enumerate(thumbnails):
        row, column = divmod(index, columns)
        y0 = row * cell_height
        x0 = column * cell_width
        canvas[y0:y0 + image.shape[0], x0:x0 + image.shape[1]] = image
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f"failed to write {output}")


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Mapping[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def make_associations(
    gt_t: np.ndarray,
    gt_xyz: np.ndarray,
    gt_q: np.ndarray,
    est_t: np.ndarray,
    est_xyz: np.ndarray,
    est_q: np.ndarray,
    hover_start: float,
    hover_end: float,
) -> Dict[str, object]:
    offset = float(estimate_offset(est_t, est_xyz, gt_t, gt_xyz))
    pairs = associate(est_t + offset, gt_t, MAX_ASSOCIATION_DIFF_S)
    filtered = [
        pair
        for pair in pairs
        if hover_start <= est_t[pair[0]] + offset <= hover_end
    ]
    if len(filtered) < 10:
        raise RuntimeError(f"only {len(filtered)} GT/VIO associations")

    match_t = np.asarray([est_t[index] + offset for index, _, _, _ in filtered])
    estimate = np.asarray([est_xyz[index] for index, _, _, _ in filtered])
    estimate_q = np.asarray([est_q[index] for index, _, _, _ in filtered])
    truth = np.asarray(
        [
            gt_xyz[lower] + weight * (gt_xyz[upper] - gt_xyz[lower])
            for _, lower, upper, weight in filtered
        ]
    )
    truth_q = np.asarray(
        [
            slerp(gt_q[lower], gt_q[upper], weight)
            for _, lower, upper, weight in filtered
        ]
    )
    ape = np.linalg.norm(estimate - truth, axis=1)
    orientation_error = np.asarray(
        [
            geodesic_deg(q_to_R(q_est), q_to_R(q_gt))
            for q_est, q_gt in zip(estimate_q, truth_q)
        ]
    )
    gt_window_t, gt_window_xyz, _ = within_window(
        gt_t, gt_xyz, gt_q, hover_start, hover_end
    )
    gt_path = uniform_path_length(gt_window_t, gt_window_xyz, hz=10.0)
    est_path = float(np.linalg.norm(np.diff(estimate, axis=0), axis=1).sum())
    full_window_coverage = float(
        (match_t[-1] - match_t[0]) / (hover_end - hover_start)
    )
    post_initialization_coverage = float(
        (match_t[-1] - match_t[0]) / max(1e-9, hover_end - match_t[0])
    )
    return {
        "time_offset_s": offset,
        "pairs": filtered,
        "time_s": match_t,
        "est_xyz": estimate,
        "est_q": estimate_q,
        "gt_xyz": truth,
        "gt_q": truth_q,
        "ape_m": ape,
        "orientation_error_deg": orientation_error,
        "metrics": {
            "associations": int(len(filtered)),
            "association_max_diff_s": MAX_ASSOCIATION_DIFF_S,
            "time_offset_s_added_to_estimate": offset,
            "spatial_alignment": "none",
            "coverage_fraction": full_window_coverage,
            "full_window_coverage_fraction": full_window_coverage,
            "post_first_estimate_coverage_fraction": post_initialization_coverage,
            "gt_path_length_m": gt_path,
            "associated_estimate_path_length_m": est_path,
            "translation_ape_rmse_m": float(np.sqrt(np.mean(ape * ape))),
            "translation_ape_mean_m": float(np.mean(ape)),
            "translation_ape_median_m": float(np.median(ape)),
            "translation_ape_p90_m": float(np.quantile(ape, 0.9)),
            "translation_ape_max_m": float(np.max(ape)),
            "translation_ape_final_m": float(ape[-1]),
            "orientation_error_rmse_deg": float(
                np.sqrt(np.mean(orientation_error * orientation_error))
            ),
        },
    }


def configure_matplotlib() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 9.0,
            "axes.labelsize": 9.5,
            "axes.titlesize": 10.5,
            "legend.fontsize": 8.5,
            "svg.fonttype": "none",
            "savefig.dpi": 240,
        }
    )


def save_figure(fig: plt.Figure, stem: Path) -> Tuple[Path, Path]:
    svg = stem.with_suffix(".svg")
    png = stem.with_suffix(".png")
    fig.savefig(svg, bbox_inches="tight", transparent=True)
    fig.savefig(png, bbox_inches="tight", transparent=True, dpi=300)
    plt.close(fig)
    return svg, png


def add_heading_arrows(
    ax: plt.Axes,
    xyz: np.ndarray,
    quaternion: np.ndarray,
    color: str,
    count: int = 9,
) -> None:
    indices = np.unique(np.linspace(0, len(xyz) - 1, count).astype(int))
    yaw = np.asarray([yaw_from_quaternion(q) for q in quaternion[indices]])
    length = 0.16
    ax.quiver(
        xyz[indices, 0],
        xyz[indices, 1],
        length * np.cos(yaw),
        length * np.sin(yaw),
        angles="xy",
        scale_units="xy",
        scale=1.0,
        color=color,
        width=0.006,
        headwidth=4.0,
        headlength=5.0,
        alpha=0.9,
        zorder=4,
    )


def style_path_axis(ax: plt.Axes, limits: Tuple[float, float, float, float]) -> None:
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(limits[0], limits[1])
    ax.set_ylim(limits[2], limits[3])
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack y [m]")
    ax.grid(True, color="#d7d7d7", linewidth=0.6, alpha=0.8)


def plot_session_path(
    session: Mapping[str, object], limits: Tuple[float, float, float, float], output: Path
) -> Tuple[Path, Path]:
    fig, ax = plt.subplots(figsize=(5.2, 4.8))
    gt = session["gt_window_xyz"]
    gt_q = session["gt_window_q"]
    assoc = session["associations"]
    ax.plot(gt[:, 0], gt[:, 1], color="#222222", linewidth=2.1, label="GT")
    ax.plot(
        assoc["est_xyz"][:, 0],
        assoc["est_xyz"][:, 1],
        color=str(session["color"]),
        linewidth=1.8,
        label="Tuned VIO (post hoc)",
    )
    add_heading_arrows(ax, gt, gt_q, "#222222")
    ax.scatter(gt[0, 0], gt[0, 1], marker="o", s=44, facecolor="white",
               edgecolor="#222222", zorder=5, label="start")
    ax.scatter(gt[-1, 0], gt[-1, 1], marker="s", s=42, color="#222222", zorder=5,
               label="end")
    style_path_axis(ax, limits)
    metrics = assoc["metrics"]
    ax.set_title(
        f"{session['display_label']} ({session['short_id']}) — recorded GT vs tuned VIO\n"
        f"no spatial alignment; APE RMSE {metrics['translation_ape_rmse_m']:.3f} m"
    )
    ax.legend(loc="best", framealpha=0.92)
    fig.tight_layout()
    return save_figure(fig, output)


def plot_session_ape(session: Mapping[str, object], ymax: float, output: Path) -> Tuple[Path, Path]:
    fig, ax = plt.subplots(figsize=(6.4, 3.25))
    assoc = session["associations"]
    time = assoc["time_s"] - float(session["hover_start_s"])
    error = assoc["ape_m"]
    rmse = float(assoc["metrics"]["translation_ape_rmse_m"])
    ax.plot(time, error, color=str(session["color"]), linewidth=1.6,
            label="translation APE")
    ax.axhline(rmse, color="#222222", linestyle="--", linewidth=1.0,
               label=f"RMSE = {rmse:.3f} m")
    ax.set_xlim(0.0, float(session["hover_duration_s"]))
    ax.set_ylim(0.0, ymax)
    ax.set_xlabel("time from stable hover [s]")
    ax.set_ylabel("translation APE [m]")
    ax.grid(True, color="#d7d7d7", linewidth=0.6, alpha=0.8)
    ax.set_title(
        f"{session['display_label']} ({session['short_id']}) — tuned VIO error "
        "(no spatial alignment)"
    )
    ax.legend(loc="upper left", framealpha=0.92)
    fig.tight_layout()
    return save_figure(fig, output)


def plot_pair_path(
    sessions: Sequence[Mapping[str, object]],
    limits: Tuple[float, float, float, float],
    output: Path,
) -> Tuple[Path, Path]:
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.5), sharex=True, sharey=True)
    for ax, session in zip(axes, sessions):
        gt = session["gt_window_xyz"]
        assoc = session["associations"]
        ax.plot(gt[:, 0], gt[:, 1], color="#222222", linewidth=2.0, label="GT")
        ax.plot(assoc["est_xyz"][:, 0], assoc["est_xyz"][:, 1],
                color=str(session["color"]), linewidth=1.7,
                label="Tuned VIO (post hoc)")
        add_heading_arrows(ax, gt, session["gt_window_q"], "#222222", count=8)
        style_path_axis(ax, limits)
        ax.set_title(
            f"{session['display_label']} ({session['short_id']})\n"
            f"APE RMSE {assoc['metrics']['translation_ape_rmse_m']:.3f} m"
        )
        ax.legend(loc="best", framealpha=0.92)
    fig.suptitle("Selected case-study flights — actual labels; no spatial alignment", y=1.01)
    fig.tight_layout()
    return save_figure(fig, output)


def plot_pair_ape(
    sessions: Sequence[Mapping[str, object]], ymax: float, output: Path
) -> Tuple[Path, Path]:
    fig, ax = plt.subplots(figsize=(7.2, 3.65))
    for session in sessions:
        assoc = session["associations"]
        t = assoc["time_s"] - float(session["hover_start_s"])
        rmse = assoc["metrics"]["translation_ape_rmse_m"]
        ax.plot(t, assoc["ape_m"], color=str(session["color"]), linewidth=1.55,
                label=f"{session['display_label']} (RMSE {rmse:.3f} m)")
    ax.set_xlim(0.0, max(float(row["hover_duration_s"]) for row in sessions))
    ax.set_ylim(0.0, ymax)
    ax.set_xlabel("time from stable hover [s]")
    ax.set_ylabel("translation APE [m]")
    ax.grid(True, color="#d7d7d7", linewidth=0.6, alpha=0.8)
    ax.set_title("Tuned VIO APE in selected case-study flights (no spatial alignment)")
    ax.legend(loc="upper left", framealpha=0.92)
    fig.tight_layout()
    return save_figure(fig, output)


def relative(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def asset_record(
    root: Path,
    path: Path,
    asset_id: str,
    session_id: str | None,
    category: str,
    description: str,
    intended_use: str,
    **extra: object,
) -> Dict[str, object]:
    record = {
        "asset_id": asset_id,
        "session_id": session_id,
        "category": category,
        "description": description,
        "intended_use": intended_use,
        "path": relative(path, root),
        "format": path.suffix.lstrip(".").lower(),
        "size_bytes": int(path.stat().st_size),
        "sha256": sha256(path),
    }
    record.update(extra)
    return record


def write_html(root: Path, manifest: Mapping[str, object]) -> Path:
    manifest_text = json.dumps(manifest, ensure_ascii=False).replace("</", "<\\/")
    caveats = "".join(f"<li>{html.escape(str(row))}</li>" for row in GLOBAL_CAVEATS)
    document = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>p1 / pm4 real-world case-study assets</title>
  <style>
    :root {{ color-scheme: light; --ink:#202124; --muted:#62666d; --line:#d9dce1; }}
    body {{ margin:0; font-family:Segoe UI,Arial,sans-serif; color:var(--ink); background:#f5f6f8; }}
    header {{ padding:24px 30px 18px; background:white; border-bottom:1px solid var(--line); }}
    h1 {{ margin:0 0 8px; font-size:25px; }} h2 {{ margin:24px 0 10px; }}
    .warning {{ margin-top:16px; padding:13px 18px; background:#fff4dd; border-left:5px solid #d08a00; }}
    .warning li {{ margin:5px 0; }}
    main {{ padding:18px 30px 40px; }}
    .controls {{ display:flex; gap:10px; flex-wrap:wrap; margin-bottom:16px; }}
    select,input {{ padding:7px 9px; border:1px solid #b8bdc5; border-radius:5px; background:white; }}
    .summary {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(260px,1fr)); gap:12px; }}
    .summary article,.card {{ background:white; border:1px solid var(--line); border-radius:7px; padding:13px; }}
    .grid {{ display:grid; grid-template-columns:repeat(auto-fill,minmax(310px,1fr)); gap:14px; }}
    .card img {{ width:100%; max-height:310px; object-fit:contain; background:repeating-conic-gradient(#f1f1f1 0 25%,#fff 0 50%) 50%/18px 18px; }}
    .card h3 {{ margin:9px 0 4px; font-size:15px; }}
    .meta {{ color:var(--muted); font-size:12px; line-height:1.45; overflow-wrap:anywhere; }}
    .actions {{ margin-top:8px; display:flex; gap:10px; }} a {{ color:#1769aa; }}
    code {{ background:#eef0f2; padding:1px 4px; border-radius:3px; }}
    .hidden {{ display:none; }}
  </style>
</head>
<body>
<header>
  <h1>Real-world p1 / pm4 paper &amp; PPT asset pack</h1>
  <div>Actual conditions: <b>p1 = PURE</b>; <b>pm4 = PURE-Mean</b>.</div>
  <div class="warning"><b>Provenance / use restrictions</b><ul>{caveats}</ul></div>
</header>
<main>
  <section><h2>Session summary</h2><div class="summary" id="summary"></div></section>
  <section>
    <h2>Assets</h2>
    <div class="controls">
      <select id="session"><option value="">All sessions</option></select>
      <select id="category"><option value="">All categories</option></select>
      <input id="query" placeholder="Filter asset id / description">
    </div>
    <div class="grid" id="grid"></div>
  </section>
</main>
<script id="manifest" type="application/json">{manifest_text}</script>
<script>
const M=JSON.parse(document.getElementById('manifest').textContent);
const sessions=Object.fromEntries(M.sessions.map(s=>[s.id,s]));
const summary=document.getElementById('summary');
M.sessions.forEach(s=>{{
  const m=s.metrics;
  const a=document.createElement('article');
  a.innerHTML=`<h3>${{s.display_label}} (${{s.short_id}})</h3>
    <div>actual condition: <code>${{s.actual_condition}}</code></div>
    <div>hover window: ${{s.hover_duration_s.toFixed(2)}} s</div>
    <div>GT path: ${{m.gt_path_length_m.toFixed(3)}} m</div>
    <div>tuned-VIO APE RMSE: ${{m.translation_ape_rmse_m.toFixed(3)}} m</div>
    <div>coverage: ${{(100*m.coverage_fraction).toFixed(1)}}%</div>`;
  summary.appendChild(a);
}});
const sessionSelect=document.getElementById('session');
M.sessions.forEach(s=>sessionSelect.add(new Option(`${{s.display_label}} (${{s.short_id}})`,s.id)));
const categories=[...new Set(M.assets.map(a=>a.category))].sort();
const categorySelect=document.getElementById('category');
categories.forEach(c=>categorySelect.add(new Option(c,c)));
const grid=document.getElementById('grid'), query=document.getElementById('query');
function render(){{
  const sid=sessionSelect.value, cat=categorySelect.value, q=query.value.trim().toLowerCase();
  grid.innerHTML='';
  M.assets.filter(a=>(!sid||a.session_id===sid)&&(!cat||a.category===cat)&&
    (!q||(a.asset_id+' '+a.description).toLowerCase().includes(q))).forEach(a=>{{
      const card=document.createElement('article'); card.className='card';
      const preview=['png','svg'].includes(a.format)?`<a href="${{a.path}}"><img loading="lazy" src="${{a.path}}"></a>`:'';
      const label=a.session_id?sessions[a.session_id].display_label:'pair/common';
      card.innerHTML=`${{preview}}<h3>${{a.asset_id}}</h3><div>${{a.description}}</div>
        <div class="meta">${{label}} · ${{a.category}} · ${{a.format.toUpperCase()}} · ${{a.intended_use}}<br>${{a.path}}</div>
        <div class="actions"><a href="${{a.path}}">Open</a><a href="${{a.path}}" download>Download</a></div>`;
      grid.appendChild(card);
    }});
}}
[sessionSelect,categorySelect,query].forEach(e=>e.addEventListener('input',render)); render();
</script>
</body></html>
"""
    path = root / "index.html"
    path.write_text(document, encoding="utf-8")
    return path


def build_session(
    spec: Mapping[str, object], window_meta: Mapping[str, object], root: Path
) -> Tuple[Dict[str, object], List[Dict[str, object]]]:
    source_bag = Path(spec["source_bag"])
    tuned_bag = Path(spec["tuned_spliced_bag"])
    provenance_path = tuned_bag.with_suffix(tuned_bag.suffix + ".provenance.json")
    for required in (source_bag, tuned_bag, provenance_path):
        if not required.is_file():
            raise FileNotFoundError(required)

    provenance = json.loads(provenance_path.read_text())
    source_file_record = file_record(source_bag)
    tuned_file_record = file_record(tuned_bag)
    provenance_file_record = file_record(provenance_path)
    if provenance.get("validation", {}).get("valid") is not True:
        raise RuntimeError(f"invalid splice provenance: {provenance_path}")
    if str(provenance["source"]["sha256"]) != source_file_record["sha256"]:
        raise RuntimeError(f"source hash disagrees with splice provenance: {source_bag}")
    if str(provenance["output"]["sha256"]) != tuned_file_record["sha256"]:
        raise RuntimeError(f"spliced bag hash disagrees with provenance: {tuned_bag}")

    windows = window_meta["windows"]
    hover_start = float(windows["hover"]["start"])
    hover_end = float(windows["hover"]["end"])
    bag_start = float(windows["full"]["start"])

    gt_t, gt_xyz, gt_q = read_traj(str(tuned_bag), GT_TOPIC)
    est_t, est_xyz, est_q = read_traj(str(tuned_bag), EST_TOPIC)
    gt_t, gt_xyz, gt_q = deduplicate_pose(gt_t, gt_xyz, gt_q)
    est_t, est_xyz, est_q = deduplicate_pose(est_t, est_xyz, est_q)
    gt_window_t, gt_window_xyz, gt_window_q = within_window(
        gt_t, gt_xyz, gt_q, hover_start, hover_end
    )
    associations = make_associations(
        gt_t, gt_xyz, gt_q, est_t, est_xyz, est_q, hover_start, hover_end
    )

    session_dir = root / "sessions" / str(spec["short_id"])
    paths_dir = session_dir / "paths"
    rgb_progress_dir = session_dir / "rgb/progress"
    rgb_spatial_dir = session_dir / "rgb/spatial"
    provenance_dir = session_dir / "provenance"
    for directory in (paths_dir, rgb_progress_dir, rgb_spatial_dir, provenance_dir):
        directory.mkdir(parents=True, exist_ok=True)
    shutil.copy2(provenance_path, provenance_dir / "splice_provenance.json")

    gt_rows = []
    for stamp, point, quat in zip(gt_window_t, gt_window_xyz, gt_window_q):
        gt_rows.append(
            {
                "absolute_time_s": f"{stamp:.9f}",
                "bag_relative_time_s": f"{stamp - bag_start:.9f}",
                "hover_relative_time_s": f"{stamp - hover_start:.9f}",
                "x_m": f"{point[0]:.9f}", "y_m": f"{point[1]:.9f}",
                "z_m": f"{point[2]:.9f}", "qx": f"{quat[0]:.12g}",
                "qy": f"{quat[1]:.12g}", "qz": f"{quat[2]:.12g}",
                "qw": f"{quat[3]:.12g}", "yaw_rad": f"{yaw_from_quaternion(quat):.9f}",
            }
        )
    gt_csv = paths_dir / "gt_path.csv"
    pose_fields = ["absolute_time_s", "bag_relative_time_s", "hover_relative_time_s",
                   "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "yaw_rad"]
    write_csv(gt_csv, pose_fields, gt_rows)

    offset = float(associations["time_offset_s"])
    estimate_mask = ((est_t + offset) >= hover_start) & ((est_t + offset) <= hover_end)
    estimate_rows = []
    for stamp, point, quat in zip(est_t[estimate_mask], est_xyz[estimate_mask], est_q[estimate_mask]):
        adjusted = stamp + offset
        estimate_rows.append(
            {
                "raw_header_time_s": f"{stamp:.9f}",
                "time_offset_s": f"{offset:.9f}",
                "associated_time_s": f"{adjusted:.9f}",
                "bag_relative_associated_time_s": f"{adjusted - bag_start:.9f}",
                "hover_relative_associated_time_s": f"{adjusted - hover_start:.9f}",
                "x_m": f"{point[0]:.9f}", "y_m": f"{point[1]:.9f}",
                "z_m": f"{point[2]:.9f}", "qx": f"{quat[0]:.12g}",
                "qy": f"{quat[1]:.12g}", "qz": f"{quat[2]:.12g}",
                "qw": f"{quat[3]:.12g}", "yaw_rad": f"{yaw_from_quaternion(quat):.9f}",
            }
        )
    est_csv = paths_dir / "tuned_vio_path.csv"
    write_csv(
        est_csv,
        ["raw_header_time_s", "time_offset_s", "associated_time_s",
         "bag_relative_associated_time_s", "hover_relative_associated_time_s",
         "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "yaw_rad"],
        estimate_rows,
    )

    associated_rows = []
    for index, (stamp, estimate, truth, q_est, q_gt, ape, angle) in enumerate(
        zip(
            associations["time_s"], associations["est_xyz"], associations["gt_xyz"],
            associations["est_q"], associations["gt_q"], associations["ape_m"],
            associations["orientation_error_deg"],
        )
    ):
        associated_rows.append(
            {
                "association_index": index,
                "absolute_time_s": f"{stamp:.9f}",
                "bag_relative_time_s": f"{stamp - bag_start:.9f}",
                "hover_relative_time_s": f"{stamp - hover_start:.9f}",
                "gt_x_m": f"{truth[0]:.9f}", "gt_y_m": f"{truth[1]:.9f}",
                "gt_z_m": f"{truth[2]:.9f}", "vio_x_m": f"{estimate[0]:.9f}",
                "vio_y_m": f"{estimate[1]:.9f}", "vio_z_m": f"{estimate[2]:.9f}",
                "translation_ape_m": f"{ape:.9f}",
                "orientation_error_deg": f"{angle:.9f}",
                "gt_yaw_rad": f"{yaw_from_quaternion(q_gt):.9f}",
                "vio_yaw_rad": f"{yaw_from_quaternion(q_est):.9f}",
            }
        )
    associated_csv = paths_dir / "associated_gt_tuned_vio.csv"
    associated_fields = ["association_index", "absolute_time_s", "bag_relative_time_s",
                         "hover_relative_time_s", "gt_x_m", "gt_y_m", "gt_z_m",
                         "vio_x_m", "vio_y_m", "vio_z_m", "translation_ape_m",
                         "orientation_error_deg", "gt_yaw_rad", "vio_yaw_rad"]
    write_csv(associated_csv, associated_fields, associated_rows)

    targets: List[Dict[str, object]] = []
    for fraction in PROGRESS_FRACTIONS:
        absolute = hover_start + float(fraction) * (hover_end - hover_start)
        point, quat = interpolate_pose(gt_t, gt_xyz, gt_q, absolute)
        targets.append(
            {
                "asset_key": f"progress_{int(round(100 * fraction)):03d}",
                "kind": "progress",
                "label": f"{int(round(100 * fraction))}% hover-window progress",
                "absolute_time_s": absolute,
                "progress_fraction": float(fraction),
                "selection_rule": "uniform_fraction_of_stable_hover_to_prelanding_window",
                "gt_xyz_m": point.tolist(),
                "gt_quaternion_xyzw": quat.tolist(),
                "gt_yaw_rad": yaw_from_quaternion(quat),
            }
        )
    spatial_events = compute_spatial_events(gt_window_t, gt_window_xyz, gt_window_q)
    for event in spatial_events:
        targets.append(
            {
                "asset_key": str(event["event_id"]),
                "kind": "spatial",
                **event,
            }
        )

    nearest = extract_nearest_rgb(tuned_bag, targets)
    progress_records: List[Dict[str, object]] = []
    spatial_records: List[Dict[str, object]] = []
    assets: List[Dict[str, object]] = []
    frame_events: List[Dict[str, object]] = []
    for target in targets:
        key = str(target["asset_key"])
        frame = nearest[key]
        image = decode_compressed(frame["data"])
        folder = rgb_progress_dir if target["kind"] == "progress" else rgb_spatial_dir
        output = folder / f"{key}.png"
        if not cv2.imwrite(str(output), image):
            raise RuntimeError(f"failed to write {output}")
        event_record = {k: v for k, v in target.items() if k not in {"kind"}}
        event_record.update(
            {
                "bag_relative_time_s": float(target["absolute_time_s"]) - bag_start,
                "hover_relative_time_s": float(target["absolute_time_s"]) - hover_start,
                "rgb_message_time_s": frame["message_time_s"],
                "rgb_time_error_s": frame["difference_s"],
                "rgb_source_format": frame["format"],
                "asset_path": relative(output, root),
            }
        )
        frame_events.append(event_record)
        if target["kind"] == "progress":
            lines = [
                f"{spec['display_label']} ({spec['short_id']})",
                f"progress {100 * float(target['progress_fraction']):.0f}% | t_hover={event_record['hover_relative_time_s']:.2f}s",
            ]
        else:
            lines = [
                f"{spec['display_label']} ({spec['short_id']})",
                f"{target['label']} | t_hover={event_record['hover_relative_time_s']:.2f}s",
            ]
        contact = {"absolute_path": output, "contact_lines": lines}
        (progress_records if target["kind"] == "progress" else spatial_records).append(contact)
        assets.append(
            asset_record(
                root, output, f"{spec['short_id']}_{key}", str(spec["id"]),
                f"rgb_{target['kind']}", str(target["label"]),
                "qualitative_case_study",
                source_topic=RGB_TOPIC,
                source_absolute_time_s=frame["message_time_s"],
                target_absolute_time_s=float(target["absolute_time_s"]),
                nearest_frame_error_s=frame["difference_s"],
                selection_rule=target["selection_rule"],
            )
        )

    progress_sheet = session_dir / "rgb/progress_contact_sheet.png"
    spatial_sheet = session_dir / "rgb/spatial_event_contact_sheet.png"
    write_contact_sheet(progress_records, progress_sheet, columns=4)
    write_contact_sheet(spatial_records, spatial_sheet, columns=3)
    assets.extend(
        [
            asset_record(root, progress_sheet, f"{spec['short_id']}_progress_contact_sheet",
                         str(spec["id"]), "rgb_contact_sheet",
                         "Onboard RGB at 0–100% stable-hover progress anchors",
                         "qualitative_case_study"),
            asset_record(root, spatial_sheet, f"{spec['short_id']}_spatial_event_contact_sheet",
                         str(spec["id"]), "rgb_contact_sheet",
                         "Onboard RGB at deterministic GT spatial events",
                         "qualitative_case_study"),
            asset_record(root, gt_csv, f"{spec['short_id']}_gt_path_csv", str(spec["id"]),
                         "path_data", "GT pose samples inside evaluation window",
                         "quantitative_case_study", source_topic=GT_TOPIC),
            asset_record(root, est_csv, f"{spec['short_id']}_tuned_vio_path_csv", str(spec["id"]),
                         "path_data", "Post-hoc tuned VIO samples, with explicit time shift",
                         "quantitative_case_study", source_topic=EST_TOPIC),
            asset_record(root, associated_csv, f"{spec['short_id']}_associated_gt_vio_csv",
                         str(spec["id"]), "path_data",
                         "Time-associated GT/VIO and APE; no spatial alignment",
                         "quantitative_case_study"),
        ]
    )

    source_copy = provenance_dir / "splice_provenance.json"
    assets.append(
        asset_record(root, source_copy, f"{spec['short_id']}_splice_provenance",
                     str(spec["id"]), "provenance", "Source-preserving bag splice provenance",
                     "audit_only")
    )

    session = {
        "id": str(spec["id"]),
        "short_id": str(spec["short_id"]),
        "actual_condition": str(spec["actual_condition"]),
        "display_label": str(spec["display_label"]),
        "color": str(spec["color"]),
        "split_in_original_campaign": str(window_meta["split"]),
        "independent_validation_eligible": False,
        "selection": "post_hoc_case_study_pair",
        "source_bag": source_file_record,
        "tuned_spliced_bag": tuned_file_record,
        "splice_provenance_file": provenance_file_record,
        "window_source": file_record(WINDOW_CACHE_INDEX),
        "full_bag_start_s": bag_start,
        "hover_start_s": hover_start,
        "hover_end_s": hover_end,
        "hover_duration_s": hover_end - hover_start,
        "hover_window_method": str(windows["hover"]["method"]),
        "metrics": dict(associations["metrics"]),
        "frame_events": frame_events,
        # Private in-memory arrays are removed before manifest serialization.
        "gt_window_xyz": gt_window_xyz,
        "gt_window_q": gt_window_q,
        "associations": associations,
    }
    return session, assets


def build(output: Path, overwrite: bool) -> Path:
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() and not overwrite:
        raise FileExistsError(f"output exists; pass --overwrite: {output}")
    if output.exists():
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = output.with_name(f"{output.name}.previous_{stamp}")
        output.rename(backup)
        print(f"moved existing output to {backup}")

    temp = Path(tempfile.mkdtemp(prefix=f".{output.name}.building_", dir=output.parent))
    try:
        configure_matplotlib()
        window_rows = load_windows()
        sessions: List[Dict[str, object]] = []
        assets: List[Dict[str, object]] = []
        for spec in SESSIONS:
            print(f"extract {spec['short_id']} ({spec['display_label']})", flush=True)
            session, session_assets = build_session(spec, window_rows[str(spec["id"])], temp)
            sessions.append(session)
            assets.extend(session_assets)

        all_xy = []
        all_ape = []
        for session in sessions:
            all_xy.extend([session["gt_window_xyz"][:, :2], session["associations"]["est_xyz"][:, :2]])
            all_ape.append(session["associations"]["ape_m"])
        xy = np.vstack(all_xy)
        xmin, ymin = np.min(xy, axis=0)
        xmax, ymax_xy = np.max(xy, axis=0)
        pad = max(0.25, 0.06 * max(xmax - xmin, ymax_xy - ymin))
        limits = (float(xmin - pad), float(xmax + pad), float(ymin - pad), float(ymax_xy + pad))
        ymax_ape = float(max(0.25, 1.08 * max(float(np.max(row)) for row in all_ape)))

        figures_dir = temp / "figures"
        figures_dir.mkdir(parents=True, exist_ok=True)
        for session in sessions:
            sid = str(session["short_id"])
            path_svg, path_png = plot_session_path(
                session, limits, figures_dir / f"{sid}_gt_vs_tuned_vio_xy"
            )
            ape_svg, ape_png = plot_session_ape(
                session, ymax_ape, figures_dir / f"{sid}_tuned_vio_ape_over_time"
            )
            for fmt_path in (path_svg, path_png):
                assets.append(asset_record(
                    temp, fmt_path, f"{sid}_gt_vs_tuned_vio_xy_{fmt_path.suffix[1:]}",
                    str(session["id"]), "trajectory_figure",
                    "Recorded GT and post-hoc tuned VIO in OptiTrack XY frame",
                    "quantitative_case_study"
                ))
            for fmt_path in (ape_svg, ape_png):
                assets.append(asset_record(
                    temp, fmt_path, f"{sid}_tuned_vio_ape_over_time_{fmt_path.suffix[1:]}",
                    str(session["id"]), "ape_figure",
                    "Translation APE over stable-hover-to-prelanding window",
                    "quantitative_case_study"
                ))

        pair_path_svg, pair_path_png = plot_pair_path(
            sessions, limits, figures_dir / "pair_gt_vs_tuned_vio_xy"
        )
        pair_ape_svg, pair_ape_png = plot_pair_ape(
            sessions, ymax_ape, figures_dir / "pair_tuned_vio_ape_over_time"
        )
        for fmt_path, category, description in (
            (pair_path_svg, "trajectory_figure", "Side-by-side GT and tuned VIO paths"),
            (pair_path_png, "trajectory_figure", "Side-by-side GT and tuned VIO paths"),
            (pair_ape_svg, "ape_figure", "Selected sessions' tuned-VIO APE over time"),
            (pair_ape_png, "ape_figure", "Selected sessions' tuned-VIO APE over time"),
        ):
            assets.append(asset_record(
                temp, fmt_path, fmt_path.stem + "_" + fmt_path.suffix[1:], None,
                category, description, "case_study_pair_only"
            ))

        metric_rows = []
        for session in sessions:
            metric_rows.append(
                {
                    "session_id": session["id"],
                    "short_id": session["short_id"],
                    "actual_condition": session["actual_condition"],
                    "display_label": session["display_label"],
                    **session["metrics"],
                }
            )
        metrics_csv = temp / "metrics_summary.csv"
        metric_fields = list(metric_rows[0].keys())
        write_csv(metrics_csv, metric_fields, metric_rows)
        assets.append(asset_record(
            temp, metrics_csv, "metrics_summary_csv", None, "metrics",
            "Case-study VIO metrics with actual condition labels", "case_study_pair_only"
        ))

        public_sessions = []
        for session in sessions:
            public_sessions.append(
                {key: value for key, value in session.items()
                 if key not in {"gt_window_xyz", "gt_window_q", "associations"}}
            )
        manifest = {
            "schema": "risk_aware_realworld_paper_asset_pack/v1",
            "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "title": "Selected p1/PURE and pm4/PURE-Mean case-study assets",
            "pack_scope": "paper_and_powerpoint_case_study_assets",
            "post_hoc_pair_selection": True,
            "independent_validation": False,
            "webcam_perspective_overlays_included": False,
            "self_contained_viewing": True,
            "self_contained_reproduction": False,
            "caveats": list(GLOBAL_CAVEATS),
            "topics": {"gt": GT_TOPIC, "tuned_vio": EST_TOPIC, "onboard_rgb": RGB_TOPIC},
            "evaluation": {
                "window": "stable_hover_to_prelanding",
                "time_offset": "speed_profile_cross_correlation_per_session",
                "association": "linear_GT_position_and_quaternion_slerp_at_shifted_VIO_time",
                "association_max_diff_s": MAX_ASSOCIATION_DIFF_S,
                "spatial_alignment": "none",
                "physical_cloth_region_aabb_min_m": PHYSICAL_CLOTH_AABB_MIN_M.tolist(),
                "physical_cloth_region_aabb_max_m": PHYSICAL_CLOTH_AABB_MAX_M.tolist(),
                "rgb_progress_fractions": list(PROGRESS_FRACTIONS),
            },
            "builder": file_record(Path(__file__)),
            "sessions": public_sessions,
            "assets": assets,
        }
        manifest_path = temp / "manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

        manifest_csv = temp / "manifest_assets.csv"
        asset_fields = ["asset_id", "session_id", "category", "description", "intended_use",
                        "path", "format", "size_bytes", "sha256"]
        write_csv(manifest_csv, asset_fields, assets)

        readme = temp / "README_FIRST.md"
        readme.write_text(
            "# Real-world p1 / pm4 case-study assets\n\n"
            "Open `index.html` directly in a browser; no web server is needed. SVG files "
            "remain editable in PowerPoint and PNG plots have transparent backgrounds.\n\n"
            "Actual labels are **p1 = PURE** and **pm4 = PURE-Mean**. This post-hoc selected "
            "pair and post-hoc tuned VIO are qualitative/case-study material, not an unbiased "
            "two-sample quantitative comparison. See `manifest.json` for source hashes, exact "
            "windows, frame-selection rules, and all caveats. Webcam perspective overlays are "
            "not included in this version.\n\n"
            "For direct ROS1 bag inspection/export on a Windows machine without ROS, start "
            "with `README_WINDOWS.md`. The large bags are intentionally referenced through "
            "`transfer_manifest.json` instead of copied into this pack.\n",
            encoding="utf-8",
        )
        shutil.copy2(Path(__file__), temp / "build_paper_asset_pack.py")
        for source in WINDOWS_PORTABLE_FILES:
            if not source.is_file():
                raise FileNotFoundError(f"missing portable Windows asset: {source}")
            shutil.copy2(source, temp / source.name)

        # Add the top-level generated files after the embedded manifest is complete. The
        # HTML embeds the complete asset list; manifest files themselves are intentionally
        # outside that list to avoid recursive self-hashes.
        write_html(temp, manifest)
        checksum_rows = []
        for path in sorted(row for row in temp.rglob("*") if row.is_file()):
            if path.name == "checksums.sha256":
                continue
            checksum_rows.append(f"{sha256(path)}  {relative(path, temp)}")
        (temp / "checksums.sha256").write_text("\n".join(checksum_rows) + "\n")

        os.replace(temp, output)
        print(f"wrote {output}")
        return output
    except BaseException:
        shutil.rmtree(temp, ignore_errors=True)
        raise


def main() -> None:
    args = parse_args()
    build(args.output, args.overwrite)


if __name__ == "__main__":
    main()

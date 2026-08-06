#!/usr/bin/env python3
"""Reproducible multi-bag FAST-LIVO campaign for the 2026-08-03 flights.

This is deliberately a thin campaign layer over the existing
``replay_fastlivo.sh`` and ``eval_fastlivo.py`` tools.  It supplies the pieces
that those single-bag tools do not have:

* reconstruct the recorded compressed colour stream into the exact 10 Hz
  image/cloud pairs consumed by FAST-LIVO;
* pin the selected/held-out bag manifest and input hashes;
* run repeatable, sequential replays; and
* score no-spatial-alignment drift without allowing early death or a frozen
  estimator to look good.

Generated bags/results live below ``tools/fastlivo/_campaign_20260803`` and are
regenerable.  The source bags are opened read-only and are never modified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image

# Reuse the established time association/evaluation implementation.
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
RECORDINGS = Path("/home/ml/webcam_recorder/recordings")
DEFAULT_ROOT = Path(__file__).resolve().parent / "_campaign_20260803"

SELECTED = (
    "00-04-39",
    "00-57-18",
    "01-16-26",
    "02-13-44",
    "02-21-43",
    "02-56-57",
    "03-43-50",
    "04-24-27",
    "04-26-56",
)
DEVELOPMENT = ("00-04-39", "01-16-26", "02-21-43", "04-24-27")
VALIDATION = tuple(x for x in SELECTED if x not in DEVELOPMENT)

TOPIC_IMAGE_COMPRESSED = "/camera/color/image_raw/compressed"
TOPIC_IMAGE_RAW = "/camera/color/image_raw_10hz"
TOPIC_CLOUD = "/camera/depth/color/points_10hz"
TOPIC_IMU = "/camera/imu"
TOPIC_INFO = "/camera/color/camera_info"
TOPIC_GT = "/vrpn_client_node/pure/pose"
TOPIC_EST = "/aft_mapped_to_optitrack"
KEEP_TOPICS = {TOPIC_CLOUD, TOPIC_IMU, TOPIC_INFO, TOPIC_GT}


def _legacy_spec() -> Dict[str, object]:
    sessions = []
    for flight_id in SELECTED:
        stem = f"flight_2026-08-03_{flight_id}"
        sessions.append({
            "id": flight_id,
            "condition": "legacy",
            "source": RECORDINGS / stem / f"{stem}.bag",
            "split": "development" if flight_id in DEVELOPMENT else "validation",
        })
    return {
        "campaign": "fastlivo_2026-08-03",
        "sessions": sessions,
        "excluded": {
            "01-15-25": "1-second recording; insufficient initialization/evaluation",
            "04-26-56_good1": "byte-identical duplicate of 04-26-56",
        },
    }


def load_spec(path: Path | None) -> Dict[str, object]:
    if path is None:
        return _legacy_spec()
    document = json.loads(path.read_text())
    recordings_root = Path(document.get("recordings_root", RECORDINGS))
    sessions = []
    seen = set()
    for raw in document.get("sessions", []):
        row = dict(raw)
        flight_id = str(row["id"])
        if flight_id in seen:
            raise ValueError(f"duplicate session id in {path}: {flight_id}")
        seen.add(flight_id)
        source = Path(row["source"])
        if not source.is_absolute():
            source = recordings_root / source
        split = str(row["split"])
        if split not in {"development", "validation"}:
            raise ValueError(f"invalid split for {flight_id}: {split}")
        sessions.append({
            "id": flight_id,
            "condition": str(row.get("condition", "unspecified")),
            "source": source,
            "split": split,
        })
    if not sessions:
        raise ValueError(f"campaign spec has no sessions: {path}")
    return {
        "campaign": str(document.get("campaign", path.stem)),
        "sessions": sessions,
        "excluded": dict(document.get("excluded", {})),
        "spec_path": str(path.resolve()),
    }


def session_index(spec: Mapping[str, object]) -> Dict[str, Mapping[str, object]]:
    return {str(row["id"]): row for row in spec["sessions"]}


def source_bag(spec: Mapping[str, object], flight_id: str) -> Path:
    return Path(session_index(spec)[flight_id]["source"])


def canonical_bag(root: Path, flight_id: str) -> Path:
    return root / "canonical" / f"flight_2026-08-03_{flight_id}_canonical.bag"


def sha256(path: Path, block: int = 8 << 20) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            data = stream.read(block)
            if not data:
                return h.hexdigest()
            h.update(data)


def stamp_ns(msg) -> int:
    return int(msg.header.stamp.secs) * 1_000_000_000 + int(msg.header.stamp.nsecs)


def _topic_counts(path: Path) -> Dict[str, int]:
    with rosbag.Bag(str(path), "r") as bag:
        return {name: int(info.message_count)
                for name, info in bag.get_type_and_topic_info().topics.items()}


def prepare_one(src: Path, dst: Path, overwrite: bool = False) -> Dict[str, object]:
    if not src.is_file():
        raise FileNotFoundError(src)
    if dst.exists() and not overwrite:
        counts = _topic_counts(dst)
        return {
            "source": str(src), "canonical": str(dst), "reused": True,
            "source_sha256": sha256(src), "canonical_sha256": sha256(dst),
            "counts": counts,
        }

    image_stamps: set[int] = set()
    cloud_stamps: set[int] = set()
    first_info = None
    with rosbag.Bag(str(src), "r") as bag:
        for topic, msg, _ in bag.read_messages(
                topics=[TOPIC_IMAGE_COMPRESSED, TOPIC_CLOUD, TOPIC_INFO]):
            if topic == TOPIC_IMAGE_COMPRESSED:
                image_stamps.add(stamp_ns(msg))
            elif topic == TOPIC_CLOUD:
                cloud_stamps.add(stamp_ns(msg))
            elif first_info is None:
                first_info = msg

    paired_stamps = image_stamps & cloud_stamps
    if not cloud_stamps:
        raise RuntimeError(f"{src}: no {TOPIC_CLOUD}")
    pair_ratio = len(paired_stamps) / len(cloud_stamps)
    if pair_ratio < 0.95:
        raise RuntimeError(
            f"{src}: exact image/cloud pair ratio {pair_ratio:.3f} < 0.95")
    if first_info is None or (first_info.width, first_info.height) != (640, 480):
        raise RuntimeError(f"{src}: unexpected/missing 640x480 CameraInfo")

    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(".bag.active")
    if tmp.exists():
        tmp.unlink()
    written = {TOPIC_IMAGE_RAW: 0, TOPIC_CLOUD: 0, TOPIC_IMU: 0,
               TOPIC_INFO: 0, TOPIC_GT: 0}
    decoded_seen: set[int] = set()
    try:
        with rosbag.Bag(str(src), "r") as inp, rosbag.Bag(
                str(tmp), "w", compression=rosbag.Compression.LZ4) as out:
            for topic, msg, bag_time in inp.read_messages(
                    topics=list(KEEP_TOPICS | {TOPIC_IMAGE_COMPRESSED})):
                if topic == TOPIC_IMAGE_COMPRESSED:
                    ns = stamp_ns(msg)
                    if ns not in paired_stamps or ns in decoded_seen:
                        continue
                    arr = np.frombuffer(msg.data, dtype=np.uint8)
                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if bgr is None or bgr.shape != (480, 640, 3):
                        raise RuntimeError(
                            f"{src}: JPEG decode returned "
                            f"{None if bgr is None else bgr.shape} at {ns}")
                    raw = Image()
                    raw.header = msg.header
                    raw.height, raw.width = bgr.shape[:2]
                    raw.encoding = "bgr8"
                    raw.is_bigendian = 0
                    raw.step = raw.width * 3
                    raw.data = bgr.tobytes()
                    out.write(TOPIC_IMAGE_RAW, raw, bag_time)
                    decoded_seen.add(ns)
                    written[TOPIC_IMAGE_RAW] += 1
                elif topic == TOPIC_CLOUD:
                    if stamp_ns(msg) not in paired_stamps:
                        continue
                    out.write(topic, msg, bag_time)
                    written[topic] += 1
                else:
                    out.write(topic, msg, bag_time)
                    written[topic] += 1
        if written[TOPIC_IMAGE_RAW] != len(paired_stamps):
            raise RuntimeError(
                f"{src}: wrote {written[TOPIC_IMAGE_RAW]} images, "
                f"expected {len(paired_stamps)}")
        if written[TOPIC_CLOUD] != len(paired_stamps):
            raise RuntimeError(
                f"{src}: wrote {written[TOPIC_CLOUD]} clouds, "
                f"expected {len(paired_stamps)}")
        tmp.replace(dst)
    except BaseException:
        if tmp.exists():
            tmp.unlink()
        raise

    return {
        "source": str(src), "canonical": str(dst), "reused": False,
        "source_sha256": sha256(src), "canonical_sha256": sha256(dst),
        "source_image_count": len(image_stamps),
        "source_cloud_count": len(cloud_stamps),
        "paired_count": len(paired_stamps), "pair_ratio": pair_ratio,
        "dropped_cloud_count": len(cloud_stamps) - len(paired_stamps),
        "camera_info": {
            "width": int(first_info.width), "height": int(first_info.height),
            "K": [float(x) for x in first_info.K],
            "distortion_model": first_info.distortion_model,
        },
        "counts": written,
    }


def prepare(root: Path, spec: Mapping[str, object], ids: Sequence[str],
            overwrite: bool) -> None:
    root.mkdir(parents=True, exist_ok=True)
    by_id = session_index(spec)
    rows = []
    for i, flight_id in enumerate(ids, 1):
        print(f"[prepare {i}/{len(ids)}] {flight_id}", flush=True)
        row = prepare_one(source_bag(spec, flight_id), canonical_bag(root, flight_id), overwrite)
        row["flight_id"] = flight_id
        row["condition"] = by_id[flight_id]["condition"]
        row["split"] = by_id[flight_id]["split"]
        rows.append(row)
        print("  pairs={}/{} ({:.2%}) canonical={:.1f} MiB".format(
            row.get("paired_count", row["counts"].get(TOPIC_CLOUD, 0)),
            row.get("source_cloud_count", row["counts"].get(TOPIC_CLOUD, 0)),
            row.get("pair_ratio", 1.0),
            canonical_bag(root, flight_id).stat().st_size / (1 << 20)), flush=True)
    manifest = {
        "campaign": spec["campaign"],
        "created_unix": time.time(),
        "selected": [str(row["id"]) for row in spec["sessions"]],
        "development": _ids_for_group("dev", spec),
        "validation": _ids_for_group("validation", spec),
        "excluded": spec.get("excluded", {}),
        "spec_path": spec.get("spec_path"),
        "bags": rows,
    }
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"[prepare] wrote {path}")


def inspect_inputs(spec: Mapping[str, object], out_dir: Path, stride: int) -> None:
    """Summarize raw RGB/cloud quality without using estimator outcomes."""
    if stride < 1:
        raise ValueError("stride must be at least one")
    rows = []
    for index, session in enumerate(spec["sessions"], 1):
        flight_id = str(session["id"])
        print(f"[inputs {index}/{len(spec['sessions'])}] {flight_id}", flush=True)
        image_luma = []
        image_contrast = []
        image_blur = []
        image_dark = []
        cloud_points = []
        cloud_median_range = []
        cloud_p90_range = []
        counts = {TOPIC_IMAGE_COMPRESSED: 0, TOPIC_CLOUD: 0}
        with rosbag.Bag(str(Path(session["source"])), "r") as bag:
            for topic, msg, _ in bag.read_messages(
                    topics=[TOPIC_IMAGE_COMPRESSED, TOPIC_CLOUD]):
                counts[topic] += 1
                if (counts[topic] - 1) % stride:
                    continue
                if topic == TOPIC_IMAGE_COMPRESSED:
                    bgr = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                                       cv2.IMREAD_GRAYSCALE)
                    if bgr is None:
                        continue
                    image_luma.append(float(np.mean(bgr)))
                    image_contrast.append(float(np.std(bgr)))
                    image_blur.append(float(cv2.Laplacian(
                        bgr, cv2.CV_64F).var()))
                    image_dark.append(float(np.mean(bgr < 30)))
                else:
                    endian = ">" if msg.is_bigendian else "<"
                    point_dtype = np.dtype({
                        "names": ("x", "y", "z"),
                        "formats": (f"{endian}f4",) * 3,
                        "offsets": (0, 4, 8),
                        "itemsize": msg.point_step,
                    })
                    points = np.frombuffer(msg.data, dtype=point_dtype)
                    xyz = np.column_stack((points["x"], points["y"], points["z"]))
                    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
                    ranges = np.linalg.norm(xyz, axis=1)
                    cloud_points.append(float(len(xyz)))
                    if len(ranges):
                        cloud_median_range.append(float(np.median(ranges)))
                        cloud_p90_range.append(float(np.quantile(ranges, 0.9)))
        metrics = {
            "image_luma_median": image_luma,
            "image_contrast_median": image_contrast,
            "image_blur_median": image_blur,
            "image_dark_fraction_median": image_dark,
            "cloud_points_median": cloud_points,
            "cloud_range_median_m": cloud_median_range,
            "cloud_range_p90_m": cloud_p90_range,
        }
        row: Dict[str, object] = {
            "flight_id": flight_id,
            "condition": session["condition"],
            "split": session["split"],
            "sample_stride": stride,
            "image_samples": len(image_luma),
            "cloud_samples": len(cloud_points),
        }
        for name, values in metrics.items():
            if not values:
                raise RuntimeError(f"{flight_id}: no values for {name}")
            row[name] = float(np.median(values))
        rows.append(row)

    out_dir.mkdir(parents=True, exist_ok=True)
    keys = list(rows[0])
    with (out_dir / "input_sessions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    condition_rows = []
    metric_names = [key for key in keys if key.endswith("_median") or
                    key.endswith("_median_m") or key.endswith("_p90_m")]
    for condition in ("pure_wodz", "pure", "pure_mean", "nominal"):
        group = [row for row in rows if row["condition"] == condition]
        item: Dict[str, object] = {"condition": condition, "n": len(group)}
        for name in metric_names:
            item[f"{name}_mean"] = float(np.mean([
                float(row[name]) for row in group]))
        condition_rows.append(item)
    with (out_dir / "input_conditions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(condition_rows[0]))
        writer.writeheader()
        writer.writerows(condition_rows)
    print(f"[inputs] wrote {out_dir}")


def uniform_path_length(t: np.ndarray, xyz: np.ndarray, hz: float = 10.0) -> float:
    if len(t) < 2:
        return 0.0
    tq = np.arange(t[0], t[-1] + 1e-9, 1.0 / hz)
    p = np.column_stack([np.interp(tq, t, xyz[:, i]) for i in range(3)])
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def translation_rpe(tm: np.ndarray, pe: np.ndarray, pg: np.ndarray,
                    delta_s: float = 1.0) -> np.ndarray:
    """World-frame relative translation error over approximately ``delta_s``."""
    values = []
    j = 1
    for i in range(len(tm)):
        j = max(j, i + 1)
        while j < len(tm) and tm[j] - tm[i] < delta_s:
            j += 1
        if j >= len(tm):
            break
        values.append(np.linalg.norm(
            (pe[j] - pe[i]) - (pg[j] - pg[i])))
    return np.asarray(values, dtype=float)


def trajectory_trend_metrics(tm: np.ndarray, pe: np.ndarray,
                             pg: np.ndarray) -> Dict[str, object]:
    """Measure whether an estimate follows GT rather than freezing/reversing.

    This deliberately performs no spatial alignment.  The associated estimate
    and GT positions are first interpolated to a common 10 Hz timeline.  Each
    overlapping one-second window is active when GT moves at least 8 cm.

    ``weighted_cosine`` is the GT-distance-weighted cosine between estimated
    and GT one-second displacements.  ``reverse_distance_fraction`` is the
    fraction of active GT distance whose displacement cosine is negative.
    ``stall_fraction`` is the fraction of active windows in which the estimate
    moves less than 3 cm.  ``progress_correlation`` correlates cumulative GT and
    estimated displacement projected onto GT's full-episode displacement.

    The trend gate is intentionally lenient relative to the well-tracked
    campaign flights: cosine >= 0.5, reverse distance <= 0.2, stall <= 0.1,
    and progress correlation >= 0.5.  Existing ``valid``/``integrity`` fields
    remain unchanged; this is an additional diagnostic gate.
    """
    names = (
        "trend_1s_weighted_cosine",
        "trend_reverse_distance_fraction",
        "trend_stall_fraction",
        "trend_progress_correlation",
        "net_displacement_cosine",
        "net_displacement_ratio",
    )
    invalid: Dict[str, object] = {name: math.nan for name in names}
    invalid["trend_consistent"] = False
    if len(tm) < 2 or len(pe) != len(tm) or len(pg) != len(tm):
        return invalid

    timeline = np.arange(float(tm[0]), float(tm[-1]) + 1e-9, 0.1)
    if len(timeline) <= 10:
        return invalid
    est = np.column_stack([
        np.interp(timeline, tm, pe[:, axis]) for axis in range(3)
    ])
    gt = np.column_stack([
        np.interp(timeline, tm, pg[:, axis]) for axis in range(3)
    ])

    gt_delta = gt[10:] - gt[:-10]
    est_delta = est[10:] - est[:-10]
    gt_distance = np.linalg.norm(gt_delta, axis=1)
    est_distance = np.linalg.norm(est_delta, axis=1)
    active = gt_distance >= 0.08
    active_weight = gt_distance[active]
    if not np.any(active) or float(np.sum(active_weight)) <= 1e-12:
        return invalid

    cosine = np.sum(gt_delta * est_delta, axis=1) / (
        gt_distance * est_distance + 1e-12)
    weighted_cosine = float(np.sum(
        active_weight * cosine[active]) / np.sum(active_weight))
    reverse_fraction = float(np.sum(
        active_weight[cosine[active] < 0.0]) / np.sum(active_weight))
    stall_fraction = float(np.mean(est_distance[active] < 0.03))

    gt_net = gt[-1] - gt[0]
    est_net = est[-1] - est[0]
    gt_net_norm = float(np.linalg.norm(gt_net))
    est_net_norm = float(np.linalg.norm(est_net))
    net_cosine = math.nan
    net_ratio = math.nan
    progress_correlation = math.nan
    if gt_net_norm > 1e-12:
        net_ratio = est_net_norm / gt_net_norm
        if est_net_norm > 1e-12:
            net_cosine = float(np.dot(gt_net, est_net) /
                               (gt_net_norm * est_net_norm))
        progress_axis = gt_net / gt_net_norm
        gt_progress = (gt - gt[0]) @ progress_axis
        est_progress = (est - est[0]) @ progress_axis
        if (float(np.std(gt_progress)) > 1e-12 and
                float(np.std(est_progress)) > 1e-12):
            progress_correlation = float(np.corrcoef(
                gt_progress, est_progress)[0, 1])

    finite_gate = all(math.isfinite(value) for value in (
        weighted_cosine, reverse_fraction, stall_fraction,
        progress_correlation))
    return {
        "trend_1s_weighted_cosine": weighted_cosine,
        "trend_reverse_distance_fraction": reverse_fraction,
        "trend_stall_fraction": stall_fraction,
        "trend_progress_correlation": progress_correlation,
        "net_displacement_cosine": net_cosine,
        "net_displacement_ratio": net_ratio,
        "trend_consistent": bool(
            finite_gate and weighted_cosine >= 0.5 and
            reverse_fraction <= 0.2 and stall_fraction <= 0.1 and
            progress_correlation >= 0.5),
    }


def score_bag(path: Path, source_path: Path | None = None) -> Dict[str, object]:
    tg, xg, qg = read_traj(str(path), TOPIC_GT)
    gt_path = uniform_path_length(tg, xg)
    gt_up = np.asarray([q_to_R(q)[:, 2] for q in qg])
    gt_reference_up = np.mean(gt_up[tg <= tg[0] + 2.0], axis=0)
    gt_reference_up /= np.linalg.norm(gt_reference_up)
    gt_tilt_deg = np.degrees(np.arccos(np.clip(
        gt_up @ gt_reference_up, -1.0, 1.0)))
    row: Dict[str, object] = {
        "result_bag": str(path), "gt_path_m": gt_path,
        "valid": False, "catastrophic": True,
    }
    try:
        te, xe, qe = read_traj(str(path), TOPIC_EST)
    except SystemExit as exc:
        row.update(failure=f"missing_estimate: {exc}", output_count=0,
                   coverage=0.0, output_ratio=0.0)
        return row

    if len(te) < 10 or not np.all(np.isfinite(xe)):
        row.update(failure="too_few_or_nonfinite_estimates", output_count=len(te),
                   coverage=0.0, output_ratio=0.0)
        return row

    offset = estimate_offset(te, xe, tg, xg)
    pairs = associate(te + offset, tg, 0.05)
    if len(pairs) < 10:
        row.update(failure="fewer_than_10_time_associations", output_count=len(te),
                   associations=len(pairs), coverage=0.0, output_ratio=0.0)
        return row
    pe = np.asarray([xe[i] for i, _, _, _ in pairs])
    pg = np.asarray([xg[k0] + u * (xg[k1] - xg[k0])
                     for i, k0, k1, u in pairs])
    qgi = np.asarray([slerp(qg[k0], qg[k1], u)
                      for _, k0, k1, u in pairs])
    orientation_error_deg = np.asarray([
        geodesic_deg(q_to_R(qe[i]), q_to_R(q_ref))
        for (i, _, _, _), q_ref in zip(pairs, qgi)
    ])
    tilt_axis_error_deg = np.asarray([
        math.degrees(math.acos(float(np.clip(
            np.dot(q_to_R(qe[i])[:, 2], q_to_R(q_ref)[:, 2]), -1.0, 1.0))))
        for (i, _, _, _), q_ref in zip(pairs, qgi)
    ])
    tm = np.asarray([te[i] + offset for i, _, _, _ in pairs])
    ape = np.linalg.norm(pe - pg, axis=1)
    rpe_1s = translation_rpe(tm, pe, pg)
    trend = trajectory_trend_metrics(tm, pe, pg)
    est_path = float(np.linalg.norm(np.diff(pe, axis=0), axis=1).sum())
    gt_assoc_path = float(np.linalg.norm(np.diff(pg, axis=0), axis=1).sum())
    eligible_span = max(1e-9, tg[-1] - tm[0])
    coverage = min(1.0, max(0.0, (tm[-1] - tm[0]) / eligible_span))
    max_gap = float(np.diff(tm).max()) if len(tm) > 1 else math.inf

    output_ratio = math.nan
    if source_path and source_path.is_file():
        expected = 0
        with rosbag.Bag(str(source_path), "r") as bag:
            for _, msg, _ in bag.read_messages(topics=[TOPIC_CLOUD]):
                ts = msg.header.stamp.to_sec()
                if tm[0] - 0.05 <= ts <= tg[-1] + 0.05:
                    expected += 1
        output_ratio = len(te) / expected if expected else 0.0

    threshold = 2.0 * gt_path
    idx = np.flatnonzero(ape > threshold)
    catastrophic = bool(len(idx))
    motion_ratio = est_path / gt_assoc_path if gt_assoc_path > 1e-6 else math.inf
    # numpy scalar comparisons return np.bool_, which json.dumps cannot
    # serialize.  Normalize the campaign boundary to plain Python types.
    integrity = bool(
        coverage >= 0.95 and max_gap <= 0.5 and
        (math.isnan(output_ratio) or output_ratio >= 0.90) and
        0.5 <= motion_ratio <= 2.0
    )
    row.update(
        output_count=len(te), associations=len(pairs),
        time_offset_s=float(offset), coverage=coverage,
        output_ratio=output_ratio, max_output_gap_s=max_gap,
        est_path_m=est_path, gt_associated_path_m=gt_assoc_path,
        gt_duration_s=float(tg[-1] - tg[0]),
        gt_tilt_p90_deg=float(np.quantile(gt_tilt_deg, 0.9)),
        gt_tilt_over_20_fraction=float(np.mean(gt_tilt_deg > 20.0)),
        motion_ratio=motion_ratio,
        rmse_m=float(np.sqrt(np.mean(ape * ape))),
        mean_ape_m=float(np.mean(ape)), max_ape_m=float(np.max(ape)),
        final_ape_m=float(ape[-1]),
        rmse_per_gt_path=float(np.sqrt(np.mean(ape * ape)) / gt_path)
        if gt_path > 1e-9 else math.inf,
        rpe_1s_rmse_m=float(np.sqrt(np.mean(rpe_1s * rpe_1s)))
        if len(rpe_1s) else math.nan,
        **trend,
        orientation_rmse_deg=float(np.sqrt(np.mean(
            orientation_error_deg * orientation_error_deg))),
        orientation_mean_deg=float(np.mean(orientation_error_deg)),
        orientation_p90_deg=float(np.quantile(orientation_error_deg, 0.9)),
        orientation_max_deg=float(np.max(orientation_error_deg)),
        tilt_axis_rmse_deg=float(np.sqrt(np.mean(
            tilt_axis_error_deg * tilt_axis_error_deg))),
        tilt_axis_mean_deg=float(np.mean(tilt_axis_error_deg)),
        tilt_axis_p90_deg=float(np.quantile(tilt_axis_error_deg, 0.9)),
        tilt_axis_max_deg=float(np.max(tilt_axis_error_deg)),
        catastrophic_threshold_m=threshold, catastrophic=catastrophic,
        catastrophic_onset_s=(float(tm[idx[0]] - tg[0]) if len(idx) else None),
        integrity=integrity, valid=(not catastrophic and integrity),
        failure=(None if not catastrophic and integrity else
                 "catastrophic" if catastrophic else "integrity_gate"),
    )
    return row


def _ids_for_group(group: str, spec: Mapping[str, object]) -> List[str]:
    sessions = spec["sessions"]
    if group == "all":
        return [str(row["id"]) for row in sessions]
    split = "development" if group == "dev" else "validation"
    return [str(row["id"]) for row in sessions if row["split"] == split]


def runtime_path(host_path: Path) -> str:
    host_path = host_path.resolve()
    native_ws = Path(os.environ.get("FASTLIVO_WS", Path.home() / "fast_livo2_d435i"))
    if sys.platform.startswith("linux") and os.uname().machine == "x86_64" and \
            (native_ws / "devel/setup.bash").is_file():
        return str(host_path)
    try:
        return "/work/" + str(host_path.relative_to(REPO))
    except ValueError as exc:
        raise ValueError(f"path must be inside mounted repository {REPO}: {host_path}") from exc


def run_replay(cmd: List[str], log_path: Path, timeout_s: float) -> int:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log:
        proc = subprocess.Popen(cmd, cwd=str(REPO), stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            return 124


def run_campaign(root: Path, spec: Mapping[str, object], group: str, tag: str,
                 config: Path | None,
                 overlay: Path | None,
                 rate: float, repeat: int, force: bool,
                 selected_ids: Sequence[str] | None = None) -> None:
    manifest_path = root / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"prepare first: {manifest_path} is missing")
    config_container = runtime_path(config) if config else None
    overlay_runtime = runtime_path(overlay) if overlay else None
    run_root = root / "runs" / tag
    run_root.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, object]] = []
    for rep in range(1, repeat + 1):
        for flight_id in (selected_ids or _ids_for_group(group, spec)):
            if flight_id not in session_index(spec):
                raise ValueError(f"unknown session id: {flight_id}")
            src = canonical_bag(root, flight_id)
            out = run_root / f"{flight_id}_r{rep}.bag"
            log = run_root / f"{flight_id}_r{rep}.log"
            print(f"[run] tag={tag} rep={rep}/{repeat} bag={flight_id} rate={rate}",
                  flush=True)
            if force or not out.is_file():
                cmd = ["bash", str(REPO / "tools/fastlivo/replay_fastlivo.sh"),
                       runtime_path(src), "--rate", str(rate),
                       "--out", runtime_path(out)]
                if config_container:
                    cmd += ["--config", config_container]
                if overlay_runtime:
                    cmd += ["--overlay", overlay_runtime]
                timeout_s = max(120.0, 2.0 * 100.0 / rate + 60.0)
                rc = run_replay(cmd, log, timeout_s)
            else:
                rc = 0
            if rc != 0 or not out.is_file():
                row = {"tag": tag, "flight_id": flight_id, "repeat": rep,
                       "rate": rate, "returncode": rc, "valid": False,
                       "catastrophic": True, "failure": "replay_failed"}
            else:
                row = score_bag(out, src)
                row.update(tag=tag, flight_id=flight_id, repeat=rep,
                           rate=rate, returncode=rc,
                           config=str(config) if config else "production-default",
                           overlay=str(overlay) if overlay else "none")
            rows.append(row)
            print("  valid={} catastrophic={} rmse={} coverage={} failure={}".format(
                row.get("valid"), row.get("catastrophic"), row.get("rmse_m"),
                row.get("coverage"), row.get("failure")), flush=True)
            _write_rows(run_root, rows)


def _write_rows(run_root: Path, rows: List[Dict[str, object]]) -> None:
    (run_root / "results.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True, allow_nan=True) + "\n")
    keys = sorted({key for row in rows for key in row})
    with (run_root / "results.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def evaluate_paths(root: Path, spec: Mapping[str, object], paths: Sequence[Path],
                   out: Path | None) -> None:
    rows = []
    selected = _ids_for_group("all", spec)
    for path in paths:
        # Campaign outputs are named <flight-id>_rN.bag.
        flight_id = next((x for x in selected if x in path.name), None)
        src = canonical_bag(root, flight_id) if flight_id else None
        row = score_bag(path, src)
        row["flight_id"] = flight_id
        rows.append(row)
        print(json.dumps(row, sort_keys=True, allow_nan=True), flush=True)
    if out:
        out.parent.mkdir(parents=True, exist_ok=True)
        _write_rows(out.parent, rows)


def _bootstrap_interval(values: np.ndarray, statistic, rng: np.random.Generator,
                        samples: int) -> Tuple[float, float]:
    draws = rng.choice(values, size=(samples, len(values)), replace=True)
    estimates = statistic(draws, axis=1)
    lo, hi = np.quantile(estimates, [0.025, 0.975])
    return float(lo), float(hi)


FUSION_MASKS = {
    "nonfinite": 1,
    "low_support": 2,
    "low_quality": 4,
    "bad_update": 8,
    "bad_motion": 16,
    "bad_tilt": 32,
}


def _fusion_stage_stats(rows: Sequence[Mapping[str, str]],
                        prefix: str) -> Dict[str, object]:
    if not rows:
        return {f"{prefix}_rows": 0}
    nfeat = np.asarray([float(x["nfeat"]) for x in rows])
    has_guard_columns = all(
        field in rows[0] for field in ("rejected", "reject_mask", "tilt_deg"))
    result: Dict[str, object] = {
        f"{prefix}_rows": len(rows),
        f"{prefix}_nfeat_median": float(np.median(nfeat)),
        f"{prefix}_nfeat_p10": float(np.quantile(nfeat, 0.1)),
        f"{prefix}_nfeat_le_50_fraction": float(np.mean(nfeat <= 50)),
        f"{prefix}_health_guard_available": has_guard_columns,
    }
    if has_guard_columns:
        rejected = np.asarray(
            [int(x["rejected"]) for x in rows], dtype=bool)
        masks = np.asarray(
            [int(x["reject_mask"]) for x in rows], dtype=np.uint32)
        tilt = np.asarray([float(x["tilt_deg"]) for x in rows])
        result[f"{prefix}_reject_fraction"] = float(np.mean(rejected))
        result[f"{prefix}_tilt_p90_deg"] = float(np.quantile(tilt, 0.9))
        for name, bit in FUSION_MASKS.items():
            result[f"{prefix}_{name}_fraction"] = float(
                np.mean((masks & bit) != 0))
    else:
        # New FAST-LIVO logs omit the retired project-specific health guard.
        # Keep the historical summary schema explicit without fabricating a
        # zero rejection rate for runs in which no guard existed.
        result[f"{prefix}_reject_fraction"] = math.nan
        result[f"{prefix}_tilt_p90_deg"] = math.nan
        for name in FUSION_MASKS:
            result[f"{prefix}_{name}_fraction"] = math.nan
    for field in (
            "lio_trans_info_ratio", "lio_rot_info_ratio",
            "lio_info_min_per_feature", "vio_trans_info_ratio",
            "vio_rot_info_ratio", "vio_info_min_per_measurement",
            "vio_inlier_ratio", "vio_error_ratio"):
        values = np.asarray([float(x[field]) for x in rows])
        result[f"{prefix}_{field}_median"] = float(np.median(values))
    return result


def load_fusion_diagnostics(fusion_dir: Path,
                            ids: Sequence[str]) -> Dict[str, Dict[str, object]]:
    diagnostics: Dict[str, Dict[str, object]] = {}
    for path in sorted(fusion_dir.glob("*_fusion.csv")):
        flight_id = next((x for x in ids if path.name.startswith(f"{x}_")), None)
        if flight_id is None:
            continue
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        item: Dict[str, object] = {"fusion_csv": str(path)}
        for stage, prefix in (("LIO", "lio"), ("VIO", "vio")):
            item.update(_fusion_stage_stats(
                [row for row in rows if row["stage"] == stage], prefix))
        diagnostics[flight_id] = item
    return diagnostics


def write_fusion_summary(rows: Sequence[Mapping[str, object]], fusion_dir: Path,
                         out_dir: Path) -> None:
    ids = [str(row["flight_id"]) for row in rows]
    diagnostics = load_fusion_diagnostics(fusion_dir, ids)
    missing = sorted(set(ids) - set(diagnostics))
    if missing:
        raise ValueError(f"fusion diagnostics missing sessions: {missing}")
    merged = []
    for row in rows:
        item = dict(row)
        item.update(diagnostics[str(row["flight_id"])])
        merged.append(item)

    keys = sorted({key for row in merged for key in row})
    with (out_dir / "fusion_sessions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(merged)

    condition_metrics = (
        "lio_nfeat_median", "lio_nfeat_le_50_fraction", "lio_reject_fraction",
        "lio_bad_motion_fraction", "lio_bad_tilt_fraction",
        "lio_lio_trans_info_ratio_median", "lio_lio_rot_info_ratio_median",
        "lio_lio_info_min_per_feature_median",
        "vio_nfeat_median", "vio_reject_fraction",
        "vio_low_support_fraction", "vio_bad_motion_fraction",
        "vio_bad_tilt_fraction", "vio_vio_trans_info_ratio_median",
        "vio_vio_rot_info_ratio_median", "vio_vio_info_min_per_measurement_median",
    )
    condition_rows = []
    for condition in ("pure_wodz", "pure", "pure_mean", "nominal"):
        group = [x for x in merged if x["condition"] == condition]
        item: Dict[str, object] = {"condition": condition, "n": len(group)}
        for metric in condition_metrics:
            item[f"{metric}_mean"] = float(np.mean([
                float(x[metric]) for x in group]))
            item[f"{metric}_median"] = float(np.median([
                float(x[metric]) for x in group]))
        condition_rows.append(item)
    condition_keys = list(condition_rows[0])
    with (out_dir / "fusion_conditions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=condition_keys)
        writer.writeheader()
        writer.writerows(condition_rows)

    diagnostic_metrics = (
        "lio_nfeat_median", "lio_nfeat_le_50_fraction", "lio_reject_fraction",
        "lio_bad_motion_fraction", "lio_bad_tilt_fraction", "lio_tilt_p90_deg",
        "lio_lio_trans_info_ratio_median", "lio_lio_rot_info_ratio_median",
        "lio_lio_info_min_per_feature_median", "vio_nfeat_median",
        "vio_reject_fraction", "vio_low_support_fraction",
        "vio_bad_motion_fraction", "vio_bad_tilt_fraction",
        "vio_tilt_p90_deg", "vio_vio_inlier_ratio_median",
        "vio_vio_trans_info_ratio_median", "vio_vio_rot_info_ratio_median",
        "vio_vio_info_min_per_measurement_median",
    )
    targets = ("rmse_m", "orientation_rmse_deg", "tilt_axis_rmse_deg",
               "motion_ratio")
    correlations: Dict[str, object] = {}
    for target in targets:
        target_values = np.asarray([float(x[target]) for x in merged])
        correlations[target] = {}
        for metric in diagnostic_metrics:
            values = np.asarray([float(x[metric]) for x in merged])
            if np.std(values) <= 1e-15 or np.std(target_values) <= 1e-15:
                correlations[target][metric] = None
            else:
                correlations[target][metric] = float(np.corrcoef(
                    target_values, values)[0, 1])
    (out_dir / "fusion_correlations.json").write_text(
        json.dumps(correlations, indent=2, sort_keys=True) + "\n")


def summarize_results(spec: Mapping[str, object], result_paths: Sequence[Path],
                      out_dir: Path, bootstrap_samples: int, seed: int,
                      fusion_dir: Path | None = None) -> None:
    by_id = session_index(spec)
    rows = []
    for path in result_paths:
        rows.extend(json.loads(path.read_text()))
    ids = [str(row["flight_id"]) for row in rows]
    duplicates = sorted({x for x in ids if ids.count(x) > 1})
    if duplicates:
        raise ValueError(f"duplicate sessions across result files: {duplicates}")
    if set(ids) != set(by_id):
        missing = sorted(set(by_id) - set(ids))
        extra = sorted(set(ids) - set(by_id))
        raise ValueError(f"result/spec mismatch; missing={missing}, extra={extra}")

    out_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    conditions = ["pure_wodz", "pure", "pure_mean", "nominal"]
    metrics = ("rmse_m", "rmse_per_gt_path", "rpe_1s_rmse_m", "motion_ratio",
               "orientation_rmse_deg", "tilt_axis_rmse_deg", "gt_tilt_p90_deg")
    grouped: Dict[str, List[Dict[str, object]]] = {name: [] for name in conditions}
    for row in rows:
        row["condition"] = by_id[str(row["flight_id"])]["condition"]
        row["split"] = by_id[str(row["flight_id"])]["split"]
        grouped[str(row["condition"])].append(row)

    summaries = {}
    for condition in conditions:
        group = grouped[condition]
        item: Dict[str, object] = {
            "n": len(group),
            "catastrophic_count": sum(bool(x["catastrophic"]) for x in group),
            "integrity_failure_count": sum(not bool(x["integrity"]) for x in group),
        }
        for metric in metrics:
            values = np.asarray([float(x[metric]) for x in group], dtype=float)
            mean_ci = _bootstrap_interval(values, np.mean, rng, bootstrap_samples)
            median_ci = _bootstrap_interval(values, np.median, rng, bootstrap_samples)
            item[metric] = {
                "mean": float(np.mean(values)),
                "median": float(np.median(values)),
                "std": float(np.std(values, ddof=1)) if len(values) > 1 else 0.0,
                "min": float(np.min(values)),
                "max": float(np.max(values)),
                "p90": float(np.quantile(values, 0.9)),
                "mean_ci95": list(mean_ci),
                "median_ci95": list(median_ci),
            }
        summaries[condition] = item

    # Stratified bootstrap preserves the deliberately unequal group sizes.
    order_hits_mean = 0
    order_hits_median = 0
    adjusted_draws = {name: [] for name in conditions}

    def adjusted_predictions(sample_rows: Sequence[Mapping[str, object]]) -> Dict[str, float]:
        paths = np.asarray([float(x["gt_path_m"]) for x in sample_rows])
        durations = np.asarray([float(x["gt_duration_s"]) for x in sample_rows])
        path_center = float(np.mean([float(x["gt_path_m"]) for x in rows]))
        duration_center = float(np.mean([float(x["gt_duration_s"]) for x in rows]))
        design = []
        target = []
        for row, path_m, duration_s in zip(sample_rows, paths, durations):
            condition = str(row["condition"])
            design.append([
                1.0,
                path_m - path_center,
                duration_s - duration_center,
                float(condition == "pure_wodz"),
                float(condition == "pure"),
                float(condition == "pure_mean"),
            ])
            target.append(float(row["rmse_m"]))
        beta, *_ = np.linalg.lstsq(np.asarray(design), np.asarray(target), rcond=None)
        return {
            "nominal": float(beta[0]),
            "pure_wodz": float(beta[0] + beta[3]),
            "pure": float(beta[0] + beta[4]),
            "pure_mean": float(beta[0] + beta[5]),
        }

    adjusted_point = adjusted_predictions(rows)
    for _ in range(bootstrap_samples):
        sample = []
        boot_stats = {}
        for condition in conditions:
            group = grouped[condition]
            indices = rng.integers(0, len(group), size=len(group))
            chosen = [group[i] for i in indices]
            sample.extend(chosen)
            values = np.asarray([float(x["rmse_m"]) for x in chosen])
            boot_stats[condition] = (float(np.mean(values)), float(np.median(values)))
        order_hits_mean += int(
            boot_stats["pure"][0] < boot_stats["pure_mean"][0] <
            boot_stats["nominal"][0])
        order_hits_median += int(
            boot_stats["pure"][1] < boot_stats["pure_mean"][1] <
            boot_stats["nominal"][1])
        predicted = adjusted_predictions(sample)
        for condition in conditions:
            adjusted_draws[condition].append(predicted[condition])

    adjusted = {
        condition: {
            "rmse_at_overall_mean_path_and_duration": adjusted_point[condition],
            "ci95": [float(x) for x in np.quantile(
                np.asarray(adjusted_draws[condition]), [0.025, 0.975])],
        }
        for condition in conditions
    }
    pure = grouped["pure"]
    pure_mean = grouped["pure_mean"]
    nominal = grouped["nominal"]
    matching_triplets = sum(
        float(a["rmse_m"]) < float(b["rmse_m"]) < float(c["rmse_m"])
        for a in pure for b in pure_mean for c in nominal)
    triplet_total = len(pure) * len(pure_mean) * len(nominal)
    result = {
        "campaign": spec["campaign"],
        "seed": seed,
        "bootstrap_samples": bootstrap_samples,
        "session_count": len(rows),
        "catastrophic_count": sum(bool(x["catastrophic"]) for x in rows),
        "integrity_failure_count": sum(not bool(x["integrity"]) for x in rows),
        "conditions": summaries,
        "adjusted_for_gt_path_and_duration": adjusted,
        "expected_order_pure_lt_pure_mean_lt_nominal": {
            "observed_mean": bool(
                summaries["pure"]["rmse_m"]["mean"] <
                summaries["pure_mean"]["rmse_m"]["mean"] <
                summaries["nominal"]["rmse_m"]["mean"]),
            "observed_median": bool(
                summaries["pure"]["rmse_m"]["median"] <
                summaries["pure_mean"]["rmse_m"]["median"] <
                summaries["nominal"]["rmse_m"]["median"]),
            "bootstrap_probability_mean": order_hits_mean / bootstrap_samples,
            "bootstrap_probability_median": order_hits_median / bootstrap_samples,
            "matching_individual_triplets": int(matching_triplets),
            "individual_triplets_total": int(triplet_total),
        },
    }
    (out_dir / "summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=True) + "\n")

    session_keys = sorted({key for row in rows for key in row})
    with (out_dir / "sessions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=session_keys)
        writer.writeheader()
        writer.writerows(rows)
    with (out_dir / "conditions.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["condition", "n", "catastrophic", "integrity_failures",
                         "rmse_mean_m", "rmse_median_m", "rmse_mean_ci95_lo",
                         "rmse_mean_ci95_hi", "rpe_1s_mean_m", "motion_ratio_median",
                         "orientation_rmse_mean_deg", "tilt_axis_rmse_mean_deg",
                         "gt_tilt_p90_mean_deg"])
        for condition in conditions:
            item = summaries[condition]
            writer.writerow([
                condition, item["n"], item["catastrophic_count"],
                item["integrity_failure_count"], item["rmse_m"]["mean"],
                item["rmse_m"]["median"], *item["rmse_m"]["mean_ci95"],
                item["rpe_1s_rmse_m"]["mean"], item["motion_ratio"]["median"],
                item["orientation_rmse_deg"]["mean"],
                item["tilt_axis_rmse_deg"]["mean"],
                item["gt_tilt_p90_deg"]["mean"],
            ])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    for axis, metric, ylabel in (
            (axes[0], "rmse_m", "No-alignment RMSE [m]"),
            (axes[1], "rmse_per_gt_path", "RMSE / GT path length")):
        data = [[float(x[metric]) for x in grouped[c]] for c in conditions]
        axis.boxplot(data, labels=conditions, showmeans=True)
        for i, values in enumerate(data, 1):
            jitter = np.linspace(-0.08, 0.08, len(values)) if len(values) > 1 else [0.0]
            axis.scatter(i + np.asarray(jitter), values, s=22, alpha=0.75)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.3)
        axis.tick_params(axis="x", rotation=20)
    fig.suptitle("FAST-LIVO frozen production config — 21 real flights")
    fig.tight_layout()
    fig.savefig(out_dir / "condition_summary.png", dpi=180)
    plt.close(fig)
    if fusion_dir is not None:
        write_fusion_summary(rows, fusion_dir, out_dir)
    print(f"[summarize] wrote {out_dir}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument(
        "--spec", type=Path,
        help="external JSON session catalog; omitted preserves the 2026-08-03 campaign")
    sub = parser.add_subparsers(dest="command", required=True)

    prep = sub.add_parser("prepare")
    prep.add_argument("--group", choices=("dev", "validation", "all"), default="all")
    prep.add_argument("--overwrite", action="store_true")

    inputs = sub.add_parser("inspect-inputs")
    inputs.add_argument("--out-dir", type=Path, required=True)
    inputs.add_argument("--stride", type=int, default=10,
                        help="sample every Nth RGB/cloud frame (default: 10)")

    run = sub.add_parser("run")
    run.add_argument("--group", choices=("dev", "validation", "all"), default="dev")
    run.add_argument("--tag", required=True)
    run.add_argument("--config", type=Path)
    run.add_argument("--overlay", type=Path)
    run.add_argument("--rate", type=float, default=1.0)
    run.add_argument("--repeat", type=int, default=1)
    run.add_argument("--force", action="store_true")
    run.add_argument(
        "--ids", nargs="+",
        help="targeted rerun subset (development tuning or promoted worst-case repeats)")

    ev = sub.add_parser("evaluate")
    ev.add_argument("bags", type=Path, nargs="+")
    ev.add_argument("--out", type=Path)

    summary = sub.add_parser("summarize")
    summary.add_argument("results", type=Path, nargs="+")
    summary.add_argument("--out-dir", type=Path, required=True)
    summary.add_argument("--bootstrap", type=int, default=20000)
    summary.add_argument("--seed", type=int, default=20260805)
    summary.add_argument("--fusion-dir", type=Path)

    args = parser.parse_args()
    root = args.root.resolve()
    spec = load_spec(args.spec.resolve() if args.spec else None)
    if args.command == "prepare":
        prepare(root, spec, _ids_for_group(args.group, spec), args.overwrite)
    elif args.command == "inspect-inputs":
        inspect_inputs(spec, args.out_dir, args.stride)
    elif args.command == "run":
        run_campaign(root, spec, args.group, args.tag,
                     args.config.resolve() if args.config else None,
                     args.overlay.resolve() if args.overlay else None,
                     args.rate, args.repeat, args.force, args.ids)
    elif args.command == "evaluate":
        evaluate_paths(root, spec, args.bags, args.out)
    elif args.command == "summarize":
        summarize_results(spec, args.results, args.out_dir,
                          args.bootstrap, args.seed,
                          args.fusion_dir.resolve() if args.fusion_dir else None)


if __name__ == "__main__":
    main()

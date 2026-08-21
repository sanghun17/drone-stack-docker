#!/usr/bin/env python3
"""Render time-aligned D435 RGB video with FAST-LIVO active measurements.

Only RGB frames that FAST-LIVO actually processed are used.  Each rendered
frame therefore has an exact set of image-plane measurement coordinates; no
point set is carried onto a different camera frame.  Frames are repeated onto
a constant-rate output timeline for broad MP4/PowerPoint compatibility.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Dict, Iterator, List, Optional, Tuple

import cv2
import numpy as np
import pandas as pd

from analyze_visual_quality import (
    decode_ros_image,
    extract_nearest_bag_images,
    ros_stamp_seconds,
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--recorded-condition", required=True)
    parser.add_argument("--frames-csv", type=Path, required=True)
    parser.add_argument("--points-csv", type=Path, required=True)
    parser.add_argument("--raw-bag", type=Path, required=True)
    parser.add_argument("--rgb-topic", default="/camera/color/image_raw/compressed")
    parser.add_argument(
        "--full-timeline-bag",
        type=Path,
        help="optional bag providing the complete RGB timeline; frames outside diagnostics are unmarked",
    )
    parser.add_argument("--full-timeline-topic", default="/camera/color/image_raw_10hz")
    parser.add_argument("--raw-rgb-first-stamp", type=float, required=True)
    parser.add_argument("--qualitative-manifest", type=Path, required=True)
    parser.add_argument("--metric", default="final_rmse")
    parser.add_argument(
        "--point-style",
        choices=("final_rmse", "solid_green"),
        default="final_rmse",
    )
    parser.add_argument("--no-hud", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--output-fps", type=float, default=30.0)
    parser.add_argument("--max-rgb-diff", type=float, default=0.001)
    parser.add_argument("--crf", type=int, default=18)
    parser.add_argument("--preset", default="veryfast")
    return parser.parse_args()


def require_columns(table: pd.DataFrame, names: Tuple[str, ...], path: Path) -> None:
    missing = [name for name in names if name not in table.columns]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")


def metric_lut() -> np.ndarray:
    values = np.arange(256, dtype=np.uint8).reshape(256, 1)
    return cv2.applyColorMap(values, cv2.COLORMAP_MAGMA).reshape(256, 3)


def translucent_panel(image: np.ndarray, pt1: Tuple[int, int], pt2: Tuple[int, int]) -> None:
    overlay = image.copy()
    cv2.rectangle(overlay, pt1, pt2, (0, 0, 0), thickness=-1)
    cv2.addWeighted(overlay, 0.62, image, 0.38, 0.0, dst=image)


def render_frame(
    rgb: np.ndarray,
    points: pd.DataFrame,
    active_count: Optional[int],
    label: str,
    raw_time_s: float,
    metric: str,
    scale: Tuple[float, float],
    lut: np.ndarray,
    point_style: str,
    show_hud: bool,
) -> np.ndarray:
    image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    height, width = image.shape[:2]
    low, high = scale

    valid = np.isfinite(points["u"].to_numpy(float)) & np.isfinite(
        points["v"].to_numpy(float)
    )
    if point_style == "final_rmse":
        valid &= np.isfinite(points[metric].to_numpy(float))
    valid_points = points.loc[valid]
    if active_count is not None and len(valid_points) != active_count:
        raise ValueError(
            f"active_count={active_count}, but {len(valid_points)} finite point rows were found"
        )

    span = max(high - low, np.finfo(float).eps)
    for row in valid_points.itertuples(index=False):
        u = int(round(float(row.u)))
        v = int(round(float(row.v)))
        if not (0 <= u < width and 0 <= v < height):
            raise ValueError(f"point ({u}, {v}) is outside {width}x{height}")
        if point_style == "solid_green":
            # Exact interior color sampled from the requested reference image:
            # RGB (98, 255, 0), represented here in OpenCV BGR order.
            color = (0, 255, 98)
        else:
            value = float(getattr(row, metric))
            color_index = int(np.clip(round(255.0 * (value - low) / span), 0, 255))
            color = tuple(int(channel) for channel in lut[color_index])
        cv2.circle(image, (u, v), 5, (0, 0, 0), thickness=-1, lineType=cv2.LINE_AA)
        cv2.circle(image, (u, v), 4, color, thickness=-1, lineType=cv2.LINE_AA)

    if show_hud:
        translucent_panel(image, (8, 8), (373, 58))
        active_text = "N/A" if active_count is None else str(active_count)
        cv2.putText(
            image,
            f"{label} | active visual measurements: {active_text}",
            (16, 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"D435 RGB timeline: {raw_time_s:6.2f} s",
            (16, 49),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (225, 225, 225),
            1,
            cv2.LINE_AA,
        )

    if point_style == "final_rmse":
        bar_x0, bar_x1 = width - 178, width - 14
        bar_y0, bar_y1 = height - 23, height - 12
        translucent_panel(image, (bar_x0 - 7, bar_y0 - 25), (bar_x1 + 7, bar_y1 + 8))
        color_indices = np.linspace(0, 255, bar_x1 - bar_x0).round().astype(np.uint8)
        gradient = np.repeat(lut[color_indices][None, :, :], bar_y1 - bar_y0, axis=0)
        image[bar_y0:bar_y1, bar_x0:bar_x1] = gradient
        cv2.rectangle(image, (bar_x0, bar_y0), (bar_x1, bar_y1), (230, 230, 230), 1)
        cv2.putText(
            image,
            "Final patch RMSE (low -> high)",
            (bar_x0, bar_y0 - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (245, 245, 245),
            1,
            cv2.LINE_AA,
        )
    return image


def bag_rgb_stamps(bag_path: Path, topic: str) -> List[float]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as error:
        raise RuntimeError("reading RGB from a bag requires the 'rosbags' package") from error
    stamps: List[float] = []
    with AnyReader([bag_path]) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            raise ValueError(f"{bag_path}: RGB topic {topic!r} absent")
        for connection, bag_timestamp_ns, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            stamps.append(ros_stamp_seconds(message, bag_timestamp_ns))
    if len(stamps) < 2 or np.any(np.diff(np.asarray(stamps)) <= 0):
        raise ValueError(f"{bag_path}: RGB timestamps are not strictly monotonic")
    return stamps


def iter_bag_rgb(bag_path: Path, topic: str) -> Iterator[Tuple[float, np.ndarray]]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as error:
        raise RuntimeError("reading RGB from a bag requires the 'rosbags' package") from error
    with AnyReader([bag_path]) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            raise ValueError(f"{bag_path}: RGB topic {topic!r} absent")
        for connection, bag_timestamp_ns, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            yield (
                ros_stamp_seconds(message, bag_timestamp_ns),
                decode_ros_image(message, connection.msgtype),
            )


def match_diagnostic_to_timeline(
    diagnostic_times: np.ndarray,
    timeline_times: np.ndarray,
    max_diff_s: float,
) -> Tuple[Dict[int, int], float]:
    mapping: Dict[int, int] = {}
    differences: List[float] = []
    for diagnostic_ordinal, target in enumerate(diagnostic_times):
        right = int(np.searchsorted(timeline_times, target))
        candidates = [index for index in (right - 1, right) if 0 <= index < len(timeline_times)]
        timeline_ordinal = min(candidates, key=lambda index: abs(float(timeline_times[index]) - target))
        difference = abs(float(timeline_times[timeline_ordinal]) - float(target))
        if difference > max_diff_s:
            raise ValueError(
                f"diagnostic time {target:.9f}: nearest full-timeline RGB differs by {difference:.9f}s"
            )
        if timeline_ordinal in mapping:
            raise ValueError(f"multiple diagnostic frames map to timeline frame {timeline_ordinal}")
        mapping[timeline_ordinal] = diagnostic_ordinal
        differences.append(difference)
    if len(mapping) != len(diagnostic_times):
        raise ValueError("not every diagnostic frame mapped uniquely to the full RGB timeline")
    return mapping, max(differences, default=0.0)


def output_repetitions(times: np.ndarray, fps: float) -> Tuple[np.ndarray, int]:
    if len(times) < 2:
        raise ValueError("at least two instrumented frames are required")
    relative = times - times[0]
    starts = np.rint(relative * fps).astype(np.int64)
    if np.any(np.diff(starts) < 1):
        raise ValueError("output FPS is too low to preserve every measurement frame")
    median_dt = float(np.median(np.diff(times)))
    final_count = int(round((float(relative[-1]) + median_dt) * fps))
    final_count = max(final_count, int(starts[-1]) + 1)
    stops = np.concatenate([starts[1:], np.asarray([final_count], dtype=np.int64)])
    repetitions = stops - starts
    if np.any(repetitions < 1):
        raise ValueError("invalid output repetition schedule")
    return repetitions, final_count


def ffprobe(path: Path) -> Dict[str, object]:
    command = [
        "ffprobe", "-v", "error", "-select_streams", "v:0",
        "-show_entries", "stream=codec_name,width,height,avg_frame_rate,nb_frames,duration",
        "-of", "json", str(path),
    ]
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    streams = json.loads(result.stdout).get("streams", [])
    if len(streams) != 1:
        raise ValueError(f"{path}: expected one video stream, got {len(streams)}")
    return streams[0]


def main() -> int:
    args = parse_args()
    for source in (args.frames_csv, args.points_csv, args.raw_bag, args.qualitative_manifest):
        if not source.is_file():
            raise FileNotFoundError(source)
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {args.output}")
    provenance_path = args.output.with_suffix(args.output.suffix + ".provenance.json")
    if provenance_path.exists():
        raise FileExistsError(f"refusing to overwrite existing provenance: {provenance_path}")

    frames = pd.read_csv(args.frames_csv)
    points = pd.read_csv(args.points_csv)
    require_columns(
        frames,
        ("frame_index", "img_time_s", "width", "height", "active_count"),
        args.frames_csv,
    )
    require_columns(points, ("frame_index", "u", "v", args.metric), args.points_csv)
    if frames.empty:
        raise ValueError(f"{args.frames_csv}: empty frame table")
    if frames["frame_index"].duplicated().any() or not frames["img_time_s"].is_monotonic_increasing:
        raise ValueError(f"{args.frames_csv}: frame IDs/timestamps are not unique and monotonic")

    expected_point_rows = int(frames["active_count"].sum())
    if len(points) != expected_point_rows:
        raise ValueError(f"point rows {len(points)} != sum(active_count) {expected_point_rows}")
    point_groups = {int(index): group for index, group in points.groupby("frame_index", sort=False)}

    manifest = json.loads(args.qualitative_manifest.read_text())
    common_scale = manifest["common_scales"][args.metric]
    scale = (float(common_scale["vmin"]), float(common_scale["vmax"]))

    times = frames["img_time_s"].to_numpy(float)
    width_values = frames["width"].astype(int).unique()
    height_values = frames["height"].astype(int).unique()
    if len(width_values) != 1 or len(height_values) != 1:
        raise ValueError("instrumented frame dimensions are not constant")
    width, height = int(width_values[0]), int(height_values[0])
    frame_rows = list(frames.itertuples(index=False))

    full_timeline = args.full_timeline_bag is not None
    timeline_stamps: Optional[np.ndarray] = None
    timeline_mapping: Dict[int, int] = {}
    images: Dict[int, Tuple[np.ndarray, float, float]] = {}
    repetitions: Optional[np.ndarray] = None
    if full_timeline:
        assert args.full_timeline_bag is not None
        if not args.full_timeline_bag.is_file():
            raise FileNotFoundError(args.full_timeline_bag)
        timeline_stamps = np.asarray(
            bag_rgb_stamps(args.full_timeline_bag, args.full_timeline_topic), dtype=float
        )
        timeline_mapping, max_rgb_diff = match_diagnostic_to_timeline(
            times, timeline_stamps, args.max_rgb_diff
        )
        expected_output_frames = len(timeline_stamps)
        encode_fps = float((len(timeline_stamps) - 1) / (timeline_stamps[-1] - timeline_stamps[0]))
        if abs(float(timeline_stamps[0]) - args.raw_rgb_first_stamp) > args.max_rgb_diff:
            raise ValueError("full RGB timeline does not start at --raw-rgb-first-stamp")
        scope = "complete_D435_RGB_10Hz_timeline_with_instrumented_interval_overlay"
        timing_policy = (
            "The full 10 Hz D435 RGB timeline is preserved. Active measurements are drawn only "
            "when that exact RGB epoch exists in the diagnostic CSV; no measurement is held or "
            "interpolated onto another camera frame. Outside the diagnostic interval the overlay "
            "is explicitly labeled 'not logged'."
        )
    else:
        targets = [
            (int(row.frame_index), float(row.img_time_s))
            for row in frame_rows
        ]
        images = extract_nearest_bag_images(args.raw_bag, args.rgb_topic, targets, args.max_rgb_diff)
        max_rgb_diff = max(float(images[index][2]) for index, _ in targets)
        repetitions, expected_output_frames = output_repetitions(times, args.output_fps)
        encode_fps = float(args.output_fps)
        scope = "instrumented_FAST_LIVO_frame_interval_only"
        timing_policy = (
            "Each source frame is the exact D435 RGB epoch processed by FAST-LIVO; "
            "the same rendered frame is repeated on a constant-rate MP4 timeline. "
            "Measurements are never carried onto a different RGB frame."
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    partial = args.output.with_name(
        f".{args.output.stem}.partial-{os.getpid()}{args.output.suffix}"
    )
    if partial.exists():
        raise FileExistsError(partial)
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "rawvideo",
        "-pix_fmt", "bgr24", "-s:v", f"{width}x{height}", "-r", f"{encode_fps:.12g}",
        "-i", "-", "-an", "-c:v", "libx264", "-preset", args.preset,
        "-crf", str(args.crf), "-pix_fmt", "yuv420p", "-movflags", "+faststart",
        str(partial),
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    lut = metric_lut()
    try:
        if process.stdin is None or process.stderr is None:
            raise RuntimeError("failed to open ffmpeg pipes")
        if full_timeline:
            assert args.full_timeline_bag is not None
            assert timeline_stamps is not None
            source_frames = iter_bag_rgb(args.full_timeline_bag, args.full_timeline_topic)
            written = 0
            for timeline_ordinal, (rgb_stamp, rgb) in enumerate(source_frames):
                if abs(float(timeline_stamps[timeline_ordinal]) - rgb_stamp) > 1e-6:
                    raise ValueError(f"timeline changed between scan and decode at {timeline_ordinal}")
                diagnostic_ordinal = timeline_mapping.get(timeline_ordinal)
                if diagnostic_ordinal is None:
                    frame_points = points.iloc[0:0]
                    active_count: Optional[int] = None
                else:
                    row = frame_rows[diagnostic_ordinal]
                    frame_index = int(row.frame_index)
                    frame_points = point_groups.get(frame_index, points.iloc[0:0])
                    active_count = int(row.active_count)
                if rgb.shape[:2] != (height, width):
                    raise ValueError(f"RGB shape {rgb.shape} != {(height, width, 3)}")
                rendered = render_frame(
                    rgb, frame_points, active_count, args.label,
                    rgb_stamp - args.raw_rgb_first_stamp, args.metric, scale, lut,
                    args.point_style, not args.no_hud,
                )
                process.stdin.write(rendered.tobytes())
                written += 1
            if written != expected_output_frames:
                raise ValueError(f"decoded timeline frames {written} != expected {expected_output_frames}")
        else:
            assert repetitions is not None
            for ordinal, row in enumerate(frame_rows):
                frame_index = int(row.frame_index)
                rgb = images[frame_index][0]
                if rgb.shape[:2] != (height, width):
                    raise ValueError(
                        f"frame {frame_index}: RGB shape {rgb.shape} != {(height, width, 3)}"
                    )
                frame_points = point_groups.get(frame_index, points.iloc[0:0])
                rendered = render_frame(
                    rgb, frame_points, int(row.active_count), args.label,
                    float(row.img_time_s) - args.raw_rgb_first_stamp,
                    args.metric, scale, lut, args.point_style, not args.no_hud,
                )
                for _ in range(int(repetitions[ordinal])):
                    process.stdin.write(rendered.tobytes())
        process.stdin.close()
        stderr = process.stderr.read().decode("utf-8", errors="replace")
        returncode = process.wait()
        if returncode != 0:
            raise RuntimeError(f"ffmpeg failed with code {returncode}: {stderr}")
        if not partial.is_file() or partial.stat().st_size == 0:
            raise RuntimeError("ffmpeg produced no output")
        stream = ffprobe(partial)
        if int(stream["width"]) != width or int(stream["height"]) != height:
            raise ValueError(f"encoded dimensions do not match: {stream}")
        if int(stream["nb_frames"]) != expected_output_frames:
            raise ValueError(
                f"encoded frame count {stream['nb_frames']} != expected {expected_output_frames}"
            )
        partial.rename(args.output)
    except Exception:
        if process.poll() is None:
            process.kill()
            process.wait()
        if partial.exists():
            partial.unlink()
        raise

    provenance = {
        "schema": "fastlivo_active_measurement_video/v1",
        "session_id": args.session_id,
        "paper_label": args.label,
        "recorded_condition": args.recorded_condition,
        "scope": scope,
        "timing_policy": timing_policy,
        "point_style": args.point_style,
        "hud_enabled": not args.no_hud,
        "solid_point_color_rgb": ([98, 255, 0] if args.point_style == "solid_green" else None),
        "metric": (args.metric if args.point_style == "final_rmse" else None),
        "metric_semantics": (
            "Photometric patch RMSE at the finalized estimator state; lower is better."
            if args.point_style == "final_rmse"
            else None
        ),
        "common_color_scale": (
            {
                "vmin": scale[0],
                "vmax": scale[1],
                "source": common_scale.get("source"),
                "percentiles": common_scale.get("percentiles"),
            }
            if args.point_style == "final_rmse"
            else None
        ),
        "instrumented_frames": len(frames),
        "active_point_rows": len(points),
        "active_count": {
            "min": int(frames["active_count"].min()),
            "mean": float(frames["active_count"].mean()),
            "p10": float(frames["active_count"].quantile(0.10)),
            "p90": float(frames["active_count"].quantile(0.90)),
            "max": int(frames["active_count"].max()),
        },
        "source_time": {
            "first_img_epoch_s": float(times[0]),
            "last_img_epoch_s": float(times[-1]),
            "first_raw_rgb_relative_s": float(times[0] - args.raw_rgb_first_stamp),
            "last_raw_rgb_relative_s": float(times[-1] - args.raw_rgb_first_stamp),
            "maximum_rgb_match_abs_diff_s": max_rgb_diff,
            "full_timeline_first_epoch_s": (
                float(timeline_stamps[0]) if timeline_stamps is not None else None
            ),
            "full_timeline_last_epoch_s": (
                float(timeline_stamps[-1]) if timeline_stamps is not None else None
            ),
            "full_timeline_frames": (
                int(len(timeline_stamps)) if timeline_stamps is not None else None
            ),
        },
        "output": {
            "path": str(args.output),
            "fps": encode_fps,
            "frames": expected_output_frames,
            "duration_s": expected_output_frames / encode_fps,
            "ffprobe": ffprobe(args.output),
            "size_bytes": args.output.stat().st_size,
            "sha256": sha256(args.output),
        },
        "sources": {
            "frames_csv": {"path": str(args.frames_csv), "sha256": sha256(args.frames_csv)},
            "points_csv": {"path": str(args.points_csv), "sha256": sha256(args.points_csv)},
            "raw_bag": {"path": str(args.raw_bag)},
            "rgb_topic": args.rgb_topic,
            "full_timeline_bag": (
                {"path": str(args.full_timeline_bag)} if args.full_timeline_bag else None
            ),
            "full_timeline_topic": args.full_timeline_topic if full_timeline else None,
            "qualitative_manifest": {
                "path": str(args.qualitative_manifest),
                "sha256": sha256(args.qualitative_manifest),
            },
        },
    }
    with provenance_path.open("x", encoding="utf-8") as stream:
        json.dump(provenance, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps({"video": str(args.output), "provenance": str(provenance_path), **provenance["output"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())

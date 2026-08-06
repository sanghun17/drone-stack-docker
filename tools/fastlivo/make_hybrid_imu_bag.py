#!/usr/bin/env python3
"""Build a non-destructive FAST-LIVO IMU diagnostic stream.

The D435 combined IMU stream supplies the timestamps and angular velocity.
Linear acceleration is linearly interpolated from MAVROS and rotated from the
vehicle base/body frame into ``camera_depth_optical_frame``.  The default
output is a small *sidecar* bag containing only ``/camera/imu_hybrid`` and a
provenance message.  Pass ``--copy-all`` only when a self-contained derivative
of the complete source bag is explicitly useful.

The source bag is always opened read-only and is never rewritten.  A JSON
manifest is written next to a generated bag so that an offline result cannot
silently lose its source/topic/transform provenance.

Example::

    python3 tools/fastlivo/make_hybrid_imu_bag.py flight.bag \
      --output flight.hybrid_imu.bag

    # A canonical sensor bag may omit MAVROS; read acceleration from the
    # original bag while copying/timestamping the canonical stream.
    python3 tools/fastlivo/make_hybrid_imu_bag.py canonical.bag \
      --mavros-source original.bag --copy-all \
      --output canonical.with_hybrid.bag

    # Inspect interpolation coverage without writing anything.
    python3 tools/fastlivo/make_hybrid_imu_bag.py flight.bag --dry-run
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from typing import Dict, Optional, Sequence, Tuple

import numpy as np
import rosbag
from std_msgs.msg import String


TOOL_VERSION = "1"
DEFAULT_D435_TOPIC = "/camera/imu"
DEFAULT_MAVROS_TOPIC = "/mavros/imu/data_raw"
DEFAULT_OUTPUT_TOPIC = "/camera/imu_hybrid"
DEFAULT_FRAME = "camera_depth_optical_frame"

# Supplied/calibrated convention: R_base<-camera_depth_optical.  With row
# vectors, a_camera = a_base @ R_BASE_FROM_CAMERA.
R_BASE_FROM_CAMERA = np.asarray([
    [-0.019284, 0.007524, 0.999786],
    [-0.999743, -0.012073, -0.019193],
    [0.011926, -0.999899, 0.007755],
], dtype=np.float64)


def stamp_sec(msg, bag_time) -> float:
    """Use a valid message stamp, otherwise fall back to the bag timestamp."""
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None:
        value = float(stamp.to_sec())
        if value > 0.0:
            return value
    return float(bag_time.to_sec())


def rotate_base_to_camera(vector: Sequence[float]) -> np.ndarray:
    """Rotate one base/body-frame row vector into camera optical axes."""
    return np.asarray(vector, dtype=np.float64) @ R_BASE_FROM_CAMERA


def rotate_covariance_base_to_camera(covariance: Sequence[float]) -> np.ndarray:
    """Rotate a 3x3 covariance using the same row-vector convention."""
    cov_base = np.asarray(covariance, dtype=np.float64).reshape(3, 3)
    return R_BASE_FROM_CAMERA.T @ cov_base @ R_BASE_FROM_CAMERA


def interpolate_sample(
        query_time: float,
        times: np.ndarray,
        values: np.ndarray,
        covariances: np.ndarray,
        max_bracket_gap: float,
        edge_policy: str = "drop",
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str, float]:
    """Interpolate one sample.

    Returns ``(value, covariance, status, support_gap_s)``.  ``value`` is
    ``None`` when the query is intentionally rejected.  The covariance is
    linearly interpolated before being rotated by the caller.
    """
    if times.size == 0:
        return None, None, "no_mavros", float("nan")

    right = int(np.searchsorted(times, query_time, side="left"))
    if right < times.size and times[right] == query_time:
        return values[right].copy(), covariances[right].copy(), "exact", 0.0

    if right == 0 or right == times.size:
        if edge_policy == "drop":
            return None, None, "outside", float("nan")
        index = 0 if right == 0 else times.size - 1
        distance = abs(float(query_time - times[index]))
        if distance > max_bracket_gap:
            return None, None, "edge_too_far", distance
        return (values[index].copy(), covariances[index].copy(),
                "nearest_edge", distance)

    left = right - 1
    gap = float(times[right] - times[left])
    if gap <= 0.0:
        return None, None, "nonpositive_gap", gap
    if gap > max_bracket_gap:
        return None, None, "gap_too_large", gap
    weight = float((query_time - times[left]) / gap)
    value = values[left] + weight * (values[right] - values[left])
    covariance = (covariances[left]
                  + weight * (covariances[right] - covariances[left]))
    return value, covariance, "interpolated", gap


def _describe(values: Sequence[float]) -> Dict[str, Optional[float]]:
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return {key: None for key in
                ("min", "p05", "median", "mean", "std", "p95", "max")}
    return {
        "min": float(np.min(array)),
        "p05": float(np.percentile(array, 5)),
        "median": float(np.median(array)),
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_revision() -> Optional[str]:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).resolve().parents[2], text=True,
            stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _load_mavros(
        source: Path, topic: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, int]]:
    rows = []
    with rosbag.Bag(str(source), "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            rows.append((
                stamp_sec(msg, bag_time),
                [msg.linear_acceleration.x,
                 msg.linear_acceleration.y,
                 msg.linear_acceleration.z],
                list(msg.linear_acceleration_covariance),
            ))
    if not rows:
        raise RuntimeError(f"{source}: no messages on {topic}")

    rows.sort(key=lambda row: row[0])
    # Duplicated timestamps cannot bracket an interpolation.  Keep the last
    # message at each timestamp and report the count in provenance.
    deduplicated = []
    for row in rows:
        if deduplicated and row[0] == deduplicated[-1][0]:
            deduplicated[-1] = row
        else:
            deduplicated.append(row)
    times = np.asarray([row[0] for row in deduplicated], dtype=np.float64)
    acceleration = np.asarray([row[1] for row in deduplicated], dtype=np.float64)
    covariance = np.asarray([row[2] for row in deduplicated], dtype=np.float64)
    return times, acceleration, covariance, {
        "messages": len(rows),
        "unique_timestamps": len(deduplicated),
        "duplicate_timestamps": len(rows) - len(deduplicated),
    }


def _file_identity(path: Path) -> Dict[str, object]:
    stat = path.stat()
    return {
        "path": str(path),
        "size_bytes": stat.st_size,
        "mtime_ns": stat.st_mtime_ns,
    }


def _base_provenance(
        args, source: Path, mavros_source: Path,
) -> Dict[str, object]:
    document: Dict[str, object] = {
        "schema": "fastlivo_hybrid_imu_sidecar/v1",
        "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "tool": str(Path(__file__).resolve()),
        "tool_version": TOOL_VERSION,
        "git_revision": _git_revision(),
        "source": _file_identity(source),
        "mavros_source": _file_identity(mavros_source),
        "topics": {
            "d435_timestamp_and_gyro": args.d435_topic,
            "mavros_acceleration": args.mavros_topic,
            "output": args.output_topic,
        },
        "output_frame": args.output_frame,
        "interpolation": {
            "method": "linear",
            "edge_policy": args.edge_policy,
            "max_bracket_gap_s": args.max_bracket_gap,
            "timestamp_basis": "header.stamp; bag timestamp only if header is zero",
        },
        "transform": {
            "name": "R_base_from_camera_depth_optical",
            "matrix_row_major": R_BASE_FROM_CAMERA.tolist(),
            "row_vector_equation": "accel_camera = accel_base @ R_base_from_camera",
            "covariance_equation": "C_camera = R.T @ C_base @ R",
        },
        "semantics": {
            "header_and_angular_velocity": "copied exactly from D435 IMU",
            "linear_acceleration": "MAVROS data_raw, time-interpolated then rotated",
            "orientation": "copied from D435 IMU (FAST-LIVO does not consume it)",
            "source_bag_modified": False,
        },
        "copy_all_source_topics": bool(args.copy_all),
    }
    if args.hash_source:
        document["source"]["sha256"] = _sha256(source)  # type: ignore[index]
    return document


def generate(args) -> Dict[str, object]:
    source = Path(args.source).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    mavros_source = Path(args.mavros_source or source).expanduser().resolve()
    if not mavros_source.is_file():
        raise FileNotFoundError(mavros_source)

    output = None if args.dry_run else Path(args.output).expanduser().resolve()
    if output is not None and output == source:
        raise ValueError("output must differ from the read-only source bag")
    if output is not None and output.exists() and not args.overwrite:
        raise FileExistsError(f"{output} exists (pass --overwrite to replace it)")

    times, accel_base, accel_cov_base, mavros_counts = _load_mavros(
        mavros_source, args.mavros_topic)
    provenance = _base_provenance(args, source, mavros_source)
    provenance["mavros"] = {
        **mavros_counts,
        "first_stamp": float(times[0]),
        "last_stamp": float(times[-1]),
        "dt_s": _describe(np.diff(times)),
        "acceleration_norm_mps2": _describe(np.linalg.norm(accel_base, axis=1)),
    }

    status_counts: Dict[str, int] = {}
    camera_count = 0
    hybrid_count = 0
    camera_accel_norm = []
    hybrid_accel_norm = []
    support_gaps = []
    first_output_bag_time = None
    tmp = None
    output_bag = None

    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        tmp = output.with_suffix(output.suffix + ".active")
        if tmp.exists():
            tmp.unlink()
        output_bag = rosbag.Bag(
            str(tmp), "w", compression=rosbag.Compression.LZ4)

    try:
        with rosbag.Bag(str(source), "r") as source_bag:
            existing = source_bag.get_type_and_topic_info().topics
            if args.output_topic in existing:
                raise RuntimeError(
                    f"source already contains {args.output_topic}; choose a new --output-topic")

            selected_topics = None if args.copy_all else [args.d435_topic]
            for topic, msg, bag_time in source_bag.read_messages(
                    topics=selected_topics):
                if output_bag is not None and args.copy_all:
                    output_bag.write(topic, msg, bag_time)
                if topic != args.d435_topic:
                    continue

                camera_count += 1
                camera_accel_norm.append(float(np.linalg.norm([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ])))
                query = stamp_sec(msg, bag_time)
                accel, covariance, status, gap = interpolate_sample(
                    query, times, accel_base, accel_cov_base,
                    args.max_bracket_gap, args.edge_policy)
                status_counts[status] = status_counts.get(status, 0) + 1
                if accel is None:
                    continue

                accel_cam = rotate_base_to_camera(accel)
                covariance_cam = rotate_covariance_base_to_camera(covariance)
                support_gaps.append(gap)
                hybrid_accel_norm.append(float(np.linalg.norm(accel_cam)))
                hybrid_count += 1

                if output_bag is not None:
                    if first_output_bag_time is None:
                        first_output_bag_time = bag_time
                        summary = dict(provenance)
                        summary["note"] = (
                            "Final counts and output hash are in the adjacent "
                            ".provenance.json manifest.")
                        output_bag.write(
                            args.output_topic + "/provenance",
                            String(data=json.dumps(summary, sort_keys=True)),
                            bag_time)
                    hybrid = copy.deepcopy(msg)
                    hybrid.header.frame_id = args.output_frame
                    hybrid.linear_acceleration.x = float(accel_cam[0])
                    hybrid.linear_acceleration.y = float(accel_cam[1])
                    hybrid.linear_acceleration.z = float(accel_cam[2])
                    hybrid.linear_acceleration_covariance = covariance_cam.reshape(-1).tolist()
                    output_bag.write(args.output_topic, hybrid, bag_time)
    finally:
        if output_bag is not None:
            output_bag.close()

    if camera_count == 0:
        if tmp is not None and tmp.exists():
            tmp.unlink()
        raise RuntimeError(f"{source}: no messages on {args.d435_topic}")
    if hybrid_count == 0:
        if tmp is not None and tmp.exists():
            tmp.unlink()
        raise RuntimeError("no hybrid samples survived interpolation policy")

    provenance["result"] = {
        "d435_messages": camera_count,
        "hybrid_messages": hybrid_count,
        "coverage_fraction": hybrid_count / camera_count,
        "dropped_messages": camera_count - hybrid_count,
        "interpolation_status": status_counts,
        "support_gap_s": _describe(support_gaps),
        "d435_acceleration_norm_mps2": _describe(camera_accel_norm),
        "hybrid_acceleration_norm_mps2": _describe(hybrid_accel_norm),
        "angular_velocity_copy": "exact by deepcopy from each D435 message",
    }

    if output is not None:
        assert tmp is not None
        if output.exists():
            # --overwrite is an explicit, narrowly-scoped replacement request.
            output.unlink()
        tmp.replace(output)
        provenance["output"] = {
            "path": str(output),
            "size_bytes": output.stat().st_size,
            "sha256": _sha256(output),
        }
        manifest = Path(str(output) + ".provenance.json")
        manifest.write_text(json.dumps(provenance, indent=2, sort_keys=True) + "\n")
        provenance["output"]["manifest"] = str(manifest)  # type: ignore[index]

    return provenance


def parse_args(argv: Optional[Sequence[str]] = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="read-only source ROS1 bag")
    parser.add_argument(
        "--mavros-source",
        help=("optional second read-only bag containing the MAVROS IMU; useful "
              "when SOURCE is a canonical sensor-only derivative"),
    )
    parser.add_argument("--output", help="derived bag path (required unless --dry-run)")
    parser.add_argument("--dry-run", action="store_true",
                        help="compute coverage/stats without writing a bag")
    parser.add_argument("--copy-all", action="store_true",
                        help="copy every source message into the derivative (large)")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--hash-source", action="store_true",
                        help="also SHA256 the source (slow for multi-GB bags)")
    parser.add_argument("--d435-topic", default=DEFAULT_D435_TOPIC)
    parser.add_argument("--mavros-topic", default=DEFAULT_MAVROS_TOPIC)
    parser.add_argument("--output-topic", default=DEFAULT_OUTPUT_TOPIC)
    parser.add_argument("--output-frame", default=DEFAULT_FRAME)
    parser.add_argument("--max-bracket-gap", type=float, default=0.05,
                        help="maximum MAVROS interpolation bracket in seconds")
    parser.add_argument("--edge-policy", choices=("drop", "nearest"), default="drop")
    args = parser.parse_args(argv)
    if not args.dry_run and not args.output:
        parser.error("--output is required unless --dry-run is used")
    if args.dry_run and args.output:
        parser.error("--dry-run and --output are mutually exclusive")
    if args.copy_all and args.dry_run:
        parser.error("--copy-all has no meaning with --dry-run")
    if args.max_bracket_gap <= 0:
        parser.error("--max-bracket-gap must be positive")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        result = generate(args)
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

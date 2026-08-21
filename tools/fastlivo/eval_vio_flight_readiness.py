#!/usr/bin/env python3
"""Strict, GT-isolated flight-readiness evaluation for FAST-LIVO replays.

This evaluator intentionally does *not* replace ``campaign_20260803.py``.  The
older campaign score answers whether a replay catastrophically diverged after
publishing a GT-anchored diagnostic pose.  This file answers the stricter
question needed before closed-loop flight:

* does the GT-independent local body pose remain accurate and continuous;
* is the IMU-rate control/planning odometry timely, smooth, and internally
  self-consistent; and
* was ground truth kept out of the estimator during the replay?

Only a single, initialization-window yaw+translation transform is fitted.  A
per-flight time shift, scale fit, or whole-trajectory SE(3) fit is never used
for a pass/fail metric.  The optional speed-correlation time offset is emitted
as a diagnostic only.

Typical use::

    python3 tools/fastlivo/eval_vio_flight_readiness.py result.bag \
      --output result.flight_readiness.json

The result bag must contain OptiTrack only as evaluation truth.  If
``/aft_mapped_to_optitrack`` is present, the report conservatively records that
GT was visible to the estimator; current FAST-LIVO code also uses that anchor
inside the propagated output, so such a replay cannot be called flight-ready.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import rosbag
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_fastlivo import estimate_offset, geodesic_deg, q_to_R, slerp  # noqa: E402


SCHEMA = "fastlivo_vio_flight_readiness/v1"
SECONDARY_SCHEMA = "fastlivo_vio_ground_hover_ranking/v1"
DEFAULT_THRESHOLDS = Path(__file__).with_name(
    "vio_flight_readiness_thresholds.yaml")
DEFAULT_GT_TOPIC = "/vrpn_client_node/pure/pose"
DEFAULT_LOCAL_TOPIC = "/aft_mapped_to_body"
DEFAULT_PROP_TOPIC = "/aft_mapped_to_body_imu_propagated"
DEFAULT_CORRECTION_COV_TOPIC = "/aft_mapped_to_body_correction_pose_cov"
GT_ANCHORED_TOPIC = "/aft_mapped_to_optitrack"


@dataclass
class PoseStream:
    topic: str
    header_time: np.ndarray
    header_stamp_ns: np.ndarray
    bag_time: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray  # xyzw
    linear_velocity: Optional[np.ndarray]
    angular_velocity: Optional[np.ndarray]
    pose_covariance_nonzero: Optional[np.ndarray]
    twist_covariance_nonzero: Optional[np.ndarray]
    frame_id: List[str]
    child_frame_id: List[str]

    def sorted_valid(self) -> "PoseStream":
        finite = (
            np.isfinite(self.header_time)
            & np.all(np.isfinite(self.position), axis=1)
            & np.all(np.isfinite(self.quaternion), axis=1)
            & (np.linalg.norm(self.quaternion, axis=1) > 1e-9)
        )
        indices = np.flatnonzero(finite)
        if len(indices):
            # Sensor epoch is preserved as int64 specifically because adjacent
            # nanoseconds collapse when a ROS epoch is represented as float
            # seconds.  Sorting and duplicate removal must use that exact key.
            order = np.argsort(self.header_stamp_ns[indices], kind="stable")
            indices = indices[order]
            # Keep the first publication at a sensor epoch.  Repeated header
            # stamps are separately reported as an integrity defect.
            ordered_stamps = self.header_stamp_ns[indices]
            keep = np.r_[True, ordered_stamps[1:] != ordered_stamps[:-1]]
            indices = indices[keep]

        def optional(values: Optional[np.ndarray]) -> Optional[np.ndarray]:
            return None if values is None else values[indices]

        return PoseStream(
            topic=self.topic,
            header_time=self.header_time[indices],
            header_stamp_ns=self.header_stamp_ns[indices],
            bag_time=self.bag_time[indices],
            position=self.position[indices],
            quaternion=_normalize_quaternions(self.quaternion[indices]),
            linear_velocity=optional(self.linear_velocity),
            angular_velocity=optional(self.angular_velocity),
            pose_covariance_nonzero=optional(self.pose_covariance_nonzero),
            twist_covariance_nonzero=optional(self.twist_covariance_nonzero),
            frame_id=[self.frame_id[index] for index in indices],
            child_frame_id=[self.child_frame_id[index] for index in indices],
        )


@dataclass
class AssociatedTrajectory:
    time: np.ndarray
    estimate_position: np.ndarray
    estimate_rotation: np.ndarray
    gt_position: np.ndarray
    gt_rotation: np.ndarray


def _normalize_quaternions(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if len(values) == 0:
        return values.reshape((-1, 4))
    norms = np.linalg.norm(values, axis=1, keepdims=True)
    return values / np.maximum(norms, 1e-15)


def _message_stamp(message: Any, bag_time: Any) -> float:
    header = getattr(message, "header", None)
    if header is not None and header.stamp.to_sec() > 0.0:
        return float(header.stamp.to_sec())
    return float(bag_time.to_sec())


def _pose_from_message(message: Any) -> Any:
    pose = message.pose
    return pose.pose if hasattr(pose, "pose") else pose


def read_pose_stream(path: Path, topic: str) -> Optional[PoseStream]:
    header_time: List[float] = []
    header_stamp_ns: List[int] = []
    bag_time: List[float] = []
    position: List[List[float]] = []
    quaternion: List[List[float]] = []
    linear_velocity: List[List[float]] = []
    angular_velocity: List[List[float]] = []
    pose_covariance_nonzero: List[bool] = []
    twist_covariance_nonzero: List[bool] = []
    frame_id: List[str] = []
    child_frame_id: List[str] = []
    has_twist = False
    has_pose_covariance = False
    has_twist_covariance = False

    with rosbag.Bag(str(path), "r") as bag:
        if topic not in bag.get_type_and_topic_info().topics:
            return None
        for _, message, record_time in bag.read_messages(topics=[topic]):
            pose = _pose_from_message(message)
            p, q = pose.position, pose.orientation
            sensor_time = _message_stamp(message, record_time)
            header_time.append(sensor_time)
            header = getattr(message, "header", None)
            header_stamp_ns.append(
                int(header.stamp.to_nsec())
                if header is not None and header.stamp.to_nsec() > 0 else
                int(record_time.to_nsec()))
            bag_time.append(float(record_time.to_sec()))
            position.append([p.x, p.y, p.z])
            quaternion.append([q.x, q.y, q.z, q.w])
            frame_id.append(str(getattr(getattr(message, "header", None),
                                        "frame_id", "")))
            child_frame_id.append(str(getattr(message, "child_frame_id", "")))

            twist_container = getattr(message, "twist", None)
            if twist_container is not None:
                twist = (twist_container.twist
                         if hasattr(twist_container, "twist")
                         else twist_container)
                v, w = twist.linear, twist.angular
                linear_velocity.append([v.x, v.y, v.z])
                angular_velocity.append([w.x, w.y, w.z])
                has_twist = True
            else:
                linear_velocity.append([math.nan] * 3)
                angular_velocity.append([math.nan] * 3)

            pose_container = getattr(message, "pose", None)
            covariance = getattr(pose_container, "covariance", None)
            if covariance is not None:
                pose_covariance_nonzero.append(
                    bool(np.any(np.abs(np.asarray(covariance, dtype=float)) > 0.0)))
                has_pose_covariance = True
            else:
                pose_covariance_nonzero.append(False)

            covariance = getattr(twist_container, "covariance", None)
            if covariance is not None:
                twist_covariance_nonzero.append(
                    bool(np.any(np.abs(np.asarray(covariance, dtype=float)) > 0.0)))
                has_twist_covariance = True
            else:
                twist_covariance_nonzero.append(False)

    count = len(header_time)
    cardinalities = {
        "bag_time": len(bag_time),
        "header_stamp_ns": len(header_stamp_ns),
        "position": len(position),
        "quaternion": len(quaternion),
        "linear_velocity": len(linear_velocity),
        "angular_velocity": len(angular_velocity),
        "pose_covariance_nonzero": len(pose_covariance_nonzero),
        "twist_covariance_nonzero": len(twist_covariance_nonzero),
        "frame_id": len(frame_id),
        "child_frame_id": len(child_frame_id),
    }
    mismatched = {
        name: length for name, length in cardinalities.items()
        if length != count
    }
    if mismatched:
        raise RuntimeError(
            f"{path}: malformed pose stream {topic}; expected {count} values "
            f"per field, got {mismatched}")

    return PoseStream(
        topic=topic,
        header_time=np.asarray(header_time, dtype=float),
        header_stamp_ns=np.asarray(header_stamp_ns, dtype=np.int64),
        bag_time=np.asarray(bag_time, dtype=float),
        position=np.asarray(position, dtype=float).reshape((-1, 3)),
        quaternion=np.asarray(quaternion, dtype=float).reshape((-1, 4)),
        linear_velocity=(np.asarray(linear_velocity, dtype=float)
                         if has_twist else None),
        angular_velocity=(np.asarray(angular_velocity, dtype=float)
                          if has_twist else None),
        pose_covariance_nonzero=(np.asarray(pose_covariance_nonzero, dtype=bool)
                                 if has_pose_covariance else None),
        twist_covariance_nonzero=(np.asarray(twist_covariance_nonzero, dtype=bool)
                                  if has_twist_covariance else None),
        frame_id=frame_id,
        child_frame_id=child_frame_id,
    )


def correction_covariance_diagnostics(
        path: Path, topic: str, local: PoseStream,
        maximum_difference_s: float) -> Dict[str, Any]:
    """Audit correction-epoch pose covariance without pretending it is high-rate.

    FAST-LIVO's lightweight IMU-rate path currently propagates only the mean.
    Covariance is therefore published separately at genuine LIO/VIO correction
    epochs.  This function checks that those matrices are finite, symmetric,
    positive-semidefinite and cover the low-rate body-pose corrections.
    """
    stamps: List[float] = []
    bag_times: List[float] = []
    frames: List[str] = []
    matrices: List[np.ndarray] = []
    positions: List[List[float]] = []
    quaternions: List[List[float]] = []
    with rosbag.Bag(str(path), "r") as bag:
        if topic not in bag.get_type_and_topic_info().topics:
            return {"evaluable": False, "failure": "missing_correction_covariance"}
        for _, message, record_time in bag.read_messages(topics=[topic]):
            values = np.asarray(message.pose.covariance, dtype=float)
            if values.size != 36:
                continue
            stamps.append(_message_stamp(message, record_time))
            bag_times.append(float(record_time.to_sec()))
            frames.append(str(message.header.frame_id))
            matrices.append(values.reshape(6, 6))
            pose = message.pose.pose
            positions.append([
                pose.position.x, pose.position.y, pose.position.z])
            quaternions.append([
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w])
    if not matrices:
        return {"evaluable": False, "failure": "empty_correction_covariance"}

    time = np.asarray(stamps, dtype=float)
    record = np.asarray(bag_times, dtype=float)
    covariance = np.asarray(matrices, dtype=float)
    position = np.asarray(positions, dtype=float)
    quaternion = np.asarray(quaternions, dtype=float)
    quaternion_norm = np.linalg.norm(quaternion, axis=1)
    pose_finite = (np.all(np.isfinite(position), axis=1) &
                   np.all(np.isfinite(quaternion), axis=1) &
                   (quaternion_norm > 1e-9))
    finite = np.all(np.isfinite(covariance), axis=(1, 2))
    nonzero = np.any(np.abs(covariance) > 0.0, axis=(1, 2))
    symmetry_error = np.max(
        np.abs(covariance - np.swapaxes(covariance, 1, 2)), axis=(1, 2))
    minimum_eigenvalue = np.full(len(covariance), math.nan)
    for index in np.flatnonzero(finite):
        symmetric = 0.5 * (covariance[index] + covariance[index].T)
        minimum_eigenvalue[index] = float(np.min(np.linalg.eigvalsh(symmetric)))
    psd = finite & (minimum_eigenvalue >= -1e-9)

    order = np.argsort(time, kind="stable")
    sorted_time = time[order]
    local_time = local.sorted_valid().header_time
    nearest_error: List[float] = []
    for stamp in local_time:
        at = int(np.searchsorted(sorted_time, stamp))
        candidates = []
        if at < len(sorted_time):
            candidates.append(abs(float(sorted_time[at] - stamp)))
        if at > 0:
            candidates.append(abs(float(sorted_time[at - 1] - stamp)))
        if candidates:
            nearest_error.append(min(candidates))
    nearest = np.asarray(nearest_error, dtype=float)
    dt = np.diff(time)
    age = record - time
    frame_values = sorted(set(frames))
    return {
        "evaluable": True,
        "topic": topic,
        "message_count": int(len(time)),
        "finite_fraction": float(np.mean(finite)),
        "nonzero_fraction": float(np.mean(nonzero)),
        "psd_fraction": float(np.mean(psd)),
        "symmetry_error_max": float(np.max(symmetry_error)),
        "minimum_eigenvalue_min": _finite(float(np.nanmin(minimum_eigenvalue))),
        "pose_finite_fraction": float(np.mean(pose_finite)),
        "quaternion_norm_error_max": _finite(float(np.max(
            np.abs(quaternion_norm - 1.0)))),
        "frame_ids": frame_values,
        "frame_stable": len(frame_values) == 1,
        "frame_is_odom": frame_values == ["odom"],
        "backward_stamp_count": int(np.count_nonzero(dt < -1e-9)),
        "duplicate_stamp_count": int(np.count_nonzero(np.abs(dt) <= 1e-9)),
        "age_min_s": float(np.min(age)),
        "age_p99_s": _quantile(age, 0.99),
        "age_max_s": float(np.max(age)),
        "local_correction_coverage": (
            float(np.mean(nearest <= maximum_difference_s))
            if len(nearest) else None),
        "local_correction_time_error_p99_s": _quantile(nearest, 0.99),
    }


def _quantile(values: np.ndarray, probability: float) -> Optional[float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return None if len(values) == 0 else float(np.quantile(values, probability))


def _rmse(values: np.ndarray) -> Optional[float]:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return None if len(values) == 0 else float(np.sqrt(np.mean(values * values)))


def _finite(value: Any) -> Any:
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(item) for item in value]
    return value


def _file_identity(path: Path) -> Dict[str, Any]:
    path = path.resolve()
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 << 20), b""):
            digest.update(block)
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": digest.hexdigest(),
    }


def stream_integrity(raw: PoseStream) -> Dict[str, Any]:
    count = len(raw.header_time)
    finite = (
        np.isfinite(raw.header_time)
        & np.all(np.isfinite(raw.position), axis=1)
        & np.all(np.isfinite(raw.quaternion), axis=1)
    )
    quaternion_norm = np.linalg.norm(raw.quaternion, axis=1) if count else np.array([])
    dt = np.diff(raw.header_time)
    age = raw.bag_time - raw.header_time
    frame_values = sorted(set(raw.frame_id))
    child_values = sorted(set(raw.child_frame_id))
    positive_dt = dt[dt > 1e-9]
    return {
        "message_count": int(count),
        "finite": bool(count > 0 and np.all(finite)),
        "nonfinite_count": int(count - np.count_nonzero(finite)),
        "quaternion_norm_error_max": (
            float(np.max(np.abs(quaternion_norm - 1.0))) if count else None),
        "backward_stamp_count": int(np.count_nonzero(dt < -1e-9)),
        "duplicate_stamp_count": int(np.count_nonzero(np.abs(dt) <= 1e-9)),
        "median_gap_s": _quantile(positive_dt, 0.5),
        "p99_gap_s": _quantile(positive_dt, 0.99),
        "max_gap_s": (float(np.max(positive_dt)) if len(positive_dt) else None),
        "rate_hz": (float(1.0 / np.median(positive_dt))
                    if len(positive_dt) and np.median(positive_dt) > 0 else None),
        "age_median_s": _quantile(age, 0.5),
        "age_p95_s": _quantile(age, 0.95),
        "age_p99_s": _quantile(age, 0.99),
        "age_max_s": (float(np.max(age)) if len(age) else None),
        "age_min_s": (float(np.min(age)) if len(age) else None),
        "frame_ids": frame_values,
        "child_frame_ids": child_values,
        "frame_stable": len(frame_values) <= 1 and len(child_values) <= 1,
        "frame_is_odom": bool(count and frame_values == ["odom"]),
        "child_frame_is_base_link": bool(
            count and child_values == ["base_link"]),
    }


def gt_valid_mask(gt: PoseStream, max_speed_mps: float,
                  max_angular_speed_deg_s: float) -> Tuple[np.ndarray, Dict[str, Any]]:
    gt = gt.sorted_valid()
    count = len(gt.header_time)
    valid = np.ones(count, dtype=bool)
    if count < 2:
        return valid, {
            "valid_count": int(count), "excluded_count": 0,
            "excluded_fraction": 0.0, "bad_speed_interval_count": 0,
            "bad_rotation_interval_count": 0,
        }
    dt = np.diff(gt.header_time)
    speed = np.full(len(dt), math.inf)
    good_dt = dt > 1e-6
    speed[good_dt] = np.linalg.norm(np.diff(gt.position, axis=0)[good_dt], axis=1) / dt[good_dt]
    rotations = np.asarray([q_to_R(value) for value in gt.quaternion])
    angle = np.asarray([
        geodesic_deg(rotations[index], rotations[index + 1])
        for index in range(count - 1)
    ])
    angular_speed = np.full(len(dt), math.inf)
    angular_speed[good_dt] = angle[good_dt] / dt[good_dt]
    bad_speed = (~good_dt) | (speed > max_speed_mps)
    bad_rotation = (~good_dt) | (angular_speed > max_angular_speed_deg_s)
    bad = bad_speed | bad_rotation
    indices = np.flatnonzero(bad)
    valid[indices] = False
    valid[np.minimum(indices + 1, count - 1)] = False
    return valid, {
        "valid_count": int(np.count_nonzero(valid)),
        "excluded_count": int(count - np.count_nonzero(valid)),
        "excluded_fraction": float(1.0 - np.mean(valid)) if count else 0.0,
        "bad_speed_interval_count": int(np.count_nonzero(bad_speed)),
        "bad_rotation_interval_count": int(np.count_nonzero(bad_rotation)),
        "speed_p99_mps": _quantile(speed[np.isfinite(speed)], 0.99),
        "angular_speed_p99_deg_s": _quantile(
            angular_speed[np.isfinite(angular_speed)], 0.99),
    }


def associate_fixed(estimate: PoseStream, gt: PoseStream, gt_valid: np.ndarray,
                    max_difference_s: float) -> AssociatedTrajectory:
    estimate = estimate.sorted_valid()
    gt = gt.sorted_valid()
    rows: List[Tuple[int, int, int, float]] = []
    for index, stamp in enumerate(estimate.header_time):
        upper = int(np.searchsorted(gt.header_time, stamp))
        if upper == 0 or upper >= len(gt.header_time):
            continue
        lower = upper - 1
        if not (gt_valid[lower] and gt_valid[upper]):
            continue
        nearest = min(abs(stamp - gt.header_time[lower]),
                      abs(gt.header_time[upper] - stamp))
        if nearest > max_difference_s:
            continue
        span = gt.header_time[upper] - gt.header_time[lower]
        if span <= 1e-9:
            continue
        fraction = float((stamp - gt.header_time[lower]) / span)
        rows.append((index, lower, upper, fraction))
    if not rows:
        empty3 = np.empty((0, 3))
        emptyR = np.empty((0, 3, 3))
        return AssociatedTrajectory(np.array([]), empty3, emptyR, empty3, emptyR)
    return AssociatedTrajectory(
        time=np.asarray([estimate.header_time[index]
                         for index, _, _, _ in rows]),
        estimate_position=np.asarray([estimate.position[index]
                                      for index, _, _, _ in rows]),
        estimate_rotation=np.asarray([q_to_R(estimate.quaternion[index])
                                      for index, _, _, _ in rows]),
        gt_position=np.asarray([
            gt.position[lower] + fraction * (gt.position[upper] - gt.position[lower])
            for _, lower, upper, fraction in rows
        ]),
        gt_rotation=np.asarray([
            q_to_R(slerp(gt.quaternion[lower], gt.quaternion[upper], fraction))
            for _, lower, upper, fraction in rows
        ]),
    )


def _yaw(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _wrap_angle(values: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(values), np.cos(values))


def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """SO(3) logarithm for the small inter-sample rotations used here."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-10:
        return 0.5 * np.asarray([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ])
    sine = math.sin(angle)
    if abs(sine) < 1e-8:
        # A near-pi step already fails the independent high-rate rotation-jump
        # gate; leave this consistency sample unavailable instead of choosing
        # an unstable logarithm axis.
        return np.full(3, math.nan)
    return (angle / (2.0 * sine)) * np.asarray([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])


def initialization_alignment(trajectory: AssociatedTrajectory,
                             duration_s: float,
                             minimum_samples: int) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    if len(trajectory.time) < minimum_samples:
        raise ValueError("too few fixed-time associations for initialization")
    mask = trajectory.time <= trajectory.time[0] + duration_s
    indices = np.flatnonzero(mask)
    if len(indices) < minimum_samples:
        indices = np.arange(min(minimum_samples, len(trajectory.time)))
    yaw_delta = np.asarray([
        _yaw(trajectory.gt_rotation[index]) -
        _yaw(trajectory.estimate_rotation[index])
        for index in indices
    ])
    yaw = math.atan2(float(np.mean(np.sin(yaw_delta))),
                     float(np.mean(np.cos(yaw_delta))))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.asarray([[cosine, -sine, 0.0],
                           [sine, cosine, 0.0],
                           [0.0, 0.0, 1.0]])
    transformed = (rotation @ trajectory.estimate_position[indices].T).T
    translation = np.mean(trajectory.gt_position[indices] - transformed, axis=0)
    residual = transformed + translation - trajectory.gt_position[indices]
    return rotation, translation, {
        "method": "initialization_window_yaw_and_translation_only",
        "scale": 1.0,
        "sensor_start_s": float(trajectory.time[indices[0]]),
        "sensor_end_s": float(trajectory.time[indices[-1]]),
        "duration_s": float(trajectory.time[indices[-1]] - trajectory.time[indices[0]]),
        "sample_count": int(len(indices)),
        "yaw_deg": math.degrees(yaw),
        "translation_m": translation.tolist(),
        "initial_residual_rmse_m": _rmse(np.linalg.norm(residual, axis=1)),
    }


def apply_alignment(trajectory: AssociatedTrajectory, rotation: np.ndarray,
                    translation: np.ndarray) -> AssociatedTrajectory:
    return AssociatedTrajectory(
        time=trajectory.time,
        estimate_position=(rotation @ trajectory.estimate_position.T).T + translation,
        estimate_rotation=np.einsum("ij,njk->nik", rotation,
                                    trajectory.estimate_rotation),
        gt_position=trajectory.gt_position,
        gt_rotation=trajectory.gt_rotation,
    )


def relative_pose_error(trajectory: AssociatedTrajectory,
                        delta_s: float) -> Tuple[np.ndarray, np.ndarray]:
    translation: List[float] = []
    orientation: List[float] = []
    for index, stamp in enumerate(trajectory.time):
        target = stamp + delta_s
        upper = int(np.searchsorted(trajectory.time, target))
        candidates = [candidate for candidate in (upper - 1, upper)
                      if candidate > index and candidate < len(trajectory.time)]
        if not candidates:
            continue
        other = min(candidates,
                    key=lambda candidate: abs(trajectory.time[candidate] - target))
        # Refuse a window whose endpoint misses the requested delta by more
        # than one local-pose period (bounded at 75 ms).
        tolerance = min(0.075, max(0.025, 0.15 * delta_s))
        if abs(trajectory.time[other] - target) > tolerance:
            continue
        est_delta = (trajectory.estimate_position[other] -
                     trajectory.estimate_position[index])
        gt_delta = trajectory.gt_position[other] - trajectory.gt_position[index]
        translation.append(float(np.linalg.norm(est_delta - gt_delta)))
        est_relative = (trajectory.estimate_rotation[index].T @
                        trajectory.estimate_rotation[other])
        gt_relative = (trajectory.gt_rotation[index].T @
                       trajectory.gt_rotation[other])
        orientation.append(geodesic_deg(est_relative, gt_relative))
    return np.asarray(translation), np.asarray(orientation)


def _uniform_positions(time: np.ndarray, position: np.ndarray,
                       hz: float = 10.0) -> Tuple[np.ndarray, np.ndarray]:
    if len(time) < 2 or time[-1] <= time[0]:
        return np.array([]), np.empty((0, 3))
    query = np.arange(time[0], time[-1] + 1e-9, 1.0 / hz)
    values = np.column_stack([
        np.interp(query, time, position[:, axis]) for axis in range(3)
    ])
    return query, values


def _path_length(position: np.ndarray) -> float:
    return (float(np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1)))
            if len(position) > 1 else 0.0)


def _weighted_direction_cosine(time: np.ndarray, estimate: np.ndarray,
                               gt: np.ndarray, delta_s: float = 1.0) -> Optional[float]:
    query, est = _uniform_positions(time, estimate)
    _, truth = _uniform_positions(time, gt)
    step = int(round(10.0 * delta_s))
    if len(query) <= step:
        return None
    est_delta = est[step:] - est[:-step]
    gt_delta = truth[step:] - truth[:-step]
    est_norm = np.linalg.norm(est_delta, axis=1)
    gt_norm = np.linalg.norm(gt_delta, axis=1)
    active = gt_norm >= 0.08
    if not np.any(active):
        return None
    cosine = np.sum(est_delta * gt_delta, axis=1) / (
        est_norm * gt_norm + 1e-12)
    return float(np.sum(gt_norm[active] * cosine[active]) /
                 np.sum(gt_norm[active]))


def _stationary_drift(trajectory: AssociatedTrajectory,
                      window_s: float) -> Dict[str, Any]:
    query, estimate = _uniform_positions(
        trajectory.time, trajectory.estimate_position, hz=10.0)
    _, gt = _uniform_positions(trajectory.time, trajectory.gt_position, hz=10.0)
    if len(query) < int(round(window_s * 10.0)) + 1:
        return {"available": False, "eligible_window_count": 0,
                "window_s": float(window_s),
                "translation_drift_max_m": None, "yaw_drift_max_deg": None}
    # A one-second-smoothed GT velocity prevents mocap jitter from declaring
    # every hover window dynamic.
    smooth_count = 11
    kernel = np.ones(smooth_count) / smooth_count
    smooth_gt = np.column_stack([
        np.convolve(np.pad(gt[:, axis], (5, 5), mode="edge"), kernel,
                    mode="valid")
        for axis in range(3)
    ])
    gt_speed = np.linalg.norm(np.gradient(smooth_gt, 0.1, axis=0), axis=1)
    steps = int(round(window_s * 10.0))
    translation_values: List[float] = []
    yaw_values: List[float] = []
    # Rotations are evaluated at original associated endpoints.
    for start in range(0, len(query) - steps, 5):
        end = start + steps
        if float(np.quantile(gt_speed[start:end + 1], 0.95)) > 0.15:
            continue
        gt_delta = gt[end] - gt[start]
        est_delta = estimate[end] - estimate[start]
        translation_values.append(float(np.linalg.norm(est_delta - gt_delta)))
        i0 = int(np.argmin(np.abs(trajectory.time - query[start])))
        i1 = int(np.argmin(np.abs(trajectory.time - query[end])))
        er = (trajectory.estimate_rotation[i0].T @
              trajectory.estimate_rotation[i1])
        gr = trajectory.gt_rotation[i0].T @ trajectory.gt_rotation[i1]
        yaw_values.append(abs(math.degrees(_wrap_angle(np.asarray([
            _yaw(er) - _yaw(gr)]))[0])))
    return {
        "available": bool(translation_values),
        "eligible_window_count": int(len(translation_values)),
        "window_s": float(window_s),
        "translation_drift_max_m": (max(translation_values)
                                    if translation_values else None),
        "yaw_drift_max_deg": max(yaw_values) if yaw_values else None,
    }


def _dynamic_freeze(trajectory: AssociatedTrajectory) -> Dict[str, Any]:
    """Find sustained estimator stasis while GT is observably moving.

    Endpoint coverage and publication rate cannot detect a node that continues
    publishing the same pose.  This check operates only on fixed-time GT
    associations and accumulates consecutive intervals in which GT moves but
    both estimated translation and rotation are effectively unchanged.
    """
    if len(trajectory.time) < 2:
        return {"evaluable": False, "maximum_duration_s": None,
                "interval_count": 0}
    dt = np.diff(trajectory.time)
    valid_dt = np.isfinite(dt) & (dt > 1e-6)
    estimate_step = np.linalg.norm(
        np.diff(trajectory.estimate_position, axis=0), axis=1)
    gt_step = np.linalg.norm(np.diff(trajectory.gt_position, axis=0), axis=1)
    estimate_rotation_step = np.asarray([
        geodesic_deg(trajectory.estimate_rotation[index],
                     trajectory.estimate_rotation[index + 1])
        for index in range(len(trajectory.time) - 1)
    ])
    gt_rotation_step = np.asarray([
        geodesic_deg(trajectory.gt_rotation[index],
                     trajectory.gt_rotation[index + 1])
        for index in range(len(trajectory.time) - 1)
    ])
    gt_dynamic = ((gt_step >= 0.05 * dt) |
                  (gt_rotation_step >= 5.0 * dt))
    estimate_static = ((estimate_step <= 0.005 * dt) &
                       (estimate_rotation_step <= 0.5 * dt))
    frozen = valid_dt & gt_dynamic & estimate_static
    maximum = current = 0.0
    for interval, is_frozen in zip(dt, frozen):
        current = current + float(interval) if is_frozen else 0.0
        maximum = max(maximum, current)
    return {
        "evaluable": True,
        "maximum_duration_s": maximum,
        "interval_count": int(np.count_nonzero(frozen)),
        "gt_dynamic_interval_count": int(np.count_nonzero(valid_dt & gt_dynamic)),
        "translation_static_threshold_mps": 0.005,
        "rotation_static_threshold_deg_s": 0.5,
        "gt_translation_dynamic_threshold_mps": 0.05,
        "gt_rotation_dynamic_threshold_deg_s": 5.0,
    }


def detect_takeoff_window(gt: PoseStream,
                          maximum_ground_z_m: float) -> Dict[str, Any]:
    """Detect a causal ground-to-stable-hover window from GT height.

    The detector is deliberately independent of estimator output.  A replay
    cropped after takeoff reports ``available=false`` instead of inventing a
    takeoff window from the estimate under test.
    """
    gt = gt.sorted_valid()
    if len(gt.header_time) < 20 or gt.header_time[-1] - gt.header_time[0] < 2.0:
        return {"available": False, "reason": "insufficient_gt"}
    query = np.arange(gt.header_time[0], gt.header_time[-1] + 1e-9, 0.02)
    z = np.interp(query, gt.header_time, gt.position[:, 2])
    initial = query <= query[0] + min(2.0, 0.2 * (query[-1] - query[0]))
    ground_z = float(np.median(z[initial]))
    if ground_z > maximum_ground_z_m:
        return {
            "available": False,
            "reason": "bag_starts_above_configured_testbed_ground",
            "ground_z_m": ground_z,
            "maximum_ground_z_m": float(maximum_ground_z_m),
        }
    if float(np.max(z) - ground_z) < 0.30:
        return {
            "available": False, "reason": "bag_starts_airborne_or_no_takeoff",
            "ground_z_m": ground_z,
        }
    kernel = np.ones(25) / 25.0  # 0.5 s at 50 Hz
    smooth_z = np.convolve(np.pad(z, (12, 12), mode="edge"), kernel,
                           mode="valid")
    vertical_speed = np.gradient(smooth_z, 0.02)
    above = smooth_z >= ground_z + 0.10
    hold = 15  # require 0.3 s above threshold
    starts = np.flatnonzero(np.convolve(
        above.astype(int), np.ones(hold, dtype=int), mode="valid") == hold)
    if not len(starts):
        return {"available": False, "reason": "no_sustained_height_crossing",
                "ground_z_m": ground_z}
    start_index = int(starts[0])
    stable = ((smooth_z >= ground_z + 0.30) &
              (np.abs(vertical_speed) <= 0.15))
    stable_hold = 25  # 0.5 s stable vertical speed
    stable_starts = np.flatnonzero(np.convolve(
        stable.astype(int), np.ones(stable_hold, dtype=int), mode="valid") ==
                                   stable_hold)
    stable_starts = stable_starts[stable_starts > start_index + hold]
    end_index = (int(stable_starts[0]) if len(stable_starts)
                 else min(len(query) - 1, start_index + int(round(5.0 / 0.02))))
    return {
        "available": True,
        "method": "gt_height_0p10m_to_height_0p30m_and_abs_vz_0p15mps",
        "ground_z_m": ground_z,
        "start_s": float(query[start_index]),
        "end_s": float(query[end_index]),
        "duration_s": float(query[end_index] - query[start_index]),
        "stable_hover_detected": bool(len(stable_starts)),
    }


def takeoff_metrics(trajectory: AssociatedTrajectory, window: Mapping[str, Any],
                    output_start_s: float) -> Dict[str, Any]:
    result = dict(window)
    if not bool(window.get("available")):
        return result
    start = float(window["start_s"])
    end = float(window["end_s"])
    indices = np.flatnonzero((trajectory.time >= start) &
                             (trajectory.time <= end))
    if len(indices) < 3:
        result.update(available=False, reason="too_few_estimator_samples")
        return result
    pe = trajectory.estimate_position[indices]
    pg = trajectory.gt_position[indices]
    translation_error = np.linalg.norm(pe - pg, axis=1)
    orientation_error = np.asarray([
        geodesic_deg(trajectory.estimate_rotation[index],
                     trajectory.gt_rotation[index]) for index in indices
    ])
    tilt_error = np.degrees(np.arccos(np.clip(np.asarray([
        np.dot(trajectory.estimate_rotation[index][:, 2],
               trajectory.gt_rotation[index][:, 2]) for index in indices
    ]), -1.0, 1.0)))
    gt_vertical = float(pg[-1, 2] - pg[0, 2])
    estimate_vertical = float(pe[-1, 2] - pe[0, 2])
    result.update({
        "sample_count": int(len(indices)),
        "output_ready_lead_s": float(start - output_start_s),
        "translation_ape_rmse_m": _rmse(translation_error),
        "translation_ape_max_m": float(np.max(translation_error)),
        "vertical_error_p95_abs_m": _quantile(np.abs(pe[:, 2] - pg[:, 2]), 0.95),
        "orientation_p95_deg": _quantile(orientation_error, 0.95),
        "tilt_p95_deg": _quantile(tilt_error, 0.95),
        "tilt_max_deg": float(np.max(tilt_error)),
        "gt_vertical_displacement_m": gt_vertical,
        "estimated_vertical_displacement_m": estimate_vertical,
        "vertical_displacement_scale": (
            estimate_vertical / gt_vertical if abs(gt_vertical) > 0.05 else None),
    })
    return result


def trajectory_metrics(trajectory: AssociatedTrajectory,
                       stream: PoseStream, gt: PoseStream,
                       alignment_config: Mapping[str, Any],
                       stationary_window_s: float,
                       maximum_ground_z_m: float, *,
                       fixed_alignment: Optional[Tuple[
                           np.ndarray, np.ndarray, Mapping[str, Any]]] = None
                       ) -> Tuple[Dict[str, Any], Optional[np.ndarray], Optional[np.ndarray]]:
    if len(trajectory.time) < int(alignment_config["minimum_samples"]):
        return ({
            "evaluable": False,
            "fixed_association_count": int(len(trajectory.time)),
            "failure": "too_few_fixed_time_associations",
        }, None, None)
    if fixed_alignment is None:
        rotation, translation, alignment = initialization_alignment(
            trajectory, float(alignment_config["duration_s"]),
            int(alignment_config["minimum_samples"]))
    else:
        rotation = np.asarray(fixed_alignment[0], dtype=float)
        translation = np.asarray(fixed_alignment[1], dtype=float)
        alignment = dict(fixed_alignment[2])
        alignment["reused_from_primary_full_result"] = True
    aligned = apply_alignment(trajectory, rotation, translation)
    position_error_vector = aligned.estimate_position - aligned.gt_position
    translation_error = np.linalg.norm(position_error_vector, axis=1)
    orientation_error = np.asarray([
        geodesic_deg(estimate, truth)
        for estimate, truth in zip(aligned.estimate_rotation, aligned.gt_rotation)
    ])
    tilt_error = np.degrees(np.arccos(np.clip(np.asarray([
        np.dot(estimate[:, 2], truth[:, 2])
        for estimate, truth in zip(aligned.estimate_rotation, aligned.gt_rotation)
    ]), -1.0, 1.0)))
    yaw_error = np.degrees(_wrap_angle(np.asarray([
        _yaw(estimate) - _yaw(truth)
        for estimate, truth in zip(aligned.estimate_rotation, aligned.gt_rotation)
    ])))
    rpe: Dict[str, Any] = {}
    for delta in (0.1, 0.5, 1.0, 2.0):
        translation_rpe, orientation_rpe = relative_pose_error(aligned, delta)
        label = str(delta).replace(".", "p")
        rpe[f"translation_rpe_{label}s_rmse_m"] = _rmse(translation_rpe)
        rpe[f"translation_rpe_{label}s_p95_m"] = _quantile(translation_rpe, 0.95)
        rpe[f"orientation_rpe_{label}s_rmse_deg"] = _rmse(orientation_rpe)
        rpe[f"orientation_rpe_{label}s_p95_deg"] = _quantile(orientation_rpe, 0.95)

    query, estimate_uniform = _uniform_positions(
        aligned.time, aligned.estimate_position)
    _, gt_uniform = _uniform_positions(aligned.time, aligned.gt_position)
    estimate_path = _path_length(estimate_uniform)
    gt_path = _path_length(gt_uniform)
    gt_start = float(gt.header_time[0])
    gt_end = float(gt.header_time[-1])
    output_start = float(stream.header_time[0])
    output_end = float(stream.header_time[-1])
    overlap_start = max(gt_start, output_start)
    overlap_end = min(gt_end, output_end)
    full_span = max(gt_end - gt_start, 1e-9)
    post_init_span = max(gt_end - overlap_start, 1e-9)
    takeoff = takeoff_metrics(
        aligned, detect_takeoff_window(gt, maximum_ground_z_m), output_start)
    alignment = dict(alignment)
    if takeoff.get("available"):
        takeoff_start = float(takeoff["start_s"])
        alignment.update({
            "first_output_to_takeoff_lead_s": takeoff_start - output_start,
            "overlaps_detected_takeoff": (
                float(alignment.get("sensor_end_s", aligned.time[0])) >=
                takeoff_start),
        })
    metrics: Dict[str, Any] = {
        "evaluable": True,
        "alignment": alignment,
        "fixed_association_count": int(len(aligned.time)),
        "fixed_association_fraction": float(len(aligned.time) /
                                            max(1, len(stream.sorted_valid().header_time))),
        "full_window_coverage": float(max(0.0, overlap_end - overlap_start) /
                                      full_span),
        "post_initialization_coverage": float(
            max(0.0, overlap_end - overlap_start) / post_init_span),
        "first_output_delay_s": float(output_start - gt_start),
        "last_output_lead_s": float(gt_end - output_end),
        "translation_ape_rmse_m": _rmse(translation_error),
        "translation_ape_mean_m": float(np.mean(translation_error)),
        "translation_ape_p95_m": _quantile(translation_error, 0.95),
        "translation_ape_max_m": float(np.max(translation_error)),
        "axis_error_rmse_m": [
            _rmse(position_error_vector[:, axis]) for axis in range(3)
        ],
        "axis_error_p95_abs_m": [
            _quantile(np.abs(position_error_vector[:, axis]), 0.95)
            for axis in range(3)
        ],
        "orientation_rmse_deg": _rmse(orientation_error),
        "orientation_p90_deg": _quantile(orientation_error, 0.90),
        "orientation_p95_deg": _quantile(orientation_error, 0.95),
        "orientation_max_deg": float(np.max(orientation_error)),
        "tilt_rmse_deg": _rmse(tilt_error),
        "tilt_p95_deg": _quantile(tilt_error, 0.95),
        "tilt_max_deg": float(np.max(tilt_error)),
        "yaw_rmse_deg": _rmse(yaw_error),
        "yaw_p95_abs_deg": _quantile(np.abs(yaw_error), 0.95),
        "yaw_max_abs_deg": float(np.max(np.abs(yaw_error))),
        "estimated_path_m": estimate_path,
        "gt_path_m": gt_path,
        "path_ratio": (estimate_path / gt_path if gt_path > 1e-9 else None),
        "direction_cosine_1s": _weighted_direction_cosine(
            aligned.time, aligned.estimate_position, aligned.gt_position),
        "stationary": _stationary_drift(aligned, stationary_window_s),
        "dynamic_freeze": _dynamic_freeze(aligned),
        "takeoff": takeoff,
        **rpe,
    }
    return metrics, rotation, translation


def propagated_diagnostics(stream: PoseStream, local: Optional[PoseStream],
                           gt: PoseStream, associated: AssociatedTrajectory,
                           alignment_rotation: Optional[np.ndarray],
                           alignment_translation: Optional[np.ndarray]) -> Dict[str, Any]:
    data = stream.sorted_valid()
    result: Dict[str, Any] = stream_integrity(stream)
    if len(data.header_time) < 3:
        result.update(evaluable=False, failure="too_few_propagated_messages")
        return result
    dt = np.diff(data.header_time)
    position_step = np.linalg.norm(np.diff(data.position, axis=0), axis=1)
    rotations = np.asarray([q_to_R(value) for value in data.quaternion])
    rotation_step = np.asarray([
        geodesic_deg(rotations[index], rotations[index + 1])
        for index in range(len(rotations) - 1)
    ])
    result.update({
        "evaluable": True,
        "position_step_p99_m": _quantile(position_step, 0.99),
        "position_step_max_m": float(np.max(position_step)),
        "rotation_step_p99_deg": _quantile(rotation_step, 0.99),
        "rotation_step_max_deg": float(np.max(rotation_step)),
        "pose_covariance_nonzero_fraction": (
            float(np.mean(data.pose_covariance_nonzero))
            if data.pose_covariance_nonzero is not None else None),
        "twist_covariance_nonzero_fraction": (
            float(np.mean(data.twist_covariance_nonzero))
            if data.twist_covariance_nonzero is not None else None),
    })
    gt_start = float(gt.header_time[0])
    gt_end = float(gt.header_time[-1])
    output_start = float(data.header_time[0])
    output_end = float(data.header_time[-1])
    overlap_start = max(gt_start, output_start)
    overlap_end = min(gt_end, output_end)
    result["post_initialization_coverage"] = float(
        max(0.0, overlap_end - overlap_start) /
        max(gt_end - overlap_start, 1e-9))

    if local is not None and len(local.sorted_valid().header_time):
        local_time = local.sorted_valid().header_time
        correction_steps = []
        for stamp in local_time:
            index = int(np.searchsorted(data.header_time, stamp))
            if 0 < index < len(data.header_time):
                correction_steps.append(position_step[index - 1])
        values = np.asarray(correction_steps)
        result["correction_step_p99_m"] = _quantile(values, 0.99)
        result["correction_step_max_m"] = (
            float(np.max(values)) if len(values) else None)

    if data.linear_velocity is not None:
        finite_velocity = np.all(np.isfinite(data.linear_velocity), axis=1)
        result["linear_twist_finite_fraction"] = float(np.mean(finite_velocity))
        # nav_msgs/Odometry requires twist in child_frame_id (base_link).
        # Rotate it into the estimator world before comparing with pose or GT.
        linear_velocity_world = np.einsum(
            "nij,nj->ni", rotations, data.linear_velocity)
        derivative = np.gradient(data.position, data.header_time, axis=0)
        consistency = np.linalg.norm(linear_velocity_world - derivative, axis=1)
        result["linear_twist_pose_rmse_mps"] = _rmse(consistency)
        result["linear_twist_pose_p95_mps"] = _quantile(consistency, 0.95)
        if (len(associated.time) >= 3 and alignment_rotation is not None and
                alignment_translation is not None):
            # Compare only at fixed header times.  The rotation maps the
            # estimator's world-frame velocity into the evaluation frame.
            aligned_velocity = (alignment_rotation @
                                linear_velocity_world.T).T
            gt_position = np.column_stack([
                np.interp(data.header_time, associated.time,
                          associated.gt_position[:, axis], left=math.nan,
                          right=math.nan)
                for axis in range(3)
            ])
            gt_velocity = np.gradient(gt_position, data.header_time, axis=0)
            error = np.linalg.norm(aligned_velocity - gt_velocity, axis=1)
            result["linear_twist_gt_rmse_mps"] = _rmse(error)
            result["linear_twist_gt_p95_mps"] = _quantile(error, 0.95)

    if data.angular_velocity is not None:
        finite_angular_velocity = np.all(
            np.isfinite(data.angular_velocity), axis=1)
        angular_norm = np.linalg.norm(data.angular_velocity, axis=1)
        pose_body_angular_velocity = np.asarray([
            _rotation_vector(rotations[index].T @ rotations[index + 1]) /
            dt[index]
            for index in range(len(dt))
        ])
        angular_consistency = np.linalg.norm(
            data.angular_velocity[:-1] - pose_body_angular_velocity, axis=1)
        pose_angular_speed = np.zeros(len(data.header_time))
        pose_angular_speed[1:] = np.radians(rotation_step) / np.maximum(dt, 1e-9)
        motion_present = bool(_quantile(pose_angular_speed, 0.90) is not None and
                              _quantile(pose_angular_speed, 0.90) > 0.05)
        angular_informative = bool(
            not motion_present or (_quantile(angular_norm, 0.90) or 0.0) > 0.01)
        result.update({
            "angular_twist_finite_fraction": float(
                np.mean(finite_angular_velocity)),
            "angular_twist_p90_rad_s": _quantile(angular_norm, 0.90),
            "pose_angular_speed_p90_rad_s": _quantile(pose_angular_speed, 0.90),
            "angular_twist_pose_rmse_rad_s": _rmse(angular_consistency),
            "angular_twist_pose_p95_rad_s": _quantile(
                angular_consistency, 0.95),
            "angular_motion_present": motion_present,
            "angular_twist_informative": angular_informative,
        })
    return result


def _get_path(document: Mapping[str, Any], dotted: str) -> Any:
    value: Any = document
    for component in dotted.split("."):
        if not isinstance(value, Mapping) or component not in value:
            return None
        value = value[component]
    return value


def evaluate_checks(document: Mapping[str, Any],
                    checks: Sequence[Mapping[str, Any]]) -> Tuple[List[Dict[str, Any]], str]:
    results: List[Dict[str, Any]] = []
    failed_required = False
    unavailable_required = False
    operators = {
        "ge": lambda actual, expected: actual >= expected,
        "le": lambda actual, expected: actual <= expected,
        "eq": lambda actual, expected: actual == expected,
    }
    for check in checks:
        actual = _get_path(document, str(check["metric"]))
        required = bool(check.get("required", True))
        expected = check["threshold"]
        if actual is None:
            status = "unavailable"
            if required:
                unavailable_required = True
            passed = None
        else:
            passed = bool(operators[str(check["operator"])](actual, expected))
            status = "pass" if passed else "fail"
            if required and not passed:
                failed_required = True
        results.append({
            "id": str(check["id"]),
            "metric": str(check["metric"]),
            "operator": str(check["operator"]),
            "threshold": expected,
            "actual": actual,
            "required": required,
            "status": status,
        })
    if failed_required:
        overall = "fail"
    elif unavailable_required:
        overall = "incomplete"
    else:
        overall = "pass"
    return results, overall


def _score_window_mask(stamp_ns: np.ndarray,
                       score_window_ns: Optional[Tuple[int, int]]) -> np.ndarray:
    if score_window_ns is None:
        return np.ones(len(stamp_ns), dtype=bool)
    start_ns, end_ns = score_window_ns
    if start_ns <= 0 or end_ns < start_ns:
        raise ValueError("score window must be positive and ordered")
    return (stamp_ns >= start_ns) & (stamp_ns <= end_ns)


def _slice_stream(stream: PoseStream, mask: np.ndarray) -> PoseStream:
    indices = np.flatnonzero(mask)
    optional = lambda value: None if value is None else value[indices]
    return PoseStream(
        topic=stream.topic,
        header_time=stream.header_time[indices],
        header_stamp_ns=stream.header_stamp_ns[indices],
        bag_time=stream.bag_time[indices],
        position=stream.position[indices],
        quaternion=stream.quaternion[indices],
        linear_velocity=optional(stream.linear_velocity),
        angular_velocity=optional(stream.angular_velocity),
        pose_covariance_nonzero=optional(stream.pose_covariance_nonzero),
        twist_covariance_nonzero=optional(stream.twist_covariance_nonzero),
        frame_id=[stream.frame_id[index] for index in indices],
        child_frame_id=[stream.child_frame_id[index] for index in indices],
    )


def evaluate_bag(result_bag: Path, thresholds: Mapping[str, Any],
                 local_topic: str = DEFAULT_LOCAL_TOPIC,
                 propagated_topic: str = DEFAULT_PROP_TOPIC,
                 gt_topic: str = DEFAULT_GT_TOPIC,
                 correction_covariance_topic: str =
                 DEFAULT_CORRECTION_COV_TOPIC, *,
                 score_window_ns: Optional[Tuple[int, int]] = None,
                 fixed_alignment: Optional[Tuple[
                     np.ndarray, np.ndarray, Mapping[str, Any]]] = None
                 ) -> Dict[str, Any]:
    result_bag = result_bag.resolve()
    local_raw = read_pose_stream(result_bag, local_topic)
    propagated_raw = read_pose_stream(result_bag, propagated_topic)
    gt_raw = read_pose_stream(result_bag, gt_topic)
    if gt_raw is None:
        raise RuntimeError(f"{result_bag}: missing evaluation GT topic {gt_topic}")
    if local_raw is None:
        raise RuntimeError(f"{result_bag}: missing local estimator topic {local_topic}")

    with rosbag.Bag(str(result_bag), "r") as bag:
        available = bag.get_type_and_topic_info().topics
        gt_anchor_message_count = (
            int(available[GT_ANCHORED_TOPIC].message_count)
            if GT_ANCHORED_TOPIC in available else 0)

    gt = gt_raw.sorted_valid()
    local = local_raw.sorted_valid()
    if score_window_ns is not None:
        gt = _slice_stream(gt, _score_window_mask(
            gt.header_stamp_ns, score_window_ns))
        local = _slice_stream(
            local, _score_window_mask(local.header_stamp_ns, score_window_ns))
        if propagated_raw is not None:
            propagated_raw = _slice_stream(
                propagated_raw,
                _score_window_mask(
                    propagated_raw.header_stamp_ns, score_window_ns))
    gt_mask, gt_filter = gt_valid_mask(
        gt, float(thresholds["gt_filter"]["max_speed_mps"]),
        float(thresholds["gt_filter"]["max_angular_speed_deg_s"]))
    maximum_difference = float(thresholds["association"]["max_difference_s"])
    local_associated = associate_fixed(local, gt, gt_mask, maximum_difference)
    local_timestamp_associated = associate_fixed(
        local, gt, np.ones(len(gt.header_time), dtype=bool), maximum_difference)
    local_metrics, local_rotation, local_translation = trajectory_metrics(
        local_associated, local, gt, thresholds["initial_alignment"],
        float(thresholds["stationary"]["window_s"]),
        float(thresholds["takeoff"]["maximum_ground_z_m"]),
        fixed_alignment=fixed_alignment)
    local_metrics["fixed_timestamp_association_fraction"] = float(
        len(local_timestamp_associated.time) /
        max(1, len(local.header_time)))
    local_metrics["integrity"] = stream_integrity(local_raw)
    correction_covariance = correction_covariance_diagnostics(
        result_bag, correction_covariance_topic, local_raw, maximum_difference)

    diagnostic_offset = None
    if len(local.header_time) > 5 and len(gt.header_time) > 5:
        diagnostic_offset = float(estimate_offset(
            local.header_time, local.position, gt.header_time, gt.position))

    propagated_metrics: Dict[str, Any]
    if propagated_raw is None:
        propagated_metrics = {
            "evaluable": False,
            "failure": "missing_propagated_odometry",
        }
    else:
        propagated = propagated_raw.sorted_valid()
        prop_associated = associate_fixed(
            propagated, gt, gt_mask, maximum_difference)
        propagated_metrics = propagated_diagnostics(
            propagated_raw, local_raw, gt, prop_associated,
            local_rotation, local_translation)
        # Also score propagated pose with the *same frozen local-frame
        # transform.  This exposes frame jumps at GT-anchor latch rather than
        # fitting them away with a second alignment.
        if (local_rotation is not None and local_translation is not None and
                len(prop_associated.time) >= 3):
            aligned_prop = apply_alignment(
                prop_associated, local_rotation, local_translation)
            error = np.linalg.norm(
                aligned_prop.estimate_position - aligned_prop.gt_position,
                axis=1)
            orientation_error = np.asarray([
                geodesic_deg(estimate, truth)
                for estimate, truth in zip(
                    aligned_prop.estimate_rotation, aligned_prop.gt_rotation)
            ])
            propagated_metrics.update({
                "fixed_local_alignment_ape_rmse_m": _rmse(error),
                "fixed_local_alignment_ape_p95_m": _quantile(error, 0.95),
                "fixed_local_alignment_ape_max_m": float(np.max(error)),
                "fixed_local_alignment_orientation_rmse_deg": _rmse(
                    orientation_error),
                "fixed_local_alignment_orientation_p90_deg": _quantile(
                    orientation_error, 0.90),
            })

    gt_anchor_free = gt_anchor_message_count == 0
    document: Dict[str, Any] = {
        "schema": SCHEMA,
        "result_bag": str(result_bag),
        "evaluation_semantics": {
            "time_offset_used_for_scoring_s": 0.0,
            "diagnostic_best_speed_offset_s": diagnostic_offset,
            "spatial_alignment": "initialization-window yaw+translation; scale=1; frozen",
            "whole_trajectory_alignment_used": False,
            "per_session_time_optimization_used": False,
            "score_window_sensor_stamp_ns": (
                None if score_window_ns is None else {
                    "start": str(score_window_ns[0]),
                    "end": str(score_window_ns[1]),
                    "boundary": "start_inclusive_end_inclusive",
                }),
            "fixed_alignment_supplied": fixed_alignment is not None,
        },
        "gt_filter": gt_filter,
        "gt_independence": {
            "gt_anchored_topic": GT_ANCHORED_TOPIC,
            "gt_anchored_message_count": gt_anchor_message_count,
            "gt_anchor_free": gt_anchor_free,
            "interpretation": (
                "GT was not observed through the anchored diagnostic topic"
                if gt_anchor_free else
                "GT anchor was active; propagated odometry is contaminated in current code"),
        },
        "local": local_metrics,
        "propagated": propagated_metrics,
        "correction_covariance": correction_covariance,
    }
    checks, status = evaluate_checks(document, thresholds["checks"])
    document["checks"] = checks
    document["status"] = status
    document["flight_ready"] = status == "pass"
    document["thresholds_schema"] = thresholds.get("schema")
    return _finite(document)


def load_thresholds(path: Path) -> Dict[str, Any]:
    with path.open() as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or "checks" not in document:
        raise ValueError(f"invalid flight-readiness thresholds: {path}")
    return document


def fixed_alignment_from_primary(
        primary_path: Path, result_bag: Path, thresholds_path: Path
        ) -> Tuple[Tuple[np.ndarray, np.ndarray, Mapping[str, Any]], Dict[str, Any]]:
    """Validate and recover the one frozen transform from a primary report."""
    primary_path = primary_path.resolve()
    result_bag = result_bag.resolve()
    thresholds_path = thresholds_path.resolve()
    try:
        primary = json.loads(primary_path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read primary report: {error}") from error
    semantics = primary.get("evaluation_semantics", {})
    bindings = primary.get("artifact_bindings", {})
    expected_bag = _file_identity(result_bag)
    expected_thresholds = _file_identity(thresholds_path)
    if (primary.get("schema") != SCHEMA or
            Path(str(primary.get("result_bag", ""))).resolve() != result_bag or
            semantics.get("score_window_sensor_stamp_ns") is not None or
            semantics.get("fixed_alignment_supplied") is not False or
            bindings.get("result_bag") != expected_bag or
            bindings.get("thresholds") != expected_thresholds or
            primary.get("thresholds_schema") !=
            load_thresholds(thresholds_path).get("schema")):
        raise ValueError(
            "primary report is not bound to this full result bag/threshold set")
    alignment = primary.get("local", {}).get("alignment", {})
    translation = alignment.get("translation_m")
    yaw_deg = alignment.get("yaw_deg")
    if (alignment.get("method") !=
            "initialization_window_yaw_and_translation_only" or
            alignment.get("scale") != 1.0 or
            not isinstance(translation, list) or len(translation) != 3 or
            not isinstance(yaw_deg, (int, float)) or isinstance(yaw_deg, bool) or
            not math.isfinite(float(yaw_deg)) or
            any(not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(float(value)) for value in translation)):
        raise ValueError("primary report lacks a finite compatible alignment")
    yaw = math.radians(float(yaw_deg))
    rotation = np.asarray([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    details = {
        **dict(alignment),
        "primary_report": str(primary_path),
        "primary_report_identity": _file_identity(primary_path),
    }
    return (rotation, np.asarray(translation, dtype=float), details), primary


def compose_ground_hover_secondary(
        masked: Mapping[str, Any], primary: Mapping[str, Any],
        primary_identity: Mapping[str, Any]) -> Dict[str, Any]:
    """Make a ranking-only report; primary full-result failures stay binding."""
    local = masked.get("local", {})
    propagated = masked.get("propagated", {})
    local_excluded = {"integrity", "takeoff", "stationary"}
    local_accuracy = {
        key: value for key, value in local.items()
        if key not in local_excluded
    }
    propagated_accuracy = {
        key: value for key, value in propagated.items()
        if key.startswith("fixed_local_alignment_")
    }
    return {
        "schema": SECONDARY_SCHEMA,
        "result_bag": masked["result_bag"],
        "role": "phase_a_ranking_compatibility_only",
        "flight_ready": False,
        "status": "ranking_only",
        "can_override_primary_failure": False,
        "primary_report_identity": dict(primary_identity),
        "primary_status": primary.get("status"),
        "primary_flight_ready": primary.get("flight_ready"),
        "evaluation_semantics": {
            **dict(masked["evaluation_semantics"]),
            "scope": (
                "hover-window local/propagated accuracy only; full-result "
                "integrity, covariance, GT isolation, and high-rate interface "
                "diagnostics are inherited from the bound primary report"),
            "primary_alignment_reused_without_refit": True,
        },
        "local_accuracy": local_accuracy,
        "propagated_accuracy": propagated_accuracy,
        "full_result_interface_inherited": {
            "gt_independence": primary.get("gt_independence"),
            "local_integrity": primary.get("local", {}).get("integrity"),
            "propagated": primary.get("propagated"),
            "correction_covariance": primary.get("correction_covariance"),
            "checks": primary.get("checks"),
            "status": primary.get("status"),
            "flight_ready": primary.get("flight_ready"),
        },
        "thresholds_schema": masked.get("thresholds_schema"),
        "artifact_bindings": masked.get("artifact_bindings"),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_bag", type=Path)
    parser.add_argument("--thresholds", type=Path, default=DEFAULT_THRESHOLDS)
    parser.add_argument("--local-topic", default=DEFAULT_LOCAL_TOPIC)
    parser.add_argument("--propagated-topic", default=DEFAULT_PROP_TOPIC)
    parser.add_argument("--correction-covariance-topic",
                        default=DEFAULT_CORRECTION_COV_TOPIC)
    parser.add_argument("--gt-topic", default=DEFAULT_GT_TOPIC)
    parser.add_argument("--score-start-ns", type=int)
    parser.add_argument("--score-end-ns", type=int)
    parser.add_argument("--fixed-alignment-report", type=Path,
                        help=("reuse local alignment from an existing primary "
                              "full-result report; never refit inside score window"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--compact", action="store_true")
    arguments = parser.parse_args()
    thresholds = load_thresholds(arguments.thresholds)
    if (arguments.score_start_ns is None) != (arguments.score_end_ns is None):
        parser.error("--score-start-ns and --score-end-ns must be supplied together")
    score_window = (None if arguments.score_start_ns is None else
                    (arguments.score_start_ns, arguments.score_end_ns))
    fixed_alignment = None
    primary = None
    if arguments.fixed_alignment_report is not None:
        if score_window is None:
            parser.error("a fixed primary alignment requires an exact score window")
        try:
            fixed_alignment, primary = fixed_alignment_from_primary(
                arguments.fixed_alignment_report, arguments.result_bag,
                arguments.thresholds)
        except ValueError as error:
            parser.error(str(error))
    document = evaluate_bag(
        arguments.result_bag, thresholds, arguments.local_topic,
        arguments.propagated_topic, arguments.gt_topic,
        arguments.correction_covariance_topic,
        score_window_ns=score_window, fixed_alignment=fixed_alignment)
    document["artifact_bindings"] = {
        "result_bag": _file_identity(arguments.result_bag),
        "thresholds": _file_identity(arguments.thresholds),
    }
    if primary is not None:
        document = compose_ground_hover_secondary(
            document, primary,
            _file_identity(arguments.fixed_alignment_report))
    payload = json.dumps(
        document, indent=None if arguments.compact else 2,
        sort_keys=True) + "\n"
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(payload)
        print(f"wrote {arguments.output}")
    else:
        print(payload, end="")
    # A completed-but-failed evaluation is not a process error; campaign
    # callers consume the explicit status field.


if __name__ == "__main__":
    main()

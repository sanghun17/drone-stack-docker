#!/usr/bin/env python3
"""Portable ROS1 bag inspection and asset export for the p1/pm4 case study.

This file intentionally does not import ROS, rosbag, cv_bridge, or project code.
It uses the pure-Python ``rosbags`` package and standard ROS1 Noetic message
definitions bundled with that package.

Typical Windows usage::

    py windows_bag_tools.py inspect bags\p1_synth.bag
    py windows_bag_tools.py export bags\p1_synth.bag --out exports\p1 \
        --audit-events --vio-time-offset 0.17

The exporter never modifies its input bag.
"""

from __future__ import annotations

import argparse
import bisect
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Iterable, Sequence

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


GT_TOPIC = "/vrpn_client_node/pure/pose"
VIO_TOPIC = "/aft_mapped_to_optitrack"
RGB_TOPIC = "/camera/color/image_raw/compressed"
PLAN_TOPIC = "/jax/optimal_trajectory"
AUDIT_TOPIC = "/jax/weight_audit"
DEBUG_TOPIC = "/jax/debug_info"
UNCERTAINTY_MARKER_TOPIC = "/local_planner/uncertainty_risk"

REPLAN_TIME_S = 0.045
AUDIT_HEADER_LEN = 18

RELEVANT_TOPICS = (
    GT_TOPIC,
    VIO_TOPIC,
    RGB_TOPIC,
    PLAN_TOPIC,
    AUDIT_TOPIC,
    DEBUG_TOPIC,
    UNCERTAINTY_MARKER_TOPIC,
    "/planner/command/trajectory",
    "/planning/trajectory",
    "/robot/odom",
    "/tf",
    "/tf_static",
)

TYPESTORE = get_typestore(Stores.ROS1_NOETIC)


def ns_to_s(value: int) -> float:
    return float(value) * 1e-9


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def header_stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    if header is None:
        return None
    value = stamp_to_ns(header.stamp)
    return value if value > 0 else None


def duration_s(duration: Any) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def sha256(path: Path, block_size: int = 8 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(block_size), b""):
            digest.update(block)
    return digest.hexdigest()


def ensure_bag(path: Path) -> Path:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"bag does not exist: {path}")
    return path


def safe_token(text: str) -> str:
    token = re.sub(r"[^A-Za-z0-9_.-]+", "_", text).strip("_.")
    return token or "event"


@dataclass
class PoseRecord:
    header_ns: int
    record_ns: int
    frame_id: str
    child_frame_id: str
    xyz: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass
class ImageRecord:
    header_ns: int
    record_ns: int
    frame_id: str
    image_format: str
    payload: bytes


@dataclass
class PlanRecord:
    header_ns: int
    record_ns: int
    frame_id: str
    reason: str
    control: np.ndarray
    message: Any

    @property
    def accept_ns(self) -> int:
        # The historical implementation scheduled activation at now()+45 ms.
        return self.header_ns - int(round(REPLAN_TIME_S * 1e9))


@dataclass
class AuditRecord:
    record_ns: int
    version: int
    n_candidates: int
    n_steps: int
    best_idx: int
    goal_only_idx: int
    emergency_idx: int
    collision_floor_m: float
    weights: np.ndarray
    safety_mode_code: int
    cvar_alpha: float
    active_n_safe: int
    planning_bounds_enabled: bool
    esdf_m: np.ndarray
    q: np.ndarray
    position_norm_m: np.ndarray
    base_cost: np.ndarray
    controls: np.ndarray
    non_weight_gate: np.ndarray

    def required_margin(self) -> np.ndarray:
        w = self.weights
        return (
            self.collision_floor_m
            + w[0] * self.q[:, :, 0]
            + w[1] * self.q[:, :, 1]
            + w[2] * self.q[:, :, 2]
            + w[3] * self.position_norm_m * self.q[:, :, 3]
        )

    def safety_slack(self) -> np.ndarray:
        return self.esdf_m - self.required_margin()


@dataclass
class DebugRecord:
    record_ns: int
    payload_length: int
    best_idx: int | None
    control: np.ndarray


@dataclass
class MarkerRecord:
    header_ns: int
    record_ns: int
    message: Any


@dataclass
class BagData:
    bag_start_ns: int
    bag_end_ns: int
    gt: list[PoseRecord]
    vio: list[PoseRecord]
    rgb: list[ImageRecord]
    plans: list[PlanRecord]
    audits: list[AuditRecord]
    debug: list[DebugRecord]
    uncertainty_markers: list[MarkerRecord]


@dataclass
class EventRequest:
    label: str
    event_ns: int
    source: str
    audit: AuditRecord | None = None
    plan: PlanRecord | None = None
    pairing_note: str = ""


def pose_from_message(message: Any, record_ns: int) -> PoseRecord:
    header_ns = header_stamp_ns(message)
    if header_ns is None:
        raise ValueError("pose message has no valid header stamp")
    pose_container = message.pose
    pose = pose_container.pose if hasattr(pose_container, "pose") else pose_container
    position = pose.position
    orientation = pose.orientation
    return PoseRecord(
        header_ns=header_ns,
        record_ns=record_ns,
        frame_id=str(message.header.frame_id),
        child_frame_id=str(getattr(message, "child_frame_id", "")),
        xyz=np.asarray([position.x, position.y, position.z], dtype=np.float64),
        quaternion_xyzw=np.asarray(
            [orientation.x, orientation.y, orientation.z, orientation.w],
            dtype=np.float64,
        ),
    )


def image_from_message(message: Any, record_ns: int) -> ImageRecord:
    header_ns = header_stamp_ns(message)
    if header_ns is None:
        raise ValueError("image message has no valid header stamp")
    data = message.data
    payload = data.tobytes() if hasattr(data, "tobytes") else bytes(data)
    return ImageRecord(
        header_ns=header_ns,
        record_ns=record_ns,
        frame_id=str(message.header.frame_id),
        image_format=str(message.format),
        payload=payload,
    )


def plan_from_message(message: Any, record_ns: int) -> PlanRecord:
    header_ns = header_stamp_ns(message)
    if header_ns is None:
        raise ValueError("plan message has no valid header stamp")
    if len(message.points) < 2 or not message.points[1].velocities:
        raise ValueError("plan has no point[1] target velocity")
    velocity = message.points[1].velocities[0]
    control = np.asarray(
        [
            velocity.linear.x,
            velocity.linear.y,
            velocity.linear.z,
            velocity.angular.z,
        ],
        dtype=np.float64,
    )
    reason = str(message.joint_names[0]) if message.joint_names else ""
    return PlanRecord(
        header_ns=header_ns,
        record_ns=record_ns,
        frame_id=str(message.header.frame_id),
        reason=reason,
        control=control,
        message=message,
    )


def decode_audit(message: Any, record_ns: int) -> AuditRecord:
    data = np.asarray(message.data, dtype=np.float32)
    if data.size < AUDIT_HEADER_LEN:
        raise ValueError(f"audit payload too short: {data.size}")
    version = int(round(float(data[0])))
    header_len = int(round(float(data[1])))
    n_candidates = int(round(float(data[2])))
    n_steps = int(round(float(data[3])))
    n_axes = int(round(float(data[4])))
    if version != 1 or header_len != AUDIT_HEADER_LEN or n_axes != 4:
        raise ValueError(
            f"unsupported audit layout: version={version}, header={header_len}, "
            f"axes={n_axes}"
        )
    expected = (
        AUDIT_HEADER_LEN
        + n_candidates * n_steps
        + n_candidates * n_steps * n_axes
        + n_candidates * n_steps
        + n_candidates
        + n_candidates * n_axes
        + n_candidates
    )
    if data.size != expected:
        raise ValueError(f"audit payload has {data.size} values, expected {expected}")

    offset = AUDIT_HEADER_LEN
    count = n_candidates * n_steps
    esdf = data[offset : offset + count].reshape(n_candidates, n_steps)
    offset += count
    count = n_candidates * n_steps * n_axes
    q = data[offset : offset + count].reshape(n_candidates, n_steps, n_axes)
    offset += count
    count = n_candidates * n_steps
    pnorm = data[offset : offset + count].reshape(n_candidates, n_steps)
    offset += count
    base_cost = data[offset : offset + n_candidates]
    offset += n_candidates
    count = n_candidates * n_axes
    controls = data[offset : offset + count].reshape(n_candidates, n_axes)
    offset += count
    non_weight_gate = data[offset : offset + n_candidates] > 0.5

    return AuditRecord(
        record_ns=record_ns,
        version=version,
        n_candidates=n_candidates,
        n_steps=n_steps,
        best_idx=int(round(float(data[5]))),
        goal_only_idx=int(round(float(data[6]))),
        emergency_idx=int(round(float(data[7]))),
        collision_floor_m=float(data[8]),
        weights=data[9:13].astype(np.float64),
        safety_mode_code=int(round(float(data[13]))),
        cvar_alpha=float(data[14]),
        active_n_safe=int(round(float(data[15]))),
        planning_bounds_enabled=bool(round(float(data[16]))),
        esdf_m=esdf.astype(np.float64),
        q=q.astype(np.float64),
        position_norm_m=pnorm.astype(np.float64),
        base_cost=base_cost.astype(np.float64),
        controls=controls.astype(np.float64),
        non_weight_gate=non_weight_gate,
    )


def debug_from_message(message: Any, record_ns: int) -> DebugRecord:
    data = np.asarray(message.data, dtype=np.float64)
    best_idx = int(round(float(data[13]))) if data.size >= 18 else None
    control = (
        data[:4].copy() if data.size >= 4 else np.full(4, np.nan, dtype=np.float64)
    )
    return DebugRecord(
        record_ns=record_ns,
        payload_length=int(data.size),
        best_idx=best_idx,
        control=control,
    )


def marker_from_message(message: Any, record_ns: int) -> MarkerRecord:
    header_values = [
        header_stamp_ns(marker)
        for marker in message.markers
        if header_stamp_ns(marker) is not None
    ]
    # MarkerArray itself has no header. The historical publisher stamped every
    # marker in one array together, so the first valid marker stamp is the
    # visualization generation/republish time.
    header_ns = int(header_values[0]) if header_values else int(record_ns)
    return MarkerRecord(header_ns=header_ns, record_ns=record_ns, message=message)


def load_bag(path: Path) -> BagData:
    wanted = {
        GT_TOPIC,
        VIO_TOPIC,
        RGB_TOPIC,
        PLAN_TOPIC,
        AUDIT_TOPIC,
        DEBUG_TOPIC,
        UNCERTAINTY_MARKER_TOPIC,
    }
    gt: list[PoseRecord] = []
    vio: list[PoseRecord] = []
    rgb: list[ImageRecord] = []
    plans: list[PlanRecord] = []
    audits: list[AuditRecord] = []
    debug: list[DebugRecord] = []
    uncertainty_markers: list[MarkerRecord] = []
    with Reader(path) as reader:
        connections = [c for c in reader.connections if c.topic in wanted]
        for connection, record_ns, raw in reader.messages(connections=connections):
            message = TYPESTORE.deserialize_ros1(raw, connection.msgtype)
            if connection.topic == GT_TOPIC:
                gt.append(pose_from_message(message, record_ns))
            elif connection.topic == VIO_TOPIC:
                vio.append(pose_from_message(message, record_ns))
            elif connection.topic == RGB_TOPIC:
                rgb.append(image_from_message(message, record_ns))
            elif connection.topic == PLAN_TOPIC:
                plans.append(plan_from_message(message, record_ns))
            elif connection.topic == AUDIT_TOPIC:
                audits.append(decode_audit(message, record_ns))
            elif connection.topic == DEBUG_TOPIC:
                debug.append(debug_from_message(message, record_ns))
            elif connection.topic == UNCERTAINTY_MARKER_TOPIC:
                uncertainty_markers.append(marker_from_message(message, record_ns))
        result = BagData(
            bag_start_ns=int(reader.start_time),
            bag_end_ns=int(reader.end_time),
            gt=gt,
            vio=vio,
            rgb=rgb,
            plans=plans,
            audits=audits,
            debug=debug,
            uncertainty_markers=uncertainty_markers,
        )
    for rows in (result.gt, result.vio, result.rgb, result.uncertainty_markers):
        rows.sort(key=lambda row: row.header_ns)
    result.plans.sort(key=lambda row: row.record_ns)
    result.audits.sort(key=lambda row: row.record_ns)
    result.debug.sort(key=lambda row: row.record_ns)
    return result


def connection_inventory(path: Path) -> dict[str, dict[str, Any]]:
    with Reader(path) as reader:
        rows: dict[str, dict[str, Any]] = {}
        for connection in reader.connections:
            if connection.topic not in RELEVANT_TOPICS:
                continue
            row = rows.setdefault(
                connection.topic,
                {"type": connection.msgtype, "messages": 0, "connections": 0},
            )
            row["messages"] += int(connection.msgcount)
            row["connections"] += 1

        header_topics = {GT_TOPIC, VIO_TOPIC, RGB_TOPIC, PLAN_TOPIC}
        bounds: dict[str, list[int]] = {}
        connections = [c for c in reader.connections if c.topic in header_topics]
        for connection, record_ns, raw in reader.messages(connections=connections):
            message = TYPESTORE.deserialize_ros1(raw, connection.msgtype)
            header_ns = header_stamp_ns(message)
            if header_ns is None:
                continue
            value = bounds.setdefault(
                connection.topic, [header_ns, header_ns, record_ns, record_ns]
            )
            value[0] = min(value[0], header_ns)
            value[1] = max(value[1], header_ns)
            value[2] = min(value[2], record_ns)
            value[3] = max(value[3], record_ns)
        for topic, value in bounds.items():
            rows[topic].update(
                {
                    "header_start_s": ns_to_s(value[0]),
                    "header_end_s": ns_to_s(value[1]),
                    "record_start_s": ns_to_s(value[2]),
                    "record_end_s": ns_to_s(value[3]),
                }
            )
        return {
            "schema": "portable_ros1_bag_inventory/v1",
            "bag": str(path),
            "size_bytes": path.stat().st_size,
            "record_start_s": ns_to_s(reader.start_time),
            "record_end_s": ns_to_s(reader.end_time),
            "duration_s": ns_to_s(reader.duration),
            "message_count": int(reader.message_count),
            "relevant_topics": rows,
        }


def print_inventory(document: dict[str, Any]) -> None:
    print(f"bag: {document['bag']}")
    print(
        "record time: "
        f"{document['record_start_s']:.9f} .. {document['record_end_s']:.9f} "
        f"({document['duration_s']:.3f} s)"
    )
    print(
        f"size: {document['size_bytes'] / (1 << 20):.1f} MiB | "
        f"messages: {document['message_count']}"
    )
    print("\nrelevant topics:")
    for topic in RELEVANT_TOPICS:
        row = document["relevant_topics"].get(topic)
        if not row:
            print(f"  {topic:<50} MISSING")
            continue
        suffix = ""
        if "header_start_s" in row:
            suffix = (
                f" | header {row['header_start_s']:.6f}.."
                f"{row['header_end_s']:.6f}"
            )
        print(
            f"  {topic:<50} {row['messages']:>6}  {row['type']}{suffix}"
        )


def nearest_by_header(rows: Sequence[Any], query_ns: int) -> Any | None:
    if not rows:
        return None
    stamps = [row.header_ns for row in rows]
    index = bisect.bisect_left(stamps, query_ns)
    if index <= 0:
        return rows[0]
    if index >= len(rows):
        return rows[-1]
    before, after = rows[index - 1], rows[index]
    return before if query_ns - before.header_ns <= after.header_ns - query_ns else after


def nearest_plan(plans: Sequence[PlanRecord], query_ns: int) -> PlanRecord | None:
    if not plans:
        return None
    return min(plans, key=lambda row: abs(row.accept_ns - query_ns))


def nearest_by_record(rows: Sequence[Any], query_ns: int) -> Any | None:
    """Return the record-time-nearest row from a record-time-sorted sequence."""
    if not rows:
        return None
    stamps = [row.record_ns for row in rows]
    index = bisect.bisect_left(stamps, query_ns)
    if index <= 0:
        return rows[0]
    if index >= len(rows):
        return rows[-1]
    before, after = rows[index - 1], rows[index]
    return before if query_ns - before.record_ns <= after.record_ns - query_ns else after


def pair_audit_to_plan(
    audit: AuditRecord, plans: Sequence[PlanRecord]
) -> tuple[PlanRecord | None, str]:
    target = audit.controls[audit.best_idx]
    causal = [
        plan
        for plan in plans
        if plan.record_ns <= audit.record_ns + 5_000_000
        and audit.record_ns - plan.record_ns <= 600_000_000
        and np.max(np.abs(plan.control - target)) <= 1e-5
    ]
    if causal:
        plan = max(causal, key=lambda row: row.record_ns)
        lag_ms = (audit.record_ns - plan.record_ns) * 1e-6
        return plan, f"exact_control_latest_causal; audit_record_lag_ms={lag_ms:.3f}"
    same_control = [
        plan
        for plan in plans
        if abs(plan.record_ns - audit.record_ns) <= 1_000_000_000
        and np.max(np.abs(plan.control - target)) <= 1e-5
    ]
    if same_control:
        plan = min(same_control, key=lambda row: abs(row.record_ns - audit.record_ns))
        lag_ms = (audit.record_ns - plan.record_ns) * 1e-6
        return plan, f"exact_control_nearest_1s; audit_record_lag_ms={lag_ms:.3f}"
    return None, "unpaired_audit"


def nearest_audit_for_event(
    audits: Sequence[AuditRecord], plans: Sequence[PlanRecord], query_ns: int
) -> tuple[AuditRecord | None, PlanRecord | None, int | None, str]:
    """Find the audit whose paired accepted-plan time is closest to an event.

    Audit messages have no header and their bag record time lags the accepted
    plan. Matching by the selected control is therefore materially better than
    comparing the audit record timestamp directly.
    """
    candidates: list[tuple[int, AuditRecord, PlanRecord, str]] = []
    for audit in audits:
        plan, note = pair_audit_to_plan(audit, plans)
        if plan is not None:
            candidates.append((abs(plan.accept_ns - query_ns), audit, plan, note))
    if candidates:
        _, audit, plan, note = min(candidates, key=lambda row: row[0])
        return audit, plan, plan.accept_ns - query_ns, note
    audit = nearest_by_record(audits, query_ns)
    if audit is None:
        return None, None, None, "weight_audit_topic_missing"
    return (
        audit,
        None,
        audit.record_ns - query_ns,
        "unpaired_nearest_audit_record_time",
    )


def pose_rows(records: Sequence[PoseRecord]) -> Iterable[Sequence[Any]]:
    for row in records:
        yield (
            ns_to_s(row.header_ns),
            ns_to_s(row.record_ns),
            row.frame_id,
            row.child_frame_id,
            *row.xyz.tolist(),
            *row.quaternion_xyzw.tolist(),
        )


def write_csv(path: Path, columns: Sequence[str], rows: Iterable[Sequence[Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(rows)


POSE_COLUMNS = (
    "header_stamp_s",
    "bag_record_stamp_s",
    "frame_id",
    "child_frame_id",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
)


def write_plan_csv(path: Path, plan: PlanRecord) -> None:
    rows = []
    for index, point in enumerate(plan.message.points):
        transform = point.transforms[0]
        velocity = point.velocities[0] if point.velocities else None
        rows.append(
            (
                index,
                duration_s(point.time_from_start),
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
                velocity.linear.x if velocity is not None else math.nan,
                velocity.linear.y if velocity is not None else math.nan,
                velocity.linear.z if velocity is not None else math.nan,
                velocity.angular.z if velocity is not None else math.nan,
            )
        )
    write_csv(
        path,
        (
            "point_index",
            "time_from_start_s",
            "x_m",
            "y_m",
            "z_m",
            "qx",
            "qy",
            "qz",
            "qw",
            "target_vx_mps",
            "target_vy_mps",
            "target_vz_mps",
            "target_wz_radps",
        ),
        rows,
    )


def plan_document(plan: PlanRecord) -> dict[str, Any]:
    points = []
    for index, point in enumerate(plan.message.points):
        transform = point.transforms[0]
        velocity = point.velocities[0] if point.velocities else None
        points.append(
            {
                "point_index": index,
                "time_from_start_s": duration_s(point.time_from_start),
                "position_m": [
                    float(transform.translation.x),
                    float(transform.translation.y),
                    float(transform.translation.z),
                ],
                "orientation_xyzw": [
                    float(transform.rotation.x),
                    float(transform.rotation.y),
                    float(transform.rotation.z),
                    float(transform.rotation.w),
                ],
                "target_velocity": None
                if velocity is None
                else {
                    "linear_mps": [
                        float(velocity.linear.x),
                        float(velocity.linear.y),
                        float(velocity.linear.z),
                    ],
                    "angular_z_radps": float(velocity.angular.z),
                },
            }
        )
    return {
        "schema": "accepted_jax_trajectory/v1",
        "topic": PLAN_TOPIC,
        "bag_record_time_s": ns_to_s(plan.record_ns),
        "approx_accept_time_s": ns_to_s(plan.accept_ns),
        "scheduled_execution_time_s": ns_to_s(plan.header_ns),
        "historical_header_frame_id": plan.frame_id,
        "coordinate_interpretation": "odom (historical header incorrectly says map)",
        "replan_reason": plan.reason,
        "point_count": len(points),
        "points": points,
    }


def write_plan_json(path: Path, plan: PlanRecord) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(plan_document(plan), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def write_uncertainty_csv(path: Path, audit: AuditRecord) -> None:
    required = audit.required_margin()
    slack = audit.safety_slack()
    rows = []
    for role, candidate in (
        ("best", audit.best_idx),
        ("goal_only", audit.goal_only_idx),
    ):
        for step in range(audit.n_steps):
            rows.append(
                (
                    role,
                    candidate,
                    step + 1,
                    (step + 1) * 0.3,
                    audit.esdf_m[candidate, step],
                    audit.q[candidate, step, 0],
                    audit.q[candidate, step, 1],
                    audit.q[candidate, step, 2],
                    audit.q[candidate, step, 3],
                    audit.position_norm_m[candidate, step],
                    required[candidate, step],
                    slack[candidate, step],
                )
            )
    write_csv(
        path,
        (
            "candidate_role",
            "candidate_index",
            "horizon_step",
            "time_from_plan_start_s",
            "esdf_m",
            "q_x_m",
            "q_y_m",
            "q_z_m",
            "q_yaw_rad",
            "position_norm_m",
            "required_margin_m",
            "safety_slack_m",
        ),
        rows,
    )


MARKER_TYPE_NAMES = {
    0: "ARROW",
    1: "CUBE",
    2: "SPHERE",
    3: "CYLINDER",
    4: "LINE_STRIP",
    5: "LINE_LIST",
    6: "CUBE_LIST",
    7: "SPHERE_LIST",
    8: "POINTS",
    9: "TEXT_VIEW_FACING",
    10: "MESH_RESOURCE",
    11: "TRIANGLE_LIST",
}
MARKER_ACTION_NAMES = {0: "ADD_OR_MODIFY", 2: "DELETE", 3: "DELETEALL"}


def marker_to_dict(marker: Any) -> dict[str, Any]:
    return {
        "header_time_s": ns_to_s(stamp_to_ns(marker.header.stamp)),
        "frame_id": str(marker.header.frame_id),
        "namespace": str(marker.ns),
        "id": int(marker.id),
        "type": int(marker.type),
        "type_name": MARKER_TYPE_NAMES.get(int(marker.type), "UNKNOWN"),
        "action": int(marker.action),
        "action_name": MARKER_ACTION_NAMES.get(int(marker.action), "UNKNOWN"),
        "pose": {
            "position_m": [
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
            ],
            "orientation_xyzw": [
                float(marker.pose.orientation.x),
                float(marker.pose.orientation.y),
                float(marker.pose.orientation.z),
                float(marker.pose.orientation.w),
            ],
        },
        "scale": [float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)],
        "color_rgba": [
            float(marker.color.r),
            float(marker.color.g),
            float(marker.color.b),
            float(marker.color.a),
        ],
        "lifetime_s": duration_s(marker.lifetime),
        "frame_locked": bool(marker.frame_locked),
        "text": str(marker.text),
        "mesh_resource": str(marker.mesh_resource),
        "points_m": [[float(point.x), float(point.y), float(point.z)] for point in marker.points],
        "point_colors_rgba": [
            [float(color.r), float(color.g), float(color.b), float(color.a)]
            for color in marker.colors
        ],
    }


def write_marker_assets(prefix: Path, record: MarkerRecord) -> tuple[Path, Path]:
    """Write a lossless-enough JSON dump and a spreadsheet-friendly summary."""
    prefix.parent.mkdir(parents=True, exist_ok=True)
    # Event labels may legitimately contain a decimal point (for example
    # ``bag_12.203s``), so Path.with_suffix() would silently truncate them.
    json_path = Path(str(prefix) + ".json")
    csv_path = Path(str(prefix) + ".csv")
    markers = [marker_to_dict(marker) for marker in record.message.markers]
    document = {
        "schema": "rviz_uncertainty_marker_array/v1",
        "topic": UNCERTAINTY_MARKER_TOPIC,
        "bag_record_time_s": ns_to_s(record.record_ns),
        "marker_header_time_s": ns_to_s(record.header_ns),
        "marker_count": len(markers),
        "warning": (
            "This is the nearest cached/republished RViz MarkerArray, not an "
            "authoritative per-plan model inference snapshot."
        ),
        "markers": markers,
    }
    json_path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    rows = []
    for marker in markers:
        px, py, pz = marker["pose"]["position_m"]
        sx, sy, sz = marker["scale"]
        r, g, b, a = marker["color_rgba"]
        rows.append(
            (
                marker["header_time_s"],
                marker["frame_id"],
                marker["namespace"],
                marker["id"],
                marker["type_name"],
                marker["action_name"],
                px,
                py,
                pz,
                sx,
                sy,
                sz,
                r,
                g,
                b,
                a,
                len(marker["points_m"]),
                marker["text"],
            )
        )
    write_csv(
        csv_path,
        (
            "header_time_s",
            "frame_id",
            "namespace",
            "marker_id",
            "marker_type",
            "action",
            "pose_x_m",
            "pose_y_m",
            "pose_z_m",
            "scale_x_m",
            "scale_y_m",
            "scale_z_m",
            "color_r",
            "color_g",
            "color_b",
            "color_a",
            "point_count",
            "text",
        ),
        rows,
    )
    return json_path, csv_path


def write_topdown_svg(path: Path, marker_record: MarkerRecord, plan: PlanRecord | None) -> None:
    """Create a transparent, editable top-down approximation of the RViz layer."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    figure, axis = plt.subplots(figsize=(7.0, 6.0), constrained_layout=True)
    figure.patch.set_alpha(0.0)
    axis.patch.set_alpha(0.0)

    if plan is not None:
        xy = np.asarray(
            [
                [point.transforms[0].translation.x, point.transforms[0].translation.y]
                for point in plan.message.points
            ],
            dtype=np.float64,
        )
        if len(xy):
            axis.plot(xy[:, 0], xy[:, 1], color="black", lw=2.0, label="selected plan", zorder=5)
            axis.scatter(xy[:, 0], xy[:, 1], color="black", s=12, zorder=6)

    selected_namespaces = {
        "uncertainty_raw_best",
        "uncertainty_risk_best",
        "uncertainty_safety_boundary_best",
    }
    labels_seen: set[str] = set()
    for marker in marker_record.message.markers:
        if int(marker.action) != 0 or str(marker.ns) not in selected_namespaces:
            continue
        rgba = (
            float(marker.color.r),
            float(marker.color.g),
            float(marker.color.b),
            max(float(marker.color.a), 0.22),
        )
        label = str(marker.ns) if str(marker.ns) not in labels_seen else None
        labels_seen.add(str(marker.ns))
        marker_type = int(marker.type)
        if marker_type in (1, 2, 3):
            ellipse = Ellipse(
                (float(marker.pose.position.x), float(marker.pose.position.y)),
                width=max(float(marker.scale.x), 1e-6),
                height=max(float(marker.scale.y), 1e-6),
                facecolor=rgba if marker_type == 2 else "none",
                edgecolor=rgba,
                linewidth=1.2,
                label=label,
                zorder=3,
            )
            axis.add_patch(ellipse)
        elif marker_type in (4, 5) and marker.points:
            points = np.asarray([[point.x, point.y] for point in marker.points], dtype=np.float64)
            if marker_type == 4:
                axis.plot(points[:, 0], points[:, 1], color=rgba, lw=max(marker.scale.x, 0.8), label=label)
            else:
                for point_index in range(0, len(points) - 1, 2):
                    axis.plot(
                        points[point_index : point_index + 2, 0],
                        points[point_index : point_index + 2, 1],
                        color=rgba,
                        lw=max(marker.scale.x, 0.8),
                        label=label if point_index == 0 else None,
                    )

    axis.set_xlabel("odom x [m]")
    axis.set_ylabel("odom y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.18)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(loc="best", fontsize=8)
    axis.set_title("Nearest recorded uncertainty layer (top-down projection)")
    path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(path, format="svg", transparent=True)
    plt.close(figure)


def image_extension(image_format: str, payload: bytes) -> str:
    lower = image_format.lower()
    if "jpeg" in lower or payload[:2] == b"\xff\xd8":
        return ".jpg"
    if "png" in lower or payload[:8] == b"\x89PNG\r\n\x1a\n":
        return ".png"
    return ".bin"


def parse_progress(value: str | None) -> list[float]:
    if value is None or not value.strip():
        return []
    result = []
    for token in value.split(","):
        number = float(token.strip())
        if not 0.0 <= number <= 1.0:
            raise ValueError(f"progress must be in [0,1], got {number}")
        result.append(number)
    return result


def build_event_requests(args: argparse.Namespace, data: BagData) -> list[EventRequest]:
    events: list[EventRequest] = []
    if args.audit_events:
        for index, audit in enumerate(data.audits):
            plan, note = pair_audit_to_plan(audit, data.plans)
            event_ns = plan.accept_ns if plan is not None else audit.record_ns
            events.append(
                EventRequest(
                    label=f"audit_{index:02d}",
                    event_ns=event_ns,
                    source="weight_audit",
                    audit=audit,
                    plan=plan,
                    pairing_note=note,
                )
            )

    progress = parse_progress(args.progress)
    if progress:
        if not data.vio:
            raise RuntimeError("--progress requires tuned VIO in the bag")
        start_ns, end_ns = data.vio[0].header_ns, data.vio[-1].header_ns
        for fraction in progress:
            event_ns = int(round(start_ns + fraction * (end_ns - start_ns)))
            events.append(
                EventRequest(
                    label=f"progress_{int(round(100 * fraction)):03d}",
                    event_ns=event_ns,
                    source="vio_progress",
                    plan=nearest_plan(data.plans, event_ns),
                )
            )

    for index, seconds in enumerate(args.event or []):
        event_ns = data.bag_start_ns + int(round(float(seconds) * 1e9))
        events.append(
            EventRequest(
                label=f"bag_{float(seconds):.3f}s_{index}",
                event_ns=event_ns,
                source="bag_relative",
                plan=nearest_plan(data.plans, event_ns),
            )
        )
    for index, seconds in enumerate(args.event_ros or []):
        event_ns = int(round(float(seconds) * 1e9))
        events.append(
            EventRequest(
                label=f"ros_{float(seconds):.6f}_{index}",
                event_ns=event_ns,
                source="absolute_ros_time",
                plan=nearest_plan(data.plans, event_ns),
            )
        )

    if not events:
        args.progress = "0,0.25,0.5,0.75,1"
        return build_event_requests(args, data)

    # Keep different semantic requests, but remove accidental exact duplicates.
    unique: list[EventRequest] = []
    seen: set[tuple[str, int]] = set()
    for event in events:
        key = (event.source, event.event_ns)
        if key not in seen:
            seen.add(key)
            unique.append(event)
    return unique


def interpolate_xyz(records: Sequence[PoseRecord], query_s: np.ndarray) -> np.ndarray:
    times = np.asarray([ns_to_s(row.header_ns) for row in records])
    xyz = np.asarray([row.xyz for row in records])
    return np.column_stack(
        [np.interp(query_s, times, xyz[:, axis]) for axis in range(3)]
    )


def make_plots(
    output: Path, data: BagData, vio_time_offset_s: float
) -> dict[str, Any]:
    if len(data.gt) < 2 or len(data.vio) < 2:
        return {"created": False, "reason": "GT or tuned VIO has fewer than 2 poses"}
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gt_t = np.asarray([ns_to_s(row.header_ns) for row in data.gt])
    gt_xyz = np.asarray([row.xyz for row in data.gt])
    vio_raw_t = np.asarray([ns_to_s(row.header_ns) for row in data.vio])
    vio_t = vio_raw_t + float(vio_time_offset_s)
    vio_xyz = np.asarray([row.xyz for row in data.vio])
    overlap = (vio_t >= gt_t[0]) & (vio_t <= gt_t[-1])
    if int(overlap.sum()) < 2:
        return {"created": False, "reason": "GT/VIO timestamps do not overlap"}
    vio_t_eval = vio_t[overlap]
    vio_xyz_eval = vio_xyz[overlap]
    gt_eval = interpolate_xyz(data.gt, vio_t_eval)
    ape = np.linalg.norm(vio_xyz_eval - gt_eval, axis=1)
    rmse = float(np.sqrt(np.mean(ape * ape)))
    mean = float(np.mean(ape))
    maximum = float(np.max(ape))

    gt_window = (gt_t >= vio_t_eval[0]) & (gt_t <= vio_t_eval[-1])
    figure, axis = plt.subplots(figsize=(7.2, 6.2), constrained_layout=True)
    axis.plot(gt_xyz[gt_window, 0], gt_xyz[gt_window, 1], label="GT", lw=2.0)
    axis.plot(vio_xyz_eval[:, 0], vio_xyz_eval[:, 1], label="tuned VIO", lw=1.6)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend()
    axis.set_title("GT and tuned VIO (no spatial alignment)")
    figure.savefig(output / "gt_vio_xy.png", dpi=180)
    plt.close(figure)

    t0 = vio_t_eval[0]
    figure, axes = plt.subplots(3, 1, figsize=(9.0, 7.5), sharex=True, constrained_layout=True)
    for index, (axis, label) in enumerate(zip(axes, ("x", "y", "z"))):
        axis.plot(gt_t[gt_window] - t0, gt_xyz[gt_window, index], label="GT", lw=1.8)
        axis.plot(vio_t_eval - t0, vio_xyz_eval[:, index], label="tuned VIO", lw=1.3)
        axis.set_ylabel(f"{label} [m]")
        axis.grid(alpha=0.25)
    axes[0].legend()
    axes[-1].set_xlabel("time from tuned-VIO overlap start [s]")
    figure.savefig(output / "gt_vio_xyz_time.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9.0, 3.8), constrained_layout=True)
    axis.plot(vio_t_eval - t0, ape, color="#D62728", lw=1.5)
    axis.axhline(rmse, color="black", ls="--", lw=1.0, label=f"RMSE {rmse:.3f} m")
    axis.set_xlabel("time from tuned-VIO overlap start [s]")
    axis.set_ylabel("translation error [m]")
    axis.set_title("Tuned VIO vs interpolated GT (no spatial alignment)")
    axis.grid(alpha=0.25)
    axis.legend()
    figure.savefig(output / "gt_vio_ape.png", dpi=180)
    plt.close(figure)

    metrics = {
        "created": True,
        "vio_time_offset_s": float(vio_time_offset_s),
        "spatial_alignment": "none",
        "associations": int(len(ape)),
        "rmse_m": rmse,
        "mean_m": mean,
        "max_m": maximum,
    }
    (output / "ape_summary.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metrics


def export_assets(args: argparse.Namespace) -> int:
    bag = ensure_bag(args.bag)
    output = args.out.expanduser().resolve()
    if output.exists() and any(output.iterdir()) and not args.allow_nonempty:
        raise RuntimeError(
            f"output is non-empty: {output}; choose a new directory or pass --allow-nonempty"
        )
    output.mkdir(parents=True, exist_ok=True)

    print(f"reading {bag}", flush=True)
    data = load_bag(bag)
    if not data.rgb:
        raise RuntimeError(f"missing onboard RGB topic {RGB_TOPIC}")
    if not data.gt:
        raise RuntimeError(f"missing GT topic {GT_TOPIC}")
    if not data.vio:
        raise RuntimeError(
            f"missing tuned VIO topic {VIO_TOPIC}; use a synthesized bag from transfer_manifest.json"
        )

    write_csv(output / "gt.csv", POSE_COLUMNS, pose_rows(data.gt))
    write_csv(output / "tuned_vio.csv", POSE_COLUMNS, pose_rows(data.vio))

    events = build_event_requests(args, data)
    event_rows: list[dict[str, Any]] = []
    rgb_dir = output / "rgb"
    plan_dir = output / "plans"
    uncertainty_dir = output / "uncertainty"
    marker_dir = output / "rviz_uncertainty_markers"
    rgb_dir.mkdir(exist_ok=True)

    for index, event in enumerate(events):
        rgb = nearest_by_header(data.rgb, event.event_ns)
        gt = nearest_by_header(data.gt, event.event_ns)
        vio_query_ns = event.event_ns - int(round(args.vio_time_offset * 1e9))
        vio = nearest_by_header(data.vio, vio_query_ns)
        plan = event.plan or nearest_plan(data.plans, event.event_ns)
        marker_record = nearest_by_header(data.uncertainty_markers, event.event_ns)
        debug_record = nearest_by_record(data.debug, event.event_ns)
        nearest_audit, nearest_audit_plan, audit_dt_ns, nearest_audit_note = (
            nearest_audit_for_event(data.audits, data.plans, event.event_ns)
        )
        audit = event.audit or nearest_audit
        audit_plan = event.plan if event.audit is not None else nearest_audit_plan
        if event.audit is not None:
            audit_dt_ns = (
                audit_plan.accept_ns - event.event_ns
                if audit_plan is not None
                else event.audit.record_ns - event.event_ns
            )
            nearest_audit_note = event.pairing_note or nearest_audit_note
        token = f"event_{index:03d}_{safe_token(event.label)}"

        rgb_path = ""
        rgb_dt_ms = math.nan
        if rgb is not None:
            extension = image_extension(rgb.image_format, rgb.payload)
            image_path = rgb_dir / f"{token}{extension}"
            image_path.write_bytes(rgb.payload)
            rgb_path = image_path.relative_to(output).as_posix()
            rgb_dt_ms = (rgb.header_ns - event.event_ns) * 1e-6

        plan_path = ""
        plan_json_path = ""
        plan_dt_ms = math.nan
        if plan is not None:
            path = plan_dir / f"{token}_selected_plan.csv"
            write_plan_csv(path, plan)
            plan_path = path.relative_to(output).as_posix()
            json_path = plan_dir / f"{token}_selected_plan.json"
            write_plan_json(json_path, plan)
            plan_json_path = json_path.relative_to(output).as_posix()
            plan_dt_ms = (plan.accept_ns - event.event_ns) * 1e-6

        uncertainty_path = ""
        best_min_slack = math.nan
        goal_min_slack = math.nan
        if audit is not None:
            path = uncertainty_dir / f"{token}_uncertainty.csv"
            write_uncertainty_csv(path, audit)
            uncertainty_path = path.relative_to(output).as_posix()
            slack = audit.safety_slack()
            best_min_slack = float(np.min(slack[audit.best_idx]))
            goal_min_slack = float(np.min(slack[audit.goal_only_idx]))

        marker_json_path = ""
        marker_csv_path = ""
        marker_svg_path = ""
        marker_dt_ms = math.nan
        marker_count = 0
        if marker_record is not None:
            prefix = marker_dir / f"{token}_nearest_marker_array"
            marker_json, marker_csv = write_marker_assets(prefix, marker_record)
            marker_json_path = marker_json.relative_to(output).as_posix()
            marker_csv_path = marker_csv.relative_to(output).as_posix()
            marker_count = len(marker_record.message.markers)
            marker_dt_ms = (marker_record.header_ns - event.event_ns) * 1e-6
            if not args.no_plots:
                marker_svg = marker_dir / f"{token}_topdown.svg"
                write_topdown_svg(marker_svg, marker_record, plan)
                marker_svg_path = marker_svg.relative_to(output).as_posix()

        event_rows.append(
            {
                "event_index": index,
                "label": event.label,
                "source": event.source,
                "event_ros_time_s": ns_to_s(event.event_ns),
                "event_bag_relative_s": ns_to_s(event.event_ns - data.bag_start_ns),
                "rgb_path": rgb_path,
                "rgb_header_time_s": ns_to_s(rgb.header_ns) if rgb else math.nan,
                "rgb_dt_ms": rgb_dt_ms,
                "gt_x_m": gt.xyz[0] if gt else math.nan,
                "gt_y_m": gt.xyz[1] if gt else math.nan,
                "gt_z_m": gt.xyz[2] if gt else math.nan,
                "gt_dt_ms": (gt.header_ns - event.event_ns) * 1e-6 if gt else math.nan,
                "vio_x_m": vio.xyz[0] if vio else math.nan,
                "vio_y_m": vio.xyz[1] if vio else math.nan,
                "vio_z_m": vio.xyz[2] if vio else math.nan,
                "vio_shifted_dt_ms":
                    ((vio.header_ns + int(round(args.vio_time_offset * 1e9))) - event.event_ns) * 1e-6
                    if vio
                    else math.nan,
                "plan_path": plan_path,
                "plan_json_path": plan_json_path,
                "plan_accept_time_s": ns_to_s(plan.accept_ns) if plan else math.nan,
                "plan_execution_time_s": ns_to_s(plan.header_ns) if plan else math.nan,
                "plan_dt_ms": plan_dt_ms,
                "plan_reason": plan.reason if plan else "",
                "uncertainty_path": uncertainty_path,
                "weight_audit_nearest_dt_ms":
                    audit_dt_ns * 1e-6 if audit_dt_ns is not None else math.nan,
                "weight_audit_record_time_s":
                    ns_to_s(audit.record_ns) if audit is not None else math.nan,
                "weight_audit_pairing_note": nearest_audit_note,
                "best_idx": audit.best_idx if audit else "",
                "goal_only_idx": audit.goal_only_idx if audit else "",
                "active_n_safe": audit.active_n_safe if audit else "",
                "best_min_safety_slack_m": best_min_slack,
                "goal_only_min_safety_slack_m": goal_min_slack,
                "debug_available": debug_record is not None,
                "debug_record_time_s":
                    ns_to_s(debug_record.record_ns) if debug_record is not None else math.nan,
                "debug_record_dt_ms":
                    (debug_record.record_ns - event.event_ns) * 1e-6
                    if debug_record is not None
                    else math.nan,
                "debug_payload_length":
                    debug_record.payload_length if debug_record is not None else "",
                "debug_best_idx":
                    debug_record.best_idx if debug_record is not None else "",
                "marker_json_path": marker_json_path,
                "marker_csv_path": marker_csv_path,
                "marker_topdown_svg_path": marker_svg_path,
                "marker_header_time_s":
                    ns_to_s(marker_record.header_ns) if marker_record is not None else math.nan,
                "marker_dt_ms": marker_dt_ms,
                "marker_count": marker_count,
                "pairing_note": event.pairing_note,
            }
        )

    event_columns = list(event_rows[0].keys())
    write_csv(
        output / "events.csv",
        event_columns,
        ([row[column] for column in event_columns] for row in event_rows),
    )

    plot_summary = (
        {"created": False, "reason": "disabled by --no-plots"}
        if args.no_plots
        else make_plots(output, data, args.vio_time_offset)
    )
    manifest = {
        "schema": "portable_ros1_bag_export/v1",
        "input_bag": str(bag),
        "input_size_bytes": bag.stat().st_size,
        "record_start_s": ns_to_s(data.bag_start_ns),
        "record_end_s": ns_to_s(data.bag_end_ns),
        "topics": {
            "gt": GT_TOPIC,
            "tuned_vio": VIO_TOPIC,
            "onboard_rgb": RGB_TOPIC,
            "accepted_plan": PLAN_TOPIC,
            "weight_audit": AUDIT_TOPIC,
            "debug_info": DEBUG_TOPIC,
            "recorded_uncertainty_marker_array": UNCERTAINTY_MARKER_TOPIC,
        },
        "counts": {
            "gt": len(data.gt),
            "tuned_vio": len(data.vio),
            "onboard_rgb": len(data.rgb),
            "accepted_plans": len(data.plans),
            "weight_audit_snapshots": len(data.audits),
            "debug_snapshots": len(data.debug),
            "uncertainty_marker_arrays": len(data.uncertainty_markers),
            "exported_events": len(events),
        },
        "vio_time_offset_s": float(args.vio_time_offset),
        "plot_summary": plot_summary,
        "caveats": [
            "No spatial alignment is applied.",
            "The tuned VIO trajectory is post-hoc and exists only over its replay interval.",
            "Weight-audit snapshots were recorded at 0.5 Hz, not for every accepted plan.",
            "Debug snapshots were recorded at 1 Hz and have no generation key.",
            "Audit Q already includes the online dead-zone scale and active safety mode.",
            "The uncertainty MarkerArray is a cached/republished RViz layer; nearest-time exports are not authoritative per-plan inference snapshots.",
            "Top-down SVGs are geometric projections, not captures of the historical RViz camera.",
            "Plan acceptance time is approximated as trajectory header stamp minus 0.045 s.",
            "The planner map/input acquisition timestamp was not recorded.",
            "The historical plan header says 'map', but its numeric transforms were used in odom coordinates.",
        ],
    }
    (output / "export_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"wrote {output} | events={len(events)} | RGB={len(data.rgb)} | "
        f"GT={len(data.gt)} | tuned VIO={len(data.vio)}"
    )
    return 0


def command_inspect(args: argparse.Namespace) -> int:
    bag = ensure_bag(args.bag)
    document = connection_inventory(bag)
    print_inventory(document)
    if args.json:
        destination = args.json.expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"\nwrote {destination}")
    missing = [topic for topic in (GT_TOPIC, VIO_TOPIC, RGB_TOPIC) if topic not in document["relevant_topics"]]
    if missing:
        print(f"\nwarning: missing required export topics: {', '.join(missing)}", file=sys.stderr)
        return 2
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser(
        "inspect", help="list relevant topics, message counts, and time ranges"
    )
    inspect_parser.add_argument("bag", type=Path)
    inspect_parser.add_argument("--json", type=Path, help="also write the inventory as JSON")
    inspect_parser.set_defaults(func=command_inspect)

    export_parser = subparsers.add_parser(
        "export", help="export RGB event frames, poses, uncertainty summaries, and plots"
    )
    export_parser.add_argument("bag", type=Path)
    export_parser.add_argument("--out", type=Path, required=True)
    export_parser.add_argument(
        "--audit-events",
        action="store_true",
        help="export every recorded 0.5 Hz weight-audit snapshot and its paired plan",
    )
    export_parser.add_argument(
        "--progress",
        default=None,
        help="comma-separated tuned-VIO progress fractions, e.g. 0,.25,.5,.75,1",
    )
    export_parser.add_argument(
        "--event",
        action="append",
        type=float,
        help="event time in seconds relative to the bag record start; repeatable",
    )
    export_parser.add_argument(
        "--event-ros",
        action="append",
        type=float,
        help="absolute ROS event time in seconds; repeatable",
    )
    export_parser.add_argument(
        "--vio-time-offset",
        type=float,
        default=0.0,
        help="seconds added to tuned-VIO header stamps for GT association/plots",
    )
    export_parser.add_argument("--no-plots", action="store_true")
    export_parser.add_argument(
        "--allow-nonempty",
        action="store_true",
        help="allow writing into a non-empty output directory (known filenames may be replaced)",
    )
    export_parser.set_defaults(func=export_assets)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        parser.exit(2, f"error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())

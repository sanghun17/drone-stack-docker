#!/usr/bin/env python3
"""Build one deterministic post-fix qualification receipt from an attempt.

This is a read-only adapter: it validates the attempt manifest and its hashed
node log/result bag/evaluation/parameter artifacts, then canonicalizes only
sensor-stamped output data.  It never launches ROS or a replay.  A small,
self-hashed execution receipt supplied by the orchestrator binds the attempt
to one of the immutable 12 plan rows and attests the fresh process UUID.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import rosbag
import yaml

from check_vio_postfix_init_qualification import (
    CANONICAL_SCHEMA,
    PLAN_SCHEMA,
    RECEIPT_SCHEMA,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    QUALIFICATION_BINDING_SCHEMA,
    RUN_SCHEMA,
    SCHEMA as CAMPAIGN_SCHEMA,
    binary_identity,
    estimator_source_identity,
    git_source_identity,
    load_json,
    object_sha256,
    sha256,
    validate_completion,
    validate_input_receipt,
    validate_plan_identity,
)


EXECUTION_SCHEMA = "fastlivo_vio_postfix_init_qualification_execution/v1"
BUILD_SCHEMA = "fastlivo_vio_postfix_build_manifest/v1"
ORCHESTRATION_SCHEMA = "fastlivo_vio_postfix_init_orchestration/v1"
DIAG_SCHEMA = "fast_livo/imu_init/v1"
STATE_FINGERPRINT_SCHEMA = "fast_livo/initial_state_ieee754_be/v1"
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
DIAG_MARKER = "[imu_init_diag]"
TOPICS = {
    "low_rate_pose": (
        "/aft_mapped_to_body", "geometry_msgs/PoseStamped"),
    "low_rate_init": (
        "/aft_mapped_to_init", "nav_msgs/Odometry"),
    "correction": (
        "/aft_mapped_to_body_correction_pose_cov",
        "geometry_msgs/PoseWithCovarianceStamped"),
    "propagated_odom": (
        "/aft_mapped_to_body_imu_propagated", "nav_msgs/Odometry"),
    "world_twist": (
        "/aft_mapped_to_body_imu_propagated_world_twist",
        "geometry_msgs/TwistStamped"),
}
ARTIFACTS = (
    "result_node.log", "result.bag", "result.flight_readiness.json",
    "result_params.yaml",
)
REVIEWED_INIT_FILES = (
    "include/IMU_Processing.h", "include/LIVMapper.h",
    "include/imu_init_buffer.h", "src/IMU_Processing.cpp",
    "src/LIVMapper.cpp",
)


def _valid_sha(value: Any) -> bool:
    return isinstance(value, str) and SHA_RE.fullmatch(value) is not None


def _self_hash(document: Mapping[str, Any], schema: str, label: str) -> str:
    if document.get("schema") != schema:
        raise CampaignError(f"{label}: wrong schema")
    declared = document.get("identity_sha256")
    if not _valid_sha(declared):
        raise CampaignError(f"{label}: missing identity SHA-256")
    core = dict(document)
    core.pop("identity_sha256", None)
    if object_sha256(core) != declared:
        raise CampaignError(f"{label}: self hash changed")
    return str(declared)


def _positive_ns(value: Any, label: str) -> int:
    if (not isinstance(value, str) or not value.isdigit() or
            value.startswith("0")):
        raise CampaignError(f"{label} must be a quoted positive decimal ns")
    parsed = int(value)
    if parsed <= 0 or parsed > (1 << 64) - 1:
        raise CampaignError(f"{label} is outside uint64")
    return parsed


def _nested(document: Mapping[str, Any], dotted: str) -> Any:
    value: Any = document
    for key in dotted.split("."):
        if not isinstance(value, Mapping) or key not in value:
            raise CampaignError(f"parameter snapshot lacks {dotted}")
        value = value[key]
    return value


def _verify_file_identity(identity: Any, label: str) -> None:
    if not isinstance(identity, Mapping):
        raise CampaignError(f"{label}: missing file identity")
    path = Path(str(identity.get("path", ""))).resolve()
    if (not path.is_file() or path.stat().st_size != identity.get("size_bytes") or
            sha256(path) != identity.get("sha256")):
        raise CampaignError(f"{label}: dependency identity changed: {path}")


def _reviewed_files(source_tree: Mapping[str, Any]) -> Dict[str, Any]:
    files = source_tree.get("files")
    if not isinstance(files, Mapping):
        raise CampaignError("build source tree has no file inventory")
    result = {}
    for relative in REVIEWED_INIT_FILES:
        identity = files.get(relative)
        if (not isinstance(identity, Mapping) or
                not _valid_sha(identity.get("sha256")) or
                not isinstance(identity.get("size_bytes"), int)):
            raise CampaignError(
                f"build source tree lacks reviewed init file {relative}")
        result[relative] = dict(identity)
    return result


def validate_build_manifest(
        build: Mapping[str, Any], *, verify_actual: bool = True) -> str:
    """Validate derived fields and, by default, re-probe build/source inputs."""
    identity = _self_hash(build, BUILD_SCHEMA, "post-fix build")
    binary = build.get("binary_identity")
    source = build.get("source_tree_identity")
    git = build.get("git_source_identity")
    if not all(isinstance(value, Mapping) for value in (binary, source, git)):
        raise CampaignError("post-fix build lacks binary/source/git identity")
    libraries = binary.get("dynamic_libraries")
    flattened = {
        str(name): row.get("sha256")
        for name, row in libraries.items()
        if isinstance(row, Mapping)
    } if isinstance(libraries, Mapping) else {}
    reviewed = _reviewed_files(source)
    if (build.get("container") != binary.get("container") or
            build.get("replay_devel") != binary.get("replay_devel") or
            build.get("executable_sha256") != binary.get("executable_sha256") or
            build.get("dynamic_libraries") != flattened or
            build.get("source_tree_sha256") != source.get("tree_sha256") or
            build.get("reviewed_init_anchor_files") != reviewed or
            build.get("reviewed_init_anchor_patch_sha256") !=
            object_sha256(reviewed)):
        raise CampaignError("post-fix build derived provenance is inconsistent")
    if verify_actual:
        actual_binary = binary_identity(
            str(build["container"]), str(build["replay_devel"]))
        actual_source = estimator_source_identity(
            Path(str(source.get("root", ""))))
        actual_git = git_source_identity(
            Path(str(source.get("root", ""))))
        if (actual_binary != binary or actual_source != source or
                actual_git != git):
            raise CampaignError(
                "post-fix build/source no longer matches actual isolated inputs")
    return identity


def _finite(values: Iterable[Any], label: str) -> List[float]:
    result = []
    for value in values:
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(float(value))):
            raise CampaignError(f"non-finite canonical value in {label}")
        result.append(float(value))
    return result


def _canonical_quaternion(quaternion: Any, label: str) -> List[float]:
    values = _finite(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w), label)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise CampaignError(f"zero quaternion in {label}")
    # Keep estimator binary64 magnitudes unchanged.  Only remove q/-q
    # ambiguity, preferring positive w and lexicographic xyz when w is zero.
    decision = [values[3], values[0], values[1], values[2]]
    first_nonzero = next((value for value in decision if value != 0.0), 1.0)
    if first_nonzero < 0.0:
        values = [-value for value in values]
    return [0.0 if value == 0.0 else value for value in values]


def _text(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack(">I", len(encoded)) + encoded


def _float_bytes(values: Iterable[float]) -> bytes:
    values = list(values)
    return struct.pack(">" + "d" * len(values), *values)


def _pose_values(pose: Any, label: str) -> List[float]:
    position = _finite(
        (pose.position.x, pose.position.y, pose.position.z), label)
    return position + _canonical_quaternion(pose.orientation, label)


def _canonical_message(name: str, topic: str, message: Any) -> Tuple[int, bytes]:
    header = getattr(message, "header", None)
    if header is None:
        raise CampaignError(f"{topic}: message has no sensor header")
    stamp_ns = int(header.stamp.to_nsec())
    if stamp_ns <= 0:
        raise CampaignError(f"{topic}: missing positive sensor stamp")
    values: List[float]
    child = str(getattr(message, "child_frame_id", ""))
    if name == "low_rate_pose":
        values = _pose_values(message.pose, topic)
    elif name == "low_rate_init":
        values = _pose_values(message.pose.pose, topic)
    elif name == "correction":
        values = (_pose_values(message.pose.pose, topic) +
                  _finite(message.pose.covariance, topic))
    elif name == "propagated_odom":
        values = (_pose_values(message.pose.pose, topic) +
                  _finite(message.pose.covariance, topic) +
                  _finite((message.twist.twist.linear.x,
                           message.twist.twist.linear.y,
                           message.twist.twist.linear.z,
                           message.twist.twist.angular.x,
                           message.twist.twist.angular.y,
                           message.twist.twist.angular.z), topic) +
                  _finite(message.twist.covariance, topic))
    elif name == "world_twist":
        values = _finite((message.twist.linear.x, message.twist.linear.y,
                          message.twist.linear.z, message.twist.angular.x,
                          message.twist.angular.y, message.twist.angular.z),
                         topic)
    else:
        raise CampaignError(f"unsupported canonical stream {name}")
    payload = (
        _text("fastlivo_sensor_stamped_message_binary64_be/v1") +
        _text(name) + _text(topic) + struct.pack(">QI", stamp_ns,
                                                   int(header.seq)) +
        _text(str(header.frame_id)) + _text(child) + _float_bytes(values)
    )
    return stamp_ns, payload


def canonicalize_result_bag(path: Path) -> Dict[str, Dict[str, Any]]:
    records: Dict[str, List[Tuple[int, bytes]]] = {
        name: [] for name in TOPICS
    }
    topic_to_name = {topic: name for name, (topic, _) in TOPICS.items()}
    with rosbag.Bag(str(path), "r") as bag:
        inventory = bag.get_type_and_topic_info().topics
        for name, (topic, expected_type) in TOPICS.items():
            info = inventory.get(topic)
            if info is None or info.msg_type != expected_type:
                raise CampaignError(
                    f"result bag {topic} type mismatch: "
                    f"{None if info is None else info.msg_type!r}")
        for topic, message, _ in bag.read_messages(
                topics=[value[0] for value in TOPICS.values()]):
            name = topic_to_name[topic]
            records[name].append(_canonical_message(name, topic, message))
    result: Dict[str, Dict[str, Any]] = {}
    for name, entries in records.items():
        if not entries:
            raise CampaignError(f"result bag has empty {name} stream")
        stamps = [stamp for stamp, _ in entries]
        monotonic = all(right >= left for left, right in zip(stamps, stamps[1:]))
        if not monotonic:
            raise CampaignError(f"result bag has backward {name} sensor stamps")
        stamp_text = "".join(f"{stamp}\n" for stamp in stamps).encode("ascii")
        payload_hasher = hashlib.sha256()
        for _, payload in entries:
            payload_hasher.update(struct.pack(">Q", len(payload)))
            payload_hasher.update(payload)
        result[name] = {
            "message_count": len(entries),
            "sensor_stamp_vector_sha256": hashlib.sha256(stamp_text).hexdigest(),
            "canonical_state_sha256": payload_hasher.hexdigest(),
            "first_sensor_stamp_ns": str(stamps[0]),
            "last_sensor_stamp_ns": str(stamps[-1]),
            "first_message_binary64_be_sha256": hashlib.sha256(
                entries[0][1]).hexdigest(),
            "all_values_finite": True,
            "sensor_stamps_monotonic_non_decreasing": True,
        }
    return result


def _diagnostic_event(text: str, source: str, ordinal: int) -> Optional[Dict[str, Any]]:
    """Parse one structured init diagnostic embedded in a ROS log line."""
    marker = text.find(DIAG_MARKER)
    if marker < 0:
        return None
    start = text.find("{", marker + len(DIAG_MARKER))
    end = text.rfind("}")
    if start < 0 or end < start:
        raise CampaignError(
            f"malformed init diagnostic in {source} at record {ordinal}")
    try:
        event = json.loads(text[start:end + 1])
    except json.JSONDecodeError as error:
        raise CampaignError(
            f"invalid init diagnostic JSON in {source} at record "
            f"{ordinal}: {error}") from error
    if event.get("schema") != DIAG_SCHEMA:
        raise CampaignError(
            f"wrong init diagnostic schema in {source} at record {ordinal}")
    event["_source"] = source
    event["_ordinal"] = ordinal
    return event


def _diagnostics(path: Path) -> Dict[str, List[Dict[str, Any]]]:
    result: Dict[str, List[Dict[str, Any]]] = {}
    for line_number, line in enumerate(path.read_text(errors="strict").splitlines(), 1):
        event = _diagnostic_event(line, str(path), line_number)
        if event is None:
            continue
        result.setdefault(str(event.get("status", "")), []).append(event)
    if result.get("failed"):
        raise CampaignError("node log contains failed IMU initialization")
    return result


def _diagnostics_from_rosout(path: Path) -> Dict[str, List[Dict[str, Any]]]:
    """Read structured diagnostics from the recorder-flushed /rosout topic."""
    result: Dict[str, List[Dict[str, Any]]] = {}
    with rosbag.Bag(str(path), "r") as bag:
        inventory = bag.get_type_and_topic_info().topics
        info = inventory.get("/rosout")
        if info is None:
            return result
        if info.msg_type != "rosgraph_msgs/Log":
            raise CampaignError(
                f"result bag /rosout type mismatch: {info.msg_type!r}")
        for ordinal, (_, message, _) in enumerate(
                bag.read_messages(topics=["/rosout"]), 1):
            event = _diagnostic_event(
                str(message.msg), f"{path}:/rosout", ordinal)
            if event is not None:
                result.setdefault(str(event.get("status", "")), []).append(event)
    return result


def _diagnostic_evidence(node_log: Path, result_bag: Path
                         ) -> Dict[str, List[Dict[str, Any]]]:
    """Merge screen and /rosout evidence, deduplicating identical events.

    Startup configuration is emitted before recording begins and therefore
    comes from the screen log.  Later timer/destructor events are guaranteed
    by the finalized rosbag.  Identical events seen through both paths count
    once; conflicting events remain distinct and fail the uniqueness gates.
    """
    merged: Dict[str, List[Dict[str, Any]]] = {}
    seen = set()
    for source_rows in (_diagnostics(node_log),
                        _diagnostics_from_rosout(result_bag)):
        for status, rows in source_rows.items():
            for event in rows:
                canonical = {
                    key: value for key, value in event.items()
                    if not key.startswith("_")
                }
                identity = object_sha256(canonical)
                if identity in seen:
                    continue
                seen.add(identity)
                merged.setdefault(status, []).append(event)
    if merged.get("failed"):
        raise CampaignError("diagnostic evidence contains failed IMU initialization")
    return merged


def _unique_event(events: Mapping[str, List[Dict[str, Any]]],
                  status: str) -> Dict[str, Any]:
    rows = events.get(status, [])
    if len(rows) != 1:
        raise CampaignError(f"node log requires exactly one {status} event")
    return rows[0]


def _load_orchestration(
        execution: Mapping[str, Any], plan: Mapping[str, Any],
        build_identity: str, run_id: str) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    plan_identity = _self_hash(plan, PLAN_SCHEMA, "qualification plan")
    path = Path(str(execution.get("orchestration_path", ""))).resolve()
    if (not path.is_file() or sha256(path) !=
            execution.get("orchestration_file_sha256")):
        raise CampaignError("execution orchestration file identity changed")
    orchestration = load_json(path)
    identity = _self_hash(
        orchestration, ORCHESTRATION_SCHEMA, "qualification orchestration")
    if (execution.get("orchestration_identity_sha256") != identity or
            orchestration.get("qualification_plan_identity_sha256") !=
            plan_identity or
            orchestration.get("build_manifest_identity_sha256") !=
            build_identity or
            orchestration.get("scope") != "development_only" or
            orchestration.get("validation_data_accessed") is not False or
            orchestration.get("replay_executed_by_generator") is not False):
        raise CampaignError("qualification orchestration provenance mismatch")
    dependencies = orchestration.get("dependencies")
    if not isinstance(dependencies, Mapping) or not dependencies:
        raise CampaignError("qualification orchestration lacks dependencies")
    for label, dependency in dependencies.items():
        _verify_file_identity(dependency, f"orchestration dependency {label}")
    plan_dependency = dependencies.get("qualification_plan", {})
    config_dependency = dependencies.get("qualification_config", {})
    anchors_dependency = dependencies.get("anchors_artifact", {})
    if (load_json(Path(str(plan_dependency.get("path", "")))).get(
            "identity_sha256") != plan_identity or
            config_dependency.get("sha256") !=
            plan.get("config_identity", {}).get("sha256") or
            anchors_dependency.get("sha256") !=
            plan.get("anchors_identity", {}).get("sha256") or
            load_json(Path(str(anchors_dependency.get("path", "")))).get(
                "identity_sha256") !=
            plan.get("anchors_artifact_identity_sha256")):
        raise CampaignError("qualification toolchain/plan provenance mismatch")
    all_rows = orchestration.get("runs")
    expected_runs = plan.get("runs")
    if (orchestration.get("expected_run_count") != 12 or
            not isinstance(all_rows, list) or len(all_rows) != 12 or
            not isinstance(expected_runs, list) or len(expected_runs) != 12 or
            {row.get("run_id") for row in all_rows if isinstance(row, Mapping)} !=
            {row.get("run_id") for row in expected_runs
             if isinstance(row, Mapping)}):
        raise CampaignError("orchestration is not the exact 12-run plan grid")
    rows = [row for row in all_rows
            if isinstance(row, Mapping) and row.get("run_id") == run_id]
    if len(rows) != 1:
        raise CampaignError("orchestration has no unique qualification run")
    return orchestration, dict(rows[0])


def _validate_campaign_attempt(
        campaign_dir: Path, run: Mapping[str, Any],
        sentinel: Mapping[str, Any], orchestration: Mapping[str, Any],
        run_spec: Mapping[str, Any], build: Mapping[str, Any],
        build_identity: str, process_uuid: str
        ) -> Tuple[Dict[str, Any], Path, Dict[str, Any], Dict[str, Path]]:
    campaign_dir = campaign_dir.resolve()
    if campaign_dir != Path(str(run_spec.get("campaign_dir", ""))).resolve():
        raise CampaignError("campaign directory differs from orchestration")
    campaign_path = campaign_dir / "campaign.json"
    campaign = load_json(campaign_path)
    campaign_identity = validate_plan_identity(campaign)
    if (campaign.get("schema") != CAMPAIGN_SCHEMA or
            campaign_identity != run_spec.get(
                "expected_campaign_identity_sha256") or
            campaign.get("campaign_id") != run_spec.get("campaign_id") or
            campaign.get("mode") != "full" or
            campaign.get("single_worker") is not True or
            campaign.get("build") != build.get("binary_identity") or
            campaign.get("replay") != {
                "rate": float(run["rate"]), "no_gt_anchor": True,
                "with_propagated": True,
                "fixed_zero_time_offset_evaluator": True,
                "ros_master_port": int(run_spec.get("port", -1)),
            }):
        raise CampaignError("campaign plan does not match qualification run")
    dependencies = campaign.get("dependencies")
    if not isinstance(dependencies, Mapping):
        raise CampaignError("campaign plan lacks dependencies")
    expected_file_dependencies = {
        "harness": orchestration["dependencies"]["campaign_harness"],
        "replay_wrapper": orchestration["dependencies"]["replay_wrapper"],
        "strict_evaluator": orchestration["dependencies"]["strict_evaluator"],
        "thresholds": orchestration["dependencies"]["thresholds"],
        "session_spec": orchestration["dependencies"]["session_spec"],
        "base_overlay": orchestration["dependencies"]["base_overlay"],
        "arms": run_spec["arm_yaml"],
        "fastlivo_base_config": orchestration[
            "dependencies"]["fastlivo_base_config"],
        "replay_launch": orchestration["dependencies"]["replay_launch"],
    }
    if (set(dependencies) != set(expected_file_dependencies) |
            {"fastlivo_source_tree", "fastlivo_git"}):
        raise CampaignError("campaign dependency inventory differs")
    for label, expected in expected_file_dependencies.items():
        if dependencies.get(label) != expected:
            raise CampaignError(f"campaign dependency differs: {label}")
        _verify_file_identity(expected, f"campaign dependency {label}")
    if (dependencies.get("fastlivo_source_tree") !=
            build.get("source_tree_identity") or
            dependencies.get("fastlivo_git") !=
            build.get("git_source_identity")):
        raise CampaignError("campaign source/build provenance differs")
    expected_arm = [{
        "id": run["arm_id"],
        "overrides": sentinel["runtime_overrides"],
        "effective_overlay_sha256": run_spec[
            "effective_overlay_sha256"],
    }]
    sessions = campaign.get("sessions")
    if campaign.get("arms") != expected_arm or not isinstance(sessions, list) or len(
            sessions) != 1:
        raise CampaignError("campaign arm/session grid is not exactly one cell")
    session = sessions[0]
    expected_session = {
        "id": run["session_id"], "split": "development",
        "input_bag": sentinel["input_bag"],
        "input_declared_sha256": sentinel["input_declared_sha256"],
        "input_provenance_sha256": sentinel["input_provenance_sha256"],
        "crop": sentinel["crop"],
    }
    for field, expected in expected_session.items():
        if session.get(field) != expected:
            raise CampaignError(f"campaign session differs: {field}")
    validate_input_receipt(
        campaign_dir / "inputs" / f"{run['session_id']}.json", session)
    binding_identity = run_spec.get("run_binding")
    _verify_file_identity(binding_identity, "qualification run binding")
    binding = load_json(Path(str(binding_identity["path"])))
    _self_hash(binding, QUALIFICATION_BINDING_SCHEMA,
               "qualification run binding")
    expected_run_spec = {
        "run_id": run["run_id"],
        "sentinel_id": run["sentinel_id"],
        "arm_id": run["arm_id"],
        "session_id": run["session_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "fresh_process_uuid": process_uuid,
        "input_bag": sentinel["input_bag"],
        "input_declared_sha256": sentinel["input_declared_sha256"],
        "input_provenance_sha256": sentinel["input_provenance_sha256"],
        "crop": sentinel["crop"],
        "runtime_overrides": sentinel["runtime_overrides"],
        "runtime_overrides_sha256": object_sha256(
            sentinel["runtime_overrides"]),
        "expected_campaign_identity_sha256": campaign_identity,
        "campaign_id": run_spec["campaign_id"],
        "port": campaign["replay"]["ros_master_port"],
        "attempt_id": run_spec["attempt_id"],
        "effective_overlay_sha256": run_spec[
            "effective_overlay_sha256"],
        "replay_command": run_spec["replay_command"],
        "evaluator_command": run_spec["evaluator_command"],
    }
    for field, expected in expected_run_spec.items():
        if run_spec.get(field) != expected:
            raise CampaignError(f"orchestration run binding mismatch: {field}")
    expected_binding = {
        "arm_id": run["arm_id"],
        "session_id": run["session_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "input_bag": sentinel["input_bag"],
        "input_declared_sha256": sentinel["input_declared_sha256"],
        "input_provenance_sha256": sentinel["input_provenance_sha256"],
        "crop": sentinel["crop"],
        "runtime_overrides": sentinel["runtime_overrides"],
        "runtime_overrides_sha256": object_sha256(
            sentinel["runtime_overrides"]),
        "expected_campaign_identity_sha256": campaign_identity,
        "campaign_id": run_spec["campaign_id"],
        "process_instance_uuid": process_uuid,
        "build_manifest_identity_sha256": build_identity,
        "qualification_plan_identity_sha256": orchestration[
            "qualification_plan_identity_sha256"],
        "qualification_run_id": run["run_id"],
        "attempt_id": run_spec["attempt_id"],
        "binary_identity_sha256": object_sha256(build["binary_identity"]),
        "effective_overlay_sha256": run_spec[
            "effective_overlay_sha256"],
        "replay_command": run_spec["replay_command"],
        "evaluator_command": run_spec["evaluator_command"],
    }
    for field, expected in expected_binding.items():
        if binding.get(field) != expected:
            raise CampaignError(f"qualification run binding mismatch: {field}")
    pointer = campaign_dir / "completed" / str(run["arm_id"]) / (
        str(run["session_id"]) + ".json")
    attempt = validate_completion(
        campaign_dir, pointer, campaign_identity, str(run["arm_id"]),
        str(run["session_id"]))
    expected_attempt = (campaign_dir / "attempts" / str(run["arm_id"]) /
                        str(run["session_id"]) /
                        str(run_spec["attempt_id"])).resolve()
    if attempt != expected_attempt:
        raise CampaignError("completion points at unexpected qualification attempt")
    manifest_path = attempt / "manifest.json"
    manifest = load_json(manifest_path)
    if (manifest.get("schema") != RUN_SCHEMA or
            manifest.get("state") != "complete" or
            manifest.get("campaign_identity_sha256") != campaign_identity or
            manifest.get("arm_id") != run["arm_id"] or
            manifest.get("session_id") != run["session_id"] or
            float(manifest.get("rate", math.nan)) != float(run["rate"]) or
            manifest.get("input_bag") != sentinel["input_bag"] or
            manifest.get("input_sha256") != sentinel["input_declared_sha256"] or
            manifest.get("crop") != sentinel["crop"] or
            manifest.get("replay_flags") !=
            ["--no-gt-anchor", "--with-propagated"] or
            manifest.get("replay_command") != run_spec["replay_command"] or
            manifest.get("evaluator_command") !=
            run_spec["evaluator_command"] or
            manifest.get("qualification_run_binding") != binding):
        raise CampaignError("attempt manifest does not match qualification run")
    identities = manifest.get("artifacts")
    if not isinstance(identities, Mapping):
        raise CampaignError("attempt manifest has no artifact identities")
    paths: Dict[str, Path] = {}
    for name in ARTIFACTS:
        path = attempt / name
        identity = identities.get(name)
        if (not isinstance(identity, Mapping) or not path.is_file() or
                path.stat().st_size != identity.get("size_bytes") or
                sha256(path) != identity.get("sha256")):
            raise CampaignError(f"attempt artifact identity mismatch: {name}")
        paths[name] = path
    return campaign, attempt, manifest, paths


def _local_objective(report: Mapping[str, Any]) -> float:
    local = report.get("local")
    if not isinstance(local, Mapping):
        raise CampaignError("evaluation lacks local metrics")
    required = {
        "translation_ape_rmse_m": 0.25,
        "translation_rpe_1p0s_rmse_m": 0.10,
        "orientation_rmse_deg": 5.0,
    }
    scores = []
    for field, denominator in required.items():
        value = _finite((local.get(field),), f"evaluation local.{field}")[0]
        scores.append(value / denominator)
    path_ratio = _finite((local.get("path_ratio"),),
                         "evaluation local.path_ratio")[0]
    scores.append(abs(path_ratio - 1.0) / 0.10)
    return max(scores)


def build_receipt(plan: Mapping[str, Any], run_id: str, attempt: Path,
                  execution: Mapping[str, Any],
                  build: Mapping[str, Any], *,
                  verify_actual_build: bool = True) -> Dict[str, Any]:
    plan_identity = _self_hash(plan, PLAN_SCHEMA, "qualification plan")
    rows = [row for row in plan.get("runs", [])
            if isinstance(row, Mapping) and row.get("run_id") == run_id]
    if len(rows) != 1:
        raise CampaignError(f"qualification plan has no unique run {run_id!r}")
    run = rows[0]
    build_identity = validate_build_manifest(
        build, verify_actual=verify_actual_build)
    execution_identity = _self_hash(
        execution, EXECUTION_SCHEMA, "qualification execution")
    process_uuid = execution.get("process_instance_uuid")
    if not isinstance(process_uuid, str) or not process_uuid:
        raise CampaignError("execution receipt lacks process instance UUID")
    orchestration, run_spec = _load_orchestration(
        execution, plan, build_identity, run_id)
    sentinel = next(
        row for row in plan["sentinels"] if row["id"] == run["sentinel_id"])
    campaign, validated_attempt, manifest, paths = _validate_campaign_attempt(
        Path(str(execution.get("campaign_dir", ""))), run, sentinel,
        orchestration, run_spec, build, build_identity, process_uuid)
    if attempt.resolve() != validated_attempt:
        raise CampaignError("receipt attempt path differs from completion pointer")
    manifest_path = validated_attempt / "manifest.json"
    manifest_sha = sha256(manifest_path)
    campaign_dir = Path(str(execution["campaign_dir"])).resolve()
    campaign_path = campaign_dir / "campaign.json"
    completion_path = (campaign_dir / "completed" / str(run["arm_id"]) /
                       (str(run["session_id"]) + ".json"))
    exact_execution = {
        "plan_identity_sha256": plan_identity,
        "run_id": run_id,
        "attempt_manifest_sha256": manifest_sha,
        "build_identity_sha256": build_identity,
        "campaign_identity_sha256": campaign["identity_sha256"],
        "campaign_plan_path": str(campaign_path),
        "campaign_plan_file_sha256": sha256(campaign_path),
        "campaign_dir": str(campaign_dir),
        "completion_pointer_path": str(completion_path),
        "completion_pointer_file_sha256": sha256(completion_path),
        "qualification_run_binding_identity_sha256": manifest[
            "qualification_run_binding"]["identity_sha256"],
        "attempt_manifest_path": str(manifest_path),
        "fresh_process": True,
    }
    for field, expected in exact_execution.items():
        if execution.get(field) != expected:
            raise CampaignError(f"execution receipt mismatch: {field}")
    try:
        params = yaml.safe_load(paths["result_params.yaml"].read_text())
    except yaml.YAMLError as error:
        raise CampaignError(f"invalid result_params.yaml: {error}") from error
    if not isinstance(params, Mapping):
        raise CampaignError("result_params.yaml is not a mapping")
    expected_anchor = sentinel["explicit_anchor"]
    anchor_parameter = _nested(params, "imu.init_anchor_stamp_ns")
    gap_parameter = _nested(params, "imu.init_anchor_max_predecessor_gap_s")
    if (anchor_parameter != expected_anchor["anchor_stamp_ns"] or
            not isinstance(anchor_parameter, str) or gap_parameter != 0.02 or
            _nested(params, "uav.runtime_reinit_enable") is not False):
        raise CampaignError("result parameters violate explicit-anchor contract")
    for section, values in sentinel["runtime_overrides"].items():
        if not isinstance(values, Mapping):
            raise CampaignError("runtime override section must be a mapping")
        for key, expected in values.items():
            actual = _nested(params, f"{section}.{key}")
            if actual != expected:
                raise CampaignError(
                    f"result parameter differs from runtime override: "
                    f"{section}.{key}")

    events = _diagnostic_evidence(
        paths["result_node.log"], paths["result.bag"])
    configured = _unique_event(events, "configured")
    covered = _unique_event(events, "anchor_covered")
    accepted_rows = [
        row for row in events.get("accepted", [])
        if row.get("initialization_gate_ready") is True
    ]
    if len(accepted_rows) != 1:
        raise CampaignError(
            "node log requires exactly one gate-ready accepted event")
    # The destructor emits a compact status=accepted summary as well; it is
    # provenance, not a second initialization acceptance.
    accepted = accepted_rows[0]
    correction_event = _unique_event(events, "first_correction_received")
    anchor_ns = expected_anchor["anchor_stamp_ns"]
    if (configured.get("anchor_mode") != "explicit" or
            configured.get("anchor_stamp_ns") != anchor_ns or
            configured.get("anchor_max_predecessor_gap_ns") != "20000000" or
            covered.get("anchor_mode") != "explicit" or
            covered.get("anchor_stamp_ns") != anchor_ns or
            covered.get("sync_epoch_ns") != anchor_ns or
            covered.get("image_epoch_ns") != anchor_ns or
            covered.get("has_pre_anchor_imu") is not True or
            covered.get("anchor_predecessor_stamp_ns") !=
            expected_anchor["predecessor_imu"]["watermark_ns"] or
            covered.get("anchor_predecessor_gap_ns") !=
            expected_anchor["predecessor_gap_ns"]):
        raise CampaignError("node log explicit-anchor coverage mismatch")
    vector = accepted.get("selected_stamp_seq")
    if (vector != expected_anchor[
            "expected_first_30_strict_post_anchor_stamp_seq"] or
            accepted.get("selected_stamp_seq_sha256") != expected_anchor[
                "expected_first_30_stamp_seq_sha256"]):
        raise CampaignError("node log selected IMU vector mismatch")
    if (accepted.get("first_used_stamp_ns") != vector[0]["stamp_ns"] or
            accepted.get("first_used_seq") != vector[0]["seq"] or
            accepted.get("last_used_stamp_ns") != vector[-1]["stamp_ns"] or
            accepted.get("last_used_seq") != vector[-1]["seq"]):
        raise CampaignError("accepted initialization endpoints mismatch vector")
    statistics = {
        "mean_acc": _finite(accepted.get("mean_acc", ()), "mean_acc"),
        "mean_gyr": _finite(accepted.get("mean_gyr", ()), "mean_gyr"),
    }
    if len(statistics["mean_acc"]) != 3 or len(statistics["mean_gyr"]) != 3:
        raise CampaignError("node log initialization mean has wrong length")
    if (accepted.get("anchor_mode") != "explicit" or
            accepted.get("anchor_stamp_ns") != anchor_ns or
            accepted.get("valid_count") != 30 or
            accepted.get("initialization_gate_ready") is not True or
            accepted.get("invalid_count") != 0 or
            accepted.get("rejected_window_count") != 0 or
            accepted.get("queue_drop_count") != 0 or
            accepted.get("initial_state_fingerprint_schema") !=
            STATE_FINGERPRINT_SCHEMA or
            not _valid_sha(accepted.get("initial_state_binary64_be_sha256"))):
        raise CampaignError("accepted initialization diagnostic failed gate")

    streams = canonicalize_result_bag(paths["result.bag"])
    correction_epoch = _positive_ns(
        str(correction_event.get("correction_epoch_ns")),
        "first correction epoch")
    if (str(correction_epoch) != streams["correction"]["first_sensor_stamp_ns"] or
            correction_event.get("qualification_gate_ready") is not True or
            correction_event.get("state_fingerprint_schema") !=
            STATE_FINGERPRINT_SCHEMA or
            correction_event.get("initial_state_binary64_be_sha256") !=
            accepted["initial_state_binary64_be_sha256"] or
            not _valid_sha(correction_event.get("state_binary64_be_sha256"))):
        raise CampaignError("first correction log/bag binding failed")
    report = load_json(paths["result.flight_readiness.json"])
    local_score = _local_objective(report)

    build_for_checker = {
        "executable_sha256": build.get("executable_sha256"),
        "dynamic_libraries": build.get("dynamic_libraries"),
        "source_tree_sha256": build.get("source_tree_sha256"),
        "reviewed_init_anchor_patch_sha256":
            build.get("reviewed_init_anchor_patch_sha256"),
    }
    if (not _valid_sha(build_for_checker["executable_sha256"]) or
            not isinstance(build_for_checker["dynamic_libraries"], Mapping) or
            not build_for_checker["dynamic_libraries"] or
            any(not _valid_sha(value) for value in
                build_for_checker["dynamic_libraries"].values()) or
            not _valid_sha(build_for_checker["source_tree_sha256"]) or
            not _valid_sha(build_for_checker[
                "reviewed_init_anchor_patch_sha256"])):
        raise CampaignError("post-fix build manifest lacks canonical hashes")
    build_for_checker["identity_sha256"] = object_sha256(build_for_checker)
    state_epoch_ns = str(accepted.get("state_epoch_ns"))
    initialization = {
        "anchor_definition":
            "earliest_explicit_eligible_full_sync_sensor_epoch",
        "anchor_mode": "explicit",
        "anchor_covered": True,
        "anchor_stamp_ns": anchor_ns,
        "image_epoch_ns": covered["image_epoch_ns"],
        "lidar_watermark_ns": covered["lidar_watermark_ns"],
        "imu_watermark_ns": covered["imu_watermark_ns"],
        "predecessor_imu_stamp_ns": covered[
            "anchor_predecessor_stamp_ns"],
        "predecessor_gap_ns": covered["anchor_predecessor_gap_ns"],
        "init_anchor_max_predecessor_gap_s": 0.02,
        "has_pre_anchor_imu": True,
        "state_epoch_ns": state_epoch_ns,
        "state_epoch_rule": "legacy_later_acceptance_sync_epoch",
        "suffix_rule": "legacy_skip_through_acceptance_state_epoch",
        "sample_sensor_stamp_seq_vector": vector,
        "sample_sensor_stamp_seq_vector_sha256": accepted[
            "selected_stamp_seq_sha256"],
        "sample_sensor_stamp_seq_hash_encoding":
            "utf8_lines_stamp_ns_comma_seq_newline",
        "valid_count": accepted["valid_count"],
        "first_used_stamp_ns": accepted["first_used_stamp_ns"],
        "first_used_seq": accepted["first_used_seq"],
        "last_used_stamp_ns": accepted["last_used_stamp_ns"],
        "last_used_seq": accepted["last_used_seq"],
        "invalid_count": accepted["invalid_count"],
        "rejected_window_count": accepted["rejected_window_count"],
        "queue_drop_count": accepted["queue_drop_count"],
        "statistics": statistics,
        "statistics_sha256": object_sha256(statistics),
        "initial_state_binary64_be_sha256": accepted[
            "initial_state_binary64_be_sha256"],
    }
    core: Dict[str, Any] = {
        "schema": RECEIPT_SCHEMA,
        "plan_identity_sha256": plan_identity,
        "run_id": run_id,
        "sentinel_id": run["sentinel_id"],
        "arm_id": run["arm_id"],
        "session_id": run["session_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "fresh_process": True,
        "process_instance_uuid": process_uuid,
        "build": build_for_checker,
        "runtime_parameters": {
            "imu/init_anchor_stamp_ns": anchor_parameter,
            "imu/init_anchor_max_predecessor_gap_s": gap_parameter,
        },
        "initialization": initialization,
        "canonicalization_schema": CANONICAL_SCHEMA,
        "quaternion_sign_canonicalized": True,
        "rosbag_record_time_used": False,
        "streams": streams,
        "first_correction": {
            "correction_epoch_ns": str(correction_epoch),
            "state_binary64_be_sha256": correction_event[
                "state_binary64_be_sha256"],
            "trajectory_index": 0,
            "all_values_finite": True,
            "qualification_gate_ready": True,
            "trajectory_message_binary64_be_sha256": streams[
                "correction"]["first_message_binary64_be_sha256"],
            "trajectory_sensor_stamp_vector_sha256": streams[
                "correction"]["sensor_stamp_vector_sha256"],
            "trajectory_binary64_be_sha256": streams[
                "correction"]["canonical_state_sha256"],
        },
        "accuracy": {
            "local_objective_normalized_max": local_score,
            "full_report_normalized_max": None,
        },
        "source": {
            "attempt": str(validated_attempt),
            "attempt_manifest_sha256": manifest_sha,
            "execution_receipt_identity_sha256": execution_identity,
            "postfix_build_manifest_identity_sha256": build_identity,
            "orchestration_identity_sha256":
                orchestration["identity_sha256"],
            "campaign_identity_sha256": campaign["identity_sha256"],
            "artifacts": {
                name: manifest["artifacts"][name] for name in ARTIFACTS
            },
        },
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _write_exclusive(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True) + "\n").encode()
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("run_id")
    parser.add_argument("attempt", type=Path)
    parser.add_argument("--execution", type=Path, required=True)
    parser.add_argument("--build-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        receipt = build_receipt(
            load_json(arguments.plan), arguments.run_id, arguments.attempt,
            load_json(arguments.execution), load_json(arguments.build_manifest))
        _write_exclusive(arguments.output, receipt)
        print(json.dumps({"output": str(arguments.output.resolve()),
                          "identity_sha256": receipt["identity_sha256"]},
                         indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, rosbag.ROSBagException,
            yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

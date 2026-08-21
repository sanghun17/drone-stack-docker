#!/usr/bin/env python3
"""Check the 12-run post-fix initialization qualification receipts.

Receipts are intentionally sensor-time/canonical-state artifacts.  Rosbag
record time, process wall time, and serialized bag hashes are neither accepted
as substitutes nor compared.  This checker is read-only except for an
optional append-only JSON report; it never launches a replay.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_json,
    object_sha256,
)


PLAN_SCHEMA = "fastlivo_vio_postfix_init_qualification_plan/v1"
RECEIPT_SCHEMA = "fastlivo_vio_postfix_init_qualification_receipt/v1"
CANONICAL_SCHEMA = "fastlivo_sensor_stamped_state_canonical/v1"
REPORT_SCHEMA = "fastlivo_vio_postfix_init_qualification_report/v1"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
STREAMS = (
    "low_rate_pose", "low_rate_init", "correction",
    "propagated_odom", "world_twist",
)
EXACT_PAYLOAD_STREAMS = ("low_rate_pose", "low_rate_init", "correction")
HIGH_RATE_STREAMS = ("propagated_odom", "world_twist")
EXPECTED_SEMANTICS = {
    "init_anchor_rule":
        "earliest_explicit_eligible_full_sync_sensor_epoch",
    "init_anchor_mode": "explicit",
    "init_anchor_max_predecessor_gap_s": 0.02,
    "init_sample_count": 30,
    "init_samples_strictly_after_anchor": True,
    "state_epoch_rule": "legacy_later_acceptance_sync_epoch",
    "suffix_rule": "legacy_skip_through_acceptance_state_epoch",
    "runtime_reinitialization_allowed": False,
    "gt_anchor_allowed": False,
    "canonicalization_schema": CANONICAL_SCHEMA,
    "compare_rosbag_record_time": False,
    "require_exact_low_rate_payload_hashes_across_rates": True,
    "high_rate_payload_hashes_are_diagnostic_only": True,
}


def _valid_hash(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _positive_decimal_ns(value: Any, label: str) -> int:
    if (not isinstance(value, str) or not value.isdigit() or
            value.startswith("0")):
        raise CampaignError(f"{label} must be a quoted positive decimal uint64 ns")
    parsed = int(value)
    if parsed <= 0 or parsed > (1 << 64) - 1:
        raise CampaignError(f"{label} is outside uint64")
    return parsed


def _stamp_seq_sha256(vector: Sequence[Mapping[str, Any]]) -> str:
    payload = "".join(
        f"{sample['stamp_ns']},{int(sample['seq'])}\n" for sample in vector)
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


def _self_hash(document: Mapping[str, Any], field: str, label: str) -> str:
    declared = document.get(field)
    if not _valid_hash(declared):
        raise CampaignError(f"{label} has no valid {field}")
    core = dict(document)
    core.pop(field, None)
    actual = object_sha256(core)
    if actual != declared:
        raise CampaignError(
            f"{label} identity changed: expected {declared}, got {actual}")
    return str(declared)


def _load_plan(path: Path) -> Tuple[Dict[str, Any], str]:
    plan = load_json(path)
    if not isinstance(plan, dict) or plan.get("schema") != PLAN_SCHEMA:
        raise CampaignError(f"not a post-fix qualification plan: {path}")
    identity = _self_hash(plan, "identity_sha256", "qualification plan")
    if (plan.get("scope") != "development_only" or
            plan.get("validation_data_accessed") is not False or
            plan.get("execution_neutral") is not True or
            plan.get("expected_run_count") != 12):
        raise CampaignError("qualification plan violates frozen scope/count")
    if (plan.get("rates") != [0.5, 1.0] or
            plan.get("fresh_process_repeats_per_rate") != 3 or
            plan.get("required_semantics") != EXPECTED_SEMANTICS):
        raise CampaignError("qualification plan semantics/rates changed")
    sentinels = plan.get("sentinels")
    if (not isinstance(sentinels, list) or len(sentinels) != 2 or
            any(not isinstance(row, Mapping) or
                row.get("session_split") != "development"
                for row in sentinels)):
        raise CampaignError("qualification plan sentinel scope changed")
    runs = plan.get("runs")
    if not isinstance(runs, list) or len(runs) != 12:
        raise CampaignError("qualification plan does not contain 12 runs")
    ids = [run.get("run_id") for run in runs if isinstance(run, Mapping)]
    if len(ids) != 12 or len(set(ids)) != 12:
        raise CampaignError("qualification plan run IDs are malformed/duplicate")
    expected_grid = {
        (sentinel["id"], rate, repeat)
        for sentinel in sentinels for rate in (0.5, 1.0)
        for repeat in (1, 2, 3)
    }
    actual_grid = {
        (run.get("sentinel_id"), run.get("rate"), run.get("repeat"))
        for run in runs if isinstance(run, Mapping)
    }
    if actual_grid != expected_grid or any(
            run.get("fresh_process_required") is not True for run in runs):
        raise CampaignError("qualification plan is not the exact 12-run grid")
    return plan, identity


def _receipt_path(root: Path, relative_value: Any) -> Path:
    relative = Path(str(relative_value))
    if relative.is_absolute() or ".." in relative.parts:
        raise CampaignError(f"unsafe receipt path in plan: {relative}")
    path = (root / relative).resolve()
    try:
        path.relative_to(root.resolve())
    except ValueError as error:
        raise CampaignError(f"receipt escapes qualification root: {path}") from error
    return path


def _validate_build(build: Any, label: str) -> Tuple[Mapping[str, Any], str]:
    if not isinstance(build, Mapping):
        raise CampaignError(f"{label}: missing post-fix build identity")
    build_hash = _self_hash(build, "identity_sha256", f"{label} build")
    if not _valid_hash(build.get("executable_sha256")):
        raise CampaignError(f"{label}: invalid executable SHA-256")
    libraries = build.get("dynamic_libraries")
    if not isinstance(libraries, Mapping) or not libraries:
        raise CampaignError(f"{label}: missing estimator-library fingerprints")
    for name, value in libraries.items():
        if not isinstance(name, str) or not _valid_hash(value):
            raise CampaignError(f"{label}: invalid library fingerprint {name!r}")
    if not _valid_hash(build.get("source_tree_sha256")):
        raise CampaignError(f"{label}: invalid source-tree fingerprint")
    if not _valid_hash(build.get("reviewed_init_anchor_patch_sha256")):
        raise CampaignError(f"{label}: missing reviewed init-anchor patch identity")
    return build, build_hash


def _validate_init(receipt: Mapping[str, Any], expected: Mapping[str, Any],
                   label: str) -> Dict[str, Any]:
    init = receipt.get("initialization")
    if not isinstance(init, Mapping):
        raise CampaignError(f"{label}: missing initialization receipt")
    anchor_ns = _positive_decimal_ns(
        init.get("anchor_stamp_ns"), f"{label} anchor_stamp_ns")
    image_ns = _positive_decimal_ns(
        init.get("image_epoch_ns"), f"{label} image_epoch_ns")
    expected_anchor_ns = _positive_decimal_ns(
        expected.get("anchor_stamp_ns"), f"{label} expected anchor")
    lidar_ns = _positive_decimal_ns(
        init.get("lidar_watermark_ns"), f"{label} lidar watermark")
    imu_ns = _positive_decimal_ns(
        init.get("imu_watermark_ns"), f"{label} imu watermark")
    state_epoch_ns = _positive_decimal_ns(
        init.get("state_epoch_ns"), f"{label} state epoch")
    if (anchor_ns != expected_anchor_ns or image_ns != anchor_ns or
            init.get("anchor_definition") !=
            "earliest_explicit_eligible_full_sync_sensor_epoch" or
            init.get("anchor_mode") != "explicit" or
            init.get("anchor_covered") is not True or
            init.get("has_pre_anchor_imu") is not True or
            lidar_ns < anchor_ns or imu_ns < anchor_ns):
        raise CampaignError(f"{label}: full-sync explicit-anchor contract failed")
    predecessor_ns = _positive_decimal_ns(
        init.get("predecessor_imu_stamp_ns"),
        f"{label} predecessor IMU stamp")
    predecessor_gap_ns = anchor_ns - predecessor_ns
    if (predecessor_ns > anchor_ns or
            init.get("predecessor_gap_ns") != str(predecessor_gap_ns) or
            predecessor_ns != int(expected["predecessor_imu"]["watermark_ns"]) or
            init.get("predecessor_gap_ns") != expected["predecessor_gap_ns"] or
            predecessor_gap_ns > 20_000_000 or
            init.get("init_anchor_max_predecessor_gap_s") != 0.02):
        raise CampaignError(f"{label}: explicit-anchor predecessor gate failed")

    vector = init.get("sample_sensor_stamp_seq_vector")
    expected_vector = expected.get(
        "expected_first_30_strict_post_anchor_stamp_seq")
    if (not isinstance(vector, list) or len(vector) != 30 or
            vector != expected_vector):
        raise CampaignError(
            f"{label}: selected stamp+seq vector differs from expected first 30")
    calculated_stamp_hash = _stamp_seq_sha256(vector)
    if (init.get("sample_sensor_stamp_seq_vector_sha256") !=
            calculated_stamp_hash or
            calculated_stamp_hash != expected.get(
                "expected_first_30_stamp_seq_sha256") or
            init.get("sample_sensor_stamp_seq_hash_encoding") !=
            "utf8_lines_stamp_ns_comma_seq_newline"):
        raise CampaignError(f"{label}: initialization stamp+seq hash mismatch")
    previous = anchor_ns
    for index, sample in enumerate(vector):
        if not isinstance(sample, Mapping):
            raise CampaignError(f"{label}: malformed initialization sample {index}")
        stamp = _positive_decimal_ns(
            sample.get("stamp_ns"), f"{label} initialization sample {index}")
        if stamp <= previous or not isinstance(sample.get("seq"), int):
            raise CampaignError(
                f"{label}: initialization samples are not strict post-anchor")
        previous = stamp
    if (init.get("valid_count") != 30 or
            init.get("first_used_stamp_ns") != vector[0]["stamp_ns"] or
            init.get("first_used_seq") != vector[0]["seq"] or
            init.get("last_used_stamp_ns") != vector[-1]["stamp_ns"] or
            init.get("last_used_seq") != vector[-1]["seq"] or
            state_epoch_ns < previous or state_epoch_ns <= anchor_ns or
            init.get("state_epoch_rule") != "legacy_later_acceptance_sync_epoch" or
            init.get("suffix_rule") !=
            "legacy_skip_through_acceptance_state_epoch"):
        raise CampaignError(f"{label}: initialization count/state/suffix contract failed")
    zero_counters = (
        "invalid_count",
        "rejected_window_count",
        "queue_drop_count",
    )
    unavailable_counters = (
        "startup_dropped_imu_count",
        "startup_duplicate_imu_count",
        "startup_time_regression_count",
        "runtime_reinitialization_count",
    )
    if any(field in init for field in unavailable_counters):
        raise CampaignError(
            f"{label}: unavailable startup/reinitialization counters must be omitted")
    for field in zero_counters:
        if init.get(field) != 0:
            raise CampaignError(f"{label}: {field} must be zero")
    statistics = init.get("statistics")
    if not isinstance(statistics, Mapping):
        raise CampaignError(f"{label}: missing initialization statistics")
    if set(statistics) != {"mean_acc", "mean_gyr"}:
        raise CampaignError(f"{label}: initialization statistics set is not exact")
    for field in ("mean_acc", "mean_gyr"):
        values = statistics[field]
        if (not isinstance(values, list) or len(values) != 3 or
                any(not isinstance(value, (int, float)) or isinstance(value, bool) or
                    not math.isfinite(float(value)) for value in values)):
            raise CampaignError(f"{label}: invalid initialization statistic {field}")
    statistics_hash = object_sha256(statistics)
    if init.get("statistics_sha256") != statistics_hash:
        raise CampaignError(f"{label}: initialization statistics hash mismatch")
    init_state_hash = init.get("initial_state_binary64_be_sha256")
    if not _valid_hash(init_state_hash):
        raise CampaignError(f"{label}: missing canonical init-state fingerprint")
    return {
        "anchor_stamp_ns": str(anchor_ns),
        "image_epoch_ns": str(image_ns),
        "lidar_watermark_ns": str(lidar_ns),
        "imu_watermark_ns": str(imu_ns),
        "state_epoch_ns": str(state_epoch_ns),
        "sample_sensor_stamp_seq_vector": vector,
        "sample_sensor_stamp_seq_vector_sha256": calculated_stamp_hash,
        "statistics": dict(statistics),
        "statistics_sha256": statistics_hash,
        "initial_state_binary64_be_sha256": init_state_hash,
    }


def _validate_streams(receipt: Mapping[str, Any], label: str) -> Dict[str, Any]:
    if receipt.get("canonicalization_schema") != CANONICAL_SCHEMA:
        raise CampaignError(f"{label}: wrong canonicalization schema")
    if receipt.get("quaternion_sign_canonicalized") is not True:
        raise CampaignError(f"{label}: quaternion sign was not canonicalized")
    if receipt.get("rosbag_record_time_used") is not False:
        raise CampaignError(f"{label}: rosbag record time entered comparison")
    streams = receipt.get("streams")
    if not isinstance(streams, Mapping) or set(streams) != set(STREAMS):
        raise CampaignError(f"{label}: output stream set must be exact")
    normalized: Dict[str, Any] = {}
    for name in STREAMS:
        stream = streams[name]
        if not isinstance(stream, Mapping):
            raise CampaignError(f"{label}: malformed {name} receipt")
        count = stream.get("message_count")
        if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
            raise CampaignError(f"{label}: empty/invalid {name} count")
        if (not _valid_hash(stream.get("sensor_stamp_vector_sha256")) or
                not _valid_hash(stream.get("canonical_state_sha256")) or
                not _valid_hash(stream.get(
                    "first_message_binary64_be_sha256")) or
                stream.get("all_values_finite") is not True or
                stream.get("sensor_stamps_monotonic_non_decreasing") is not True):
            raise CampaignError(f"{label}: {name} canonical-state gate failed")
        normalized[name] = {
            "message_count": count,
            "sensor_stamp_vector_sha256": stream["sensor_stamp_vector_sha256"],
            "canonical_state_sha256": stream["canonical_state_sha256"],
            "first_message_binary64_be_sha256": stream[
                "first_message_binary64_be_sha256"],
        }
    if (normalized["propagated_odom"]["message_count"] !=
            normalized["world_twist"]["message_count"] or
            normalized["propagated_odom"]["sensor_stamp_vector_sha256"] !=
            normalized["world_twist"]["sensor_stamp_vector_sha256"]):
        raise CampaignError(f"{label}: propagated odom/world-twist pairing failed")
    if (normalized["low_rate_pose"]["message_count"] !=
            normalized["low_rate_init"]["message_count"] or
            normalized["low_rate_pose"]["sensor_stamp_vector_sha256"] !=
            normalized["low_rate_init"]["sensor_stamp_vector_sha256"]):
        raise CampaignError(f"{label}: low-rate body/init pairing failed")
    first_correction = receipt.get("first_correction")
    if not isinstance(first_correction, Mapping):
        raise CampaignError(f"{label}: missing first correction receipt")
    first_stamp_ns = _positive_decimal_ns(
        first_correction.get("correction_epoch_ns"),
        f"{label} first correction epoch")
    if (not _valid_hash(first_correction.get("state_binary64_be_sha256")) or
            first_correction.get("trajectory_index") != 0 or
            first_correction.get("all_values_finite") is not True or
            first_correction.get("qualification_gate_ready") is not True):
        raise CampaignError(f"{label}: first correction canonical gate failed")
    correction_stream = normalized["correction"]
    if (first_correction.get("trajectory_sensor_stamp_vector_sha256") !=
            correction_stream["sensor_stamp_vector_sha256"] or
            first_correction.get("trajectory_binary64_be_sha256") !=
            correction_stream["canonical_state_sha256"] or
            first_correction.get("trajectory_message_binary64_be_sha256") !=
            correction_stream["first_message_binary64_be_sha256"]):
        raise CampaignError(
            f"{label}: first correction is not bound to correction trajectory")
    normalized["first_correction"] = {
        "correction_epoch_ns": str(first_stamp_ns),
        "state_binary64_be_sha256":
            first_correction["state_binary64_be_sha256"],
        "trajectory_sensor_stamp_vector_sha256":
            first_correction["trajectory_sensor_stamp_vector_sha256"],
        "trajectory_binary64_be_sha256":
            first_correction["trajectory_binary64_be_sha256"],
        "trajectory_message_binary64_be_sha256":
            first_correction["trajectory_message_binary64_be_sha256"],
    }
    return normalized


def _validate_accuracy(receipt: Mapping[str, Any], label: str) -> Dict[str, Any]:
    accuracy = receipt.get("accuracy")
    if not isinstance(accuracy, Mapping):
        raise CampaignError(f"{label}: missing accuracy receipt")
    local = accuracy.get("local_objective_normalized_max")
    full = accuracy.get("full_report_normalized_max")
    if (not isinstance(local, (int, float)) or isinstance(local, bool) or
            not math.isfinite(float(local))):
        raise CampaignError(f"{label}: non-finite local objective score")
    if (full is not None and
            (not isinstance(full, (int, float)) or isinstance(full, bool) or
             not math.isfinite(float(full)))):
        raise CampaignError(f"{label}: non-finite full-report diagnostic score")
    return {
        "local_objective_normalized_max": float(local),
        "full_report_normalized_max": None if full is None else float(full),
    }


def check_qualification(plan_path: Path, receipt_root: Path) -> Dict[str, Any]:
    plan_path = plan_path.resolve()
    receipt_root = receipt_root.resolve()
    plan, plan_identity = _load_plan(plan_path)
    expected_anchor_by_sentinel = {
        str(row["id"]): row["explicit_anchor"]
        for row in plan["sentinels"]
    }
    run_receipts: List[Dict[str, Any]] = []
    process_instances = set()
    build_hashes = set()
    build_documents: Dict[str, Mapping[str, Any]] = {}
    for run in plan["runs"]:
        run_id = str(run["run_id"])
        path = _receipt_path(receipt_root, run["expected_receipt"])
        receipt = load_json(path)
        label = f"receipt {run_id}"
        if not isinstance(receipt, Mapping) or receipt.get("schema") != RECEIPT_SCHEMA:
            raise CampaignError(f"{label}: wrong receipt schema")
        _self_hash(receipt, "identity_sha256", label)
        exact = {
            "plan_identity_sha256": plan_identity,
            "run_id": run_id,
            "sentinel_id": run["sentinel_id"],
            "arm_id": run["arm_id"],
            "session_id": run["session_id"],
            "rate": run["rate"],
            "repeat": run["repeat"],
        }
        for field, expected in exact.items():
            if receipt.get(field) != expected:
                raise CampaignError(
                    f"{label}: {field} mismatch ({receipt.get(field)!r} != {expected!r})")
        if receipt.get("fresh_process") is not True:
            raise CampaignError(f"{label}: run was not a fresh process")
        expected_anchor = expected_anchor_by_sentinel[run["sentinel_id"]]
        runtime_parameters = receipt.get("runtime_parameters")
        if (not isinstance(runtime_parameters, Mapping) or
                set(runtime_parameters) != {
                    "imu/init_anchor_stamp_ns",
                    "imu/init_anchor_max_predecessor_gap_s",
                } or
                runtime_parameters.get("imu/init_anchor_stamp_ns") !=
                expected_anchor["anchor_stamp_ns"] or
                runtime_parameters.get(
                    "imu/init_anchor_max_predecessor_gap_s") != 0.02):
            raise CampaignError(
                f"{label}: runtime explicit-anchor parameter snapshot mismatch")
        process_id = receipt.get("process_instance_uuid")
        if not isinstance(process_id, str) or not process_id:
            raise CampaignError(f"{label}: missing process instance UUID")
        if process_id in process_instances:
            raise CampaignError(f"{label}: process instance was reused")
        process_instances.add(process_id)
        build, build_hash = _validate_build(receipt.get("build"), label)
        build_hashes.add(build_hash)
        build_documents[build_hash] = build
        init = _validate_init(
            receipt, expected_anchor, label)
        streams = _validate_streams(receipt, label)
        if (streams["first_correction"]["correction_epoch_ns"] !=
                init["state_epoch_ns"]):
            raise CampaignError(
                f"{label}: first correction epoch differs from acceptance state epoch")
        accuracy = _validate_accuracy(receipt, label)
        run_receipts.append({
            "run_id": run_id,
            "sentinel_id": run["sentinel_id"],
            "arm_id": run["arm_id"],
            "session_id": run["session_id"],
            "rate": run["rate"],
            "repeat": run["repeat"],
            "process_instance_uuid": process_id,
            "receipt": str(path),
            "receipt_identity_sha256": receipt["identity_sha256"],
            "build_identity_sha256": build_hash,
            "initialization": init,
            "streams": streams,
            "accuracy": accuracy,
        })

    failures: List[Dict[str, Any]] = []
    if len(build_hashes) != 1:
        failures.append({
            "gate": "single_postfix_build",
            "detail": f"observed {len(build_hashes)} build identities",
        })
    build_hash = next(iter(build_hashes)) if len(build_hashes) == 1 else None
    reference_build = plan.get("reference_phase_a_build")
    if build_hash is not None and isinstance(reference_build, Mapping):
        post_build = build_documents[build_hash]
        old_exe = reference_build.get("executable_sha256")
        old_libraries = {
            name: row.get("sha256") if isinstance(row, Mapping) else None
            for name, row in reference_build.get("dynamic_libraries", {}).items()
        } if isinstance(reference_build.get("dynamic_libraries"), Mapping) else {}
        if (post_build.get("executable_sha256") == old_exe and
                dict(post_build.get("dynamic_libraries", {})) == old_libraries):
            failures.append({
                "gate": "postfix_build_differs_from_reference",
                "detail": "post-fix executable/libraries equal the pre-fix build",
            })

    sentinel_results: List[Dict[str, Any]] = []
    warnings: List[Dict[str, Any]] = []
    for sentinel in plan["sentinels"]:
        sentinel_id = sentinel["id"]
        rows = [row for row in run_receipts
                if row["sentinel_id"] == sentinel_id]
        if len(rows) != 6:
            raise CampaignError(f"sentinel {sentinel_id} does not have six receipts")
        init_hashes = {
            row["initialization"]["sample_sensor_stamp_seq_vector_sha256"]
            for row in rows
        }
        anchor_stamps = {
            row["initialization"]["anchor_stamp_ns"] for row in rows
        }
        statistics_hashes = {
            row["initialization"]["statistics_sha256"] for row in rows
        }
        state_epochs = {
            row["initialization"]["state_epoch_ns"] for row in rows
        }
        init_state_hashes = {
            row["initialization"]["initial_state_binary64_be_sha256"]
            for row in rows
        }
        if (len(init_hashes) != 1 or len(anchor_stamps) != 1 or
                len(statistics_hashes) != 1 or len(state_epochs) != 1 or
                len(init_state_hashes) != 1):
            failures.append({
                "gate": "identical_init_anchor_vector_stats_state_epoch_across_repeats_and_rates",
                "sentinel_id": sentinel_id,
                "stamp_vector_hash_count": len(init_hashes),
                "anchor_count": len(anchor_stamps),
                "statistics_hash_count": len(statistics_hashes),
                "state_epoch_count": len(state_epochs),
                "init_state_fingerprint_count": len(init_state_hashes),
            })
        stream_inventory_agreement: Dict[str, bool] = {}
        stream_payload_agreement: Dict[str, bool] = {}
        for stream in STREAMS:
            inventory_signatures = {
                (row["streams"][stream]["message_count"],
                 row["streams"][stream]["sensor_stamp_vector_sha256"])
                for row in rows
            }
            payload_signatures = {
                row["streams"][stream]["canonical_state_sha256"]
                for row in rows
            }
            stream_inventory_agreement[stream] = len(inventory_signatures) == 1
            stream_payload_agreement[stream] = len(payload_signatures) == 1
            if len(inventory_signatures) != 1:
                failures.append({
                    "gate": "identical_stream_count_and_sensor_stamp_inventory_across_repeats_and_rates",
                    "sentinel_id": sentinel_id,
                    "stream": stream,
                    "signature_count": len(inventory_signatures),
                })
            if stream in EXACT_PAYLOAD_STREAMS and len(payload_signatures) != 1:
                failures.append({
                    "gate": "identical_low_rate_payload_across_repeats_and_rates",
                    "sentinel_id": sentinel_id,
                    "stream": stream,
                    "payload_hash_count": len(payload_signatures),
                })
            elif stream in HIGH_RATE_STREAMS and len(payload_signatures) != 1:
                warnings.append({
                    "diagnostic": "high_rate_payload_differs_across_repeats_or_rates",
                    "known_separate_interface_blocker": True,
                    "sentinel_id": sentinel_id,
                    "stream": stream,
                    "payload_hash_count": len(payload_signatures),
                })
        first_correction_signatures = {
            tuple(sorted(row["streams"]["first_correction"].items()))
            for row in rows
        }
        if len(first_correction_signatures) != 1:
            failures.append({
                "gate": "identical_first_correction_across_repeats_and_rates",
                "sentinel_id": sentinel_id,
                "signature_count": len(first_correction_signatures),
            })
        local_scores = [
            row["accuracy"]["local_objective_normalized_max"] for row in rows
        ]
        local_score_envelope = max(local_scores) - min(local_scores)
        if local_score_envelope != 0.0:
            failures.append({
                "gate": "identical_local_objective_accuracy_across_repeats_and_rates",
                "sentinel_id": sentinel_id,
                "envelope": local_score_envelope,
            })
        full_scores = [
            row["accuracy"]["full_report_normalized_max"] for row in rows
            if row["accuracy"]["full_report_normalized_max"] is not None
        ]
        full_score_envelope = (
            max(full_scores) - min(full_scores) if full_scores else None)
        sentinel_results.append({
            "sentinel_id": sentinel_id,
            "run_count": len(rows),
            "init_stamp_vector_agreement": len(init_hashes) == 1,
            "anchor_agreement": len(anchor_stamps) == 1,
            "initialization_statistics_agreement": len(statistics_hashes) == 1,
            "acceptance_state_epoch_agreement": len(state_epochs) == 1,
            "canonical_init_state_agreement": len(init_state_hashes) == 1,
            "first_correction_agreement": len(first_correction_signatures) == 1,
            "stream_inventory_agreement": stream_inventory_agreement,
            "stream_payload_agreement": stream_payload_agreement,
            "local_objective_normalized_min": min(local_scores),
            "local_objective_normalized_max": max(local_scores),
            "local_objective_repeat_envelope": local_score_envelope,
            "full_report_normalized_repeat_envelope_diagnostic":
                full_score_envelope,
        })

    old_executable = (
        reference_build.get("executable_sha256")
        if isinstance(reference_build, Mapping) else None)
    core: Dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "plan": str(plan_path),
        "plan_identity_sha256": plan_identity,
        "receipt_root": str(receipt_root),
        "receipt_count": len(run_receipts),
        "fresh_process_instance_count": len(process_instances),
        "postfix_build_identity_sha256": build_hash,
        "postfix_executable_sha256": (
            build_documents[build_hash].get("executable_sha256")
            if build_hash is not None else None),
        "reference_prefix_executable_sha256": old_executable,
        "run_receipts": run_receipts,
        "sentinels": sentinel_results,
        "maximum_local_objective_repeat_envelope": max(
            result["local_objective_repeat_envelope"]
            for result in sentinel_results),
        "warnings": warnings,
        "failures": failures,
        "status": "pass" if not failures else "fail",
        "go_for_postfix_phase_a_rebaseline": not failures,
        "not_a_flight_readiness_decision": True,
        "flight_ready": False,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _write_report(path: Path, report: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(report, indent=2, sort_keys=True,
                          ensure_ascii=False) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("receipt_root", type=Path)
    parser.add_argument("--report", type=Path)
    arguments = parser.parse_args(argv)
    try:
        report = check_qualification(arguments.plan, arguments.receipt_root)
        if arguments.report:
            _write_report(arguments.report, report)
        else:
            print(json.dumps(report, indent=2, sort_keys=True,
                             ensure_ascii=False))
        return 0 if report["status"] == "pass" else 1
    except (CampaignError, FileExistsError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

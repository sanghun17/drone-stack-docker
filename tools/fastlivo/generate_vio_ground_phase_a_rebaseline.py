#!/usr/bin/env python3
"""Prepare, but never execute, the qualified ground-init Phase-A rebaseline.

This generator is deliberately narrower than the legacy hover-crop campaign
harness.  It accepts only a self-hashed *ground-init* qualification report
that explicitly grants ``low_rate_estimator_rebaseline_go``.  It then freezes
the existing eight-arm, five-development-session grid into forty independent
full-bag-start-to-cached-landing cells.  Each cell carries its session's tight
stationarity thresholds and explicit initialization anchor.

Every cell requires two evaluations of the same result bag: a primary full
safety report and a secondary hover-to-landing report which reuses (and may
not refit) the primary spatial alignment.  The generated workflow never
promotes a flight candidate and never clears the separately known high-rate
interface blocker.

The generator hashes inputs and probes the qualified isolated build, but it
does not invoke rosbag replay, ROS, Docker build, or the evaluator.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

from build_vio_postfix_init_qualification_receipt import (
    BUILD_SCHEMA,
    validate_build_manifest,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    EVALUATOR,
    FASTLIVO_BASE_CONFIG,
    REPLAY,
    REPLAY_LAUNCH,
    REQUIRED_ESTIMATOR_LIBRARIES,
    effective_overlay,
    file_identity,
    load_arms,
    load_json,
    object_sha256,
    sha256,
    validate_effective_overlay,
    validate_plan_identity,
    within_repo,
)


SCHEMA = "fastlivo_vio_ground_phase_a_orchestration/v1"
CELL_SCHEMA = "fastlivo_vio_ground_phase_a_cell/v1"
COMMANDS_SCHEMA = "fastlivo_vio_ground_phase_a_commands/v1"
ARMS_SCHEMA = "fastlivo_vio_tuning_arms/v1"
GROUND_ANCHORS_SCHEMA = "fastlivo_ground_init_anchors/v1"
QUALIFICATION_PLAN_SCHEMA = "fastlivo_vio_postfix_init_qualification_plan/v1"
QUALIFICATION_REPORT_SCHEMA = "fastlivo_vio_ground_init_qualification_report/v1"
QUALIFICATION_VARIANT = "ground_init_full_start_to_landing/v1"

TOOLS = Path(__file__).resolve().parent
PHASE_A_ARMS = TOOLS / "vio_flight_tuning_arms_phase_a.yaml"
RUNNER = TOOLS / "run_vio_ground_phase_a_rebaseline.py"
REPORTER = TOOLS / "build_vio_ground_phase_a_rebaseline_report.py"
DEFAULT_REFERENCE = (
    TOOLS / "_campaign_vio_flight_20260814" /
    "tuning_campaigns/phase_a_ofat_clean_v2"
)

SESSION_IDS: Tuple[str, ...] = (
    "pw1_20260804_052639",
    "pm2_20260805_020515",
    "p0_20260804_211027",
    "n0_20260805_021950",
    "pw3_20260804_053018",
)
ARM_IDS: Tuple[str, ...] = (
    "baseline_acc10_img1000_out1000",
    "acc5",
    "acc20",
    "img300",
    "img3000",
    "outlier100",
    "outlier300",
    "outlier600",
)
TIGHT_INIT_PARAMETERS: Mapping[str, Any] = {
    "imu_int_frame": 30,
    "init_anchor_max_predecessor_gap_s": 0.02,
    "init_max_gyr_mean": 0.01,
    "init_max_gyr_std": 0.04,
    "init_max_acc_std": 0.25,
    "init_acc_norm_tolerance": 0.10,
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _self_hash(document: Mapping[str, Any], schema: str, label: str) -> str:
    if document.get("schema") != schema:
        raise CampaignError(f"{label}: wrong schema")
    declared = document.get("identity_sha256")
    core = dict(document)
    core.pop("identity_sha256", None)
    if (not isinstance(declared, str) or
            SHA256_RE.fullmatch(declared) is None or
            object_sha256(core) != declared):
        raise CampaignError(f"{label}: self hash changed or is missing")
    return declared


def _write_bytes_exclusive(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _write_json_exclusive(path: Path, document: Mapping[str, Any]) -> None:
    _write_bytes_exclusive(
        path, (json.dumps(document, indent=2, sort_keys=True,
                          ensure_ascii=False) + "\n").encode("utf-8"))


def _load_yaml(path: Path, label: str) -> Dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise CampaignError(f"{label} must be a YAML mapping: {path}")
    return value


def _decimal_uint64(value: Any, label: str) -> str:
    if (not isinstance(value, str) or not value.isdigit() or
            value.startswith("0")):
        raise CampaignError(f"{label} must be a quoted positive decimal uint64")
    parsed = int(value)
    if parsed <= 0 or parsed > (1 << 64) - 1:
        raise CampaignError(f"{label} is outside uint64")
    return value


def _positive_finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise CampaignError(f"{label} must be finite and positive") from error
    if not math.isfinite(result) or result <= 0.0:
        raise CampaignError(f"{label} must be finite and positive")
    return result


def _file_binding(path: Path, *, known_sha256: Optional[str] = None) -> Dict[str, Any]:
    path = path.resolve()
    if not path.is_file():
        raise CampaignError(f"missing bound file: {path}")
    digest = sha256(path) if known_sha256 is None else known_sha256
    if SHA256_RE.fullmatch(str(digest)) is None:
        raise CampaignError(f"invalid bound file SHA-256: {path}")
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "mtime_ns": path.stat().st_mtime_ns,
        "sha256": str(digest),
    }


def _bind_document(path: Path, document: Mapping[str, Any], label: str) -> None:
    path = path.resolve()
    if not path.is_file() or load_json(path) != document:
        raise CampaignError(f"{label} path/document binding changed: {path}")


def _require_reference_dependency(
        reference: Mapping[str, Any], key: str, path: Path) -> None:
    dependencies = reference.get("dependencies")
    identity = dependencies.get(key) if isinstance(dependencies, Mapping) else None
    path = path.resolve()
    if (not isinstance(identity, Mapping) or
            Path(str(identity.get("path", ""))).resolve() != path or
            not path.is_file() or identity.get("sha256") != sha256(path)):
        raise CampaignError(
            f"controlled Phase-A dependency differs from clean-v2: {key}")


def _normalized_reference_arms(plan: Mapping[str, Any]) -> List[Dict[str, Any]]:
    raw = plan.get("arms")
    if not isinstance(raw, list):
        raise CampaignError("reference Phase-A plan lacks arms")
    result: List[Dict[str, Any]] = []
    for row in raw:
        if not isinstance(row, Mapping) or not isinstance(
                row.get("overrides"), Mapping):
            raise CampaignError("reference Phase-A arm is malformed")
        result.append({
            "id": str(row.get("id", "")),
            "overrides": copy.deepcopy(dict(row["overrides"])),
        })
    return result


def validate_reference(
        reference: Mapping[str, Any],
        canonical_arms: Sequence[Mapping[str, Any]]) -> Tuple[str, List[Mapping[str, Any]]]:
    identity = validate_plan_identity(reference)
    expected_arms = [
        {"id": str(row["id"]),
         "overrides": copy.deepcopy(dict(row["overrides"]))}
        for row in canonical_arms
    ]
    if ([row["id"] for row in expected_arms] != list(ARM_IDS) or
            _normalized_reference_arms(reference) != expected_arms):
        raise CampaignError("reference does not contain the exact frozen eight arms")
    replay = reference.get("replay")
    if (reference.get("campaign_id") != "phase_a_ofat_clean_v2" or
            reference.get("mode") != "full" or
            not isinstance(replay, Mapping) or replay.get("rate") != 1.0 or
            replay.get("no_gt_anchor") is not True or
            replay.get("with_propagated") is not True):
        raise CampaignError("reference is not the frozen clean-v2 Phase-A campaign")
    sessions = reference.get("sessions")
    if (not isinstance(sessions, list) or len(sessions) != len(SESSION_IDS) or
            [str(row.get("id", "")) for row in sessions
             if isinstance(row, Mapping)] != list(SESSION_IDS) or
            any(not isinstance(row, Mapping) or row.get("split") != "development"
                for row in sessions)):
        raise CampaignError("reference does not contain the exact five dev sessions")
    return identity, list(sessions)


def validate_ground_plan(
        plan: Mapping[str, Any], *, reference_identity: str,
        anchors_identity: str,
        canonical_arms: Sequence[Mapping[str, Any]]) -> str:
    identity = _self_hash(
        plan, QUALIFICATION_PLAN_SCHEMA, "ground qualification plan")
    contract = plan.get("postfix_phase_a_rebaseline")
    decision = plan.get("decision_contract")
    if (plan.get("qualification_variant") != QUALIFICATION_VARIANT or
            plan.get("session_window_mode") != "ground_to_landing" or
            plan.get("scope") != "development_only" or
            plan.get("validation_data_accessed") is not False or
            plan.get("execution_neutral") is not True or
            plan.get("reference_phase_a_campaign_identity_sha256") !=
            reference_identity or
            plan.get("anchors_artifact_identity_sha256") != anchors_identity or
            plan.get("frozen_phase_a_arms_sha256") !=
            object_sha256(list(canonical_arms)) or
            plan.get("predecessor_qualification_remains_fail_no_go") is not True or
            plan.get("high_rate_interface_remains_no_go") is not True):
        raise CampaignError("ground qualification plan scope/provenance changed")
    if (not isinstance(contract, Mapping) or contract.get("rate") != 1.0 or
            contract.get("repeats_per_arm_session") != 1 or
            contract.get("expected_arm_count") != 8 or
            contract.get("expected_run_count") != 40 or
            contract.get("expected_session_ids") != list(SESSION_IDS) or
            contract.get("old_scores_may_be_pooled") is not False or
            contract.get("reuse_old_completion_pointers") is not False or
            contract.get("validation_access_allowed") is not False or
            contract.get("window_mode") != "ground_to_landing"):
        raise CampaignError("ground Phase-A contract changed")
    if (not isinstance(decision, Mapping) or
            decision.get("high_rate_flight_interface_can_be_cleared_by_this_plan")
            is not False or
            decision.get("primary_full_result_safety_report_required") is not True or
            decision.get("secondary_hover_report_required") is not True or
            decision.get("secondary_cannot_override_primary_failure") is not True):
        raise CampaignError("ground qualification decision contract changed")
    return identity


def validate_anchors(
        anchors: Mapping[str, Any], *, reference_identity: str,
        reference_sessions: Sequence[Mapping[str, Any]],
        verify_actual_inputs: bool) -> Tuple[str, List[Dict[str, Any]]]:
    identity = _self_hash(anchors, GROUND_ANCHORS_SCHEMA, "ground anchors")
    if (anchors.get("scope") != "development_only" or
            anchors.get("validation_data_accessed") is not False or
            anchors.get("selection_uses_ground_truth") is not False or
            anchors.get("selection_uses_estimator_output") is not False or
            anchors.get("post_selection_ground_truth_audit_only") is not True or
            anchors.get("reference_phase_a_campaign_identity_sha256") !=
            reference_identity or anchors.get("session_count") != 5 or
            anchors.get("session_ids") != list(SESSION_IDS)):
        raise CampaignError("ground anchor scope/provenance changed")
    raw = anchors.get("sessions")
    if not isinstance(raw, list) or len(raw) != 5:
        raise CampaignError("ground anchor artifact lacks five sessions")
    reference_by_id = {str(row["id"]): row for row in reference_sessions}
    result: List[Dict[str, Any]] = []
    for expected_id, row in zip(SESSION_IDS, raw):
        if (not isinstance(row, Mapping) or
                row.get("session_id") != expected_id or
                row.get("split") != "development"):
            raise CampaignError("ground anchor session order/scope changed")
        input_row = row.get("input")
        anchor = row.get("anchor")
        stationarity = row.get("stationarity")
        audit = row.get("post_selection_ground_hard_gate")
        windows = row.get("windows")
        if not all(isinstance(value, Mapping) for value in (
                input_row, anchor, stationarity, audit, windows)):
            raise CampaignError(f"incomplete ground anchor session: {expected_id}")
        reference = reference_by_id[expected_id]
        if (input_row.get("path") != reference.get("input_bag") or
                input_row.get("declared_sha256") !=
                reference.get("input_declared_sha256") or
                input_row.get("input_provenance_sha256") !=
                reference.get("input_provenance_sha256") or
                input_row.get("verified_sha256") !=
                input_row.get("declared_sha256") or
                input_row.get("full_file_sha256_verified") is not True):
            raise CampaignError(f"ground input provenance mismatch: {expected_id}")
        bag = Path(str(input_row["path"])).resolve()
        if (not bag.is_file() or bag.stat().st_size !=
                int(input_row.get("size_bytes", -1))):
            raise CampaignError(f"ground input file changed: {bag}")
        if verify_actual_inputs and sha256(bag) != input_row["declared_sha256"]:
            raise CampaignError(f"ground input full SHA-256 changed: {bag}")
        provenance = Path(str(bag) + ".provenance.json").resolve()
        if (not provenance.is_file() or
                sha256(provenance) != input_row["input_provenance_sha256"]):
            raise CampaignError(f"ground input sidecar changed: {provenance}")
        sidecar = load_json(provenance)
        sidecar_output = sidecar.get("output")
        if (not isinstance(sidecar_output, Mapping) or
                sidecar_output.get("sha256") != input_row["declared_sha256"] or
                sidecar.get("topics", {}).get("output") !=
                "/camera/imu_hybrid"):
            raise CampaignError(f"ground input sidecar contract changed: {provenance}")
        cache = Path(str(audit.get("window_cache", ""))).resolve()
        if (not cache.is_file() or sha256(cache) !=
                audit.get("window_cache_sha256")):
            raise CampaignError(f"ground window cache changed: {cache}")
        if (stationarity.get("accepted") is not True or
                not isinstance(stationarity.get("checks"), Mapping) or
                not all(value is True for value in
                        stationarity["checks"].values()) or
                audit.get("passed") is not True or
                not isinstance(audit.get("checks"), Mapping) or
                not all(value is True for value in audit["checks"].values()) or
                audit.get("affects_anchor_selection") is not False or
                audit.get("uses_ground_truth_only_after_anchor_selection")
                is not True):
            raise CampaignError(f"ground stationarity/audit did not pass: {expected_id}")
        stamp = _decimal_uint64(
            anchor.get("anchor_stamp_ns"), f"{expected_id} anchor")
        if (anchor.get("anchor_mode_required") != "explicit" or
                anchor.get("anchor_definition") !=
                "earliest_explicit_eligible_full_sync_sensor_epoch" or
                anchor.get("init_anchor_max_predecessor_gap_s") != 0.02):
            raise CampaignError(f"ground anchor semantics changed: {expected_id}")
        replay = windows.get("replay")
        primary = windows.get("primary_evaluation")
        secondary = windows.get("secondary_hover_evaluation")
        if (not isinstance(replay, Mapping) or
                replay.get("start_offset_s") != 0.0 or
                replay.get("end_definition") != "frozen_cached_landing" or
                not isinstance(primary, Mapping) or primary.get("interval") !=
                "complete_recorded_ground_to_landing_result" or
                not isinstance(secondary, Mapping) or
                secondary.get("purpose") !=
                "phase_a_ranking_compatibility_only" or
                secondary.get("boundary") !=
                "start_inclusive_end_inclusive" or
                secondary.get("mask_basis") !=
                "result_stream_sensor_header_epoch" or
                secondary.get("interpretation") !=
                "absolute_ros_epoch_numeric_mask_with_mixed_frozen_sources" or
                secondary.get("start_epoch_origin") !=
                "gt_pose_header_time_from_frozen_cache" or
                secondary.get("end_epoch_origin") !=
                "mavros_landing_record_time_from_frozen_cache"):
            raise CampaignError(f"ground replay/evaluation window changed: {expected_id}")
        duration = _positive_finite(
            replay.get("duration_s"), f"{expected_id} replay duration")
        score_start = _decimal_uint64(
            secondary.get("start_absolute_ros_epoch_ns"),
            f"{expected_id} secondary start")
        score_end = _decimal_uint64(
            secondary.get("end_absolute_ros_epoch_ns"),
            f"{expected_id} secondary end")
        if int(score_end) <= int(score_start):
            raise CampaignError(f"secondary window is empty: {expected_id}")
        result.append({
            "session_id": expected_id,
            "condition": str(row.get("condition", "unspecified")),
            "split": "development",
            "input": {
                "path": str(bag),
                "size_bytes": bag.stat().st_size,
                "mtime_ns": bag.stat().st_mtime_ns,
                "declared_sha256": input_row["declared_sha256"],
                "verified_sha256": input_row["verified_sha256"],
                "full_file_sha256_verified_at_anchor_extraction": True,
                "full_file_sha256_reverified_at_phase_a_prepare":
                    verify_actual_inputs,
                "provenance": _file_binding(
                    provenance,
                    known_sha256=input_row["input_provenance_sha256"]),
                "window_cache": _file_binding(
                    cache, known_sha256=audit["window_cache_sha256"]),
            },
            "explicit_anchor": {
                "anchor_stamp_ns": stamp,
                "anchor_mode": "explicit",
                "anchor_definition": anchor["anchor_definition"],
                "init_anchor_max_predecessor_gap_s": 0.02,
            },
            "replay": {
                "rate": 1.0,
                "start_s": 0.0,
                "duration_s": duration,
                "end_definition": "frozen_cached_landing",
                "no_gt_anchor": True,
                "with_propagated": True,
            },
            "primary_evaluation": {
                "role": "full_result_safety_primary",
                "score_window_sensor_stamp_ns": None,
                "fixed_alignment_report": None,
            },
            "secondary_evaluation": {
                "role": "hover_to_landing_low_rate_ranking_only",
                "score_start_ns": score_start,
                "score_end_ns": score_end,
                "boundary": "start_inclusive_end_inclusive",
                "mask_basis": "result_stream_sensor_header_epoch",
                "interpretation":
                    "absolute_ros_epoch_numeric_mask_with_mixed_frozen_sources",
                "start_epoch_origin":
                    "gt_pose_header_time_from_frozen_cache",
                "end_epoch_origin":
                    "mavros_landing_record_time_from_frozen_cache",
                "alignment": "reuse_primary_without_refit",
                "can_override_primary_failure": False,
                "can_clear_high_rate_interface": False,
            },
        })
    return identity, result


def validate_qualification_report(
        report: Mapping[str, Any], *, plan_identity: str,
        build_identity: str, executable_sha256: str) -> str:
    identity = _self_hash(
        report, QUALIFICATION_REPORT_SCHEMA, "ground qualification report")
    if (report.get("qualification_variant") != QUALIFICATION_VARIANT or
            report.get("scope") != "development_only" or
            report.get("validation_data_accessed") is not False or
            report.get("status") != "pass" or report.get("failures") != [] or
            report.get("low_rate_estimator_rebaseline_go") is not True or
            report.get("go_for_ground_init_low_rate_phase_a_rebaseline")
            is not True or
            report.get("high_rate_interface_remains_no_go") is not True or
            report.get("high_rate_interface_status") != "NO_GO" or
            report.get("primary_strict_flight_status") != "NO_GO" or
            report.get("secondary_can_override_primary_failure") is not False or
            report.get("predecessor_qualification_status") != "fail" or
            report.get("predecessor_qualification_superseded") is not False or
            report.get("plan_identity_sha256") != plan_identity or
            report.get("build_manifest_identity_sha256") != build_identity or
            report.get("postfix_build_identity_sha256") != build_identity or
            report.get("postfix_executable_sha256") != executable_sha256 or
            report.get("receipt_count") != 12 or
            report.get("fresh_process_instance_count") != 12 or
            report.get("not_a_flight_readiness_decision") is not True or
            report.get("flight_ready") is not False):
        raise CampaignError(
            "ground qualification report is not an explicit, bound low-rate GO")
    if report.get("high_rate_flight_interface_go") not in (None, False):
        raise CampaignError("ground qualification attempted to clear high-rate GO")
    return identity


def _validate_build(
        build: Mapping[str, Any], reference: Mapping[str, Any], *,
        verify_actual_build: bool) -> str:
    if build.get("schema") != BUILD_SCHEMA:
        raise CampaignError("qualified build manifest has the wrong schema")
    identity = validate_build_manifest(
        build, verify_actual=verify_actual_build)
    if build.get("derived_from_actual_isolated_devel_and_source") is not True:
        raise CampaignError("qualified build is not derived from isolated inputs")
    if set(build.get("dynamic_libraries", {})) != set(
            REQUIRED_ESTIMATOR_LIBRARIES):
        raise CampaignError("qualified build library inventory is not exact")
    old = reference.get("build")
    if isinstance(old, Mapping):
        old_libraries = {
            str(name): row.get("sha256")
            for name, row in old.get("dynamic_libraries", {}).items()
            if isinstance(row, Mapping)
        } if isinstance(old.get("dynamic_libraries"), Mapping) else {}
        if (build.get("executable_sha256") == old.get("executable_sha256") and
                build.get("dynamic_libraries") == old_libraries):
            raise CampaignError("ground-qualified build equals the pre-fix build")
    return identity


def validate_runtime_constant_binding(
        plan: Mapping[str, Any], anchors: Mapping[str, Any],
        build: Mapping[str, Any]) -> Dict[str, Any]:
    """Bind the anchor gate's exact 9.81 constant to qualified source bytes."""
    binding = anchors.get("runtime_constant_binding")
    plan_binding = plan.get("runtime_constant_binding")
    required_keys = (
        "gravity_macro", "expected_literal",
        "source_relative_to_estimator_root", "source_sha256",
    )
    if (not isinstance(binding, Mapping) or
            not isinstance(plan_binding, Mapping) or
            {key: binding.get(key) for key in required_keys} !=
            {key: plan_binding.get(key) for key in required_keys} or
            binding.get("gravity_macro") != "G_m_s2" or
            binding.get("expected_literal") != 9.81 or
            binding.get("source_relative_to_estimator_root") !=
            "include/common_lib.h" or
            SHA256_RE.fullmatch(str(binding.get("source_sha256", ""))) is None):
        raise CampaignError("runtime gravity-constant binding changed")
    source = build.get("source_tree_identity")
    files = source.get("files") if isinstance(source, Mapping) else None
    identity = files.get("include/common_lib.h") \
        if isinstance(files, Mapping) else None
    if (not isinstance(identity, Mapping) or
            identity.get("sha256") != binding["source_sha256"] or
            not isinstance(identity.get("size_bytes"), int) or
            identity.get("size_bytes", 0) <= 0):
        raise CampaignError(
            "qualified build source does not contain the bound 9.81 constant file")
    return {key: copy.deepcopy(binding[key]) for key in required_keys}


def _runtime_overrides(
        arm: Mapping[str, Any], anchor_stamp_ns: str) -> Dict[str, Any]:
    overrides = copy.deepcopy(dict(arm["overrides"]))
    raw_imu = overrides.get("imu")
    if raw_imu is not None and not isinstance(raw_imu, Mapping):
        raise CampaignError(f"arm {arm['id']} has scalar imu overrides")
    imu = dict(raw_imu or {})
    forbidden = set(TIGHT_INIT_PARAMETERS) | {"init_anchor_stamp_ns"}
    overlap = sorted(forbidden.intersection(imu))
    if overlap:
        raise CampaignError(
            f"arm {arm['id']} modifies frozen ground-init keys: {overlap}")
    imu.update(copy.deepcopy(dict(TIGHT_INIT_PARAMETERS)))
    imu["init_anchor_stamp_ns"] = anchor_stamp_ns
    overrides["imu"] = imu
    return overrides


def prepare_orchestration(
        qualification_plan: Mapping[str, Any],
        qualification_report: Mapping[str, Any], build: Mapping[str, Any],
        anchors: Mapping[str, Any], reference: Mapping[str, Any],
        canonical_arms: Sequence[Mapping[str, Any]],
        base_overlay: Mapping[str, Any], *,
        qualification_plan_path: Path, qualification_report_path: Path,
        build_path: Path, anchors_path: Path, reference_campaign: Path,
        arms_path: Path, base_overlay_path: Path, thresholds_path: Path,
        output_root: Path, port: int, python_executable: str = sys.executable,
        verify_actual_build: bool = True,
        verify_actual_inputs: bool = True) -> Dict[str, Any]:
    reference_identity, reference_sessions = validate_reference(
        reference, canonical_arms)
    anchors_identity, sessions = validate_anchors(
        anchors, reference_identity=reference_identity,
        reference_sessions=reference_sessions,
        verify_actual_inputs=verify_actual_inputs)
    plan_identity = validate_ground_plan(
        qualification_plan, reference_identity=reference_identity,
        anchors_identity=anchors_identity, canonical_arms=canonical_arms)
    build_identity = _validate_build(
        build, reference, verify_actual_build=verify_actual_build)
    runtime_constant_binding = validate_runtime_constant_binding(
        qualification_plan, anchors, build)
    report_identity = validate_qualification_report(
        qualification_report, plan_identity=plan_identity,
        build_identity=build_identity,
        executable_sha256=str(build.get("executable_sha256", "")))

    bindings = (
        (qualification_plan_path, qualification_plan, "qualification plan"),
        (qualification_report_path, qualification_report, "qualification report"),
        (build_path, build, "qualified build manifest"),
        (anchors_path, anchors, "ground anchor artifact"),
        (reference_campaign / "campaign.json", reference,
         "pre-fix reference campaign"),
    )
    for path, document, label in bindings:
        _bind_document(path, document, label)
    if load_arms(arms_path.resolve()) != list(canonical_arms):
        raise CampaignError("frozen Phase-A arm file changed")
    if _load_yaml(base_overlay_path.resolve(), "base overlay") != base_overlay:
        raise CampaignError("base overlay path/document binding changed")
    for path, label in (
            (thresholds_path, "thresholds"), (RUNNER, "ground Phase-A runner"),
            (REPORTER, "ground Phase-A report builder"),
            (EVALUATOR, "strict evaluator"), (REPLAY, "replay wrapper"),
            (REPLAY_LAUNCH, "replay launch"),
            (FASTLIVO_BASE_CONFIG, "FAST-LIVO base config")):
        if not path.resolve().is_file():
            raise CampaignError(f"missing {label}: {path.resolve()}")
    # Preserve the controlled OFAT comparison inputs that did not need a
    # post-fix implementation change.  The replay wrapper and evaluator are
    # intentionally current and independently bound because they contain the
    # provenance/dual-window fixes this workflow requires.
    for key, path in (
            ("arms", arms_path),
            ("base_overlay", base_overlay_path),
            ("thresholds", thresholds_path),
            ("replay_launch", REPLAY_LAUNCH)):
        _require_reference_dependency(reference, key, path)

    expected_ground_overrides = qualification_plan.get(
        "postfix_phase_a_explicit_anchor_overrides")
    if not isinstance(expected_ground_overrides, Mapping):
        raise CampaignError("qualification plan lacks ground-init overrides")
    session_by_id = {row["session_id"]: row for row in sessions}
    for session_id in SESSION_IDS:
        expected = {"imu": {
            **copy.deepcopy(dict(TIGHT_INIT_PARAMETERS)),
            "init_anchor_stamp_ns": session_by_id[session_id][
                "explicit_anchor"]["anchor_stamp_ns"],
        }}
        if expected_ground_overrides.get(session_id) != expected:
            raise CampaignError(
                f"qualification plan tight override differs: {session_id}")

    output_root = within_repo(
        output_root.resolve(), "ground Phase-A output root")
    if output_root.exists():
        raise FileExistsError(output_root)
    if not 1024 <= int(port) <= 65535:
        raise CampaignError("port must be between 1024 and 65535")

    dependencies = {
        "generator": file_identity(Path(__file__).resolve()),
        "runner": file_identity(RUNNER.resolve()),
        "phase_a_reporter": file_identity(REPORTER.resolve()),
        "strict_evaluator": file_identity(EVALUATOR.resolve()),
        "replay_wrapper": file_identity(REPLAY.resolve()),
        "replay_launch": file_identity(REPLAY_LAUNCH.resolve()),
        "fastlivo_base_config": file_identity(FASTLIVO_BASE_CONFIG.resolve()),
        "thresholds": file_identity(thresholds_path.resolve()),
        "base_overlay": file_identity(base_overlay_path.resolve()),
        "frozen_phase_a_arms": file_identity(arms_path.resolve()),
        "qualification_plan": file_identity(qualification_plan_path.resolve()),
        "qualification_report": file_identity(
            qualification_report_path.resolve()),
        "qualified_build_manifest": file_identity(build_path.resolve()),
        "ground_anchors": file_identity(anchors_path.resolve()),
        "prefix_v2_reference_plan": file_identity(
            (reference_campaign / "campaign.json").resolve()),
    }

    output_root.mkdir(parents=True, exist_ok=False)
    cells_dir = output_root / "cells"
    cell_rows: List[Dict[str, Any]] = []
    commands: List[List[str]] = []
    ordinal = 0
    for arm in canonical_arms:
        for session_id in SESSION_IDS:
            ordinal += 1
            session = session_by_id[session_id]
            runtime = _runtime_overrides(
                arm, session["explicit_anchor"]["anchor_stamp_ns"])
            effective = effective_overlay(
                base_overlay, {"id": arm["id"], "overrides": runtime})
            validate_effective_overlay(effective)
            run_id = f"ga{ordinal:02d}_{arm['id']}__{session_id}"
            campaign_id = f"ground_a_{ordinal:02d}_{arm['id']}_{session_id}"
            cell_core: Dict[str, Any] = {
                "schema": CELL_SCHEMA,
                "scope": "development_only",
                "validation_data_accessed": False,
                "ordinal": ordinal,
                "run_id": run_id,
                "campaign_id": campaign_id,
                "arm_id": arm["id"],
                "phase_a_arm_overrides": copy.deepcopy(dict(arm["overrides"])),
                "runtime_overrides": runtime,
                "effective_overlay_sha256": object_sha256(effective),
                "session": copy.deepcopy(session),
                "qualification": {
                    "variant": QUALIFICATION_VARIANT,
                    "plan_identity_sha256": plan_identity,
                    "report_identity_sha256": report_identity,
                    "low_rate_estimator_rebaseline_go": True,
                },
                "qualified_build_identity_sha256": build_identity,
                "qualified_executable_sha256": build["executable_sha256"],
                "replay_process": {
                    "fresh_campaign_required": True,
                    "fresh_process_required": True,
                    "sequential_execution_required": True,
                    "rate": 1.0,
                },
                "evaluation_contract": {
                    "primary_full_result_safety_report_required": True,
                    "secondary_hover_report_required": True,
                    "secondary_reuses_primary_alignment_without_refit": True,
                    "secondary_cannot_override_primary_failure": True,
                    "primary_and_secondary_must_bind_same_result_bag": True,
                },
                "decision_contract": {
                    "old_prefix_scores_role": "diagnostic_only",
                    "old_scores_may_be_pooled": False,
                    "candidate_promotion_allowed": False,
                    "flight_ready_can_be_declared": False,
                    "high_rate_interface_remains_no_go": True,
                },
            }
            cell = {**cell_core, "identity_sha256": object_sha256(cell_core)}
            cell_path = cells_dir / f"{run_id}.json"
            _write_json_exclusive(cell_path, cell)
            command = [
                python_executable, str(RUNNER.resolve()), "cell",
                str((output_root / "orchestration.json").resolve()),
                "--cell-id", run_id,
            ]
            commands.append(command)
            cell_rows.append({
                "ordinal": ordinal,
                "run_id": run_id,
                "campaign_id": campaign_id,
                "arm_id": arm["id"],
                "session_id": session_id,
                "cell": file_identity(cell_path),
                "cell_object_identity_sha256": cell["identity_sha256"],
                "command": command,
            })

    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "qualification_variant": QUALIFICATION_VARIANT,
        "qualification_gate": {
            "plan_identity_sha256": plan_identity,
            "report_identity_sha256": report_identity,
            "low_rate_estimator_rebaseline_go": True,
            "self_hashed_report_required": True,
        },
        "qualified_build": {
            "identity_sha256": build_identity,
            "container": build["container"],
            "replay_devel": str(build["replay_devel"]).rstrip("/"),
            "executable_sha256": build["executable_sha256"],
            "dynamic_libraries": copy.deepcopy(build["dynamic_libraries"]),
            "source_tree_sha256": build["source_tree_sha256"],
            "actual_build_and_source_verified_at_prepare": verify_actual_build,
        },
        "runtime_constant_binding": runtime_constant_binding,
        "design": {
            "arm_ids": list(ARM_IDS),
            "session_ids": list(SESSION_IDS),
            "arm_count": 8,
            "session_count": 5,
            "expected_run_count": 40,
            "repeats_per_arm_session": 1,
            "rate": 1.0,
            "window": "full_bag_record_start_to_frozen_cached_landing",
            "fresh_campaign_per_cell": True,
            "fresh_process_per_cell": True,
            "strictly_sequential": True,
        },
        "evaluation_contract": {
            "primary": "complete_ground_to_landing_safety",
            "secondary": "hover_to_landing_low_rate_ranking_only",
            "secondary_alignment": "reuse_primary_without_refit",
            "secondary_can_override_primary_failure": False,
            "same_result_bag_required": True,
        },
        "decision_contract": {
            "old_prefix_scores_role": "diagnostic_only",
            "old_scores_may_be_pooled": False,
            "reuse_old_completion_pointers": False,
            "candidate_promotion_allowed": False,
            "flight_ready_can_be_declared": False,
            "high_rate_interface_remains_no_go": True,
        },
        "prefix_v2_reference": {
            "campaign_identity_sha256": reference_identity,
            "comparison_role": "diagnostic_only_no_pooling",
        },
        "output_root": str(output_root),
        "ros_master_port": int(port),
        "dependencies": dependencies,
        "cells": cell_rows,
        "commands": commands,
        "execution": {
            "generator_executed_replay": False,
            "generator_executed_build": False,
            "run_exact_commands_in_list_order": True,
            "sequence_command": [
                python_executable, str(RUNNER.resolve()), "sequence",
                str((output_root / "orchestration.json").resolve()),
                "--commands",
                str((output_root / "commands.json").resolve()),
            ],
            "report_command": [
                python_executable, str(REPORTER.resolve()),
                str((output_root / "orchestration.json").resolve()),
                "--output", str((output_root / "phase_a_report.json").resolve()),
            ],
        },
    }
    orchestration = {**core, "identity_sha256": object_sha256(core)}
    orchestration_path = output_root / "orchestration.json"
    _write_json_exclusive(orchestration_path, orchestration)
    commands_core: Dict[str, Any] = {
        "schema": COMMANDS_SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "orchestration_path": str(orchestration_path.resolve()),
        "orchestration_identity_sha256": orchestration["identity_sha256"],
        "strictly_sequential": True,
        "fresh_campaign_per_cell": True,
        "command_count": 40,
        "commands": commands,
    }
    commands_document = {
        **commands_core, "identity_sha256": object_sha256(commands_core)}
    _write_json_exclusive(output_root / "commands.json", commands_document)
    return orchestration


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("qualification_plan", type=Path)
    parser.add_argument("qualification_report", type=Path)
    parser.add_argument("build_manifest", type=Path)
    parser.add_argument("ground_anchors", type=Path)
    parser.add_argument("--reference-campaign", type=Path,
                        default=DEFAULT_REFERENCE)
    parser.add_argument("--arms", type=Path, default=PHASE_A_ARMS)
    parser.add_argument("--base-overlay", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--port", type=int, default=11431)
    arguments = parser.parse_args(argv)
    try:
        required = (
            arguments.qualification_plan, arguments.qualification_report,
            arguments.build_manifest, arguments.ground_anchors,
            arguments.reference_campaign / "campaign.json", arguments.arms,
            arguments.base_overlay, arguments.thresholds,
        )
        missing = [str(path.resolve()) for path in required
                   if not path.resolve().is_file()]
        if missing:
            raise CampaignError("missing input: " + ", ".join(missing))
        orchestration = prepare_orchestration(
            load_json(arguments.qualification_plan.resolve()),
            load_json(arguments.qualification_report.resolve()),
            load_json(arguments.build_manifest.resolve()),
            load_json(arguments.ground_anchors.resolve()),
            load_json((arguments.reference_campaign / "campaign.json").resolve()),
            load_arms(arguments.arms.resolve()),
            _load_yaml(arguments.base_overlay.resolve(), "base overlay"),
            qualification_plan_path=arguments.qualification_plan.resolve(),
            qualification_report_path=arguments.qualification_report.resolve(),
            build_path=arguments.build_manifest.resolve(),
            anchors_path=arguments.ground_anchors.resolve(),
            reference_campaign=arguments.reference_campaign.resolve(),
            arms_path=arguments.arms.resolve(),
            base_overlay_path=arguments.base_overlay.resolve(),
            thresholds_path=arguments.thresholds.resolve(),
            output_root=arguments.output_root,
            port=arguments.port,
        )
        print(json.dumps({
            "orchestration": str(
                (arguments.output_root / "orchestration.json").resolve()),
            "commands": str((arguments.output_root / "commands.json").resolve()),
            "identity_sha256": orchestration["identity_sha256"],
            "expected_run_count": 40,
            "low_rate_estimator_rebaseline_go": True,
            "high_rate_interface_remains_no_go": True,
            "replay_executed": False,
            "build_executed": False,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

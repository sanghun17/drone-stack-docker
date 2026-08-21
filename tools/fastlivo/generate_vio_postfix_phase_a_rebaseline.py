#!/usr/bin/env python3
"""Prepare the qualified post-fix Phase-A rebaseline without running it.

The generator is deliberately execution-neutral.  It accepts only a passing,
self-hashed init-qualification report, binds the exact qualified isolated build,
and writes five session-specific copies of the frozen eight-arm OFAT design.
Each copy adds that development session's explicit initialization anchor.  The
commands it emits use the standard append-only campaign harness at rate 1.0;
the generator itself never opens a bag, invokes Docker, builds code, or starts
ROS.

One standard-harness campaign is used per development session.  Consequently
each of its eight arm/session cells has one append-only attempt/completion
pointer, while an interrupted session command remains safely resumable.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import yaml

from run_vio_flight_tuning_campaign import (
    CampaignError,
    EVALUATOR,
    REPLAY,
    REPLAY_LAUNCH,
    REQUIRED_ESTIMATOR_LIBRARIES,
    deep_merge,
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


SCHEMA = "fastlivo_vio_postfix_phase_a_orchestration/v1"
COMMANDS_SCHEMA = "fastlivo_vio_postfix_phase_a_commands/v1"
ARMS_SCHEMA = "fastlivo_vio_tuning_arms/v1"
ANCHORS_SCHEMA = "fastlivo_earliest_full_sync_anchors/v1"
QUALIFICATION_PLAN_SCHEMA = "fastlivo_vio_postfix_init_qualification_plan/v1"
QUALIFICATION_REPORT_SCHEMA = "fastlivo_vio_postfix_init_qualification_report/v1"
BUILD_SCHEMA = "fastlivo_vio_postfix_build_manifest/v1"
PHASE_A_ARMS = Path(__file__).with_name("vio_flight_tuning_arms_phase_a.yaml")
HARNESS = Path(__file__).with_name("run_vio_flight_tuning_campaign.py")
DEFAULT_REFERENCE = (
    Path(__file__).with_name("_campaign_vio_flight_20260814") /
    "tuning_campaigns/phase_a_ofat_clean_v2"
)

# This order is copied from the complete clean-v2 campaign and from the
# qualification preregistration.  It is intentionally narrower than the
# broader eight-ID development allow-list used by exploratory tools.
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
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def _self_hash(document: Mapping[str, Any], schema: str, label: str) -> str:
    if document.get("schema") != schema:
        raise CampaignError(f"{label} has the wrong schema")
    declared = document.get("identity_sha256")
    if not isinstance(declared, str) or not SHA256_RE.fullmatch(declared):
        raise CampaignError(f"{label} has no valid identity SHA-256")
    core = dict(document)
    core.pop("identity_sha256", None)
    if object_sha256(core) != declared:
        raise CampaignError(f"{label} self-hash changed")
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
        path,
        (json.dumps(document, indent=2, sort_keys=True, ensure_ascii=False) +
         "\n").encode("utf-8"),
    )


def _decimal_uint64(value: Any, label: str) -> str:
    if (not isinstance(value, str) or not value.isdigit() or
            value.startswith("0")):
        raise CampaignError(f"{label} must be a quoted positive decimal uint64")
    parsed = int(value)
    if parsed <= 0 or parsed > (1 << 64) - 1:
        raise CampaignError(f"{label} is outside uint64")
    return value


def _require_reference_dependency(
        dependencies: Mapping[str, Any], key: str, actual_path: Path) -> None:
    identity = dependencies.get(key)
    actual_path = actual_path.resolve()
    if (not isinstance(identity, Mapping) or not actual_path.is_file() or
            Path(str(identity.get("path", ""))).resolve() != actual_path or
            identity.get("sha256") != sha256(actual_path)):
        raise CampaignError(
            f"controlled-comparison dependency differs from pre-fix v2: {key}")


def _normalized_plan_arms(plan: Mapping[str, Any]) -> List[Dict[str, Any]]:
    raw = plan.get("arms")
    if not isinstance(raw, list):
        raise CampaignError("reference Phase-A plan has no arms")
    result: List[Dict[str, Any]] = []
    for row in raw:
        if not isinstance(row, Mapping) or not isinstance(
                row.get("overrides"), Mapping):
            raise CampaignError("reference Phase-A plan has a malformed arm")
        result.append({
            "id": str(row.get("id", "")),
            "overrides": copy.deepcopy(dict(row["overrides"])),
        })
    return result


def _library_hashes_from_campaign_build(
        build: Mapping[str, Any]) -> Dict[str, str]:
    raw = build.get("dynamic_libraries")
    if not isinstance(raw, Mapping):
        raise CampaignError("reference campaign has no estimator libraries")
    result: Dict[str, str] = {}
    for name in REQUIRED_ESTIMATOR_LIBRARIES:
        row = raw.get(name)
        value = row.get("sha256") if isinstance(row, Mapping) else None
        if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
            raise CampaignError(f"reference build has invalid {name} fingerprint")
        result[name] = value
    return result


def validate_build_manifest(build: Mapping[str, Any]) -> str:
    identity = _self_hash(build, BUILD_SCHEMA, "post-fix build manifest")
    if build.get("derived_from_actual_isolated_devel_and_source") is not True:
        raise CampaignError("post-fix build was not derived from an actual isolated build")
    executable = build.get("executable_sha256")
    source = build.get("source_tree_sha256")
    if (not isinstance(executable, str) or not SHA256_RE.fullmatch(executable) or
            not isinstance(source, str) or not SHA256_RE.fullmatch(source)):
        raise CampaignError("post-fix build has invalid executable/source identity")
    libraries = build.get("dynamic_libraries")
    if not isinstance(libraries, Mapping) or set(libraries) != set(
            REQUIRED_ESTIMATOR_LIBRARIES):
        raise CampaignError("post-fix build library inventory is not exact")
    if any(not isinstance(libraries[name], str) or
           not SHA256_RE.fullmatch(str(libraries[name]))
           for name in REQUIRED_ESTIMATOR_LIBRARIES):
        raise CampaignError("post-fix build has an invalid library fingerprint")
    binary = build.get("binary_identity")
    if (not isinstance(binary, Mapping) or
            binary.get("container") != build.get("container") or
            binary.get("replay_devel") != build.get("replay_devel") or
            binary.get("executable_sha256") != executable):
        raise CampaignError("post-fix build binary identity is internally inconsistent")
    replay_devel = str(build.get("replay_devel", "")).rstrip("/")
    if replay_devel in {"/work/ws/fast-livo/devel", "/work/devel", ""}:
        raise CampaignError("a fresh isolated post-fix devel is mandatory")
    return identity


def validate_reference(
        reference_plan: Mapping[str, Any],
        canonical_arms: Sequence[Mapping[str, Any]]) -> Tuple[str, List[Mapping[str, Any]]]:
    identity = validate_plan_identity(reference_plan)
    if (reference_plan.get("mode") != "full" or
            reference_plan.get("replay", {}).get("rate") != 1.0 or
            reference_plan.get("replay", {}).get("no_gt_anchor") is not True or
            reference_plan.get("replay", {}).get("with_propagated") is not True):
        raise CampaignError("reference is not the frozen full rate-1 Phase-A campaign")
    expected_arms = [
        {"id": str(row["id"]), "overrides": copy.deepcopy(row["overrides"])}
        for row in canonical_arms
    ]
    if ([row["id"] for row in expected_arms] != list(ARM_IDS) or
            _normalized_plan_arms(reference_plan) != expected_arms):
        raise CampaignError("reference does not use the exact frozen eight Phase-A arms")
    sessions = reference_plan.get("sessions")
    if (not isinstance(sessions, list) or
            [str(row.get("id", "")) for row in sessions
             if isinstance(row, Mapping)] != list(SESSION_IDS) or
            len(sessions) != len(SESSION_IDS) or
            any(not isinstance(row, Mapping) or row.get("split") != "development"
                for row in sessions)):
        raise CampaignError("reference does not use the exact five-session dev grid")
    return identity, list(sessions)


def validate_anchors(
        anchors: Mapping[str, Any], reference_identity: str,
        reference_sessions: Sequence[Mapping[str, Any]]) -> Tuple[str, Dict[str, str]]:
    identity = _self_hash(anchors, ANCHORS_SCHEMA, "explicit-anchor artifact")
    if (anchors.get("scope") != "development_only" or
            anchors.get("validation_data_accessed") is not False or
            anchors.get("reference_phase_a_campaign_identity_sha256") !=
            reference_identity or anchors.get("session_count") != len(SESSION_IDS) or
            anchors.get("session_ids") != list(SESSION_IDS)):
        raise CampaignError("explicit-anchor artifact scope/reference/grid changed")
    raw = anchors.get("sessions")
    if not isinstance(raw, list) or len(raw) != len(SESSION_IDS):
        raise CampaignError("explicit-anchor artifact does not cover five sessions")
    reference_by_id = {str(row["id"]): row for row in reference_sessions}
    result: Dict[str, str] = {}
    for row in raw:
        if not isinstance(row, Mapping):
            raise CampaignError("malformed explicit-anchor session")
        session_id = str(row.get("session_id", ""))
        if (session_id not in SESSION_IDS or session_id in result or
                row.get("split") != "development"):
            raise CampaignError(f"unexpected explicit-anchor session {session_id!r}")
        reference = reference_by_id[session_id]
        input_row = row.get("input")
        crop = row.get("crop")
        anchor = row.get("anchor")
        if not all(isinstance(value, Mapping)
                   for value in (input_row, crop, anchor)):
            raise CampaignError(f"malformed input/crop/anchor for {session_id}")
        if (input_row.get("path") != reference.get("input_bag") or
                input_row.get("declared_sha256") !=
                reference.get("input_declared_sha256") or
                input_row.get("input_provenance_sha256") !=
                reference.get("input_provenance_sha256") or
                input_row.get("full_file_sha256_verified") is not True):
            raise CampaignError(f"anchor input provenance differs for {session_id}")
        reference_crop = reference.get("crop", {})
        for field in ("basis", "start_s", "duration_s", "full_duration_s",
                      "smoke_truncated", "window_method"):
            if crop.get(field) != reference_crop.get(field):
                raise CampaignError(f"anchor crop differs for {session_id}: {field}")
        stamp = _decimal_uint64(
            anchor.get("anchor_stamp_ns"), f"{session_id} anchor_stamp_ns")
        if (anchor.get("anchor_mode_required") != "explicit" or
                anchor.get("anchor_definition") !=
                "earliest_explicit_eligible_full_sync_sensor_epoch" or
                anchor.get("init_anchor_max_predecessor_gap_s") != 0.02):
            raise CampaignError(f"anchor semantics differ for {session_id}")
        result[session_id] = stamp
    if list(result) != list(SESSION_IDS):
        raise CampaignError("explicit-anchor session order changed")
    return identity, result


def validate_qualification_go(
        qualification_plan: Mapping[str, Any],
        qualification_report: Mapping[str, Any],
        build: Mapping[str, Any], reference_plan: Mapping[str, Any],
        reference_identity: str, anchors_identity: str,
        anchor_stamps: Mapping[str, str], canonical_arms: Sequence[Mapping[str, Any]],
        *, container: str, replay_devel: str) -> Tuple[str, str, str]:
    plan_identity = _self_hash(
        qualification_plan, QUALIFICATION_PLAN_SCHEMA, "qualification plan")
    report_identity = _self_hash(
        qualification_report, QUALIFICATION_REPORT_SCHEMA,
        "qualification report")
    build_identity = validate_build_manifest(build)
    if (qualification_report.get("scope") != "development_only" or
            qualification_report.get("validation_data_accessed") is not False or
            qualification_report.get("status") != "pass" or
            qualification_report.get("go_for_postfix_phase_a_rebaseline") is not True or
            qualification_report.get("failures") != [] or
            qualification_report.get("plan_identity_sha256") != plan_identity or
            qualification_report.get("postfix_build_identity_sha256") != build_identity or
            qualification_report.get("postfix_executable_sha256") !=
            build.get("executable_sha256")):
        raise CampaignError("post-fix initialization qualification is not a bound GO")
    if (qualification_plan.get("scope") != "development_only" or
            qualification_plan.get("validation_data_accessed") is not False or
            qualification_plan.get("reference_phase_a_campaign_identity_sha256") !=
            reference_identity or
            qualification_plan.get("anchors_artifact_identity_sha256") !=
            anchors_identity or
            qualification_plan.get("frozen_phase_a_arms_sha256") !=
            object_sha256(list(canonical_arms))):
        raise CampaignError("qualification plan is not bound to this Phase-A design")
    contract = qualification_plan.get("postfix_phase_a_rebaseline")
    if (not isinstance(contract, Mapping) or contract.get("rate") != 1.0 or
            contract.get("repeats_per_arm_session") != 1 or
            contract.get("expected_arm_count") != 8 or
            contract.get("expected_run_count") != 40 or
            contract.get("expected_session_ids") != list(SESSION_IDS) or
            contract.get("old_scores_may_be_pooled") is not False or
            contract.get("reuse_old_completion_pointers") is not False or
            contract.get("validation_access_allowed") is not False):
        raise CampaignError("qualification plan Phase-A contract changed")
    expected_overrides = {
        session_id: {"imu": {
            "init_anchor_stamp_ns": anchor_stamps[session_id],
            "init_anchor_max_predecessor_gap_s": 0.02,
        }} for session_id in SESSION_IDS
    }
    if qualification_plan.get(
            "postfix_phase_a_explicit_anchor_overrides") != expected_overrides:
        raise CampaignError("qualification plan explicit anchors differ from artifact")
    if (build.get("container") != container or
            str(build.get("replay_devel", "")).rstrip("/") !=
            replay_devel.rstrip("/")):
        raise CampaignError("requested runner does not name the qualified build")
    old_build = reference_plan.get("build")
    if not isinstance(old_build, Mapping):
        raise CampaignError("reference Phase-A build is missing")
    same_executable = (old_build.get("executable_sha256") ==
                       build.get("executable_sha256"))
    same_libraries = (_library_hashes_from_campaign_build(old_build) ==
                      dict(build.get("dynamic_libraries", {})))
    if same_executable and same_libraries:
        raise CampaignError("qualified build is identical to the pre-fix build")
    return plan_identity, report_identity, build_identity


def _session_arm_document(
        canonical_arms: Sequence[Mapping[str, Any]], session_id: str,
        anchor_stamp_ns: str, *, reference_identity: str,
        qualification_report_identity: str,
        build_identity: str, anchors_identity: str) -> Dict[str, Any]:
    arms: List[Dict[str, Any]] = []
    for arm in canonical_arms:
        overrides = copy.deepcopy(dict(arm["overrides"]))
        if (isinstance(overrides.get("imu"), Mapping) and
                any(key in overrides["imu"] for key in (
                    "init_anchor_stamp_ns", "init_anchor_max_predecessor_gap_s"))):
            raise CampaignError("frozen Phase-A arm already modifies init anchor")
        deep_merge(overrides, {"imu": {
            "init_anchor_stamp_ns": anchor_stamp_ns,
            "init_anchor_max_predecessor_gap_s": 0.02,
        }})
        arms.append({"id": str(arm["id"]), "overrides": overrides})
    return {
        "schema": ARMS_SCHEMA,
        "postfix_phase_a_provenance": {
            "session_id": session_id,
            "reference_prefix_campaign_identity_sha256": reference_identity,
            "qualification_report_identity_sha256": qualification_report_identity,
            "qualified_build_identity_sha256": build_identity,
            "anchors_artifact_identity_sha256": anchors_identity,
            "anchor_stamp_ns": anchor_stamp_ns,
            "init_anchor_max_predecessor_gap_s": 0.02,
            "old_scores_may_be_pooled": False,
            "validation_data_accessed": False,
        },
        "arms": arms,
    }


def prepare_orchestration(
        qualification_plan: Mapping[str, Any],
        qualification_report: Mapping[str, Any], build: Mapping[str, Any],
        anchors: Mapping[str, Any], reference_plan: Mapping[str, Any],
        canonical_arms: Sequence[Mapping[str, Any]], base_overlay: Mapping[str, Any],
        *, qualification_plan_path: Path, qualification_report_path: Path,
        build_path: Path, anchors_path: Path, reference_campaign: Path,
        arms_source_path: Path, base_overlay_path: Path, thresholds_path: Path,
        output_root: Path, container: str, replay_devel: str, port: int,
        python_executable: str = sys.executable) -> Dict[str, Any]:
    reference_identity, reference_sessions = validate_reference(
        reference_plan, canonical_arms)
    anchors_identity, anchor_stamps = validate_anchors(
        anchors, reference_identity, reference_sessions)
    qualification_plan_identity, qualification_report_identity, build_identity = (
        validate_qualification_go(
            qualification_plan, qualification_report, build, reference_plan,
            reference_identity, anchors_identity, anchor_stamps, canonical_arms,
            container=container, replay_devel=replay_devel))

    # Bind the validated in-memory documents to the exact files that will be
    # recorded in the orchestration before creating any output directory.
    document_bindings = (
        (qualification_plan_path, qualification_plan, "qualification plan"),
        (qualification_report_path, qualification_report, "qualification report"),
        (build_path, build, "qualified build manifest"),
        (anchors_path, anchors, "anchor artifact"),
        (reference_campaign / "campaign.json", reference_plan,
         "pre-fix v2 campaign plan"),
    )
    for path, expected_document, label in document_bindings:
        path = path.resolve()
        if not path.is_file() or load_json(path) != expected_document:
            raise CampaignError(f"{label} path/document binding changed: {path}")
    if load_arms(arms_source_path.resolve()) != list(canonical_arms):
        raise CampaignError("frozen Phase-A arms path/document binding changed")
    if _load_yaml(base_overlay_path.resolve(), "base overlay") != base_overlay:
        raise CampaignError("base overlay path/document binding changed")
    if not thresholds_path.resolve().is_file():
        raise CampaignError("flight-readiness thresholds file is missing")

    output_root = output_root.resolve()
    within_repo(output_root, "post-fix Phase-A output root")
    reference_by_id = {str(row["id"]): row for row in reference_sessions}
    dependencies = reference_plan.get("dependencies")
    if not isinstance(dependencies, Mapping):
        raise CampaignError("reference Phase-A dependency provenance is missing")
    # These inputs define the replay/evaluation comparison and must remain
    # byte-identical to clean v2.  The harness itself may differ because its
    # provenance/binding checks were strengthened after v2.
    for key, path in (
            ("arms", arms_source_path),
            ("base_overlay", base_overlay_path),
            ("thresholds", thresholds_path),
            ("strict_evaluator", EVALUATOR),
            ("replay_wrapper", REPLAY),
            ("replay_launch", REPLAY_LAUNCH)):
        _require_reference_dependency(dependencies, key, path)
    session_spec = (dependencies.get("session_spec")
                    if isinstance(dependencies, Mapping) else None)
    if not isinstance(session_spec, Mapping):
        raise CampaignError("reference Phase-A session-spec provenance is missing")
    spec_path = Path(str(session_spec.get("path", ""))).resolve()
    if (not spec_path.is_file() or
            (session_spec.get("sha256") is not None and
             sha256(spec_path) != session_spec.get("sha256"))):
        raise CampaignError("reference Phase-A session-spec identity changed")
    hybrid_parents = {str(Path(str(row["input_bag"])).parent)
                      for row in reference_sessions}
    cache_parents = {str(Path(str(row["window_cache"])).parent)
                     for row in reference_sessions}
    if len(hybrid_parents) != 1 or len(cache_parents) != 1:
        raise CampaignError("reference Phase-A inputs do not share frozen roots")
    hybrid_dir = next(iter(hybrid_parents))
    window_cache = next(iter(cache_parents))

    output_root.mkdir(parents=True, exist_ok=False)
    arms_dir = output_root / "arms"
    campaign_root = output_root / "campaigns"
    commands: List[List[str]] = []
    sessions: List[Dict[str, Any]] = []

    for session_id in SESSION_IDS:
        arm_document = _session_arm_document(
            canonical_arms, session_id, anchor_stamps[session_id],
            reference_identity=reference_identity,
            qualification_report_identity=qualification_report_identity,
            build_identity=build_identity, anchors_identity=anchors_identity)
        for arm in arm_document["arms"]:
            effective = effective_overlay(base_overlay, arm)
            validate_effective_overlay(effective)
        arm_path = arms_dir / f"{session_id}.yaml"
        _write_bytes_exclusive(
            arm_path, yaml.safe_dump(arm_document, sort_keys=False).encode("utf-8"))
        campaign_id = f"postfix_a_{session_id}"
        command = [
            "env", "-u", "FASTLIVO_QUALIFICATION_RUN_BINDING",
            python_executable, str(HARNESS),
            "--campaign-id", campaign_id,
            "--arms", str(arm_path),
            "--container", container,
            "--replay-devel", replay_devel.rstrip("/"),
            "--root", str(campaign_root),
            "--spec", str(spec_path),
            "--hybrid-dir", hybrid_dir,
            "--window-cache", window_cache,
            "--base-overlay", str(base_overlay_path.resolve()),
            "--thresholds", str(thresholds_path.resolve()),
            "--session", session_id,
            "--rate", "1",
            "--port", str(port),
        ]
        commands.append(command)
        reference = reference_by_id[session_id]
        sessions.append({
            "session_id": session_id,
            "split": "development",
            "anchor_stamp_ns": anchor_stamps[session_id],
            "init_anchor_max_predecessor_gap_s": 0.02,
            "input_bag": reference["input_bag"],
            "input_declared_sha256": reference["input_declared_sha256"],
            "input_provenance_sha256": reference["input_provenance_sha256"],
            "crop": copy.deepcopy(reference["crop"]),
            "arms_file": file_identity(arm_path),
            "campaign_id": campaign_id,
            "campaign_dir": str((campaign_root / campaign_id).resolve()),
            "expected_cell_count": len(ARM_IDS),
            "expected_cells": [
                {"arm_id": arm_id, "session_id": session_id, "repeat": 1}
                for arm_id in ARM_IDS
            ],
            "harness_command": command,
        })

    commands_core: Dict[str, Any] = {
        "schema": COMMANDS_SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "replay_executed_by_generator": False,
        "build_executed_by_generator": False,
        "qualification_report_identity_sha256": qualification_report_identity,
        "qualified_build_identity_sha256": build_identity,
        "sequential_execution_required": True,
        "commands": commands,
    }
    commands_document = {
        **commands_core, "identity_sha256": object_sha256(commands_core)}
    commands_path = output_root / "commands.json"
    _write_json_exclusive(commands_path, commands_document)

    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "qualification_gate_required": True,
        "qualification_status": "pass",
        "go_for_postfix_phase_a_rebaseline": True,
        "replay_executed_by_generator": False,
        "build_executed_by_generator": False,
        "rate": 1.0,
        "repeats_per_arm_session": 1,
        "arm_ids": list(ARM_IDS),
        "session_ids": list(SESSION_IDS),
        "expected_arm_count": 8,
        "expected_session_count": 5,
        "expected_run_count": 40,
        "standard_harness_append_only_cell_attempts": True,
        "one_completion_pointer_per_arm_session_cell": True,
        "session_campaigns_are_sequential": True,
        "qualification_run_binding_environment_unset": True,
        "reuse_prefix_completion_pointers": False,
        "old_scores_may_be_pooled": False,
        "sensitivity_comparator_cannot_promote": True,
        "qualification": {
            "plan": file_identity(qualification_plan_path.resolve()),
            "plan_identity_sha256": qualification_plan_identity,
            "report": file_identity(qualification_report_path.resolve()),
            "report_identity_sha256": qualification_report_identity,
        },
        "qualified_build": {
            "manifest": file_identity(build_path.resolve()),
            "identity_sha256": build_identity,
            "container": container,
            "replay_devel": replay_devel.rstrip("/"),
            "executable_sha256": build["executable_sha256"],
            "dynamic_libraries": copy.deepcopy(build["dynamic_libraries"]),
            "source_tree_sha256": build["source_tree_sha256"],
        },
        "reference_prefix_v2": {
            "campaign": str(reference_campaign.resolve()),
            "campaign_plan": file_identity(
                (reference_campaign / "campaign.json").resolve()),
            "campaign_identity_sha256": reference_identity,
            "comparison_role": "sensitivity_diagnostic_only",
        },
        "anchors": {
            "artifact": file_identity(anchors_path.resolve()),
            "identity_sha256": anchors_identity,
        },
        "dependencies": {
            "generator": file_identity(Path(__file__).resolve()),
            "standard_harness": file_identity(HARNESS.resolve()),
            "frozen_phase_a_arms": file_identity(arms_source_path.resolve()),
            "base_overlay": file_identity(base_overlay_path.resolve()),
            "thresholds": file_identity(thresholds_path.resolve()),
            "strict_evaluator": file_identity(EVALUATOR.resolve()),
            "replay_wrapper": file_identity(REPLAY.resolve()),
            "replay_launch": file_identity(REPLAY_LAUNCH.resolve()),
            "session_spec": file_identity(spec_path),
        },
        "commands": file_identity(commands_path),
        "sessions": sessions,
    }
    result = {**core, "identity_sha256": object_sha256(core)}
    _write_json_exclusive(output_root / "orchestration.json", result)
    return result


def _load_yaml(path: Path, label: str) -> Dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read {label}: {error}") from error
    if not isinstance(document, dict):
        raise CampaignError(f"{label} is not a YAML mapping")
    return document


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("qualification_plan", type=Path)
    parser.add_argument("qualification_report", type=Path)
    parser.add_argument("build_manifest", type=Path)
    parser.add_argument("anchors", type=Path)
    parser.add_argument("--reference-campaign", type=Path,
                        default=DEFAULT_REFERENCE)
    parser.add_argument("--arms", type=Path, default=PHASE_A_ARMS)
    parser.add_argument("--base-overlay", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--container", required=True)
    parser.add_argument("--replay-devel", required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--port", type=int, default=11421)
    arguments = parser.parse_args(argv)
    try:
        if not 1024 <= arguments.port <= 65535:
            raise CampaignError("--port must be between 1024 and 65535")
        inputs = [
            arguments.qualification_plan, arguments.qualification_report,
            arguments.build_manifest, arguments.anchors,
            arguments.reference_campaign / "campaign.json", arguments.arms,
            arguments.base_overlay, arguments.thresholds,
        ]
        missing = [str(path) for path in inputs if not path.resolve().is_file()]
        if missing:
            raise CampaignError("missing input: " + ", ".join(missing))
        orchestration = prepare_orchestration(
            load_json(arguments.qualification_plan.resolve()),
            load_json(arguments.qualification_report.resolve()),
            load_json(arguments.build_manifest.resolve()),
            load_json(arguments.anchors.resolve()),
            load_json((arguments.reference_campaign / "campaign.json").resolve()),
            load_arms(arguments.arms.resolve()),
            _load_yaml(arguments.base_overlay.resolve(), "base overlay"),
            qualification_plan_path=arguments.qualification_plan.resolve(),
            qualification_report_path=arguments.qualification_report.resolve(),
            build_path=arguments.build_manifest.resolve(),
            anchors_path=arguments.anchors.resolve(),
            reference_campaign=arguments.reference_campaign.resolve(),
            arms_source_path=arguments.arms.resolve(),
            base_overlay_path=arguments.base_overlay.resolve(),
            thresholds_path=arguments.thresholds.resolve(),
            output_root=arguments.output_root,
            container=arguments.container,
            replay_devel=arguments.replay_devel,
            port=arguments.port,
        )
        print(json.dumps({
            "orchestration": str(
                (arguments.output_root / "orchestration.json").resolve()),
            "identity_sha256": orchestration["identity_sha256"],
            "commands": str((arguments.output_root / "commands.json").resolve()),
            "command_count": len(orchestration["sessions"]),
            "expected_run_count": orchestration["expected_run_count"],
            "replay_executed": False,
            "build_executed": False,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Execute the qualified ground-init Phase-A cells, one fresh replay at a time.

``sequence`` consumes the immutable forty-command list in order.  ``cell`` is
the only mode that starts replay; it revalidates the self-hashed low-rate GO,
qualified build/source, dependencies, development input bag and provenance
before creating an append-only attempt.  A global worker lock plus the replay
process preflight prevents overlapping estimator processes.

Each completed cell contains a full-result primary safety report and a
hover-to-landing ranking-only report generated from the same bag using the
primary report's frozen alignment.  The manifest always keeps flight
promotion disabled and the high-rate interface NO-GO.
"""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple
import uuid

import yaml

from build_vio_postfix_init_qualification_receipt import (
    validate_build_manifest,
)
from generate_vio_ground_phase_a_rebaseline import (
    ARM_IDS,
    CELL_SCHEMA,
    COMMANDS_SCHEMA,
    QUALIFICATION_PLAN_SCHEMA,
    QUALIFICATION_VARIANT,
    SCHEMA as ORCHESTRATION_SCHEMA,
    SESSION_IDS,
    TIGHT_INIT_PARAMETERS,
    validate_qualification_report,
    validate_runtime_constant_binding,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    EVALUATOR,
    REPLAY,
    bag_topic_inventory,
    check_no_live_worker,
    container_path,
    effective_overlay,
    hash_artifacts,
    load_json,
    load_yaml_mapping,
    object_sha256,
    run_logged,
    sha256,
    validate_evaluation,
    validate_output_topic_inventory,
    validate_parameter_snapshot,
    within_repo,
)


CAMPAIGN_SCHEMA = "fastlivo_vio_ground_phase_a_campaign/v1"
MANIFEST_SCHEMA = "fastlivo_vio_ground_phase_a_run/v1"
COMPLETION_SCHEMA = "fastlivo_vio_ground_phase_a_completion/v1"
SEQUENCE_ATTEMPT_SCHEMA = "fastlivo_vio_ground_phase_a_sequence_attempt/v1"
PRIMARY_SCHEMA = "fastlivo_vio_flight_readiness/v1"
SECONDARY_SCHEMA = "fastlivo_vio_ground_hover_ranking/v1"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


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


def _verify_file_identity(identity: Any, label: str) -> Path:
    if not isinstance(identity, Mapping):
        raise CampaignError(f"{label}: missing file identity")
    path = Path(str(identity.get("path", ""))).resolve()
    if (not path.is_file() or path.stat().st_size !=
            int(identity.get("size_bytes", -1)) or
            sha256(path) != identity.get("sha256")):
        raise CampaignError(f"{label}: file identity changed: {path}")
    return path


def _file_identity(path: Path) -> Dict[str, Any]:
    path = path.resolve()
    if not path.is_file():
        raise CampaignError(f"missing artifact: {path}")
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def _load_orchestration(path: Path) -> Tuple[Dict[str, Any], str]:
    path = path.resolve()
    document = load_json(path)
    identity = _self_hash(
        document, ORCHESTRATION_SCHEMA, "ground Phase-A orchestration")
    output_root = Path(str(document.get("output_root", ""))).resolve()
    if (output_root != path.parent or
            document.get("scope") != "development_only" or
            document.get("validation_data_accessed") is not False or
            document.get("qualification_variant") != QUALIFICATION_VARIANT or
            document.get("design", {}).get("expected_run_count") != 40 or
            document.get("design", {}).get("arm_ids") != list(ARM_IDS) or
            document.get("design", {}).get("session_ids") != list(SESSION_IDS) or
            document.get("design", {}).get("arm_count") != 8 or
            document.get("design", {}).get("session_count") != 5 or
            document.get("design", {}).get("repeats_per_arm_session") != 1 or
            document.get("design", {}).get("rate") != 1.0 or
            document.get("design", {}).get("window") !=
            "full_bag_record_start_to_frozen_cached_landing" or
            document.get("design", {}).get("fresh_campaign_per_cell") is not True or
            document.get("design", {}).get("fresh_process_per_cell") is not True or
            document.get("design", {}).get("strictly_sequential") is not True or
            document.get("decision_contract", {}).get(
                "high_rate_interface_remains_no_go") is not True or
            document.get("decision_contract", {}).get(
                "old_scores_may_be_pooled") is not False or
            document.get("decision_contract", {}).get(
                "candidate_promotion_allowed") is not False or
            document.get("decision_contract", {}).get(
                "flight_ready_can_be_declared") is not False or
            document.get("evaluation_contract", {}).get("primary") !=
            "complete_ground_to_landing_safety" or
            document.get("evaluation_contract", {}).get("secondary") !=
            "hover_to_landing_low_rate_ranking_only" or
            document.get("evaluation_contract", {}).get(
                "secondary_alignment") != "reuse_primary_without_refit" or
            document.get("evaluation_contract", {}).get(
                "secondary_can_override_primary_failure") is not False or
            document.get("evaluation_contract", {}).get(
                "same_result_bag_required") is not True or
            document.get("execution", {}).get(
                "generator_executed_replay") is not False or
            document.get("execution", {}).get(
                "generator_executed_build") is not False):
        raise CampaignError("ground Phase-A orchestration safety contract changed")
    cells = document.get("cells")
    commands = document.get("commands")
    if (not isinstance(cells, list) or len(cells) != 40 or
            not isinstance(commands, list) or len(commands) != 40):
        raise CampaignError("ground Phase-A orchestration is not an exact 40-cell grid")
    if len({row.get("run_id") for row in cells if isinstance(row, Mapping)}) != 40:
        raise CampaignError("ground Phase-A cell IDs are malformed/duplicate")
    expected_grid = [
        (arm_id, session_id)
        for arm_id in ARM_IDS for session_id in SESSION_IDS
    ]
    observed_grid = [
        (row.get("arm_id"), row.get("session_id"))
        for row in cells if isinstance(row, Mapping)
    ]
    if (len(observed_grid) != 40 or observed_grid != expected_grid or
            [row.get("ordinal") for row in cells
             if isinstance(row, Mapping)] != list(range(1, 41))):
        raise CampaignError("ground Phase-A grid/order changed")
    return dict(document), identity


def _load_cell(
        orchestration: Mapping[str, Any], run_id: str) -> Tuple[Dict[str, Any], Path]:
    rows = [row for row in orchestration["cells"]
            if isinstance(row, Mapping) and row.get("run_id") == run_id]
    if len(rows) != 1:
        raise CampaignError(f"orchestration has no unique cell {run_id!r}")
    row = rows[0]
    path = _verify_file_identity(row.get("cell"), f"cell {run_id}")
    cell = load_json(path)
    identity = _self_hash(cell, CELL_SCHEMA, f"cell {run_id}")
    session = cell.get("session")
    qualification = cell.get("qualification")
    replay_process = cell.get("replay_process")
    evaluation = cell.get("evaluation_contract")
    if (identity != row.get("cell_object_identity_sha256") or
            cell.get("run_id") != run_id or
            cell.get("campaign_id") != row.get("campaign_id") or
            cell.get("arm_id") != row.get("arm_id") or
            not isinstance(session, Mapping) or
            session.get("session_id") != row.get("session_id") or
            session.get("split") != "development" or
            not isinstance(qualification, Mapping) or
            qualification.get("variant") != QUALIFICATION_VARIANT or
            qualification.get("plan_identity_sha256") !=
            orchestration.get("qualification_gate", {}).get(
                "plan_identity_sha256") or
            qualification.get("report_identity_sha256") !=
            orchestration.get("qualification_gate", {}).get(
                "report_identity_sha256") or
            qualification.get("low_rate_estimator_rebaseline_go") is not True or
            cell.get("qualified_build_identity_sha256") !=
            orchestration.get("qualified_build", {}).get("identity_sha256") or
            cell.get("qualified_executable_sha256") !=
            orchestration.get("qualified_build", {}).get("executable_sha256") or
            not isinstance(replay_process, Mapping) or
            replay_process.get("fresh_campaign_required") is not True or
            replay_process.get("fresh_process_required") is not True or
            replay_process.get("sequential_execution_required") is not True or
            replay_process.get("rate") != 1.0 or
            not isinstance(evaluation, Mapping) or
            evaluation.get("primary_full_result_safety_report_required") is not True or
            evaluation.get("secondary_hover_report_required") is not True or
            evaluation.get("secondary_reuses_primary_alignment_without_refit")
            is not True or
            evaluation.get("secondary_cannot_override_primary_failure")
            is not True or
            evaluation.get("primary_and_secondary_must_bind_same_result_bag")
            is not True or
            cell.get("scope") != "development_only" or
            cell.get("validation_data_accessed") is not False or
            cell.get("decision_contract", {}).get(
                "high_rate_interface_remains_no_go") is not True or
            cell.get("decision_contract", {}).get(
                "candidate_promotion_allowed") is not False or
            cell.get("decision_contract", {}).get(
                "old_scores_may_be_pooled") is not False):
        raise CampaignError(f"cell safety contract changed: {run_id}")
    return dict(cell), path


def _validate_dependencies(orchestration: Mapping[str, Any]) -> Dict[str, Path]:
    dependencies = orchestration.get("dependencies")
    if not isinstance(dependencies, Mapping):
        raise CampaignError("orchestration lacks dependency inventory")
    required = {
        "generator", "runner", "strict_evaluator", "replay_wrapper",
        "phase_a_reporter",
        "replay_launch", "fastlivo_base_config", "thresholds",
        "base_overlay", "frozen_phase_a_arms", "qualification_plan",
        "qualification_report", "qualified_build_manifest", "ground_anchors",
        "prefix_v2_reference_plan",
    }
    if set(dependencies) != required:
        raise CampaignError("orchestration dependency inventory is not exact")
    return {
        label: _verify_file_identity(identity, f"dependency {label}")
        for label, identity in dependencies.items()
    }


def _validate_gate_and_build(
        orchestration: Mapping[str, Any], paths: Mapping[str, Path], *,
        verify_actual_build: bool = True) -> None:
    plan = load_json(paths["qualification_plan"])
    report = load_json(paths["qualification_report"])
    build = load_json(paths["qualified_build_manifest"])
    plan_identity = _self_hash(
        plan, QUALIFICATION_PLAN_SCHEMA, "ground qualification plan")
    if (plan.get("qualification_variant") != QUALIFICATION_VARIANT or
            plan.get("session_window_mode") != "ground_to_landing" or
            plan.get("scope") != "development_only" or
            plan.get("validation_data_accessed") is not False or
            plan.get("high_rate_interface_remains_no_go") is not True):
        raise CampaignError("ground qualification plan runtime contract changed")
    build_identity = validate_build_manifest(
        build, verify_actual=verify_actual_build)
    gate = orchestration["qualification_gate"]
    qualified = orchestration["qualified_build"]
    if (gate.get("plan_identity_sha256") != plan_identity or
            qualified.get("identity_sha256") != build_identity or
            qualified.get("executable_sha256") !=
            build.get("executable_sha256") or
            qualified.get("source_tree_sha256") !=
            build.get("source_tree_sha256")):
        raise CampaignError("orchestration plan/build binding changed")
    anchors = load_json(paths["ground_anchors"])
    runtime_binding = validate_runtime_constant_binding(plan, anchors, build)
    if orchestration.get("runtime_constant_binding") != runtime_binding:
        raise CampaignError("runtime gravity-constant/build binding changed")
    report_identity = validate_qualification_report(
        report, plan_identity=plan_identity, build_identity=build_identity,
        executable_sha256=str(build.get("executable_sha256", "")))
    if (gate.get("report_identity_sha256") != report_identity or
            gate.get("low_rate_estimator_rebaseline_go") is not True):
        raise CampaignError("low-rate rebaseline GO binding changed")


def _validate_input(cell: Mapping[str, Any]) -> str:
    session = cell["session"]
    input_row = session.get("input")
    if not isinstance(input_row, Mapping):
        raise CampaignError("cell lacks input binding")
    bag = Path(str(input_row.get("path", ""))).resolve()
    if (not bag.is_file() or bag.stat().st_size !=
            int(input_row.get("size_bytes", -1)) or
            bag.stat().st_mtime_ns != int(input_row.get("mtime_ns", -1))):
        raise CampaignError(f"development input stat changed: {bag}")
    actual = sha256(bag)
    if (actual != input_row.get("declared_sha256") or
            actual != input_row.get("verified_sha256")):
        raise CampaignError(f"development input SHA-256 changed: {bag}")
    _verify_file_identity(input_row.get("provenance"), "hybrid provenance")
    _verify_file_identity(input_row.get("window_cache"), "development window cache")
    return actual


def _campaign_plan(
        orchestration: Mapping[str, Any], orchestration_identity: str,
        cell: Mapping[str, Any], cell_path: Path) -> Dict[str, Any]:
    core = {
        "schema": CAMPAIGN_SCHEMA,
        "campaign_id": cell["campaign_id"],
        "scope": "development_only",
        "validation_data_accessed": False,
        "orchestration_identity_sha256": orchestration_identity,
        "cell": _file_identity(cell_path),
        "cell_object_identity_sha256": cell["identity_sha256"],
        "qualification_gate": copy.deepcopy(orchestration["qualification_gate"]),
        "qualified_build": copy.deepcopy(orchestration["qualified_build"]),
        "run_id": cell["run_id"],
        "arm_id": cell["arm_id"],
        "session_id": cell["session"]["session_id"],
        "runtime_overrides": copy.deepcopy(cell["runtime_overrides"]),
        "replay": copy.deepcopy(cell["session"]["replay"]),
        "primary_evaluation": copy.deepcopy(
            cell["session"]["primary_evaluation"]),
        "secondary_evaluation": copy.deepcopy(
            cell["session"]["secondary_evaluation"]),
        "fresh_process_required": True,
        "candidate_promotion_allowed": False,
        "flight_ready_can_be_declared": False,
        "high_rate_interface_remains_no_go": True,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _validate_or_create_campaign_plan(path: Path, proposed: Mapping[str, Any]) -> None:
    if path.exists():
        if load_json(path) != proposed:
            raise CampaignError(f"immutable campaign plan differs: {path}")
        _self_hash(load_json(path), CAMPAIGN_SCHEMA, "cell campaign plan")
        return
    _write_json_exclusive(path, proposed)


def _validate_primary_report(
        path: Path, result_bag: Path, thresholds: Path) -> Dict[str, Any]:
    report = validate_evaluation(path, result_bag)
    semantics = report.get("evaluation_semantics", {})
    bindings = report.get("artifact_bindings", {})
    if (report.get("schema") != PRIMARY_SCHEMA or
            semantics.get("score_window_sensor_stamp_ns") is not None or
            semantics.get("fixed_alignment_supplied") is not False or
            bindings.get("result_bag") != _file_identity(result_bag) or
            bindings.get("thresholds") != _file_identity(thresholds)):
        raise CampaignError("primary report is not a bound, unmasked full report")
    return report


def _validate_secondary_report(
        path: Path, result_bag: Path, thresholds: Path, primary_path: Path,
        primary: Mapping[str, Any], score_start_ns: str,
        score_end_ns: str) -> Dict[str, Any]:
    report = load_json(path)
    semantics = report.get("evaluation_semantics", {})
    expected_window = {
        "start": score_start_ns,
        "end": score_end_ns,
        "boundary": "start_inclusive_end_inclusive",
    }
    inherited = report.get("full_result_interface_inherited", {})
    primary_identity = _file_identity(primary_path)
    primary_alignment = primary.get("local", {}).get("alignment")
    secondary_alignment = report.get("local_accuracy", {}).get("alignment")
    if (report.get("schema") != SECONDARY_SCHEMA or
            Path(str(report.get("result_bag", ""))).resolve() !=
            result_bag.resolve() or report.get("role") !=
            "phase_a_ranking_compatibility_only" or
            report.get("status") != "ranking_only" or
            report.get("flight_ready") is not False or
            report.get("can_override_primary_failure") is not False or
            report.get("primary_report_identity") != primary_identity or
            report.get("primary_status") != primary.get("status") or
            report.get("primary_flight_ready") != primary.get("flight_ready") or
            semantics.get("score_window_sensor_stamp_ns") != expected_window or
            semantics.get("fixed_alignment_supplied") is not True or
            semantics.get("primary_alignment_reused_without_refit") is not True or
            primary_alignment is None or secondary_alignment is None or
            secondary_alignment.get("yaw_deg") != primary_alignment.get("yaw_deg") or
            secondary_alignment.get("translation_m") !=
            primary_alignment.get("translation_m") or
            secondary_alignment.get("scale") != primary_alignment.get("scale") or
            secondary_alignment.get("reused_from_primary_full_result") is not True or
            secondary_alignment.get("primary_report_identity") != primary_identity or
            report.get("artifact_bindings", {}).get("result_bag") !=
            _file_identity(result_bag) or
            report.get("artifact_bindings", {}).get("thresholds") !=
            _file_identity(thresholds) or
            inherited.get("status") != primary.get("status") or
            inherited.get("flight_ready") != primary.get("flight_ready") or
            inherited.get("gt_independence") != primary.get("gt_independence") or
            inherited.get("local_integrity") !=
            primary.get("local", {}).get("integrity") or
            inherited.get("propagated") != primary.get("propagated") or
            inherited.get("correction_covariance") !=
            primary.get("correction_covariance")):
        raise CampaignError(
            "secondary report did not reuse the bound primary alignment/safety state")
    return report


def _validate_manifest(attempt: Path, manifest: Mapping[str, Any]) -> None:
    _self_hash(manifest, MANIFEST_SCHEMA, "ground Phase-A run manifest")
    if (manifest.get("state") != "complete" or
            manifest.get("candidate_promotion_allowed") is not False or
            manifest.get("flight_ready") is not False or
            manifest.get("high_rate_interface_remains_no_go") is not True or
            manifest.get("secondary_can_override_primary_failure") is not False):
        raise CampaignError(f"completed manifest safety state changed: {attempt}")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping) or not artifacts:
        raise CampaignError(f"manifest lacks artifacts: {attempt}")
    for name, identity in artifacts.items():
        if not isinstance(name, str) or Path(name).name != name:
            raise CampaignError(f"unsafe artifact name: {name!r}")
        path = attempt / name
        if (not path.is_file() or path.stat().st_size !=
                int(identity.get("size_bytes", -1)) or
                sha256(path) != identity.get("sha256")):
            raise CampaignError(f"completed artifact changed: {path}")
    inventory = bag_topic_inventory(attempt / "result.bag")
    if inventory != manifest.get("output_topic_inventory"):
        raise CampaignError(f"result bag topic inventory changed: {attempt}")
    validate_output_topic_inventory(inventory)


def _validate_completion(
        campaign_dir: Path, pointer_path: Path,
        campaign_identity: str, run_id: str) -> Path:
    pointer = load_json(pointer_path)
    _self_hash(pointer, COMPLETION_SCHEMA, "ground Phase-A completion")
    if (pointer.get("campaign_identity_sha256") != campaign_identity or
            pointer.get("run_id") != run_id):
        raise CampaignError(f"completion belongs to another cell: {pointer_path}")
    relative = Path(str(pointer.get("attempt", "")))
    if relative.is_absolute() or ".." in relative.parts:
        raise CampaignError(f"unsafe completion attempt path: {pointer_path}")
    attempt = (campaign_dir / relative).resolve()
    try:
        attempt.relative_to(campaign_dir.resolve())
    except ValueError as error:
        raise CampaignError("completion attempt escapes campaign") from error
    manifest_path = attempt / "manifest.json"
    if (not manifest_path.is_file() or
            sha256(manifest_path) != pointer.get("manifest_sha256")):
        raise CampaignError(f"completion manifest changed: {manifest_path}")
    manifest = load_json(manifest_path)
    if (manifest.get("campaign_identity_sha256") != campaign_identity or
            manifest.get("run_id") != run_id):
        raise CampaignError(f"completion manifest identity mismatch: {manifest_path}")
    _validate_manifest(attempt, manifest)
    return attempt


def _require_predecessor_completions(
        orchestration: Mapping[str, Any], orchestration_identity: str,
        target: Mapping[str, Any]) -> None:
    """Prevent direct ``cell`` calls from bypassing the frozen list order.

    This is a lightweight ordering proof: full artifact revalidation is done
    when each completion is written and again by the final report builder.
    Here we bind each predecessor's immutable campaign/completion/manifest
    identities without repeatedly hashing every prior result bag.
    """
    target_ordinal = int(target.get("ordinal", -1))
    if not 1 <= target_ordinal <= 40:
        raise CampaignError("target cell has invalid ordinal")
    output_root = Path(str(orchestration["output_root"])).resolve()
    for row in orchestration["cells"][:target_ordinal - 1]:
        if not isinstance(row, Mapping):
            raise CampaignError("malformed predecessor cell row")
        cell, cell_path = _load_cell(
            orchestration, str(row["run_id"]))
        campaign = _campaign_plan(
            orchestration, orchestration_identity, cell, cell_path)
        campaign_dir = output_root / "campaigns" / str(cell["campaign_id"])
        campaign_path = campaign_dir / "campaign.json"
        completion_path = campaign_dir / "completion.json"
        if (not campaign_path.is_file() or
                load_json(campaign_path) != campaign or
                not completion_path.is_file()):
            raise CampaignError(
                f"cell {target_ordinal} cannot precede incomplete "
                f"cell {row['ordinal']}")
        completion = load_json(completion_path)
        _self_hash(
            completion, COMPLETION_SCHEMA,
            f"predecessor completion {row['run_id']}")
        if (completion.get("campaign_identity_sha256") !=
                campaign["identity_sha256"] or
                completion.get("run_id") != row["run_id"]):
            raise CampaignError("predecessor completion identity changed")
        relative = Path(str(completion.get("attempt", "")))
        if relative.is_absolute() or ".." in relative.parts:
            raise CampaignError("unsafe predecessor attempt path")
        manifest_path = campaign_dir / relative / "manifest.json"
        if (not manifest_path.is_file() or
                sha256(manifest_path) != completion.get("manifest_sha256")):
            raise CampaignError("predecessor manifest identity changed")
        manifest = load_json(manifest_path)
        _self_hash(
            manifest, MANIFEST_SCHEMA,
            f"predecessor manifest {row['run_id']}")
        if (manifest.get("state") != "complete" or
                manifest.get("campaign_identity_sha256") !=
                campaign["identity_sha256"] or
                manifest.get("run_id") != row["run_id"] or
                manifest.get("candidate_promotion_allowed") is not False or
                manifest.get("high_rate_interface_remains_no_go") is not True):
            raise CampaignError("predecessor manifest safety state changed")


def _evaluation_commands(
        result_bag: Path, thresholds: Path, primary: Path, secondary: Path,
        score_start_ns: str, score_end_ns: str) -> Tuple[list[str], list[str]]:
    primary_command = [
        sys.executable, str(EVALUATOR), str(result_bag),
        "--thresholds", str(thresholds),
        "--output", str(primary),
    ]
    secondary_command = [
        sys.executable, str(EVALUATOR), str(result_bag),
        "--thresholds", str(thresholds),
        "--score-start-ns", score_start_ns,
        "--score-end-ns", score_end_ns,
        "--fixed-alignment-report", str(primary),
        "--output", str(secondary),
    ]
    return primary_command, secondary_command


def execute_cell(orchestration_path: Path, run_id: str) -> str:
    orchestration, orchestration_identity = _load_orchestration(
        orchestration_path)
    paths = _validate_dependencies(orchestration)
    if (paths["runner"] != Path(__file__).resolve() or
            paths["strict_evaluator"] != EVALUATOR.resolve() or
            paths["replay_wrapper"] != REPLAY.resolve()):
        raise CampaignError("runtime tool path differs from bound dependency")
    _validate_gate_and_build(orchestration, paths)
    cell, cell_path = _load_cell(orchestration, run_id)
    input_hash = _validate_input(cell)
    output_root = within_repo(
        Path(str(orchestration["output_root"])).resolve(),
        "ground Phase-A output root")
    campaign_dir = output_root / "campaigns" / str(cell["campaign_id"])
    campaign = _campaign_plan(
        orchestration, orchestration_identity, cell, cell_path)

    lock_path = output_root / ".worker.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+b") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError("another Phase-A replay worker is active") from error
        _require_predecessor_completions(
            orchestration, orchestration_identity, cell)
        campaign_dir.mkdir(parents=True, exist_ok=True)
        campaign_path = campaign_dir / "campaign.json"
        _validate_or_create_campaign_plan(campaign_path, campaign)
        completion_path = campaign_dir / "completion.json"
        if completion_path.exists():
            attempt = _validate_completion(
                campaign_dir, completion_path,
                str(campaign["identity_sha256"]), run_id)
            print(f"SKIP validated {run_id} -> {attempt}")
            return "skipped"

        process_uuid = str(uuid.uuid4())
        stamp = dt.datetime.now(dt.timezone.utc).strftime(
            "%Y%m%dT%H%M%S.%fZ")
        attempt_id = f"{stamp}_{uuid.uuid4().hex[:12]}"
        attempt = campaign_dir / "attempts" / attempt_id
        attempt.mkdir(parents=True, exist_ok=False)
        base = load_yaml_mapping(paths["base_overlay"], "base overlay")
        arm = {"id": cell["arm_id"],
               "overrides": copy.deepcopy(cell["runtime_overrides"])}
        overlay = effective_overlay(base, arm)
        if object_sha256(overlay) != cell.get("effective_overlay_sha256"):
            raise CampaignError("effective overlay identity changed")
        imu = overlay.get("imu", {})
        expected_imu = {
            **dict(TIGHT_INIT_PARAMETERS),
            "init_anchor_stamp_ns": cell["session"]["explicit_anchor"][
                "anchor_stamp_ns"],
        }
        if any(imu.get(key) != value for key, value in expected_imu.items()):
            raise CampaignError("tight ground-init runtime parameters changed")
        overlay_path = attempt / "overlay.yaml"
        _write_bytes_exclusive(
            overlay_path,
            yaml.safe_dump(overlay, sort_keys=True).encode("utf-8"))
        result_bag = attempt / "result.bag"
        primary_path = attempt / "result.full.flight_readiness.json"
        secondary_path = attempt / "result.hover.ranking.json"
        replay = cell["session"]["replay"]
        if (replay.get("start_s") != 0.0 or replay.get("rate") != 1.0 or
                replay.get("end_definition") != "frozen_cached_landing" or
                replay.get("no_gt_anchor") is not True or
                replay.get("with_propagated") is not True):
            raise CampaignError("cell replay contract changed")
        replay_command = [
            "bash", str(REPLAY),
            container_path(Path(cell["session"]["input"]["path"])),
            "--rate", "1",
            "--start", "0",
            "--duration", format(float(replay["duration_s"]), ".17g"),
            "--overlay", container_path(overlay_path),
            "--out", container_path(result_bag),
            "--no-gt-anchor", "--with-propagated",
        ]
        secondary = cell["session"]["secondary_evaluation"]
        primary_command, secondary_command = _evaluation_commands(
            result_bag, paths["thresholds"], primary_path, secondary_path,
            str(secondary["score_start_ns"]),
            str(secondary["score_end_ns"]))
        environment = os.environ.copy()
        environment.pop("FASTLIVO_QUALIFICATION_RUN_BINDING", None)
        environment.update({
            "FASTLIVO_REPLAY_CONTAINER": str(
                orchestration["qualified_build"]["container"]),
            "FASTLIVO_REPLAY_DEVEL": str(
                orchestration["qualified_build"]["replay_devel"]),
            "FASTLIVO_REPLAY_PORT": str(orchestration["ros_master_port"]),
        })
        started = utc_now()
        try:
            check_no_live_worker(
                str(orchestration["qualified_build"]["container"]),
                int(orchestration["ros_master_port"]))
            run_logged(replay_command, environment,
                       attempt / "replay.stdout.log")
            validate_parameter_snapshot(
                attempt / "result_params.yaml", overlay)
            run_logged(primary_command, environment,
                       attempt / "evaluate.primary.stdout.log")
            primary = _validate_primary_report(
                primary_path, result_bag, paths["thresholds"])
            run_logged(secondary_command, environment,
                       attempt / "evaluate.secondary.stdout.log")
            secondary_report = _validate_secondary_report(
                secondary_path, result_bag, paths["thresholds"],
                primary_path, primary,
                str(secondary["score_start_ns"]),
                str(secondary["score_end_ns"]))
            inventory = bag_topic_inventory(result_bag)
            validate_output_topic_inventory(inventory)
            names = [
                "overlay.yaml", "result.bag", "result_params.yaml",
                "result_node.log", "result.full.flight_readiness.json",
                "result.hover.ranking.json", "replay.stdout.log",
                "evaluate.primary.stdout.log",
                "evaluate.secondary.stdout.log",
            ]
            if (attempt / "result_fusion.csv").is_file():
                names.append("result_fusion.csv")
            artifacts = hash_artifacts(attempt, names)
            manifest_core: Dict[str, Any] = {
                "schema": MANIFEST_SCHEMA,
                "state": "complete",
                "created_utc": started,
                "completed_utc": utc_now(),
                "campaign_identity_sha256": campaign["identity_sha256"],
                "orchestration_identity_sha256": orchestration_identity,
                "cell_identity_sha256": cell["identity_sha256"],
                "run_id": run_id,
                "arm_id": cell["arm_id"],
                "session_id": cell["session"]["session_id"],
                "fresh_process_instance_uuid": process_uuid,
                "input_bag": cell["session"]["input"]["path"],
                "input_sha256": input_hash,
                "qualified_build_identity_sha256":
                    orchestration["qualified_build"]["identity_sha256"],
                "qualified_executable_sha256":
                    orchestration["qualified_build"]["executable_sha256"],
                "source_tree_sha256":
                    orchestration["qualified_build"]["source_tree_sha256"],
                "runtime_overrides": copy.deepcopy(cell["runtime_overrides"]),
                "replay": copy.deepcopy(replay),
                "replay_command": replay_command,
                "primary_evaluator_command": primary_command,
                "secondary_evaluator_command": secondary_command,
                "primary_status": primary["status"],
                "primary_flight_ready_diagnostic": bool(
                    primary.get("flight_ready", False)),
                "secondary_status": secondary_report["status"],
                "secondary_ranking_metrics_available": True,
                "secondary_can_override_primary_failure": False,
                "old_prefix_scores_role": "diagnostic_only_no_pooling",
                "candidate_promotion_allowed": False,
                "flight_ready": False,
                "high_rate_interface_remains_no_go": True,
                "output_topic_inventory": inventory,
                "artifacts": artifacts,
            }
            manifest = {
                **manifest_core,
                "identity_sha256": object_sha256(manifest_core),
            }
            manifest_path = attempt / "manifest.json"
            _write_json_exclusive(manifest_path, manifest)
            _validate_manifest(attempt, manifest)
            completion_core = {
                "schema": COMPLETION_SCHEMA,
                "created_utc": utc_now(),
                "campaign_identity_sha256": campaign["identity_sha256"],
                "run_id": run_id,
                "attempt": attempt.relative_to(campaign_dir).as_posix(),
                "manifest_sha256": sha256(manifest_path),
                "manifest_identity_sha256": manifest["identity_sha256"],
            }
            completion = {
                **completion_core,
                "identity_sha256": object_sha256(completion_core),
            }
            _write_json_exclusive(completion_path, completion)
            _validate_completion(
                campaign_dir, completion_path,
                str(campaign["identity_sha256"]), run_id)
            print(
                f"DONE {run_id}: primary={primary['status']} "
                f"secondary=ranking_only -> {attempt}")
            return "completed"
        except BaseException as error:
            failure_path = attempt / "failure.json"
            if not failure_path.exists():
                _write_json_exclusive(failure_path, {
                    "schema": MANIFEST_SCHEMA,
                    "state": "failed_or_interrupted",
                    "created_utc": started,
                    "failed_utc": utc_now(),
                    "campaign_identity_sha256": campaign["identity_sha256"],
                    "run_id": run_id,
                    "fresh_process_instance_uuid": process_uuid,
                    "error_type": type(error).__name__,
                    "error": str(error),
                })
            raise


def _validate_commands(
        orchestration_path: Path, orchestration: Mapping[str, Any],
        path: Path) -> list[list[str]]:
    document = load_json(path.resolve())
    _self_hash(document, COMMANDS_SCHEMA, "ground Phase-A command list")
    commands = document.get("commands")
    if (document.get("scope") != "development_only" or
            document.get("validation_data_accessed") is not False or
            Path(str(document.get("orchestration_path", ""))).resolve() !=
            orchestration_path.resolve() or
            document.get("orchestration_identity_sha256") !=
            orchestration["identity_sha256"] or
            document.get("strictly_sequential") is not True or
            document.get("fresh_campaign_per_cell") is not True or
            document.get("command_count") != 40 or
            commands != orchestration.get("commands")):
        raise CampaignError("command list differs from immutable orchestration")
    if not all(isinstance(command, list) and
               all(isinstance(item, str) for item in command)
               for command in commands):
        raise CampaignError("command list is malformed")
    return [list(command) for command in commands]


def execute_sequence(
        orchestration_path: Path, commands_path: Path) -> None:
    orchestration, _ = _load_orchestration(orchestration_path)
    commands = _validate_commands(
        orchestration_path.resolve(), orchestration, commands_path)
    output_root = within_repo(
        Path(str(orchestration["output_root"])).resolve(),
        "ground Phase-A output root")
    sequence_lock_path = output_root / ".sequence.lock"
    with sequence_lock_path.open("a+b") as sequence_lock:
        try:
            fcntl.flock(
                sequence_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError("another Phase-A sequence is active") from error
        attempt_id = "sequence_" + uuid.uuid4().hex
        attempt_root = output_root / "sequence_attempts" / attempt_id
        attempt_root.mkdir(parents=True, exist_ok=False)
        started = utc_now()
        completed = 0
        try:
            for index, command in enumerate(commands, start=1):
                expected_prefix = [
                    sys.executable, str(Path(__file__).resolve()), "cell",
                    str(orchestration_path.resolve()), "--cell-id",
                ]
                if command[:5] != expected_prefix or len(command) != 6:
                    raise CampaignError(
                        f"command {index} is not an exact bound cell invocation")
                log_path = attempt_root / f"{index:02d}_{command[-1]}.log"
                descriptor = os.open(
                    log_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
                with os.fdopen(descriptor, "wb") as stream:
                    stream.write(("command: " + shlex.join(command) + "\n").encode())
                    stream.flush()
                    process = subprocess.run(
                        command, stdout=stream, stderr=subprocess.STDOUT)
                if process.returncode:
                    raise CampaignError(
                        f"cell {index}/40 failed with {process.returncode}; "
                        f"see {log_path}")
                completed += 1
            core = {
                "schema": SEQUENCE_ATTEMPT_SCHEMA,
                "state": "complete",
                "created_utc": started,
                "completed_utc": utc_now(),
                "orchestration_identity_sha256":
                    orchestration["identity_sha256"],
                "command_count": 40,
                "completed_command_count": completed,
                "strictly_sequential": True,
                "candidate_promotion_allowed": False,
                "high_rate_interface_remains_no_go": True,
            }
            _write_json_exclusive(
                attempt_root / "sequence_receipt.json",
                {**core, "identity_sha256": object_sha256(core)})
        except BaseException as error:
            _write_json_exclusive(attempt_root / "failure.json", {
                "schema": SEQUENCE_ATTEMPT_SCHEMA,
                "state": "failed_or_interrupted",
                "created_utc": started,
                "failed_utc": utc_now(),
                "completed_command_count": completed,
                "error_type": type(error).__name__,
                "error": str(error),
            })
            raise


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)
    cell = subparsers.add_parser("cell")
    cell.add_argument("orchestration", type=Path)
    cell.add_argument("--cell-id", required=True)
    sequence = subparsers.add_parser("sequence")
    sequence.add_argument("orchestration", type=Path)
    sequence.add_argument("--commands", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.mode == "cell":
            execute_cell(arguments.orchestration, arguments.cell_id)
        else:
            execute_sequence(arguments.orchestration, arguments.commands)
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Preflight or execute the frozen 60-cell ground Phase-B interaction grid."""

from __future__ import annotations

import argparse
import copy
import datetime as dt
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple
import uuid

import rosbag
import yaml

from build_vio_postfix_init_qualification_receipt import validate_build_manifest
from generate_vio_ground_phase_a_rebaseline import TIGHT_INIT_PARAMETERS
from generate_vio_ground_phase_b_interaction import (
    CELL_SCHEMA,
    COMMANDS_SCHEMA,
    CONFIG_IDS,
    CONFIGURATIONS,
    QUALIFICATION_VARIANT,
    REPEAT_IDS,
    SCHEMA as ORCHESTRATION_SCHEMA,
    SESSION_IDS,
    frozen_schedule,
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
    validate_output_topic_inventory,
    validate_parameter_snapshot,
    within_repo,
)
from run_vio_ground_phase_a_rebaseline import (
    _validate_primary_report,
    _validate_secondary_report,
)


CAMPAIGN_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_campaign/v1"
MANIFEST_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_run/v1"
COMPLETION_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_completion/v1"
SEQUENCE_ATTEMPT_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_sequence/v1"
PREFLIGHT_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_preflight/v1"
RETRY_ERROR_TEXT = "explicit anchor was not observed as a full-sync sensor epoch"
ESTIMATOR_OUTPUT_TOPICS = (
    "/aft_mapped_to_body",
    "/aft_mapped_to_init",
    "/aft_mapped_to_body_correction_pose_cov",
)


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def _self_hash(document: Mapping[str, Any], schema: str, label: str) -> str:
    if document.get("schema") != schema:
        raise CampaignError(f"{label}: wrong schema")
    declared = document.get("identity_sha256")
    core = dict(document)
    core.pop("identity_sha256", None)
    if not isinstance(declared, str) or object_sha256(core) != declared:
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


def _file_identity(path: Path) -> Dict[str, Any]:
    path = path.resolve()
    if not path.is_file():
        raise CampaignError(f"missing file: {path}")
    return {"path": str(path), "size_bytes": path.stat().st_size,
            "sha256": sha256(path)}


def _verify_file_identity(identity: Any, label: str) -> Path:
    if not isinstance(identity, Mapping):
        raise CampaignError(f"{label}: missing file identity")
    path = Path(str(identity.get("path", ""))).resolve()
    if (not path.is_file() or path.stat().st_size !=
            int(identity.get("size_bytes", -1)) or
            sha256(path) != identity.get("sha256")):
        raise CampaignError(f"{label}: file identity changed: {path}")
    return path


def _load_orchestration(path: Path) -> Tuple[Dict[str, Any], str]:
    path = path.resolve()
    document = load_json(path)
    identity = _self_hash(document, ORCHESTRATION_SCHEMA,
                          "ground Phase-B orchestration")
    design = document.get("design", {})
    evaluation = document.get("evaluation_contract", {})
    decision = document.get("decision_contract", {})
    retry = document.get("infrastructure_retry_contract", {})
    ranking = document.get("ranking_contract", {})
    if (Path(str(document.get("output_root", ""))).resolve() != path.parent or
            document.get("scope") != "development_only" or
            document.get("validation_data_accessed") is not False or
            document.get("qualification_variant") != QUALIFICATION_VARIANT or
            design.get("configuration_ids") != list(CONFIG_IDS) or
            design.get("session_ids") != list(SESSION_IDS) or
            design.get("repeat_ids") != list(REPEAT_IDS) or
            design.get("configuration_count") != 4 or
            design.get("session_count") != 5 or
            design.get("repeats_per_configuration_session") != 3 or
            design.get("expected_run_count") != 60 or
            design.get("rate") != 1.0 or
            design.get("fresh_campaign_per_cell") is not True or
            design.get("fresh_process_per_cell") is not True or
            design.get("strictly_sequential") is not True or
            design.get("order") !=
            "three_round_session_blocks_cyclic_configuration_rotation" or
            design.get("four_cell_blocks_are_complete_factorials") is not True or
            design.get("configuration_position_balance_max_minus_min") != 1 or
            design.get("within_session_repeat_position_balance_max_minus_min") != 1 or
            evaluation.get("primary") != "complete_ground_to_landing_safety" or
            evaluation.get("secondary_alignment") !=
            "reuse_primary_without_refit" or
            evaluation.get("secondary_can_override_primary_failure") is not False or
            evaluation.get("same_result_bag_required") is not True or
            decision.get("candidate_promotion_allowed") is not False or
            decision.get("flight_ready_can_be_declared") is not False or
            decision.get("global_high_rate_interface_remains_no_go") is not True or
            ranking.get("global_high_rate_blocker_is_separate") is not True or
            ranking.get("low_rate_repeat_determinism_is_hard_gate") is not True or
            retry.get("maximum_retries_per_cell") != 1 or
            retry.get("eligible_error_text_exact") != RETRY_ERROR_TEXT or
            retry.get("zero_estimator_output_required") is not True or
            retry.get("unapproved_failure_stops_sequence") is not True or
            retry.get("second_occurrence_stops_sequence") is not True or
            document.get("execution", {}).get(
                "generator_executed_replay") is not False or
            document.get("execution", {}).get(
                "generator_executed_build") is not False):
        raise CampaignError("ground Phase-B orchestration safety contract changed")
    cells = document.get("cells")
    commands = document.get("commands")
    if not isinstance(cells, list) or len(cells) != 60 or not isinstance(
            commands, list) or len(commands) != 60:
        raise CampaignError("ground Phase-B orchestration is not 60 cells")
    expected = frozen_schedule()
    observed = [
        (row.get("repeat_id"), row.get("session_id"),
         row.get("configuration_id"))
        for row in cells if isinstance(row, Mapping)
    ]
    if (observed != expected or
            [row.get("ordinal") for row in cells
             if isinstance(row, Mapping)] != list(range(1, 61)) or
            len({row.get("run_id") for row in cells
                 if isinstance(row, Mapping)}) != 60):
        raise CampaignError("ground Phase-B frozen grid/order changed")
    return dict(document), identity


def _validate_dependencies(orchestration: Mapping[str, Any]) -> Dict[str, Path]:
    dependencies = orchestration.get("dependencies")
    required = {
        "generator", "runner", "phase_b_reporter", "strict_evaluator",
        "replay_wrapper", "replay_launch", "fastlivo_base_config",
        "thresholds", "base_overlay", "phase_a_orchestration",
        "phase_a_report", "phase_a_generator", "phase_a_runner",
        "phase_a_reporter", "phase_a_selector", "phase_a_summarizer",
        "qualification_plan", "qualification_report",
        "qualified_build_manifest", "ground_anchors",
    }
    if not isinstance(dependencies, Mapping) or set(dependencies) != required:
        raise CampaignError("Phase-B dependency inventory is not exact")
    return {name: _verify_file_identity(value, f"dependency {name}")
            for name, value in dependencies.items()}


def _validate_gate_and_build(
        orchestration: Mapping[str, Any], paths: Mapping[str, Path], *,
        verify_actual_build: bool = True) -> None:
    phase_a, phase_a_identity = __import__(
        "run_vio_ground_phase_a_rebaseline")._load_orchestration(
            paths["phase_a_orchestration"])
    report = load_json(paths["phase_a_report"])
    _self_hash(report, "fastlivo_vio_ground_phase_a_report/v1",
               "ground Phase-A report")
    build = load_json(paths["qualified_build_manifest"])
    build_identity = validate_build_manifest(
        build, verify_actual=verify_actual_build)
    gate = orchestration["phase_a_gate"]
    qualified = orchestration["qualified_build"]
    if (gate.get("orchestration_identity_sha256") != phase_a_identity or
            gate.get("report_identity_sha256") !=
            report.get("identity_sha256") or
            gate.get("completed_run_count") != 40 or
            gate.get("fresh_process_instance_count") != 40 or
            gate.get("selected_main_effects") != ["outlier600", "acc5"] or
            report.get("selected_top_two_development_directions") !=
            ["outlier600", "acc5"] or
            qualified.get("identity_sha256") != build_identity or
            qualified.get("identity_sha256") !=
            phase_a.get("qualified_build", {}).get("identity_sha256") or
            qualified.get("executable_sha256") != build.get("executable_sha256") or
            qualified.get("source_tree_sha256") != build.get("source_tree_sha256") or
            orchestration.get("runtime_constant_binding") !=
            phase_a.get("runtime_constant_binding")):
        raise CampaignError("Phase-B Phase-A/build/source binding changed")


def _load_cell(orchestration: Mapping[str, Any], run_id: str) -> Tuple[Dict[str, Any], Path]:
    rows = [row for row in orchestration["cells"]
            if isinstance(row, Mapping) and row.get("run_id") == run_id]
    if len(rows) != 1:
        raise CampaignError(f"no unique Phase-B cell {run_id!r}")
    row = rows[0]
    path = _verify_file_identity(row["cell"], f"cell {run_id}")
    cell = load_json(path)
    identity = _self_hash(cell, CELL_SCHEMA, f"cell {run_id}")
    if (identity != row.get("cell_object_identity_sha256") or
            cell.get("ordinal") != row.get("ordinal") or
            cell.get("run_id") != run_id or
            cell.get("campaign_id") != row.get("campaign_id") or
            cell.get("repeat_id") != row.get("repeat_id") or
            cell.get("session", {}).get("session_id") != row.get("session_id") or
            cell.get("configuration", {}).get("id") !=
            row.get("configuration_id") or
            cell.get("scope") != "development_only" or
            cell.get("validation_data_accessed") is not False or
            cell.get("decision_contract", {}).get(
                "candidate_promotion_allowed") is not False or
            cell.get("decision_contract", {}).get(
                "flight_ready_can_be_declared") is not False or
            cell.get("decision_contract", {}).get(
                "global_high_rate_interface_remains_no_go") is not True):
        raise CampaignError(f"Phase-B cell safety contract changed: {run_id}")
    return dict(cell), path


def _validate_input(cell: Mapping[str, Any]) -> str:
    row = cell["session"]["input"]
    bag = Path(str(row["path"])).resolve()
    if (not bag.is_file() or bag.stat().st_size != int(row["size_bytes"]) or
            bag.stat().st_mtime_ns != int(row["mtime_ns"])):
        raise CampaignError(f"development input stat changed: {bag}")
    actual = sha256(bag)
    if actual != row.get("declared_sha256") or actual != row.get("verified_sha256"):
        raise CampaignError(f"development input SHA-256 changed: {bag}")
    _verify_file_identity(row["provenance"], "hybrid provenance")
    _verify_file_identity(row["window_cache"], "development window cache")
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
        "phase_a_gate": copy.deepcopy(orchestration["phase_a_gate"]),
        "qualified_build": copy.deepcopy(orchestration["qualified_build"]),
        "run_id": cell["run_id"],
        "repeat_id": cell["repeat_id"],
        "session_id": cell["session"]["session_id"],
        "configuration": copy.deepcopy(cell["configuration"]),
        "runtime_overrides": copy.deepcopy(cell["runtime_overrides"]),
        "replay": copy.deepcopy(cell["session"]["replay"]),
        "primary_evaluation": copy.deepcopy(cell["session"]["primary_evaluation"]),
        "secondary_evaluation": copy.deepcopy(cell["session"]["secondary_evaluation"]),
        "infrastructure_retry_contract": copy.deepcopy(
            orchestration["infrastructure_retry_contract"]),
        "fresh_process_required": True,
        "candidate_promotion_allowed": False,
        "flight_ready_can_be_declared": False,
        "global_high_rate_interface_remains_no_go": True,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _validate_manifest(attempt: Path, manifest: Mapping[str, Any]) -> None:
    _self_hash(manifest, MANIFEST_SCHEMA, "Phase-B run manifest")
    if (manifest.get("state") != "complete" or
            manifest.get("candidate_promotion_allowed") is not False or
            manifest.get("flight_ready") is not False or
            manifest.get("global_high_rate_interface_remains_no_go") is not True or
            manifest.get("secondary_can_override_primary_failure") is not False):
        raise CampaignError(f"completed Phase-B manifest changed: {attempt}")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping) or not artifacts:
        raise CampaignError(f"Phase-B manifest lacks artifacts: {attempt}")
    for name, identity in artifacts.items():
        if not isinstance(identity, Mapping):
            raise CampaignError(f"malformed Phase-B artifact identity: {name}")
        path = attempt / name
        if (Path(name).name != name or not path.is_file() or
                path.stat().st_size != int(identity.get("size_bytes", -1)) or
                sha256(path) != identity.get("sha256")):
            raise CampaignError(f"Phase-B artifact changed: {path}")
    inventory = bag_topic_inventory(attempt / "result.bag")
    if inventory != manifest.get("output_topic_inventory"):
        raise CampaignError("Phase-B result bag inventory changed")
    validate_output_topic_inventory(inventory)


def _validate_completion(
        campaign_dir: Path, pointer_path: Path, campaign_identity: str,
        run_id: str) -> Path:
    pointer = load_json(pointer_path)
    _self_hash(pointer, COMPLETION_SCHEMA, "Phase-B completion")
    relative = Path(str(pointer.get("attempt", "")))
    if (pointer.get("campaign_identity_sha256") != campaign_identity or
            pointer.get("run_id") != run_id or relative.is_absolute() or
            ".." in relative.parts):
        raise CampaignError("Phase-B completion pointer changed")
    attempt = (campaign_dir / relative).resolve()
    try:
        attempt.relative_to(campaign_dir.resolve())
    except ValueError as error:
        raise CampaignError("Phase-B completion attempt escapes campaign") from error
    manifest_path = attempt / "manifest.json"
    if (not manifest_path.is_file() or
            sha256(manifest_path) != pointer.get("manifest_sha256")):
        raise CampaignError("Phase-B completion manifest changed")
    manifest = load_json(manifest_path)
    if (manifest.get("campaign_identity_sha256") != campaign_identity or
            manifest.get("run_id") != run_id):
        raise CampaignError("Phase-B completion belongs to another cell")
    _validate_manifest(attempt, manifest)
    return attempt


def _require_predecessors(
        orchestration: Mapping[str, Any], orchestration_identity: str,
        target: Mapping[str, Any]) -> None:
    output_root = Path(str(orchestration["output_root"])).resolve()
    ordinal = int(target["ordinal"])
    for row in orchestration["cells"][:ordinal - 1]:
        cell, cell_path = _load_cell(orchestration, str(row["run_id"]))
        campaign = _campaign_plan(
            orchestration, orchestration_identity, cell, cell_path)
        campaign_dir = output_root / "campaigns" / cell["campaign_id"]
        campaign_path = campaign_dir / "campaign.json"
        completion_path = campaign_dir / "completion.json"
        if (not campaign_path.is_file() or load_json(campaign_path) != campaign or
                not completion_path.is_file()):
            raise CampaignError(
                f"cell {ordinal} cannot precede incomplete cell {row['ordinal']}")
        _validate_completion(campaign_dir, completion_path,
                             campaign["identity_sha256"], row["run_id"])


def _structured_failed_events(attempt: Path) -> list[Dict[str, Any]]:
    events: list[Dict[str, Any]] = []
    bag = attempt / "result.bag"
    if bag.is_file():
        try:
            with rosbag.Bag(str(bag), "r") as stream:
                info = stream.get_type_and_topic_info().topics.get("/rosout")
                if info is None or info.msg_type != "rosgraph_msgs/Log":
                    return []
                for _, message, _ in stream.read_messages(topics=["/rosout"]):
                    text = str(getattr(message, "msg", ""))
                    marker = text.find("[imu_init_diag]")
                    if marker < 0:
                        continue
                    start = text.find("{", marker)
                    end = text.rfind("}")
                    if start < 0 or end < start:
                        return []
                    event = json.loads(text[start:end + 1])
                    if event.get("status") == "failed":
                        events.append(event)
        except Exception:
            return []
    return events


def _eligible_startup_retry(attempt: Path) -> Tuple[bool, Dict[str, Any]]:
    bag = attempt / "result.bag"
    try:
        inventory = bag_topic_inventory(bag) if bag.is_file() else {}
    except Exception:
        inventory = {}
    counts = {topic: int(inventory.get(topic, {}).get("message_count", 0))
              for topic in ESTIMATOR_OUTPUT_TOPICS}
    events = _structured_failed_events(attempt)
    event = events[0] if len(events) == 1 else {}
    anchor_ns = str(event.get("anchor_stamp_ns", ""))
    sync_ns = str(event.get("sync_epoch_ns", ""))
    configured_anchor_ns = ""
    params_path = attempt / "result_params.yaml"
    if params_path.is_file():
        try:
            params = yaml.safe_load(params_path.read_text())
            if isinstance(params, Mapping) and isinstance(params.get("imu"), Mapping):
                configured_anchor_ns = str(
                    params["imu"].get("init_anchor_stamp_ns", ""))
        except (OSError, yaml.YAMLError):
            configured_anchor_ns = ""
    exact_error_observed = (
        len(events) == 1 and
        event.get("schema") == "fast_livo/imu_init/v1" and
        event.get("status") == "failed" and
        event.get("anchor_mode") == "explicit" and
        event.get("reason") == RETRY_ERROR_TEXT and
        anchor_ns.isdigit() and sync_ns.isdigit() and
        int(anchor_ns) > 0 and int(sync_ns) > int(anchor_ns) and
        configured_anchor_ns == anchor_ns
    )
    zero_outputs = all(value == 0 for value in counts.values())
    return exact_error_observed and zero_outputs, {
        "eligible_error_text": RETRY_ERROR_TEXT,
        "eligible_error_text_observed": exact_error_observed,
        "structured_rosout_failed_event_count": len(events),
        "structured_rosout_failed_event": event or None,
        "configured_anchor_stamp_ns": configured_anchor_ns or None,
        "event_anchor_matches_result_parameters":
            bool(anchor_ns and configured_anchor_ns == anchor_ns),
        "estimator_output_message_counts": counts,
        "zero_estimator_outputs": zero_outputs,
    }


def _inventory_attempt_directories(
        campaign_dir: Path, completed_attempt: Optional[Path] = None
        ) -> list[Path]:
    """Reject orphan/extra attempts and return the approved failed attempt.

    A SIGKILL can leave neither failure.json nor manifest.json.  Such a
    directory must never be silently skipped by a later retry or report.
    """
    attempts_root = campaign_dir / "attempts"
    if not attempts_root.exists():
        if completed_attempt is not None:
            raise CampaignError("completion points into missing attempts directory")
        return []
    children = sorted(attempts_root.iterdir())
    if any(not child.is_dir() for child in children):
        raise CampaignError("Phase-B attempts inventory contains a non-directory")
    completed = completed_attempt.resolve() if completed_attempt is not None else None
    success_count = 0
    approved_failures: list[Path] = []
    for attempt in children:
        manifest_path = attempt / "manifest.json"
        failure_path = attempt / "failure.json"
        is_completed = completed is not None and attempt.resolve() == completed
        if is_completed:
            if not manifest_path.is_file() or failure_path.exists():
                raise CampaignError("completed Phase-B attempt inventory is malformed")
            success_count += 1
            continue
        if manifest_path.exists() or not failure_path.is_file():
            raise CampaignError(
                f"orphan/incomplete/extra Phase-B attempt directory: {attempt}")
        failure = load_json(failure_path)
        eligible, _ = _eligible_startup_retry(attempt)
        if (not eligible or
                failure.get("state") != "failed_or_interrupted" or
                failure.get("infrastructure_retry_approved") is not True or
                failure.get("process_attempt_ordinal") != 1 or
                failure.get("excluded_from_accuracy") is not True or
                failure.get("enters_configuration_tuning_rank") is not False):
            raise CampaignError(
                f"unapproved Phase-B failed attempt directory: {attempt}")
        approved_failures.append(attempt)
    if completed is not None and success_count != 1:
        raise CampaignError("Phase-B attempt inventory lacks one completed attempt")
    if len(approved_failures) > 1:
        raise CampaignError("Phase-B attempt inventory exceeds retry limit")
    expected_count = success_count + len(approved_failures)
    if len(children) != expected_count:
        raise CampaignError("Phase-B attempt inventory contains extra directories")
    return approved_failures


def _evaluation_commands(
        result_bag: Path, thresholds: Path, primary: Path, secondary: Path,
        score_start_ns: str, score_end_ns: str) -> Tuple[list[str], list[str]]:
    primary_command = [sys.executable, str(EVALUATOR), str(result_bag),
                       "--thresholds", str(thresholds), "--output", str(primary)]
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
    orchestration, orchestration_identity = _load_orchestration(orchestration_path)
    paths = _validate_dependencies(orchestration)
    if (paths["runner"] != Path(__file__).resolve() or
            paths["strict_evaluator"] != EVALUATOR.resolve() or
            paths["replay_wrapper"] != REPLAY.resolve()):
        raise CampaignError("Phase-B runtime tool differs from bound dependency")
    _validate_gate_and_build(orchestration, paths)
    cell, cell_path = _load_cell(orchestration, run_id)
    input_hash = _validate_input(cell)
    output_root = within_repo(Path(orchestration["output_root"]).resolve(),
                              "ground Phase-B output root")
    campaign_dir = output_root / "campaigns" / cell["campaign_id"]
    campaign = _campaign_plan(
        orchestration, orchestration_identity, cell, cell_path)

    lock_path = output_root / ".worker.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+b") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError("another Phase-B replay worker is active") from error
        _require_predecessors(orchestration, orchestration_identity, cell)
        campaign_dir.mkdir(parents=True, exist_ok=True)
        campaign_path = campaign_dir / "campaign.json"
        if campaign_path.exists():
            if load_json(campaign_path) != campaign:
                raise CampaignError("immutable Phase-B campaign differs")
        else:
            _write_json_exclusive(campaign_path, campaign)
        completion_path = campaign_dir / "completion.json"
        if completion_path.exists():
            attempt = _validate_completion(
                campaign_dir, completion_path, campaign["identity_sha256"], run_id)
            _inventory_attempt_directories(campaign_dir, attempt)
            print(f"SKIP validated {run_id} -> {attempt}")
            return "skipped"

        # Existing failed attempts may only be the one preregistered transient
        # startup miss; this also makes interrupted reruns fail closed.
        existing_failed_attempts = _inventory_attempt_directories(campaign_dir)
        process_attempt_ordinal = len(existing_failed_attempts)
        while process_attempt_ordinal < 2:
            process_attempt_ordinal += 1
            process_uuid = str(uuid.uuid4())
            stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            attempt = campaign_dir / "attempts" / (
                f"{stamp}_{uuid.uuid4().hex[:12]}")
            attempt.mkdir(parents=True, exist_ok=False)
            base = load_yaml_mapping(paths["base_overlay"], "base overlay")
            overlay = effective_overlay(base, {
                "id": cell["configuration"]["id"],
                "overrides": copy.deepcopy(cell["runtime_overrides"]),
            })
            if object_sha256(overlay) != cell["effective_overlay_sha256"]:
                raise CampaignError("Phase-B effective overlay changed")
            imu = overlay.get("imu", {})
            expected_imu = {
                **dict(TIGHT_INIT_PARAMETERS),
                "init_anchor_stamp_ns":
                    cell["session"]["explicit_anchor"]["anchor_stamp_ns"],
            }
            if (any(imu.get(key) != value for key, value in expected_imu.items()) or
                    imu.get("acc_cov") != cell["configuration"]["acc_cov"] or
                    overlay.get("vio", {}).get("img_point_cov") != 1000.0 or
                    overlay.get("vio", {}).get("outlier_threshold") !=
                    cell["configuration"]["outlier_threshold"]):
                raise CampaignError("Phase-B runtime factor/init parameters changed")
            overlay_path = attempt / "overlay.yaml"
            _write_bytes_exclusive(
                overlay_path, yaml.safe_dump(overlay, sort_keys=True).encode())
            result_bag = attempt / "result.bag"
            primary_path = attempt / "result.full.flight_readiness.json"
            secondary_path = attempt / "result.hover.ranking.json"
            replay = cell["session"]["replay"]
            replay_command = [
                "bash", str(REPLAY),
                container_path(Path(cell["session"]["input"]["path"])),
                "--rate", "1", "--start", "0",
                "--duration", format(float(replay["duration_s"]), ".17g"),
                "--overlay", container_path(overlay_path),
                "--out", container_path(result_bag),
                "--no-gt-anchor", "--with-propagated",
            ]
            secondary = cell["session"]["secondary_evaluation"]
            primary_command, secondary_command = _evaluation_commands(
                result_bag, paths["thresholds"], primary_path, secondary_path,
                str(secondary["score_start_ns"]), str(secondary["score_end_ns"]))
            environment = os.environ.copy()
            environment.pop("FASTLIVO_QUALIFICATION_RUN_BINDING", None)
            environment.update({
                "FASTLIVO_REPLAY_CONTAINER": orchestration["qualified_build"]["container"],
                "FASTLIVO_REPLAY_DEVEL": orchestration["qualified_build"]["replay_devel"],
                "FASTLIVO_REPLAY_PORT": str(orchestration["ros_master_port"]),
            })
            started = utc_now()
            try:
                check_no_live_worker(orchestration["qualified_build"]["container"],
                                     int(orchestration["ros_master_port"]))
                run_logged(replay_command, environment,
                           attempt / "replay.stdout.log")
                validate_parameter_snapshot(attempt / "result_params.yaml", overlay)
                run_logged(primary_command, environment,
                           attempt / "evaluate.primary.stdout.log")
                primary = _validate_primary_report(
                    primary_path, result_bag, paths["thresholds"])
                run_logged(secondary_command, environment,
                           attempt / "evaluate.secondary.stdout.log")
                secondary_report = _validate_secondary_report(
                    secondary_path, result_bag, paths["thresholds"],
                    primary_path, primary, str(secondary["score_start_ns"]),
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
                    "repeat_id": cell["repeat_id"],
                    "session_id": cell["session"]["session_id"],
                    "configuration_id": cell["configuration"]["id"],
                    "fresh_process_instance_uuid": process_uuid,
                    "process_attempt_ordinal": process_attempt_ordinal,
                    "startup_retry_was_used": process_attempt_ordinal == 2,
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
                    "secondary_status": secondary_report["status"],
                    "secondary_ranking_metrics_available": True,
                    "secondary_can_override_primary_failure": False,
                    "candidate_promotion_allowed": False,
                    "flight_ready": False,
                    "global_high_rate_interface_remains_no_go": True,
                    "output_topic_inventory": inventory,
                    "artifacts": artifacts,
                }
                manifest = {**manifest_core,
                            "identity_sha256": object_sha256(manifest_core)}
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
                    "process_attempt_count": process_attempt_ordinal,
                    "retained_failed_attempt_count": process_attempt_ordinal - 1,
                }
                completion = {**completion_core,
                              "identity_sha256": object_sha256(completion_core)}
                _write_json_exclusive(completion_path, completion)
                _validate_completion(campaign_dir, completion_path,
                                     campaign["identity_sha256"], run_id)
                print(f"DONE {run_id}: attempt={process_attempt_ordinal} "
                      f"primary={primary['status']} secondary=ranking_only")
                return "completed"
            except BaseException as error:
                eligible, evidence = _eligible_startup_retry(attempt)
                retry_approved = eligible and process_attempt_ordinal == 1
                failure = {
                    "schema": MANIFEST_SCHEMA,
                    "state": "failed_or_interrupted",
                    "created_utc": started,
                    "failed_utc": utc_now(),
                    "campaign_identity_sha256": campaign["identity_sha256"],
                    "run_id": run_id,
                    "fresh_process_instance_uuid": process_uuid,
                    "process_attempt_ordinal": process_attempt_ordinal,
                    "error_type": type(error).__name__,
                    "error": str(error),
                    "infrastructure_retry_evidence": evidence,
                    "infrastructure_retry_approved": retry_approved,
                    "failure_retained_append_only": True,
                    "excluded_from_accuracy": True,
                    "counts_as_global_operational_warning": True,
                    "enters_configuration_tuning_rank": False,
                }
                failure_path = attempt / "failure.json"
                if not failure_path.exists():
                    _write_json_exclusive(failure_path, failure)
                if retry_approved:
                    print(f"RETRY approved exact startup miss {run_id}",
                          file=sys.stderr)
                    continue
                raise
        raise CampaignError("Phase-B retry budget exhausted")


def _validate_commands(
        orchestration_path: Path, orchestration: Mapping[str, Any],
        path: Path) -> list[list[str]]:
    document = load_json(path.resolve())
    _self_hash(document, COMMANDS_SCHEMA, "Phase-B command list")
    commands = document.get("commands")
    if (document.get("scope") != "development_only" or
            document.get("validation_data_accessed") is not False or
            Path(str(document.get("orchestration_path", ""))).resolve() !=
            orchestration_path.resolve() or
            document.get("orchestration_identity_sha256") !=
            orchestration["identity_sha256"] or
            document.get("strictly_sequential") is not True or
            document.get("stop_on_unapproved_or_second_failure") is not True or
            document.get("eligible_startup_failure_retry_limit") != 1 or
            document.get("fresh_campaign_per_cell") is not True or
            document.get("command_count") != 60 or
            commands != orchestration.get("commands")):
        raise CampaignError("Phase-B command list differs from orchestration")
    return [list(command) for command in commands]


def preflight(
        orchestration_path: Path, commands_path: Path, *,
        verify_actual_build: bool = True,
        verify_inputs: bool = True) -> Dict[str, Any]:
    orchestration, identity = _load_orchestration(orchestration_path)
    paths = _validate_dependencies(orchestration)
    _validate_gate_and_build(
        orchestration, paths, verify_actual_build=verify_actual_build)
    commands = _validate_commands(orchestration_path, orchestration, commands_path)
    check_no_live_worker(
        str(orchestration["qualified_build"]["container"]),
        int(orchestration["ros_master_port"]))
    verified_sessions = set()
    for row in orchestration["cells"]:
        cell, _ = _load_cell(orchestration, row["run_id"])
        session_id = cell["session"]["session_id"]
        if verify_inputs and session_id not in verified_sessions:
            _validate_input(cell)
            verified_sessions.add(session_id)
    core = {
        "schema": PREFLIGHT_SCHEMA,
        "status": "go",
        "orchestration_identity_sha256": identity,
        "command_count": len(commands),
        "unique_development_inputs_verified": len(verified_sessions),
        "actual_build_and_source_verified": verify_actual_build,
        "no_active_worker_verified": True,
        "worker_check_container": orchestration["qualified_build"]["container"],
        "worker_check_ros_master_port": orchestration["ros_master_port"],
        "validation_data_accessed": False,
        "replay_executed": False,
        "build_executed": False,
        "candidate_promotion_allowed": False,
        "flight_ready": False,
        "global_high_rate_interface_remains_no_go": True,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def execute_sequence(orchestration_path: Path, commands_path: Path) -> None:
    orchestration, _ = _load_orchestration(orchestration_path)
    commands = _validate_commands(orchestration_path, orchestration, commands_path)
    output_root = within_repo(Path(orchestration["output_root"]).resolve(),
                              "ground Phase-B output root")
    with (output_root / ".sequence.lock").open("a+b") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise CampaignError("another Phase-B sequence is active") from error
        attempt_root = output_root / "sequence_attempts" / (
            "sequence_" + uuid.uuid4().hex)
        attempt_root.mkdir(parents=True, exist_ok=False)
        started = utc_now()
        completed = 0
        try:
            for index, command in enumerate(commands, start=1):
                expected = [sys.executable, str(Path(__file__).resolve()), "cell",
                            str(orchestration_path.resolve()), "--cell-id"]
                if command[:5] != expected or len(command) != 6:
                    raise CampaignError(f"command {index} is not exact")
                log_path = attempt_root / f"{index:02d}_{command[-1]}.log"
                descriptor = os.open(log_path, os.O_WRONLY | os.O_CREAT |
                                     os.O_EXCL, 0o644)
                with os.fdopen(descriptor, "wb") as stream:
                    stream.write(("command: " + shlex.join(command) + "\n").encode())
                    process = subprocess.run(command, stdout=stream,
                                             stderr=subprocess.STDOUT)
                if process.returncode:
                    raise CampaignError(
                        f"cell {index}/60 permanently failed with "
                        f"{process.returncode}; see {log_path}")
                completed += 1
            core = {
                "schema": SEQUENCE_ATTEMPT_SCHEMA,
                "state": "complete",
                "created_utc": started,
                "completed_utc": utc_now(),
                "orchestration_identity_sha256":
                    orchestration["identity_sha256"],
                "command_count": 60,
                "completed_command_count": completed,
                "strictly_sequential": True,
                "unapproved_or_second_failure_stops_sequence": True,
                "candidate_promotion_allowed": False,
                "global_high_rate_interface_remains_no_go": True,
            }
            _write_json_exclusive(attempt_root / "sequence_receipt.json",
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
    sub = parser.add_subparsers(dest="mode", required=True)
    cell = sub.add_parser("cell")
    cell.add_argument("orchestration", type=Path)
    cell.add_argument("--cell-id", required=True)
    sequence = sub.add_parser("sequence")
    sequence.add_argument("orchestration", type=Path)
    sequence.add_argument("--commands", type=Path, required=True)
    check = sub.add_parser("preflight")
    check.add_argument("orchestration", type=Path)
    check.add_argument("--commands", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.mode == "cell":
            execute_cell(arguments.orchestration, arguments.cell_id)
        elif arguments.mode == "sequence":
            execute_sequence(arguments.orchestration, arguments.commands)
        else:
            print(json.dumps(preflight(arguments.orchestration,
                                       arguments.commands), indent=2,
                             sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

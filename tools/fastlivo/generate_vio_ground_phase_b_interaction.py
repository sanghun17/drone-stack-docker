#!/usr/bin/env python3
"""Freeze the development-only ground-init Phase-B interaction experiment.

The input is the completed official ground Phase-A report, not the legacy
prefix campaign.  Phase-A selected ``outlier600`` and ``acc5``; this workflow
therefore freezes the 2 x 2 interaction acc_cov {10, 5} x
outlier_threshold {1000, 600}, with img_point_cov fixed at 1000.  Every
configuration/session pair is repeated in three fresh processes (60 cells).

Cells are emitted in a frozen, block-interleaved order.  Each four-cell block
contains every configuration once, and cyclic within-block rotations spread
configuration position over wall time.  Generation hashes inputs and the
qualified build but never starts ROS, replay, an evaluator, or a build.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import rosbag

from build_vio_ground_phase_a_rebaseline_report import SCHEMA as PHASE_A_REPORT_SCHEMA
from generate_vio_ground_phase_a_rebaseline import (
    QUALIFICATION_VARIANT,
    SESSION_IDS,
    TIGHT_INIT_PARAMETERS,
    _load_yaml,
    _self_hash,
)
import run_vio_ground_phase_a_rebaseline as phase_a_runner
from run_vio_flight_tuning_campaign import (
    CampaignError,
    EVALUATOR,
    FASTLIVO_BASE_CONFIG,
    REPLAY,
    REPLAY_LAUNCH,
    bag_topic_inventory,
    effective_overlay,
    file_identity,
    load_json,
    object_sha256,
    sha256,
    validate_effective_overlay,
    within_repo,
)


SCHEMA = "fastlivo_vio_ground_phase_b_interaction_orchestration/v1"
CELL_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_cell/v1"
COMMANDS_SCHEMA = "fastlivo_vio_ground_phase_b_interaction_commands/v1"

TOOLS = Path(__file__).resolve().parent
RUNNER = TOOLS / "run_vio_ground_phase_b_interaction.py"
REPORTER = TOOLS / "build_vio_ground_phase_b_interaction_report.py"
PHASE_A_SELECTOR = TOOLS / "select_vio_flight_tuning_phase_a.py"
PHASE_A_SUMMARIZER = TOOLS / "summarize_vio_flight_tuning_campaign.py"
DEFAULT_PHASE_A_ROOT = (
    TOOLS / "_campaign_vio_flight_20260814" /
    "ground_phase_a_rebaseline_v1"
)
DEFAULT_FAILED_ATTEMPT = (
    DEFAULT_PHASE_A_ROOT / "campaigns" /
    "ground_a_33_outlier300_p0_20260804_211027" / "attempts" /
    "20260813T222537.864438Z_5242dceede0c"
)

CONFIGURATIONS: Tuple[Mapping[str, Any], ...] = (
    {
        "id": "acc10_img1000_out1000",
        "acc_cov": 10.0,
        "img_point_cov": 1000.0,
        "outlier_threshold": 1000.0,
    },
    {
        "id": "acc5_img1000_out1000",
        "acc_cov": 5.0,
        "img_point_cov": 1000.0,
        "outlier_threshold": 1000.0,
    },
    {
        "id": "acc10_img1000_out600",
        "acc_cov": 10.0,
        "img_point_cov": 1000.0,
        "outlier_threshold": 600.0,
    },
    {
        "id": "acc5_img1000_out600",
        "acc_cov": 5.0,
        "img_point_cov": 1000.0,
        "outlier_threshold": 600.0,
    },
)
CONFIG_IDS: Tuple[str, ...] = tuple(str(row["id"]) for row in CONFIGURATIONS)
REPEAT_IDS: Tuple[str, ...] = ("r1", "r2", "r3")

# Each session occurs once in each repeat, at a deliberately changed temporal
# position.  The configuration order is separately rotated by global block
# index, so every four consecutive cells remain a complete 2 x 2 block.
ROUND_SESSION_ORDERS: Tuple[Tuple[str, ...], ...] = (
    tuple(SESSION_IDS),
    (SESSION_IDS[3], SESSION_IDS[4], SESSION_IDS[0],
     SESSION_IDS[1], SESSION_IDS[2]),
    (SESSION_IDS[1], SESSION_IDS[2], SESSION_IDS[3],
     SESSION_IDS[4], SESSION_IDS[0]),
)


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


def _file_binding(path: Path) -> Dict[str, Any]:
    path = path.resolve()
    if not path.is_file():
        raise CampaignError(f"missing bound file: {path}")
    return {
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "mtime_ns": path.stat().st_mtime_ns,
        "sha256": sha256(path),
    }


def frozen_schedule() -> List[Tuple[str, str, str]]:
    """Return (repeat, session, configuration) in the exact 60-cell order."""
    schedule: List[Tuple[str, str, str]] = []
    block = 0
    for repeat_id, sessions in zip(REPEAT_IDS, ROUND_SESSION_ORDERS):
        for session_id in sessions:
            rotation = block % len(CONFIG_IDS)
            order = CONFIG_IDS[rotation:] + CONFIG_IDS[:rotation]
            schedule.extend((repeat_id, session_id, config_id)
                            for config_id in order)
            block += 1
    if (len(schedule) != 60 or len(set(schedule)) != 60 or
            set(schedule) != {
                (repeat_id, session_id, config_id)
                for repeat_id in REPEAT_IDS
                for session_id in SESSION_IDS
                for config_id in CONFIG_IDS
            }):
        raise AssertionError("invalid frozen Phase-B schedule")
    return schedule


def _validate_position_balance(schedule: Sequence[Tuple[str, str, str]]) -> None:
    counts = {config_id: [0, 0, 0, 0] for config_id in CONFIG_IDS}
    per_session = {
        (session_id, config_id): [0, 0, 0, 0]
        for session_id in SESSION_IDS for config_id in CONFIG_IDS
    }
    for start in range(0, len(schedule), 4):
        block = schedule[start:start + 4]
        if len(block) != 4 or {row[2] for row in block} != set(CONFIG_IDS):
            raise CampaignError("Phase-B order is not complete four-cell blocks")
        if len({(row[0], row[1]) for row in block}) != 1:
            raise CampaignError("Phase-B block mixes repeat/session strata")
        for position, (_, _, config_id) in enumerate(block):
            counts[config_id][position] += 1
            per_session[(block[0][1], config_id)][position] += 1
    if any(max(row) - min(row) > 1 for row in counts.values()):
        raise CampaignError("Phase-B configuration position is not balanced")
    if any(max(row) - min(row) > 1 for row in per_session.values()):
        raise CampaignError(
            "Phase-B within-session repeat position is not balanced")


def _validate_phase_a_report(
        report: Mapping[str, Any], orchestration: Mapping[str, Any],
        orchestration_identity: str) -> str:
    identity = _self_hash(report, PHASE_A_REPORT_SCHEMA, "ground Phase-A report")
    selection = report.get("selection")
    summary = report.get("summary")
    completions = report.get("completions")
    if (report.get("scope") != "development_only" or
            report.get("validation_data_accessed") is not False or
            report.get("orchestration_identity_sha256") !=
            orchestration_identity or
            report.get("qualified_build_identity_sha256") !=
            orchestration.get("qualified_build", {}).get("identity_sha256") or
            report.get("expected_run_count") != 40 or
            report.get("completed_run_count") != 40 or
            report.get("fresh_process_instance_count") != 40 or
            report.get("selected_top_two_development_directions") !=
            ["outlier600", "acc5"] or
            report.get("selected_top_two_are_not_promoted_candidates") is not True or
            report.get("candidate_promotion_allowed") is not False or
            report.get("flight_ready") is not False or
            report.get("high_rate_interface_remains_no_go") is not True or
            not isinstance(selection, Mapping) or
            selection.get("selection_complete") is not True or
            selection.get("selected_top_two") != ["outlier600", "acc5"] or
            selection.get("development_ranking_only") is not True or
            selection.get("candidate_promotion_allowed") is not False or
            not isinstance(summary, Mapping) or
            summary.get("validation_data_accessed") is not False or
            summary.get("old_scores_may_be_pooled") is not False or
            not isinstance(completions, list) or len(completions) != 40 or
            len({row.get("fresh_process_instance_uuid") for row in completions
                 if isinstance(row, Mapping)}) != 40):
        raise CampaignError("Phase-A report is not the completed official dev result")
    return identity


def _sessions_from_phase_a(
        orchestration: Mapping[str, Any]) -> Dict[str, Dict[str, Any]]:
    by_id: Dict[str, Dict[str, Any]] = {}
    for row in orchestration["cells"]:
        cell, _ = phase_a_runner._load_cell(
            orchestration, str(row["run_id"]))
        session_id = str(cell["session"]["session_id"])
        proposed = copy.deepcopy(dict(cell["session"]))
        if session_id in by_id and by_id[session_id] != proposed:
            raise CampaignError(f"Phase-A session contract differs across arms: {session_id}")
        by_id[session_id] = proposed
    if list(session_id for session_id in SESSION_IDS if session_id in by_id) != list(SESSION_IDS):
        raise CampaignError("Phase-A lacks exact five development sessions")
    return by_id


def _validate_failed_attempt(
        attempt: Path, phase_a_root: Path) -> Dict[str, Any]:
    attempt = attempt.resolve()
    try:
        attempt.relative_to(phase_a_root.resolve())
    except ValueError as error:
        raise CampaignError("Phase-A failed attempt escapes official root") from error
    failure_path = attempt / "failure.json"
    failure = load_json(failure_path)
    if (failure.get("state") != "failed_or_interrupted" or
            failure.get("run_id") !=
            "ga33_outlier300__p0_20260804_211027" or
            failure.get("error_type") != "CampaignError" or
            not isinstance(failure.get("fresh_process_instance_uuid"), str)):
        raise CampaignError("unexpected Phase-A operational failure provenance")
    inventory = {
        path.name: _file_binding(path)
        for path in sorted(attempt.iterdir()) if path.is_file()
    }
    if set(inventory) != {
            "failure.json", "overlay.yaml", "replay.stdout.log",
            "result.bag", "result_params.yaml"}:
        raise CampaignError("Phase-A failed-attempt inventory changed")
    result_bag = attempt / "result.bag"
    topic_inventory = bag_topic_inventory(result_bag)
    estimator_counts = {
        topic: int(topic_inventory.get(topic, {}).get("message_count", 0))
        for topic in (
            "/aft_mapped_to_body", "/aft_mapped_to_init",
            "/aft_mapped_to_body_correction_pose_cov",
        )
    }
    events: List[Dict[str, Any]] = []
    with rosbag.Bag(str(result_bag), "r") as bag:
        info = bag.get_type_and_topic_info().topics.get("/rosout")
        if info is None or info.msg_type != "rosgraph_msgs/Log":
            raise CampaignError("Phase-A failed attempt lacks typed /rosout")
        for _, message, _ in bag.read_messages(topics=["/rosout"]):
            text = str(message.msg)
            marker = text.find("[imu_init_diag]")
            if marker < 0:
                continue
            start = text.find("{", marker)
            end = text.rfind("}")
            if start < 0 or end < start:
                raise CampaignError("malformed Phase-A IMU-init diagnostic")
            events.append(json.loads(text[start:end + 1]))
    if len(events) != 1:
        raise CampaignError("Phase-A failure needs exactly one IMU-init event")
    event = events[0]
    anchor_ns = str(event.get("anchor_stamp_ns", ""))
    sync_ns = str(event.get("sync_epoch_ns", ""))
    params = _load_yaml(attempt / "result_params.yaml", "failed result parameters")
    configured_anchor_ns = str(
        params.get("imu", {}).get("init_anchor_stamp_ns", ""))
    if (event.get("schema") != "fast_livo/imu_init/v1" or
            event.get("status") != "failed" or
            event.get("anchor_mode") != "explicit" or
            event.get("reason") !=
            "explicit anchor was not observed as a full-sync sensor epoch" or
            not anchor_ns.isdigit() or not sync_ns.isdigit() or
            configured_anchor_ns != anchor_ns or
            int(anchor_ns) <= 0 or int(sync_ns) <= int(anchor_ns) or
            any(estimator_counts.values())):
        raise CampaignError(
            "Phase-A operational failure lacks exact zero-output anchor evidence")
    return {
        "role": "operational_provenance_only",
        "attempt": str(attempt),
        "run_id": failure["run_id"],
        "fresh_process_instance_uuid": failure["fresh_process_instance_uuid"],
        "state": failure["state"],
        "files": inventory,
        "structured_rosout_failure_event": event,
        "estimator_output_message_counts": estimator_counts,
        "zero_estimator_outputs": True,
        "preserved_append_only": True,
        "excluded_from_phase_a_completed_cells": True,
        "excluded_from_phase_b_numeric_pool": True,
        "does_not_clear_or_create_high_rate_go": True,
    }


def _runtime_overrides(config: Mapping[str, Any], anchor_stamp_ns: str) -> Dict[str, Any]:
    return {
        "imu": {
            "acc_cov": float(config["acc_cov"]),
            **copy.deepcopy(dict(TIGHT_INIT_PARAMETERS)),
            "init_anchor_stamp_ns": anchor_stamp_ns,
        },
        "vio": {
            "img_point_cov": float(config["img_point_cov"]),
            "outlier_threshold": float(config["outlier_threshold"]),
        },
    }


def prepare_orchestration(
        phase_a_orchestration_path: Path, phase_a_report_path: Path,
        base_overlay_path: Path, thresholds_path: Path, output_root: Path,
        failed_attempt_path: Path, *, port: int,
        python_executable: str = sys.executable,
        verify_actual_build: bool = True) -> Dict[str, Any]:
    phase_a_orchestration_path = phase_a_orchestration_path.resolve()
    phase_a_report_path = phase_a_report_path.resolve()
    phase_a, phase_a_identity = phase_a_runner._load_orchestration(
        phase_a_orchestration_path)
    phase_a_paths = phase_a_runner._validate_dependencies(phase_a)
    phase_a_runner._validate_gate_and_build(
        phase_a, phase_a_paths, verify_actual_build=verify_actual_build)
    report = load_json(phase_a_report_path)
    report_identity = _validate_phase_a_report(
        report, phase_a, phase_a_identity)
    sessions = _sessions_from_phase_a(phase_a)
    base_overlay = _load_yaml(base_overlay_path.resolve(), "base overlay")
    if phase_a_paths["base_overlay"] != base_overlay_path.resolve():
        raise CampaignError("Phase-B base overlay differs from official Phase-A")
    if phase_a_paths["thresholds"] != thresholds_path.resolve():
        raise CampaignError("Phase-B thresholds differ from official Phase-A")
    for path, label in ((RUNNER, "Phase-B runner"),
                        (REPORTER, "Phase-B reporter"),
                        (PHASE_A_SELECTOR, "Phase-A selector"),
                        (PHASE_A_SUMMARIZER, "Phase-A summarizer"),
                        (EVALUATOR, "strict evaluator"),
                        (REPLAY, "replay wrapper"),
                        (REPLAY_LAUNCH, "replay launch"),
                        (FASTLIVO_BASE_CONFIG, "FAST-LIVO base config")):
        if not path.resolve().is_file():
            raise CampaignError(f"missing {label}: {path.resolve()}")

    phase_a_root = phase_a_orchestration_path.parent
    failure_provenance = _validate_failed_attempt(
        failed_attempt_path, phase_a_root)
    schedule = frozen_schedule()
    _validate_position_balance(schedule)
    output_root = within_repo(output_root.resolve(), "ground Phase-B output root")
    if output_root.exists():
        raise FileExistsError(output_root)
    if not 1024 <= int(port) <= 65535:
        raise CampaignError("port must be between 1024 and 65535")

    dependencies = {
        "generator": file_identity(Path(__file__).resolve()),
        "runner": file_identity(RUNNER.resolve()),
        "phase_b_reporter": file_identity(REPORTER.resolve()),
        "strict_evaluator": file_identity(EVALUATOR.resolve()),
        "replay_wrapper": file_identity(REPLAY.resolve()),
        "replay_launch": file_identity(REPLAY_LAUNCH.resolve()),
        "fastlivo_base_config": file_identity(FASTLIVO_BASE_CONFIG.resolve()),
        "thresholds": file_identity(thresholds_path.resolve()),
        "base_overlay": file_identity(base_overlay_path.resolve()),
        "phase_a_orchestration": file_identity(phase_a_orchestration_path),
        "phase_a_report": file_identity(phase_a_report_path),
        "phase_a_generator": file_identity(phase_a_paths["generator"]),
        "phase_a_runner": file_identity(phase_a_paths["runner"]),
        "phase_a_reporter": file_identity(phase_a_paths["phase_a_reporter"]),
        # These two transitive producers were not in the Phase-A orchestration
        # inventory; Phase-B binds them explicitly to close that audit gap.
        "phase_a_selector": file_identity(PHASE_A_SELECTOR.resolve()),
        "phase_a_summarizer": file_identity(PHASE_A_SUMMARIZER.resolve()),
        "qualification_plan": file_identity(phase_a_paths["qualification_plan"]),
        "qualification_report": file_identity(phase_a_paths["qualification_report"]),
        "qualified_build_manifest": file_identity(
            phase_a_paths["qualified_build_manifest"]),
        "ground_anchors": file_identity(phase_a_paths["ground_anchors"]),
    }

    output_root.mkdir(parents=True, exist_ok=False)
    config_by_id = {str(row["id"]): row for row in CONFIGURATIONS}
    cell_rows: List[Dict[str, Any]] = []
    commands: List[List[str]] = []
    for ordinal, (repeat_id, session_id, config_id) in enumerate(schedule, start=1):
        session = sessions[session_id]
        config = config_by_id[config_id]
        runtime = _runtime_overrides(
            config, str(session["explicit_anchor"]["anchor_stamp_ns"]))
        effective = effective_overlay(
            base_overlay, {"id": config_id, "overrides": runtime})
        validate_effective_overlay(effective)
        run_id = f"gb{ordinal:02d}_{repeat_id}_{session_id}__{config_id}"
        campaign_id = f"ground_b_{ordinal:02d}_{repeat_id}_{session_id}_{config_id}"
        cell_core: Dict[str, Any] = {
            "schema": CELL_SCHEMA,
            "scope": "development_only",
            "validation_data_accessed": False,
            "ordinal": ordinal,
            "run_id": run_id,
            "campaign_id": campaign_id,
            "repeat_id": repeat_id,
            "session": copy.deepcopy(session),
            "configuration": copy.deepcopy(dict(config)),
            "runtime_overrides": runtime,
            "effective_overlay_sha256": object_sha256(effective),
            "phase_a_provenance": {
                "orchestration_identity_sha256": phase_a_identity,
                "report_identity_sha256": report_identity,
                "selected_main_effects": ["outlier600", "acc5"],
            },
            "qualified_build_identity_sha256":
                phase_a["qualified_build"]["identity_sha256"],
            "qualified_executable_sha256":
                phase_a["qualified_build"]["executable_sha256"],
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
                "development_interaction_ranking_only": True,
                "validation_access_allowed": False,
                "candidate_promotion_allowed": False,
                "flight_ready_can_be_declared": False,
                "global_high_rate_interface_remains_no_go": True,
                "global_high_rate_blocker_excluded_from_per_cell_reliability_rank": True,
            },
        }
        cell = {**cell_core, "identity_sha256": object_sha256(cell_core)}
        cell_path = output_root / "cells" / f"{run_id}.json"
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
            "repeat_id": repeat_id,
            "session_id": session_id,
            "configuration_id": config_id,
            "cell": file_identity(cell_path),
            "cell_object_identity_sha256": cell["identity_sha256"],
            "command": command,
        })

    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "qualification_variant": QUALIFICATION_VARIANT,
        "phase_a_gate": {
            "orchestration_identity_sha256": phase_a_identity,
            "report_identity_sha256": report_identity,
            "completed_run_count": 40,
            "fresh_process_instance_count": 40,
            "selected_main_effects": ["outlier600", "acc5"],
        },
        "qualified_build": copy.deepcopy(phase_a["qualified_build"]),
        "runtime_constant_binding": copy.deepcopy(
            phase_a["runtime_constant_binding"]),
        "design": {
            "factor_levels": {
                "imu.acc_cov": [10.0, 5.0],
                "vio.outlier_threshold": [1000.0, 600.0],
                "vio.img_point_cov_fixed": 1000.0,
            },
            "configuration_ids": list(CONFIG_IDS),
            "session_ids": list(SESSION_IDS),
            "repeat_ids": list(REPEAT_IDS),
            "configuration_count": 4,
            "session_count": 5,
            "repeats_per_configuration_session": 3,
            "expected_run_count": 60,
            "rate": 1.0,
            "window": "full_bag_record_start_to_frozen_cached_landing",
            "fresh_campaign_per_cell": True,
            "fresh_process_per_cell": True,
            "strictly_sequential": True,
            "order": "three_round_session_blocks_cyclic_configuration_rotation",
            "four_cell_blocks_are_complete_factorials": True,
            "configuration_position_balance_max_minus_min": 1,
            "within_session_repeat_position_balance_max_minus_min": 1,
        },
        "evaluation_contract": {
            "primary": "complete_ground_to_landing_safety",
            "secondary": "hover_to_landing_low_rate_ranking_only",
            "secondary_alignment": "reuse_primary_without_refit",
            "secondary_can_override_primary_failure": False,
            "same_result_bag_required": True,
        },
        "ranking_contract": {
            "global_high_rate_blocker_is_separate": True,
            "global_high_rate_blocker_enters_configuration_rank": False,
            "lexicographic_order": [
                "incomplete_or_unapproved_cell_failure_count",
                "low_rate_repeat_determinism_failure_count",
                "low_rate_output_reliability_failed_check_count",
                "cells_with_low_rate_output_reliability_failures",
                "worst_repeat_session_normalized_accuracy",
                "mean_repeat_session_normalized_accuracy",
                "frozen_configuration_order",
            ],
            "accuracy_source": "secondary_hover_primary_alignment_reused",
            "accuracy_refit_allowed": False,
            "low_rate_repeat_determinism_is_hard_gate": True,
            "exact_across_three_repeats": [
                "low_rate_pose_q_sign_canonical",
                "low_rate_init_q_sign_canonical",
                "correction_pose_cov_q_sign_canonical",
                "initialization_sample_vector",
                "initial_state",
                "first_correction",
                "secondary_ranking_metrics_binary64",
            ],
            "high_rate_payload_role": "diagnostic_only_no_go",
            "approved_startup_retry_enters_tuning_rank": False,
        },
        "infrastructure_retry_contract": {
            "maximum_retries_per_cell": 1,
            "eligible_error_text_exact":
                "explicit anchor was not observed as a full-sync sensor epoch",
            "zero_estimator_output_required": True,
            "retry_requires_fresh_process": True,
            "failed_attempt_retained_append_only": True,
            "failed_attempt_is_global_operational_warning": True,
            "approved_retry_enters_configuration_tuning_rank": False,
            "failed_attempt_excluded_from_accuracy": True,
            "unapproved_failure_stops_sequence": True,
            "second_occurrence_stops_sequence": True,
        },
        "decision_contract": {
            "development_interaction_ranking_only": True,
            "old_phase_a_scores_may_be_pooled": False,
            "reuse_phase_a_completion_pointers": False,
            "candidate_promotion_allowed": False,
            "flight_ready_can_be_declared": False,
            "global_high_rate_interface_remains_no_go": True,
        },
        "phase_a_failed_attempt_operational_provenance": failure_provenance,
        "output_root": str(output_root),
        "ros_master_port": int(port),
        "dependencies": dependencies,
        "cells": cell_rows,
        "commands": commands,
        "execution": {
            "generator_executed_replay": False,
            "generator_executed_build": False,
            "run_exact_commands_in_list_order": True,
            "sequence_stops_on_unapproved_or_second_failure": True,
            "eligible_startup_failure_is_retried_once_inside_cell": True,
            "preflight_command": [
                python_executable, str(RUNNER.resolve()), "preflight",
                str((output_root / "orchestration.json").resolve()),
                "--commands", str((output_root / "commands.json").resolve()),
            ],
            "sequence_command": [
                python_executable, str(RUNNER.resolve()), "sequence",
                str((output_root / "orchestration.json").resolve()),
                "--commands", str((output_root / "commands.json").resolve()),
            ],
            "report_command": [
                python_executable, str(REPORTER.resolve()),
                str((output_root / "orchestration.json").resolve()),
                "--output", str((output_root / "phase_b_report.json").resolve()),
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
        "stop_on_unapproved_or_second_failure": True,
        "eligible_startup_failure_retry_limit": 1,
        "fresh_campaign_per_cell": True,
        "command_count": 60,
        "commands": commands,
    }
    commands_document = {
        **commands_core, "identity_sha256": object_sha256(commands_core)}
    _write_json_exclusive(output_root / "commands.json", commands_document)
    return orchestration


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase_a_orchestration", type=Path)
    parser.add_argument("phase_a_report", type=Path)
    parser.add_argument("--base-overlay", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--failed-attempt", type=Path,
                        default=DEFAULT_FAILED_ATTEMPT)
    parser.add_argument("--port", type=int, default=11432)
    arguments = parser.parse_args(argv)
    try:
        orchestration = prepare_orchestration(
            arguments.phase_a_orchestration,
            arguments.phase_a_report,
            arguments.base_overlay,
            arguments.thresholds,
            arguments.output_root,
            arguments.failed_attempt,
            port=arguments.port,
        )
        print(json.dumps({
            "orchestration": str(
                (arguments.output_root / "orchestration.json").resolve()),
            "commands": str((arguments.output_root / "commands.json").resolve()),
            "identity_sha256": orchestration["identity_sha256"],
            "expected_run_count": 60,
            "validation_data_accessed": False,
            "candidate_promotion_allowed": False,
            "flight_ready": False,
            "global_high_rate_interface_remains_no_go": True,
            "replay_executed": False,
            "build_executed": False,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

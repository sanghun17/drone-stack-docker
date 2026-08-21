#!/usr/bin/env python3
"""Validate, determinism-gate, and rank completed ground Phase-B development runs."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from build_vio_postfix_init_qualification_receipt import (
    _diagnostic_evidence,
    canonicalize_result_bag,
)
from generate_vio_ground_phase_b_interaction import (
    CONFIG_IDS,
    REPEAT_IDS,
    SESSION_IDS,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_json,
    object_sha256,
)
from run_vio_ground_phase_b_interaction import (
    _campaign_plan,
    _eligible_startup_retry,
    _inventory_attempt_directories,
    _load_cell,
    _load_orchestration,
    _validate_completion,
    _validate_dependencies,
    _validate_gate_and_build,
    _validate_primary_report,
    _validate_secondary_report,
)
from select_vio_flight_tuning_phase_a import normalized_session_score


SCHEMA = "fastlivo_vio_ground_phase_b_interaction_report/v1"
LOW_RATE_STREAMS = ("low_rate_pose", "low_rate_init", "correction")
HIGH_RATE_DIAGNOSTIC_STREAMS = ("propagated_odom", "world_twist")
ACCURACY_METRICS = (
    "translation_ape_rmse_m",
    "translation_rpe_1p0s_rmse_m",
    "orientation_rmse_deg",
    "path_ratio",
)
LOW_RATE_RELIABILITY_CHECKS = frozenset({
    "local_finite", "local_monotonic_header", "local_unique_header",
    "local_stable_frame", "local_expected_frame",
    "local_quaternion_normalized", "local_max_gap", "local_pose_coverage",
    "local_fixed_association_fraction", "gt_was_not_visible_to_estimator",
    "correction_covariance_evaluable", "correction_covariance_finite",
    "correction_pose_finite", "correction_pose_quaternion_normalized",
    "correction_covariance_nonzero", "correction_covariance_psd",
    "correction_covariance_symmetric", "correction_covariance_coverage",
    "correction_covariance_frame", "correction_covariance_expected_frame",
    "correction_covariance_monotonic_header",
    "correction_covariance_unique_header",
})
RECORD_TIME_AGE_DIAGNOSTIC_CHECKS = (
    "local_nonnegative_sensor_age",
    "correction_covariance_age",
    "correction_covariance_nonnegative_age",
)


def _write_json_exclusive(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True,
                          ensure_ascii=False) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _finite_metrics(local: Mapping[str, Any]) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for name in ACCURACY_METRICS:
        value = local.get(name)
        if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(float(value))):
            raise CampaignError(f"missing/non-finite Phase-B metric {name}")
        result[name] = float(value)
    return result


def _all_local_metrics_signature(local: Mapping[str, Any]) -> str:
    # Paths embedded in alignment provenance are intentionally excluded; all
    # actual numeric/evaluable/count fields must remain byte-for-byte equal as
    # JSON binary64 values across the three fresh repeats.
    metrics = {key: value for key, value in local.items() if key != "alignment"}
    return object_sha256(metrics)


def _initialization_signature(node_log: Path, result_bag: Path) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    events = _diagnostic_evidence(node_log, result_bag)
    accepted = [row for row in events.get("accepted", [])
                if row.get("initialization_gate_ready") is True]
    corrections = events.get("first_correction_received", [])
    if len(accepted) != 1 or len(corrections) != 1:
        raise CampaignError("Phase-B needs one accepted init and first correction")
    row = accepted[0]
    correction = corrections[0]
    init = {
        "anchor_mode": row.get("anchor_mode"),
        "anchor_stamp_ns": row.get("anchor_stamp_ns"),
        "state_epoch_ns": row.get("state_epoch_ns"),
        "selected_stamp_seq": row.get("selected_stamp_seq"),
        "selected_stamp_seq_sha256": row.get("selected_stamp_seq_sha256"),
        "valid_count": row.get("valid_count"),
        "invalid_count": row.get("invalid_count"),
        "rejected_window_count": row.get("rejected_window_count"),
        "queue_drop_count": row.get("queue_drop_count"),
        "mean_acc": row.get("mean_acc"),
        "mean_gyr": row.get("mean_gyr"),
        "initial_state_fingerprint_schema":
            row.get("initial_state_fingerprint_schema"),
        "initial_state_binary64_be_sha256":
            row.get("initial_state_binary64_be_sha256"),
    }
    first = {
        "correction_epoch_ns": correction.get("correction_epoch_ns"),
        "state_fingerprint_schema": correction.get("state_fingerprint_schema"),
        "initial_state_binary64_be_sha256":
            correction.get("initial_state_binary64_be_sha256"),
        "state_binary64_be_sha256": correction.get("state_binary64_be_sha256"),
        "qualification_gate_ready": correction.get("qualification_gate_ready"),
    }
    if (init["anchor_mode"] != "explicit" or init["valid_count"] != 30 or
            init["invalid_count"] != 0 or init["rejected_window_count"] != 0 or
            init["queue_drop_count"] != 0 or
            first["qualification_gate_ready"] is not True or
            first["initial_state_binary64_be_sha256"] !=
            init["initial_state_binary64_be_sha256"]):
        raise CampaignError("Phase-B deterministic initialization gate changed")
    return init, first


def _low_rate_reliability(primary: Mapping[str, Any]) -> Dict[str, Any]:
    rows = primary.get("checks")
    if not isinstance(rows, list):
        raise CampaignError("primary report lacks checks")
    indexed = {str(row.get("id")): row for row in rows
               if isinstance(row, Mapping)}
    missing = sorted(LOW_RATE_RELIABILITY_CHECKS - set(indexed))
    if missing:
        raise CampaignError(f"primary report lacks reliability checks: {missing}")
    failures = sorted(
        check_id for check_id in LOW_RATE_RELIABILITY_CHECKS
        if indexed[check_id].get("status") != "pass")
    age_diagnostics = {
        check_id: indexed.get(check_id, {}).get("status", "missing")
        for check_id in RECORD_TIME_AGE_DIAGNOSTIC_CHECKS
    }
    return {
        "checked_ids": sorted(LOW_RATE_RELIABILITY_CHECKS),
        "failed_ids": failures,
        "failed_check_count": len(failures),
        "passed": not failures,
        "propagated_high_rate_checks_excluded": True,
        "objective_accuracy_checks_excluded": True,
        "global_high_rate_blocker_excluded": True,
        "rosbag_callback_record_time_age_checks_excluded_from_rank":
            list(RECORD_TIME_AGE_DIAGNOSTIC_CHECKS),
        "rosbag_callback_record_time_age_diagnostic_status": age_diagnostics,
    }


def _repeat_determinism(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    if len(rows) != 3 or {row.get("repeat_id") for row in rows} != set(REPEAT_IDS):
        raise CampaignError("determinism group lacks exact three repeats")
    fields = {
        "low_rate_pose": {row["stream_signatures"]["low_rate_pose"] for row in rows},
        "low_rate_init": {row["stream_signatures"]["low_rate_init"] for row in rows},
        "correction": {row["stream_signatures"]["correction"] for row in rows},
        "initialization": {row["initialization_signature_sha256"] for row in rows},
        "first_correction": {row["first_correction_signature_sha256"] for row in rows},
        "secondary_metrics": {row["all_local_metrics_signature_sha256"] for row in rows},
    }
    exact = {name: len(signatures) == 1
             for name, signatures in fields.items()}
    high_rate = {
        name: len({row["stream_signatures"][name] for row in rows})
        for name in HIGH_RATE_DIAGNOSTIC_STREAMS
    }
    return {
        "exact": exact,
        "all_required_exact": all(exact.values()),
        "signature_counts": {name: len(values) for name, values in fields.items()},
        "quaternion_sign_canonicalized": True,
        "high_rate_diagnostic_signature_counts": high_rate,
        "high_rate_payload_is_not_determinism_gate": True,
        "global_high_rate_interface_remains_no_go": True,
    }


def rank_configurations(
        rows: Sequence[Mapping[str, Any]],
        determinism_groups: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """Apply the frozen reliability-first, then minimax/mean ranking."""
    failures = [row for row in determinism_groups
                if row.get("all_required_exact") is not True]
    if failures:
        return {
            "selection_complete": False,
            "selection_failure": "low_rate_repeat_determinism_hard_gate",
            "failed_group_count": len(failures),
            "ranking": [],
        }
    ranking = []
    for plan_order, config_id in enumerate(CONFIG_IDS):
        selected = [row for row in rows if row["configuration_id"] == config_id]
        if len(selected) != 15:
            return {
                "selection_complete": False,
                "selection_failure": "incomplete_or_unapproved_cell_failure",
                "failed_group_count": 0,
                "ranking": [],
            }
        reliability_count = sum(
            int(row["low_rate_output_reliability"]["failed_check_count"])
            for row in selected)
        reliability_cells = sum(
            not row["low_rate_output_reliability"]["passed"] for row in selected)
        scores = [float(row["normalized_accuracy_max"]) for row in selected]
        key = [0, 0, reliability_count, reliability_cells, max(scores),
               statistics.fmean(scores), plan_order]
        ranking.append({
            "configuration_id": config_id,
            "completed_cell_count": 15,
            "incomplete_or_unapproved_cell_failure_count": 0,
            "low_rate_repeat_determinism_failure_count": 0,
            "low_rate_output_reliability_failed_check_count": reliability_count,
            "cells_with_low_rate_output_reliability_failures": reliability_cells,
            "worst_repeat_session_normalized_accuracy": max(scores),
            "mean_repeat_session_normalized_accuracy": statistics.fmean(scores),
            "frozen_configuration_order": plan_order,
            "lexicographic_key": key,
            "candidate_promotion_allowed": False,
            "flight_ready": False,
        })
    ranking.sort(key=lambda row: tuple(row["lexicographic_key"]))
    for rank, row in enumerate(ranking, 1):
        row["rank"] = rank
    return {
        "selection_complete": True,
        "selection_failure": None,
        "ranking": ranking,
        "selection_rule": [
            "incomplete_or_unapproved_cell_failure_count",
            "low_rate_repeat_determinism_failure_count",
            "low_rate_output_reliability_failed_check_count",
            "cells_with_low_rate_output_reliability_failures",
            "worst_repeat_session_normalized_accuracy",
            "mean_repeat_session_normalized_accuracy",
            "frozen_configuration_order",
        ],
        "best_development_configuration": ranking[0]["configuration_id"],
        "candidate_promotion_allowed": False,
        "flight_ready": False,
    }


def factorial_decomposition(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    """Return diagnostic 2 x 2 main effects and interaction per stratum."""
    config_keys = {
        "acc10_out1000": "acc10_img1000_out1000",
        "acc5_out1000": "acc5_img1000_out1000",
        "acc10_out600": "acc10_img1000_out600",
        "acc5_out600": "acc5_img1000_out600",
    }
    outcomes = ACCURACY_METRICS + ("normalized_accuracy_max",)
    strata = []
    effect_values: Dict[str, Dict[str, List[Tuple[float, str]]]] = {
        outcome: {name: [] for name in (
            "acc_cov_main_effect_5_minus_10",
            "outlier_main_effect_600_minus_1000",
            "interaction_delta_delta",
        )} for outcome in outcomes
    }
    for repeat_id in REPEAT_IDS:
        for session_id in SESSION_IDS:
            selected = [row for row in rows
                        if row.get("repeat_id") == repeat_id and
                        row.get("session_id") == session_id]
            by_config = {str(row.get("configuration_id")): row
                         for row in selected}
            if len(selected) != 4 or set(by_config) != set(CONFIG_IDS):
                raise CampaignError(
                    f"factorial stratum is incomplete: {repeat_id}/{session_id}")
            raw: Dict[str, Dict[str, float]] = {}
            for label, config_id in config_keys.items():
                row = by_config[config_id]
                metrics = row.get("accuracy_metrics")
                if not isinstance(metrics, Mapping):
                    raise CampaignError("factorial row lacks accuracy metrics")
                raw[label] = {
                    **{name: float(metrics[name]) for name in ACCURACY_METRICS},
                    "normalized_accuracy_max":
                        float(row["normalized_accuracy_max"]),
                }
            effects: Dict[str, Dict[str, float]] = {}
            stratum_id = f"{repeat_id}__{session_id}"
            for outcome in outcomes:
                y00 = raw["acc10_out1000"][outcome]
                y10 = raw["acc5_out1000"][outcome]
                y01 = raw["acc10_out600"][outcome]
                y11 = raw["acc5_out600"][outcome]
                values = {
                    "acc_cov_main_effect_5_minus_10":
                        ((y10 + y11) - (y00 + y01)) / 2.0,
                    "outlier_main_effect_600_minus_1000":
                        ((y01 + y11) - (y00 + y10)) / 2.0,
                    "interaction_delta_delta": y11 - y10 - y01 + y00,
                }
                effects[outcome] = values
                for name, value in values.items():
                    effect_values[outcome][name].append((value, stratum_id))
            strata.append({
                "stratum_id": stratum_id,
                "repeat_id": repeat_id,
                "session_id": session_id,
                "raw_outcomes": raw,
                "effects": effects,
            })

    summary: Dict[str, Dict[str, Any]] = {}
    for outcome, by_effect in effect_values.items():
        summary[outcome] = {}
        for name, pairs in by_effect.items():
            values = [value for value, _ in pairs]
            worst_value, worst_stratum = max(
                pairs, key=lambda pair: (abs(pair[0]), pair[1]))
            positive = sum(value > 0.0 for value in values)
            negative = sum(value < 0.0 for value in values)
            zero = len(values) - positive - negative
            nonzero = positive + negative
            summary[outcome][name] = {
                "stratum_count": len(values),
                "mean": statistics.fmean(values),
                "median": statistics.median(values),
                "worst_absolute_value": abs(worst_value),
                "signed_value_at_worst_absolute": worst_value,
                "worst_absolute_stratum_id": worst_stratum,
                "positive_count": positive,
                "negative_count": negative,
                "zero_count": zero,
                "dominant_nonzero_sign_consistency_fraction":
                    (max(positive, negative) / nonzero
                     if nonzero else 1.0),
            }
    return {
        "role": "diagnostic_only_does_not_change_frozen_selection_rule",
        "factor_coding": {
            "acc_cov_main_effect":
                "mean(outcomes at acc5)-mean(outcomes at acc10)",
            "outlier_main_effect":
                "mean(outcomes at out600)-mean(outcomes at out1000)",
            "interaction_delta_delta":
                "y(acc5,out600)-y(acc5,out1000)-y(acc10,out600)+y(acc10,out1000)",
        },
        "outcomes": list(outcomes),
        "stratum_count": len(strata),
        "strata": strata,
        "effect_summary": summary,
        "candidate_promotion_allowed": False,
        "flight_ready": False,
    }


def build_report(
        orchestration_path: Path, *,
        verify_actual_build: bool = True) -> Dict[str, Any]:
    orchestration_path = orchestration_path.resolve()
    orchestration, orchestration_identity = _load_orchestration(orchestration_path)
    paths = _validate_dependencies(orchestration)
    if paths["phase_b_reporter"] != Path(__file__).resolve():
        raise CampaignError("Phase-B reporter differs from bound dependency")
    _validate_gate_and_build(
        orchestration, paths, verify_actual_build=verify_actual_build)
    output_root = Path(orchestration["output_root"]).resolve()
    run_rows: List[Dict[str, Any]] = []
    completions: List[Dict[str, Any]] = []
    successful_uuids = set()
    failed_uuids = set()
    approved_retries = []
    for expected_ordinal, row in enumerate(orchestration["cells"], 1):
        cell, cell_path = _load_cell(orchestration, row["run_id"])
        campaign = _campaign_plan(
            orchestration, orchestration_identity, cell, cell_path)
        campaign_dir = output_root / "campaigns" / cell["campaign_id"]
        campaign_path = campaign_dir / "campaign.json"
        completion_path = campaign_dir / "completion.json"
        if (not campaign_path.is_file() or load_json(campaign_path) != campaign or
                not completion_path.is_file()):
            raise CampaignError(
                f"Phase-B incomplete/unapproved failure at cell {expected_ordinal}")
        attempt = _validate_completion(
            campaign_dir, completion_path, campaign["identity_sha256"],
            row["run_id"])
        retained_failed_attempts = _inventory_attempt_directories(
            campaign_dir, attempt)
        manifest = load_json(attempt / "manifest.json")
        process_uuid = manifest.get("fresh_process_instance_uuid")
        if (not isinstance(process_uuid, str) or not process_uuid or
                process_uuid in successful_uuids or process_uuid in failed_uuids or
                manifest.get("orchestration_identity_sha256") !=
                orchestration_identity or
                manifest.get("cell_identity_sha256") != cell["identity_sha256"] or
                manifest.get("qualified_build_identity_sha256") !=
                orchestration["qualified_build"]["identity_sha256"]):
            raise CampaignError("Phase-B successful process binding changed")
        successful_uuids.add(process_uuid)
        failure_paths = [path / "failure.json"
                         for path in retained_failed_attempts]
        for failure_path in failure_paths:
            failure = load_json(failure_path)
            failure_uuid = failure.get("fresh_process_instance_uuid")
            eligible, evidence = _eligible_startup_retry(failure_path.parent)
            if (not eligible or failure.get("infrastructure_retry_approved") is not True or
                    failure.get("excluded_from_accuracy") is not True or
                    not isinstance(failure_uuid, str) or not failure_uuid or
                    failure_uuid in successful_uuids or failure_uuid in failed_uuids):
                raise CampaignError("Phase-B retained failure is not approved startup retry")
            failed_uuids.add(failure_uuid)
            approved_retries.append({
                "run_id": row["run_id"],
                "configuration_id": row["configuration_id"],
                "session_id": row["session_id"],
                "repeat_id": row["repeat_id"],
                "attempt": str(failure_path.parent),
                "fresh_process_instance_uuid": failure_uuid,
                "evidence": evidence,
                "excluded_from_accuracy": True,
                "enters_configuration_tuning_rank": False,
            })
        if int(manifest.get("process_attempt_ordinal", -1)) != len(failure_paths) + 1:
            raise CampaignError("Phase-B process attempt count differs")
        result_bag = attempt / "result.bag"
        primary_path = attempt / "result.full.flight_readiness.json"
        secondary_path = attempt / "result.hover.ranking.json"
        primary = _validate_primary_report(
            primary_path, result_bag, paths["thresholds"])
        contract = cell["session"]["secondary_evaluation"]
        secondary = _validate_secondary_report(
            secondary_path, result_bag, paths["thresholds"], primary_path,
            primary, str(contract["score_start_ns"]),
            str(contract["score_end_ns"]))
        local = secondary.get("local_accuracy")
        if not isinstance(local, Mapping):
            raise CampaignError("Phase-B secondary lacks local accuracy")
        metrics = _finite_metrics(local)
        score_input = {**metrics, "accuracy_rankable": True}
        normalized = normalized_session_score(score_input)["normalized_max"]
        streams = canonicalize_result_bag(result_bag)
        init, first = _initialization_signature(
            attempt / "result_node.log", result_bag)
        stream_signatures = {
            name: object_sha256(streams[name])
            for name in LOW_RATE_STREAMS + HIGH_RATE_DIAGNOSTIC_STREAMS
        }
        run_rows.append({
            "ordinal": expected_ordinal,
            "run_id": row["run_id"],
            "configuration_id": row["configuration_id"],
            "session_id": row["session_id"],
            "repeat_id": row["repeat_id"],
            "attempt": str(attempt),
            "primary_status": primary["status"],
            "secondary_status": secondary["status"],
            "accuracy_metrics": metrics,
            "all_local_metrics_signature_sha256":
                _all_local_metrics_signature(local),
            "normalized_accuracy_max": normalized,
            "low_rate_output_reliability": _low_rate_reliability(primary),
            "stream_signatures": stream_signatures,
            "stream_evidence": streams,
            "initialization_signature_sha256": object_sha256(init),
            "initialization_evidence": init,
            "first_correction_signature_sha256": object_sha256(first),
            "first_correction_evidence": first,
            "successful_fresh_process_instance_uuid": process_uuid,
            "startup_retry_count": len(failure_paths),
            "candidate_promotion_allowed": False,
            "flight_ready": False,
            "global_high_rate_interface_remains_no_go": True,
        })
        completions.append({
            "ordinal": expected_ordinal,
            "run_id": row["run_id"],
            "completion": str(completion_path),
            "manifest_identity_sha256": manifest["identity_sha256"],
            "successful_fresh_process_instance_uuid": process_uuid,
            "process_attempt_count": len(failure_paths) + 1,
            "approved_startup_retry_count": len(failure_paths),
        })
    if len(run_rows) != 60 or len(successful_uuids) != 60:
        raise CampaignError("Phase-B lacks sixty successful fresh cells")

    determinism_groups = []
    for config_id in CONFIG_IDS:
        for session_id in SESSION_IDS:
            rows = [row for row in run_rows
                    if row["configuration_id"] == config_id and
                    row["session_id"] == session_id]
            gate = _repeat_determinism(rows)
            determinism_groups.append({
                "configuration_id": config_id,
                "session_id": session_id,
                **gate,
            })
    selection = rank_configurations(run_rows, determinism_groups)
    decomposition = factorial_decomposition(run_rows)
    determinism_pass = all(
        row["all_required_exact"] for row in determinism_groups)
    status = "pass" if determinism_pass else "fail"
    failures = [] if determinism_pass else [
        {"gate": "exact_low_rate_repeat_determinism",
         "failed_groups": [
             {"configuration_id": row["configuration_id"],
              "session_id": row["session_id"], "exact": row["exact"]}
             for row in determinism_groups
             if not row["all_required_exact"]
         ]}
    ]
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "status": status,
        "failures": failures,
        "scope": "development_only",
        "validation_data_accessed": False,
        "orchestration": str(orchestration_path),
        "orchestration_identity_sha256": orchestration_identity,
        "phase_a_orchestration_identity_sha256":
            orchestration["phase_a_gate"]["orchestration_identity_sha256"],
        "phase_a_report_identity_sha256":
            orchestration["phase_a_gate"]["report_identity_sha256"],
        "qualified_build_identity_sha256":
            orchestration["qualified_build"]["identity_sha256"],
        "expected_successful_cell_count": 60,
        "successful_cell_count": len(run_rows),
        "successful_fresh_process_count": len(successful_uuids),
        "approved_startup_retry_count": len(approved_retries),
        "failed_fresh_process_count": len(failed_uuids),
        "actual_fresh_process_attempt_count":
            len(successful_uuids) + len(failed_uuids),
        "approved_startup_retries": approved_retries,
        "startup_retry_operational_warning": bool(approved_retries),
        "startup_retry_enters_configuration_tuning_rank": False,
        "completions": completions,
        "runs": run_rows,
        "low_rate_repeat_determinism": {
            "hard_gate_passed": determinism_pass,
            "group_count": len(determinism_groups),
            "groups": determinism_groups,
            "quaternion_sign_canonicalized": True,
            "metrics_exact_required": True,
        },
        "selection": selection,
        "factorial_decomposition": decomposition,
        "development_ranking_only": True,
        "candidate_promotion_allowed": False,
        "flight_ready": False,
        "global_high_rate_interface_remains_no_go": True,
        "high_rate_payload_role": "diagnostic_only_no_go",
        "phase_a_failed_attempt_operational_provenance":
            orchestration["phase_a_failed_attempt_operational_provenance"],
    }
    return {**core, "identity_sha256": object_sha256(core)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("orchestration", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        report = build_report(arguments.orchestration)
        _write_json_exclusive(arguments.output.resolve(), report)
        print(json.dumps({
            "report": str(arguments.output.resolve()),
            "identity_sha256": report["identity_sha256"],
            "status": report["status"],
            "successful_cell_count": report["successful_cell_count"],
            "actual_fresh_process_attempt_count":
                report["actual_fresh_process_attempt_count"],
            "approved_startup_retry_count":
                report["approved_startup_retry_count"],
            "candidate_promotion_allowed": False,
            "flight_ready": False,
            "global_high_rate_interface_remains_no_go": True,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

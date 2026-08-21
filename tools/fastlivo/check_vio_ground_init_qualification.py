#!/usr/bin/env python3
"""Check the 12 append-only ground-init qualification receipts.

Only initialization, low-rate pose/init, and correction determinism can grant
the development Phase-A rebaseline gate.  High-rate inventory/payload and the
full-result flight-readiness reports remain visible, authoritative NO-GO
evidence and can never be cleared by this checker.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence

from build_vio_ground_init_qualification_receipt import (
    EXPECTED_RUNTIME_PARAMETERS,
    SCHEMA as RECEIPT_SCHEMA,
)
from build_vio_postfix_init_qualification_receipt import (
    _self_hash,
    _verify_file_identity,
)
from check_vio_postfix_init_qualification import (
    PLAN_SCHEMA,
    RECEIPT_SCHEMA as BASE_RECEIPT_SCHEMA,
    _receipt_path,
    _validate_accuracy,
    _validate_build,
    _validate_init,
    _validate_streams,
)
from extract_vio_ground_init_anchors import SCHEMA as ANCHOR_SCHEMA
from prepare_vio_ground_init_qualification import VARIANT
from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_json,
    object_sha256,
    sha256,
)


REPORT_SCHEMA = "fastlivo_vio_ground_init_qualification_report/v1"
LOW_RATE_EXACT_STREAMS = ("low_rate_pose", "low_rate_init", "correction")
HIGH_RATE_DIAGNOSTIC_STREAMS = ("propagated_odom", "world_twist")
STD_SOURCE = (
    "frozen_preregistered_recomputation_from_exact_30_sample_"
    "measurement_vector_not_runtime_log")
EXPECTED_STATIONARITY_GATE = {
    "accumulator": "sequential_welford_binary64_in_stamp_seq_order",
    "comparison": "inclusive",
    "gravity_m_s2": 9.81,
    "max_abs_mean_acc_norm_error_m_s2": 0.1,
    "max_acc_sample_std_vector_norm_m_s2": 0.25,
    "max_gyr_sample_std_vector_norm_rad_s": 0.04,
    "max_mean_gyr_vector_norm_rad_s": 0.01,
    "measurement_vector_hash_encoding": (
        "concatenated_big_endian_uint64_stamp_ns_uint32_seq_6x_"
        "ieee754_binary64_accxyz_gyrxyz"),
    "variance_denominator": "N_minus_1",
    "vector_norm": "euclidean_l2",
}
EXPECTED_HARD_GATE_CONTRACT = {
    "affects_anchor_selection": False,
    "minimum_30th_sample_to_takeoff_lead_s": 0.5,
    "minimum_anchor_to_hover_score_start_lead_s": 3.0,
    "source": "frozen_development_window_cache",
}
EXPECTED_DECISION_CONTRACT = {
    "high_rate_flight_interface_can_be_cleared_by_this_plan": False,
    "low_rate_determinism_can_be_qualified_separately": True,
    "primary_full_result_safety_report_required": True,
    "secondary_cannot_override_primary_failure": True,
    "secondary_hover_report_required": True,
}


def _load_ground_plan(path: Path) -> tuple[Dict[str, Any], str]:
    plan = load_json(path)
    identity = _self_hash(plan, PLAN_SCHEMA, "ground qualification plan")
    if (plan.get("qualification_variant") != VARIANT or
            plan.get("scope") != "development_only" or
            plan.get("validation_data_accessed") is not False or
            plan.get("execution_neutral") is not True or
            plan.get("expected_run_count") != 12 or
            plan.get("rates") != [0.5, 1.0] or
            plan.get("fresh_process_repeats_per_rate") != 3 or
            plan.get("session_window_mode") != "ground_to_landing" or
            plan.get("stationarity_gate") != EXPECTED_STATIONARITY_GATE or
            plan.get("post_selection_ground_hard_gate_contract") !=
            EXPECTED_HARD_GATE_CONTRACT or
            plan.get("decision_contract") != EXPECTED_DECISION_CONTRACT or
            plan.get("predecessor_qualification_remains_fail_no_go") is not True or
            plan.get("high_rate_interface_remains_no_go") is not True):
        raise CampaignError("ground qualification plan contract changed")
    sentinels = plan.get("sentinels")
    runs = plan.get("runs")
    if (not isinstance(sentinels, list) or len(sentinels) != 2 or
            not isinstance(runs, list) or len(runs) != 12 or
            any(row.get("session_split") != "development"
                for row in sentinels)):
        raise CampaignError("ground qualification scope/grid changed")
    expected_grid = {
        (row["id"], rate, repeat) for row in sentinels
        for rate in (0.5, 1.0) for repeat in (1, 2, 3)
    }
    actual_grid = {(row.get("sentinel_id"), row.get("rate"), row.get("repeat"))
                   for row in runs if isinstance(row, Mapping)}
    if (actual_grid != expected_grid or
            len({row.get("run_id") for row in runs}) != 12 or
            any(row.get("fresh_process_required") is not True for row in runs)):
        raise CampaignError("ground qualification is not the exact 12-cell grid")
    _verify_file_identity(plan.get("config_identity"), "ground config")
    _verify_file_identity(plan.get("anchors_identity"), "ground anchors")
    anchors = load_json(Path(str(plan["anchors_identity"]["path"])))
    if (_self_hash(anchors, ANCHOR_SCHEMA, "ground anchors") !=
            plan.get("anchors_artifact_identity_sha256")):
        raise CampaignError("ground anchor self-hash changed")
    predecessor = plan.get("predecessor_qualification", {})
    predecessor_path = Path(str(predecessor.get("path", ""))).resolve()
    predecessor_report = load_json(predecessor_path)
    if (sha256(predecessor_path) != predecessor.get("sha256") or
            predecessor_report.get("identity_sha256") !=
            predecessor.get("identity_sha256") or
            predecessor_report.get("status") != "fail" or
            predecessor.get("status") != "fail" or
            predecessor.get("superseded") is not False or
            predecessor.get("remains_authoritative_for_high_rate_interface")
            is not True):
        raise CampaignError("predecessor qualification FAIL binding changed")
    return plan, identity


def _expected_stationarity(sentinel: Mapping[str, Any]) -> Dict[str, Any]:
    stationarity = sentinel["stationarity"]
    return {
        "measurement_vector_sha256": stationarity["measurement_vector_sha256"],
        "measurement_vector_hash_encoding": stationarity[
            "measurement_vector_hash_encoding"],
        "mean_acc": stationarity["mean_acc"],
        "mean_acc_norm_m_s2": stationarity["mean_acc_norm_m_s2"],
        "acc_sample_std": stationarity["acc_sample_std"],
        "acc_sample_std_vector_norm_m_s2": stationarity[
            "acc_sample_std_vector_norm_m_s2"],
        "mean_gyr": stationarity["mean_gyr"],
        "mean_gyr_vector_norm_rad_s": stationarity[
            "mean_gyr_vector_norm_rad_s"],
        "gyr_sample_std": stationarity["gyr_sample_std"],
        "gyr_sample_std_vector_norm_rad_s": stationarity[
            "gyr_sample_std_vector_norm_rad_s"],
        "variance_denominator": stationarity["variance_denominator"],
        "checks": stationarity["checks"],
        "accepted": True,
        "runtime_means_binary64_exact": True,
        "sample_std_source": STD_SOURCE,
    }


def _file_is_current(identity: Any, label: str) -> None:
    _verify_file_identity(identity, label)


def _norm3(values: Any, label: str) -> float:
    if (not isinstance(values, list) or len(values) != 3 or
            any(not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(float(value)) for value in values)):
        raise CampaignError(f"{label}: expected three finite values")
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _validate_ground_receipt(
        receipt: Mapping[str, Any], run: Mapping[str, Any],
        sentinel: Mapping[str, Any], plan_identity: str,
        label: str) -> Dict[str, Any]:
    _self_hash(receipt, RECEIPT_SCHEMA, label)
    exact = {
        "plan_identity_sha256": plan_identity,
        "run_id": run["run_id"],
        "sentinel_id": run["sentinel_id"],
        "arm_id": run["arm_id"],
        "session_id": run["session_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "fresh_process": True,
        "high_rate_interface_qualification": "NO_GO",
        "not_a_flight_readiness_decision": True,
    }
    for field, expected in exact.items():
        if receipt.get(field) != expected:
            raise CampaignError(f"{label}: exact field differs: {field}")
    expected_runtime = {
        **EXPECTED_RUNTIME_PARAMETERS,
        "imu/init_anchor_stamp_ns": sentinel["explicit_anchor"][
            "anchor_stamp_ns"],
    }
    if receipt.get("runtime_parameters") != expected_runtime:
        raise CampaignError(f"{label}: tight runtime snapshot differs")
    if receipt.get("stationarity_evidence") != _expected_stationarity(sentinel):
        raise CampaignError(f"{label}: frozen stationarity evidence differs")
    stationarity = sentinel["stationarity"]
    acc_mean_norm = _norm3(stationarity.get("mean_acc"), f"{label} mean_acc")
    acc_std_norm = _norm3(
        stationarity.get("acc_sample_std"), f"{label} acc sample std")
    gyr_mean_norm = _norm3(
        stationarity.get("mean_gyr"), f"{label} mean_gyr")
    gyr_std_norm = _norm3(
        stationarity.get("gyr_sample_std"), f"{label} gyr sample std")
    declared_norms = (
        (acc_mean_norm, stationarity.get("mean_acc_norm_m_s2")),
        (acc_std_norm, stationarity.get(
            "acc_sample_std_vector_norm_m_s2")),
        (gyr_mean_norm, stationarity.get(
            "mean_gyr_vector_norm_rad_s")),
        (gyr_std_norm, stationarity.get(
            "gyr_sample_std_vector_norm_rad_s")),
    )
    if (stationarity.get("sample_count") != 30 or
            stationarity.get("variance_denominator") != "N_minus_1" or
            stationarity.get("accumulator") !=
            "sequential_welford_binary64_in_stamp_seq_order" or
            any(not isinstance(declared, (int, float)) or
                isinstance(declared, bool) or
                not math.isclose(calculated, float(declared),
                                 rel_tol=0.0, abs_tol=1e-12)
                for calculated, declared in declared_norms) or
            abs(acc_mean_norm - 9.81) > 0.10 or
            acc_std_norm > 0.25 or gyr_mean_norm > 0.01 or
            gyr_std_norm > 0.04 or stationarity.get("accepted") is not True or
            not all(stationarity.get("checks", {}).values())):
        raise CampaignError(f"{label}: numerical stationarity gate failed")
    hard_gate = sentinel["post_selection_ground_hard_gate"]
    try:
        takeoff_lead_ns = int(hard_gate["30th_sample_to_takeoff_lead_ns"])
        hover_lead_ns = int(hard_gate[
            "anchor_to_hover_score_start_lead_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise CampaignError(f"{label}: malformed hard lead gate") from error
    if (receipt.get("post_selection_ground_hard_gate") != hard_gate or
            hard_gate.get("passed") is not True or
            not all(hard_gate.get("checks", {}).values()) or
            takeoff_lead_ns < 500_000_000 or
            hover_lead_ns < 3_000_000_000):
        raise CampaignError(f"{label}: hard pre-takeoff lead gate differs")

    constant = receipt.get("ground_runtime_constant_binding", {})
    frozen_constant = sentinel.get("runtime_constant_binding")
    # The constant is plan-global; sentinel deliberately does not duplicate it.
    if (constant.get("verified") is not True or
            constant.get("gravity_macro") != "G_m_s2" or
            constant.get("expected_literal") != 9.81 or
            constant.get("source_sha256") !=
            "e89d7b8a80c9f9bb88663a6eaa7e4921c222b21a7ad7f898262158d307679796" or
            constant.get("source_file_identity", {}).get("sha256") !=
            constant.get("source_sha256") or frozen_constant is not None):
        raise CampaignError(f"{label}: exact runtime gravity binding differs")

    base = receipt.get("base_deterministic_receipt")
    if not isinstance(base, Mapping):
        raise CampaignError(f"{label}: missing embedded deterministic receipt")
    _self_hash(base, BASE_RECEIPT_SCHEMA, f"{label} base receipt")
    if (base.get("plan_identity_sha256") != plan_identity or
            base.get("run_id") != run["run_id"] or
            base.get("process_instance_uuid") !=
            receipt.get("process_instance_uuid")):
        raise CampaignError(f"{label}: embedded deterministic binding differs")
    build, base_build_identity = _validate_build(base.get("build"), label)
    init = _validate_init(base, sentinel["explicit_anchor"], label)
    streams = _validate_streams(base, label)
    accuracy = _validate_accuracy(base, label)
    if streams["first_correction"]["correction_epoch_ns"] != init[
            "state_epoch_ns"]:
        raise CampaignError(f"{label}: correction/acceptance epoch differs")

    primary = receipt.get("primary_full_result", {})
    secondary = receipt.get("secondary_hover_ranking", {})
    _file_is_current(primary.get("identity"), f"{label} primary report")
    _file_is_current(secondary.get("identity"), f"{label} secondary report")
    if (primary.get("status") not in {"pass", "fail", "incomplete"} or
            not isinstance(primary.get("flight_ready"), bool) or
            primary.get("flight_ready") != (primary.get("status") == "pass") or
            primary.get("is_authoritative_for_interface_safety") is not True or
            secondary.get("status") != "ranking_only" or
            secondary.get("flight_ready") is not False or
            secondary.get("can_override_primary_failure") is not False):
        raise CampaignError(f"{label}: primary/secondary decision scope differs")
    alignment = primary.get("alignment_takeoff_diagnostic", {})
    lead = alignment.get("first_output_to_detected_takeoff_lead_s")
    if (alignment.get("source") != "primary_evaluator_gt_detected_takeoff" or
            alignment.get("not_claimed_ground_only") is not True or
            not isinstance(alignment.get(
                "alignment_overlaps_detected_takeoff"), bool) or
            not isinstance(lead, (int, float)) or isinstance(lead, bool) or
            not math.isfinite(float(lead))):
        raise CampaignError(f"{label}: alignment/takeoff diagnostic differs")
    return {
        "run_id": run["run_id"],
        "sentinel_id": run["sentinel_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "process_instance_uuid": receipt["process_instance_uuid"],
        "receipt_identity_sha256": receipt["identity_sha256"],
        "build_manifest_identity_sha256": receipt[
            "build_manifest_identity_sha256"],
        "base_build_identity_sha256": base_build_identity,
        "build": dict(build),
        "initialization": init,
        "streams": streams,
        "accuracy_diagnostic": accuracy,
        "primary_status": primary["status"],
        "primary_flight_ready": primary["flight_ready"],
        "alignment_takeoff_diagnostic": dict(alignment),
    }


def check_qualification(plan_path: Path, receipt_root: Path) -> Dict[str, Any]:
    plan_path = plan_path.resolve()
    receipt_root = receipt_root.resolve()
    plan, plan_identity = _load_ground_plan(plan_path)
    sentinels = {row["id"]: row for row in plan["sentinels"]}
    rows: List[Dict[str, Any]] = []
    process_instances = set()
    for run in plan["runs"]:
        path = _receipt_path(receipt_root, run["expected_receipt"])
        row = _validate_ground_receipt(
            load_json(path), run, sentinels[run["sentinel_id"]],
            plan_identity, f"receipt {run['run_id']}")
        if row["process_instance_uuid"] in process_instances:
            raise CampaignError("fresh process UUID was reused")
        process_instances.add(row["process_instance_uuid"])
        row["receipt"] = str(path)
        rows.append(row)
    if len(rows) != 12 or len(process_instances) != 12:
        raise CampaignError("ground qualification lacks 12 fresh receipts")

    failures: List[Dict[str, Any]] = []
    warnings: List[Dict[str, Any]] = []
    full_builds = {row["build_manifest_identity_sha256"] for row in rows}
    base_builds = {row["base_build_identity_sha256"] for row in rows}
    if len(full_builds) != 1 or len(base_builds) != 1:
        failures.append({
            "gate": "single_exact_build_across_12_cells",
            "full_build_identity_count": len(full_builds),
            "base_build_identity_count": len(base_builds),
        })

    sentinel_results: List[Dict[str, Any]] = []
    for sentinel_id, sentinel in sentinels.items():
        group = [row for row in rows if row["sentinel_id"] == sentinel_id]
        if len(group) != 6:
            raise CampaignError(f"{sentinel_id}: expected six receipts")
        init_signatures = {
            object_sha256(row["initialization"]) for row in group
        }
        low_rate: Dict[str, bool] = {}
        for stream in LOW_RATE_EXACT_STREAMS:
            signatures = {object_sha256(row["streams"][stream])
                          for row in group}
            low_rate[stream] = len(signatures) == 1
            if len(signatures) != 1:
                failures.append({
                    "gate": "exact_low_rate_stream_across_rates_and_repeats",
                    "sentinel_id": sentinel_id,
                    "stream": stream,
                    "signature_count": len(signatures),
                })
        correction_signatures = {
            object_sha256(row["streams"]["first_correction"])
            for row in group
        }
        if len(init_signatures) != 1:
            failures.append({
                "gate": "exact_initialization_across_rates_and_repeats",
                "sentinel_id": sentinel_id,
                "signature_count": len(init_signatures),
            })
        if len(correction_signatures) != 1:
            failures.append({
                "gate": "exact_first_correction_across_rates_and_repeats",
                "sentinel_id": sentinel_id,
                "signature_count": len(correction_signatures),
            })
        high_rate: Dict[str, Dict[str, int]] = {}
        for stream in HIGH_RATE_DIAGNOSTIC_STREAMS:
            inventory = {
                (row["streams"][stream]["message_count"],
                 row["streams"][stream]["sensor_stamp_vector_sha256"])
                for row in group
            }
            payload = {row["streams"][stream]["canonical_state_sha256"]
                       for row in group}
            high_rate[stream] = {
                "inventory_signature_count": len(inventory),
                "payload_signature_count": len(payload),
            }
            if len(inventory) != 1 or len(payload) != 1:
                warnings.append({
                    "diagnostic": "high_rate_stream_differs",
                    "sentinel_id": sentinel_id,
                    "stream": stream,
                    **high_rate[stream],
                    "cannot_clear_high_rate_no_go": True,
                })
        sentinel_results.append({
            "sentinel_id": sentinel_id,
            "run_count": 6,
            "exact_initialization": len(init_signatures) == 1,
            "exact_low_rate_and_correction_streams": low_rate,
            "exact_first_correction": len(correction_signatures) == 1,
            "high_rate_diagnostics_only": high_rate,
            "primary_statuses": [row["primary_status"] for row in group],
            "primary_flight_ready": [row["primary_flight_ready"]
                                     for row in group],
            "alignment_takeoff_diagnostics": [
                row["alignment_takeoff_diagnostic"] for row in group],
        })

    low_rate_go = not failures
    primary_all_pass = all(row["primary_flight_ready"] for row in rows)
    core: Dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "qualification_variant": VARIANT,
        "scope": "development_only",
        "validation_data_accessed": False,
        "plan": str(plan_path),
        "plan_identity_sha256": plan_identity,
        "receipt_root": str(receipt_root),
        "receipt_count": len(rows),
        "fresh_process_instance_count": len(process_instances),
        "build_manifest_identity_sha256": (
            next(iter(full_builds)) if len(full_builds) == 1 else None),
        "postfix_build_identity_sha256": (
            next(iter(full_builds)) if len(full_builds) == 1 else None),
        "postfix_executable_sha256": (
            rows[0]["build"].get("executable_sha256")
            if len(base_builds) == 1 else None),
        "run_receipts": rows,
        "sentinels": sentinel_results,
        "decision_basis": (
            "exact initialization + low_rate_pose + low_rate_init + "
            "correction + first_correction only"),
        "failures": failures,
        "warnings": warnings,
        "status": "pass" if low_rate_go else "fail",
        "low_rate_estimator_rebaseline_go": low_rate_go,
        "go_for_ground_init_low_rate_phase_a_rebaseline": low_rate_go,
        "primary_full_result_all_pass": primary_all_pass,
        "primary_strict_flight_status": "NO_GO",
        "high_rate_interface_status": "NO_GO",
        "high_rate_interface_remains_no_go": True,
        "high_rate_inventory_and_payload_are_diagnostic_only": True,
        "predecessor_qualification_status": "fail",
        "predecessor_qualification_superseded": False,
        "secondary_can_override_primary_failure": False,
        "not_a_flight_readiness_decision": True,
        "flight_ready": False,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _write_report(path: Path, report: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(report, indent=2, sort_keys=True) + "\n").encode()
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
            print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["status"] == "pass" else 1
    except (CampaignError, FileExistsError, OSError, KeyError,
            TypeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

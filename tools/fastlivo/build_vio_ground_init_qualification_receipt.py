#!/usr/bin/env python3
"""Build one ground-init receipt from a completed, bound campaign cell.

The existing deterministic-init receipt builder remains the canonical parser
for node diagnostics and sensor-stamped streams.  This adapter adds the exact
runtime stationarity parameters, frozen ground-anchor statistics, and the
primary/full plus secondary/ranking report bindings.  It performs no replay.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Dict, Mapping, Optional, Sequence

import yaml

from build_vio_postfix_init_qualification_receipt import (
    EXECUTION_SCHEMA,
    _self_hash,
    _verify_file_identity,
    build_receipt as build_base_receipt,
    validate_build_manifest,
)
from check_vio_postfix_init_qualification import PLAN_SCHEMA
from prepare_vio_ground_init_qualification import VARIANT
from run_vio_flight_tuning_campaign import (
    CampaignError,
    file_identity,
    load_json,
    object_sha256,
)
from run_vio_ground_init_qualification_cell import POSTPROCESS_SCHEMA


SCHEMA = "fastlivo_vio_ground_init_qualification_receipt/v1"
EXPECTED_RUNTIME_PARAMETERS = {
    "imu/init_anchor_max_predecessor_gap_s": 0.02,
    "imu/imu_int_frame": 30,
    "imu/init_max_gyr_mean": 0.01,
    "imu/init_max_gyr_std": 0.04,
    "imu/init_max_acc_std": 0.25,
    "imu/init_acc_norm_tolerance": 0.10,
}


def _nested(document: Mapping[str, Any], dotted: str) -> Any:
    value: Any = document
    for key in dotted.split("/"):
        if not isinstance(value, Mapping) or key not in value:
            raise CampaignError(f"parameter snapshot lacks {dotted}")
        value = value[key]
    return value


def _exact_finite_vector(actual: Any, expected: Any, label: str) -> None:
    if (not isinstance(actual, list) or not isinstance(expected, list) or
            len(actual) != 3 or len(expected) != 3 or
            any(not isinstance(value, (int, float)) or isinstance(value, bool) or
                not math.isfinite(float(value)) for value in actual + expected) or
            [float(value).hex() for value in actual] !=
            [float(value).hex() for value in expected]):
        raise CampaignError(f"runtime/artifact {label} differs at binary64")


def build_ground_receipt(
        plan: Mapping[str, Any], run_id: str, attempt: Path,
        execution: Mapping[str, Any], build: Mapping[str, Any], *,
        verify_actual_build: bool = True) -> Dict[str, Any]:
    plan_identity = _self_hash(plan, PLAN_SCHEMA, "ground qualification plan")
    if (plan.get("qualification_variant") != VARIANT or
            plan.get("high_rate_interface_remains_no_go") is not True or
            plan.get("predecessor_qualification_remains_fail_no_go") is not True):
        raise CampaignError("not the frozen ground qualification variant")
    execution_identity = _self_hash(
        execution, EXECUTION_SCHEMA, "ground execution")
    build_identity = validate_build_manifest(
        build, verify_actual=verify_actual_build)
    ground_constant = build.get("ground_runtime_constant_binding")
    if (not isinstance(ground_constant, Mapping) or
            ground_constant.get("verified") is not True or
            ground_constant.get("source_sha256") !=
            plan["runtime_constant_binding"]["source_sha256"] or
            ground_constant.get("expected_literal") != 9.81):
        raise CampaignError("ground build lacks exact runtime gravity binding")
    runs = [row for row in plan["runs"] if row.get("run_id") == run_id]
    if len(runs) != 1:
        raise CampaignError("ground plan has no unique requested run")
    run = runs[0]
    sentinel = next(row for row in plan["sentinels"]
                    if row["id"] == run["sentinel_id"])
    attempt = attempt.resolve()
    base = build_base_receipt(
        plan, run_id, attempt, execution, build,
        verify_actual_build=verify_actual_build)
    params_path = attempt / "result_params.yaml"
    try:
        params = yaml.safe_load(params_path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read ground runtime params: {error}") from error
    if not isinstance(params, Mapping):
        raise CampaignError("ground runtime params are not a mapping")
    expected_parameters = {
        **EXPECTED_RUNTIME_PARAMETERS,
        "imu/init_anchor_stamp_ns": sentinel["explicit_anchor"][
            "anchor_stamp_ns"],
    }
    runtime_parameters: Dict[str, Any] = {}
    for dotted, expected in expected_parameters.items():
        actual = _nested(params, dotted)
        if actual != expected:
            raise CampaignError(
                f"ground runtime parameter differs {dotted}: {actual!r}")
        runtime_parameters[dotted] = actual
    if _nested(params, "uav/runtime_reinit_enable") is not False:
        raise CampaignError("runtime reinitialization was enabled")
    stationarity = sentinel["stationarity"]
    init = base["initialization"]
    _exact_finite_vector(
        init["statistics"]["mean_acc"], stationarity["mean_acc"],
        "mean_acc")
    _exact_finite_vector(
        init["statistics"]["mean_gyr"], stationarity["mean_gyr"],
        "mean_gyr")
    if (init["sample_sensor_stamp_seq_vector"] !=
            sentinel["explicit_anchor"][
                "expected_first_30_strict_post_anchor_stamp_seq"] or
            not all(stationarity["checks"].values()) or
            stationarity["accepted"] is not True or
            sentinel["post_selection_ground_hard_gate"]["passed"] is not True):
        raise CampaignError("ground init vector/stationarity/hard gate mismatch")

    postprocess_path = attempt / "ground_postprocess.json"
    postprocess = load_json(postprocess_path)
    postprocess_identity = _self_hash(
        postprocess, POSTPROCESS_SCHEMA, "ground postprocess")
    primary_path = attempt / "result.flight_readiness.json"
    secondary_path = attempt / "result.hover_ranking.json"
    primary = load_json(primary_path)
    secondary = load_json(secondary_path)
    orchestration = load_json(Path(str(execution["orchestration_path"])))
    thresholds_identity = orchestration["dependencies"]["thresholds"]
    primary_alignment = primary.get("local", {}).get("alignment", {})
    primary_semantics = primary.get("evaluation_semantics", {})
    first_output_lead = primary_alignment.get(
        "first_output_to_takeoff_lead_s")
    if (primary.get("schema") != "fastlivo_vio_flight_readiness/v1" or
            Path(str(primary.get("result_bag", ""))).resolve() !=
            (attempt / "result.bag").resolve() or
            primary_semantics.get("score_window_sensor_stamp_ns") is not None or
            primary_semantics.get("fixed_alignment_supplied") is not False or
            primary.get("artifact_bindings", {}).get("result_bag") !=
            file_identity(attempt / "result.bag") or
            primary.get("artifact_bindings", {}).get("thresholds") !=
            thresholds_identity or
            not isinstance(first_output_lead, (int, float)) or
            isinstance(first_output_lead, bool) or
            not math.isfinite(float(first_output_lead)) or
            not isinstance(primary_alignment.get(
                "overlaps_detected_takeoff"), bool)):
        raise CampaignError("primary full-result evaluator binding failed")
    exact_postprocess = {
        "plan_identity_sha256": plan_identity,
        "orchestration_identity_sha256": execution[
            "orchestration_identity_sha256"],
        "build_manifest_identity_sha256": build_identity,
        "run_id": run_id,
        "attempt": str(attempt),
        "primary_report": file_identity(primary_path),
        "secondary_report": file_identity(secondary_path),
        "secondary_log": file_identity(attempt / "evaluate.hover.stdout.log"),
        "secondary_cannot_override_primary": True,
    }
    for field, expected in exact_postprocess.items():
        if postprocess.get(field) != expected:
            raise CampaignError(f"ground postprocess differs: {field}")
    hover = sentinel["secondary_hover_evaluation"]
    secondary_alignment = secondary.get("local_accuracy", {}).get(
        "alignment", {})
    alignment_fields = ("method", "scale", "yaw_deg", "translation_m")
    artifact_bindings = secondary.get("artifact_bindings", {})
    if (secondary.get("schema") != "fastlivo_vio_ground_hover_ranking/v1" or
            secondary.get("result_bag") != str((attempt / "result.bag").resolve()) or
            secondary.get("role") != "phase_a_ranking_compatibility_only" or
            secondary.get("status") != "ranking_only" or
            secondary.get("primary_report_identity") !=
            file_identity(primary_path) or
            secondary.get("primary_status") != primary.get("status") or
            secondary.get("primary_flight_ready") !=
            primary.get("flight_ready") or
            secondary.get("flight_ready") is not False or
            secondary.get("can_override_primary_failure") is not False or
            artifact_bindings.get("result_bag") !=
            file_identity(attempt / "result.bag") or
            artifact_bindings.get("thresholds") != thresholds_identity or
            secondary.get("evaluation_semantics", {}).get(
                "score_window_sensor_stamp_ns") != {
                    "start": hover["start_absolute_ros_epoch_ns"],
                    "end": hover["end_absolute_ros_epoch_ns"],
                    "boundary": "start_inclusive_end_inclusive",
                } or secondary.get("evaluation_semantics", {}).get(
                    "primary_alignment_reused_without_refit") is not True or
            not isinstance(primary_alignment, Mapping) or
            not isinstance(secondary_alignment, Mapping) or
            any(secondary_alignment.get(field) != primary_alignment.get(field)
                for field in alignment_fields)):
        raise CampaignError("secondary hover ranking report binding failed")
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "plan_identity_sha256": plan_identity,
        "run_id": run_id,
        "sentinel_id": run["sentinel_id"],
        "arm_id": run["arm_id"],
        "session_id": run["session_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "fresh_process": True,
        "process_instance_uuid": base["process_instance_uuid"],
        "build_manifest_identity_sha256": build_identity,
        "ground_runtime_constant_binding": dict(ground_constant),
        "runtime_parameters": runtime_parameters,
        "stationarity_evidence": {
            "measurement_vector_sha256": stationarity[
                "measurement_vector_sha256"],
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
            "sample_std_source": (
                "frozen_preregistered_recomputation_from_exact_30_sample_"
                "measurement_vector_not_runtime_log"),
        },
        "post_selection_ground_hard_gate": sentinel[
            "post_selection_ground_hard_gate"],
        "primary_full_result": {
            "identity": file_identity(primary_path),
            "status": primary.get("status"),
            "flight_ready": primary.get("flight_ready"),
            "is_authoritative_for_interface_safety": True,
            "alignment_takeoff_diagnostic": {
                "source": "primary_evaluator_gt_detected_takeoff",
                "alignment_sensor_start_s": primary_alignment.get(
                    "sensor_start_s"),
                "alignment_sensor_end_s": primary_alignment.get(
                    "sensor_end_s"),
                "first_output_to_detected_takeoff_lead_s":
                    first_output_lead,
                "alignment_overlaps_detected_takeoff": primary_alignment[
                    "overlaps_detected_takeoff"],
                "not_claimed_ground_only": True,
            },
        },
        "secondary_hover_ranking": {
            "identity": file_identity(secondary_path),
            "status": "ranking_only",
            "flight_ready": False,
            "can_override_primary_failure": False,
            "bound_epoch_origins": {
                "start": hover["start_epoch_origin"],
                "end": hover["end_epoch_origin"],
            },
        },
        "postprocess_identity_sha256": postprocess_identity,
        "execution_receipt_identity_sha256": execution_identity,
        "base_deterministic_receipt": base,
        "high_rate_interface_qualification": "NO_GO",
        "not_a_flight_readiness_decision": True,
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
        receipt = build_ground_receipt(
            load_json(arguments.plan), arguments.run_id,
            arguments.attempt, load_json(arguments.execution),
            load_json(arguments.build_manifest))
        _write_exclusive(arguments.output, receipt)
        print(json.dumps({
            "receipt": str(arguments.output.resolve()),
            "identity_sha256": receipt["identity_sha256"],
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError,
            ValueError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

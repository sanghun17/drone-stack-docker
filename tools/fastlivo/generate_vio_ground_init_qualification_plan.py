#!/usr/bin/env python3
"""Generate an immutable 12-run ground-init qualification plan.

The plan is execution-neutral.  It freezes two development sentinels, two
replay rates, three fresh-process repeats, the full-start-to-landing replay
window, a primary whole-result safety evaluation, and a secondary bounded
hover-to-landing evaluation.  It does not launch ROS or replay a bag.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence

import yaml

from extract_vio_ground_init_anchors import (
    CONFIG_SCHEMA,
    DEFAULT_CONFIG,
    SCHEMA as ANCHORS_SCHEMA,
    _load_config,
    measurement_vector_sha256,
)
from generate_vio_postfix_init_qualification_plan import DEFAULT_REFERENCE
from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_arms,
    load_json,
    object_sha256,
    sha256,
    validate_plan_identity,
)
from select_vio_flight_tuning_phase_a import PHASE_A_ARMS
from select_vio_flight_tuning_phase_b import verify_phase_a_plan
from check_vio_postfix_init_qualification import (
    EXPECTED_SEMANTICS,
    PLAN_SCHEMA,
)


SCHEMA = PLAN_SCHEMA


def _write_json_exclusive(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True) + "\n").encode()
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _self_hash(document: Mapping[str, Any], schema: str, label: str) -> str:
    if document.get("schema") != schema:
        raise CampaignError(f"{label}: wrong schema")
    declared = str(document.get("identity_sha256", ""))
    core = dict(document)
    core.pop("identity_sha256", None)
    if len(declared) != 64 or object_sha256(core) != declared:
        raise CampaignError(f"{label}: identity changed")
    return declared


def _validate_anchors(
        artifact: Mapping[str, Any], reference_plan: Mapping[str, Any],
        session_ids: Sequence[str]) -> Dict[str, Mapping[str, Any]]:
    _self_hash(artifact, ANCHORS_SCHEMA, "ground anchors")
    if (artifact.get("scope") != "development_only" or
            artifact.get("validation_data_accessed") is not False or
            artifact.get("selection_uses_ground_truth") is not False or
            artifact.get("selection_uses_estimator_output") is not False or
            artifact.get("reference_phase_a_campaign_identity_sha256") !=
            reference_plan.get("identity_sha256") or
            artifact.get("session_ids") != list(session_ids) or
            artifact.get("session_count") != 5):
        raise CampaignError("ground anchor scope/provenance contract mismatch")
    rows = artifact.get("sessions")
    if not isinstance(rows, list) or len(rows) != 5:
        raise CampaignError("ground anchors lack exact five-session grid")
    result: Dict[str, Mapping[str, Any]] = {}
    reference = {str(row["id"]): row for row in reference_plan["sessions"]}
    for row in rows:
        if not isinstance(row, Mapping):
            raise CampaignError("malformed ground anchor row")
        session_id = str(row.get("session_id", ""))
        if session_id not in session_ids or session_id in result:
            raise CampaignError(f"unexpected/duplicate ground session {session_id}")
        input_row = row.get("input")
        expected = reference[session_id]
        if (not isinstance(input_row, Mapping) or
                input_row.get("path") != expected["input_bag"] or
                input_row.get("declared_sha256") !=
                expected["input_declared_sha256"] or
                input_row.get("verified_sha256") !=
                expected["input_declared_sha256"] or
                input_row.get("full_file_sha256_verified") is not True or
                input_row.get("input_provenance_sha256") !=
                expected["input_provenance_sha256"]):
            raise CampaignError(f"ground input identity mismatch: {session_id}")
        anchor = row.get("anchor")
        stationarity = row.get("stationarity")
        audit = row.get("post_selection_ground_hard_gate")
        windows = row.get("windows")
        if not all(isinstance(value, Mapping)
                   for value in (anchor, stationarity, audit, windows)):
            raise CampaignError(f"incomplete ground anchor row: {session_id}")
        vector = stationarity.get("measurement_vector")
        expected_vector = anchor.get(
            "expected_first_30_strict_post_anchor_stamp_seq")
        if (not isinstance(vector, list) or len(vector) != 30 or
                [(sample.get("stamp_ns"), sample.get("seq")) for sample in vector] !=
                [(sample.get("stamp_ns"), sample.get("seq"))
                 for sample in expected_vector] or
                measurement_vector_sha256(vector) !=
                stationarity.get("measurement_vector_sha256") or
                stationarity.get("accepted") is not True or
                not all(stationarity.get("checks", {}).values()) or
                audit.get("passed") is not True or
                not all(audit.get("checks", {}).values())):
            raise CampaignError(f"ground anchor stationarity/audit failed: {session_id}")
        result[session_id] = row
    if list(result) != list(session_ids):
        raise CampaignError("ground anchor session order changed")
    return result


def generate_plan(
        config: Mapping[str, Any], reference_plan: Mapping[str, Any],
        expected_arms: Sequence[Mapping[str, Any]],
        anchors_artifact: Mapping[str, Any], *,
        config_identity: Optional[Mapping[str, Any]] = None,
        anchors_identity: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    if config.get("schema") != CONFIG_SCHEMA:
        raise CampaignError("wrong ground-init config schema")
    session_ids = verify_phase_a_plan(reference_plan, expected_arms)
    anchor_identity = _self_hash(
        anchors_artifact, ANCHORS_SCHEMA, "ground anchors")
    exact_contracts = {
        "stationarity_gate": config.get("stationarity_gate"),
        "runtime_constant_binding": config.get("runtime_constant_binding"),
        "post_selection_ground_hard_gate_contract": config.get(
            "post_selection_ground_hard_gate"),
        "window_contract": config.get("windows"),
    }
    for field, expected in exact_contracts.items():
        actual = anchors_artifact.get(field)
        if field == "runtime_constant_binding" and isinstance(actual, Mapping):
            actual = {key: actual.get(key) for key in expected}
        if actual != expected:
            raise CampaignError(f"ground anchor/config contract mismatch: {field}")
    anchors = _validate_anchors(anchors_artifact, reference_plan, session_ids)
    by_arm = {str(row["id"]): copy.deepcopy(dict(row["overrides"]))
              for row in expected_arms}
    raw_sentinels = config.get("sentinels")
    if not isinstance(raw_sentinels, list) or len(raw_sentinels) != 2:
        raise CampaignError("ground qualification requires exactly two sentinels")
    sentinels: List[Dict[str, Any]] = []
    seen = set()
    for raw in raw_sentinels:
        sentinel_id = str(raw.get("id", ""))
        arm_id = str(raw.get("arm_id", ""))
        session_id = str(raw.get("session_id", ""))
        if (not sentinel_id or sentinel_id in seen or arm_id not in by_arm or
                session_id not in anchors):
            raise CampaignError("invalid/duplicate ground sentinel")
        seen.add(sentinel_id)
        row = anchors[session_id]
        runtime = copy.deepcopy(by_arm[arm_id])
        imu = runtime.setdefault("imu", {})
        if not isinstance(imu, dict) or any(key in imu for key in (
                "init_anchor_stamp_ns", "init_anchor_max_predecessor_gap_s",
                "init_max_gyr_mean", "init_max_gyr_std", "init_max_acc_std",
                "init_acc_norm_tolerance", "imu_int_frame")):
            raise CampaignError(f"arm conflicts with ground-init contract: {arm_id}")
        imu.update({
            "init_anchor_stamp_ns": row["anchor"]["anchor_stamp_ns"],
            "init_anchor_max_predecessor_gap_s": 0.02,
            "imu_int_frame": 30,
            "init_max_gyr_mean": 0.01,
            "init_max_gyr_std": 0.04,
            "init_max_acc_std": 0.25,
            "init_acc_norm_tolerance": 0.10,
        })
        replay = copy.deepcopy(dict(row["windows"]["replay"]))
        crop = {
            "basis": "full_bag_record_start_through_frozen_cached_landing",
            "start_s": replay["start_offset_s"],
            "duration_s": replay["duration_s"],
            "full_duration_s": replay["duration_s"],
            "smoke_truncated": False,
            "window_method": "ground_init_full_start_to_cached_landing",
        }
        sentinels.append({
            "id": sentinel_id,
            "arm_id": arm_id,
            "phase_a_arm_overrides": by_arm[arm_id],
            "runtime_overrides": runtime,
            "session_id": session_id,
            "session_split": "development",
            "input_bag": row["input"]["path"],
            "input_declared_sha256": row["input"]["declared_sha256"],
            "input_provenance_sha256": row["input"][
                "input_provenance_sha256"],
            "crop": crop,
            "explicit_anchor": copy.deepcopy(dict(row["anchor"])),
            "stationarity": copy.deepcopy(dict(row["stationarity"])),
            "post_selection_ground_hard_gate": copy.deepcopy(dict(
                row["post_selection_ground_hard_gate"])),
            "primary_evaluation": copy.deepcopy(dict(
                row["windows"]["primary_evaluation"])),
            "secondary_hover_evaluation": copy.deepcopy(dict(
                row["windows"]["secondary_hover_evaluation"])),
        })
    rates = config.get("rates")
    repeats = config.get("fresh_process_repeats_per_rate")
    if rates != [0.5, 1.0] or repeats != 3:
        raise CampaignError("ground qualification grid must be 2 rates x 3 repeats")
    runs: List[Dict[str, Any]] = []
    for sentinel in sentinels:
        for rate in rates:
            rate_label = str(rate).replace(".", "p")
            for repeat in range(1, repeats + 1):
                runs.append({
                    "run_id": f"{sentinel['id']}__rate{rate_label}__r{repeat}",
                    "sentinel_id": sentinel["id"],
                    "arm_id": sentinel["arm_id"],
                    "session_id": sentinel["session_id"],
                    "rate": rate,
                    "repeat": repeat,
                    "fresh_process_required": True,
                    "expected_receipt": f"receipts/{sentinel['id']}__rate{rate_label}__r{repeat}.json",
                    "replay_arguments": {
                        "start_s": sentinel["crop"]["start_s"],
                        "duration_s": sentinel["crop"]["duration_s"],
                        "no_gt_anchor": True,
                        "with_propagated": True,
                    },
                    "evaluation_arguments": {
                        "primary": {
                            "score_window": None,
                            "role": "safety_primary",
                        },
                        "secondary": {
                            "score_start_ns": sentinel[
                                "secondary_hover_evaluation"][
                                    "start_absolute_ros_epoch_ns"],
                            "score_end_ns": sentinel[
                                "secondary_hover_evaluation"][
                                    "end_absolute_ros_epoch_ns"],
                            "role": "phase_a_ranking_compatibility_only",
                        },
                    },
                })
    if len(runs) != 12 or len({row["run_id"] for row in runs}) != 12:
        raise CampaignError("ground qualification did not produce exact 12-run grid")
    predecessor = anchors_artifact["predecessor_qualification"]
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "execution_neutral": True,
        "replay_executed_by_generator": False,
        "qualification_variant": "ground_init_full_start_to_landing/v1",
        "session_window_mode": "ground_to_landing",
        "not_a_flight_readiness_decision": True,
        "predecessor_qualification_remains_fail_no_go": True,
        "high_rate_interface_remains_no_go": True,
        "predecessor_qualification": copy.deepcopy(dict(predecessor)),
        "reference_phase_a_campaign_identity_sha256": reference_plan[
            "identity_sha256"],
        "config_identity": dict(config_identity or {
            "object_sha256": object_sha256(config)}),
        "anchors_identity": dict(anchors_identity or {
            "object_sha256": object_sha256(anchors_artifact)}),
        "anchors_artifact_identity_sha256": anchor_identity,
        "stationarity_gate": copy.deepcopy(dict(config["stationarity_gate"])),
        "runtime_constant_binding": copy.deepcopy(dict(
            config["runtime_constant_binding"])),
        "post_selection_ground_hard_gate_contract": copy.deepcopy(dict(
            config["post_selection_ground_hard_gate"])),
        "window_contract": copy.deepcopy(dict(config["windows"])),
        "rates": list(rates),
        "fresh_process_repeats_per_rate": repeats,
        "expected_run_count": len(runs),
        "sentinels": sentinels,
        "runs": runs,
        "required_semantics": copy.deepcopy(EXPECTED_SEMANTICS),
        "reference_phase_a_campaign_id": reference_plan["campaign_id"],
        "reference_phase_a_build": copy.deepcopy(reference_plan["build"]),
        "frozen_phase_a_arms_sha256": object_sha256(expected_arms),
        "postfix_phase_a_rebaseline": {
            "rate": 1.0,
            "repeats_per_arm_session": 1,
            "expected_arm_count": 8,
            "expected_run_count": 40,
            "expected_session_ids": list(session_ids),
            "old_scores_may_be_pooled": False,
            "reuse_old_completion_pointers": False,
            "validation_access_allowed": False,
            "window_mode": "ground_to_landing",
        },
        "postfix_phase_a_explicit_anchor_overrides": {
            session_id: {
                "imu": {
                    "init_anchor_stamp_ns": anchors[session_id]["anchor"][
                        "anchor_stamp_ns"],
                    "init_anchor_max_predecessor_gap_s": 0.02,
                    "imu_int_frame": 30,
                    "init_max_gyr_mean": 0.01,
                    "init_max_gyr_std": 0.04,
                    "init_max_acc_std": 0.25,
                    "init_acc_norm_tolerance": 0.10,
                },
            } for session_id in session_ids
        },
        "decision_contract": {
            "primary_full_result_safety_report_required": True,
            "secondary_hover_report_required": True,
            "secondary_cannot_override_primary_failure": True,
            "low_rate_determinism_can_be_qualified_separately": True,
            "high_rate_flight_interface_can_be_cleared_by_this_plan": False,
        },
    }
    return {**core, "identity_sha256": object_sha256(core)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("anchors", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--reference-campaign", type=Path,
                        default=DEFAULT_REFERENCE)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        config_path = arguments.config.resolve()
        anchors_path = arguments.anchors.resolve()
        config = _load_config(config_path)
        reference_plan = load_json(
            arguments.reference_campaign.resolve() / "campaign.json")
        validate_plan_identity(reference_plan)
        artifact = load_json(anchors_path)
        plan = generate_plan(
            config, reference_plan, load_arms(PHASE_A_ARMS), artifact,
            config_identity={
                "path": str(config_path),
                "size_bytes": config_path.stat().st_size,
                "sha256": sha256(config_path),
                "object_sha256": object_sha256(config),
            },
            anchors_identity={
                "path": str(anchors_path),
                "size_bytes": anchors_path.stat().st_size,
                "sha256": sha256(anchors_path),
                "object_sha256": object_sha256(artifact),
            })
        _write_json_exclusive(arguments.output, plan)
        print(json.dumps({
            "output": str(arguments.output.resolve()),
            "identity_sha256": plan["identity_sha256"],
            "expected_run_count": plan["expected_run_count"],
            "sentinels": [row["id"] for row in plan["sentinels"]],
            "replay_executed": False,
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, KeyError, ValueError,
            yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

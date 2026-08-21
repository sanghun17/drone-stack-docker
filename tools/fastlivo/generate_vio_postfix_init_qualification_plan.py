#!/usr/bin/env python3
"""Generate an immutable, execution-neutral 12-run init qualification plan.

The output describes exactly two development-only sentinels, two replay rates,
and three fresh-process repeats.  It contains no executable command and does
not launch ROS, Docker, FAST-LIVO, or a replay.  A later orchestrator must
produce one canonical sensor-time receipt per planned run for the companion
checker.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence

import yaml

from run_vio_flight_tuning_campaign import (
    CampaignError,
    load_arms,
    load_json,
    object_sha256,
    sha256,
    validate_plan_identity,
)
from select_vio_flight_tuning_phase_a import (
    DEVELOPMENT_SESSION_IDS,
    PHASE_A_ARMS,
)
from select_vio_flight_tuning_phase_b import verify_phase_a_plan


SCHEMA = "fastlivo_vio_postfix_init_qualification_plan/v1"
CONFIG_SCHEMA = "fastlivo_vio_postfix_init_qualification_config/v1"
DEFAULT_CONFIG = Path(__file__).with_name(
    "vio_postfix_init_qualification_config.yaml")
DEFAULT_REFERENCE = (
    Path(__file__).with_name("_campaign_vio_flight_20260814") /
    "tuning_campaigns/phase_a_ofat_clean_v2")
ANCHORS_SCHEMA = "fastlivo_earliest_full_sync_anchors/v1"


def _load_yaml(path: Path) -> Dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read config {path}: {error}") from error
    if not isinstance(document, dict):
        raise CampaignError("qualification config must be a YAML mapping")
    return document


def _write_json_exclusive(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True,
                          ensure_ascii=False) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


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


def _validated_anchors(
        artifact: Mapping[str, Any], config: Mapping[str, Any],
        reference_plan: Mapping[str, Any], session_ids: Sequence[str],
        *, verify_identity: bool = True) -> Tuple[str, Dict[str, Dict[str, Any]]]:
    if artifact.get("schema") != ANCHORS_SCHEMA:
        raise CampaignError("wrong earliest-full-sync anchor artifact schema")
    declared = str(artifact.get("identity_sha256", ""))
    core = dict(artifact)
    core.pop("identity_sha256", None)
    if (verify_identity and
            (len(declared) != 64 or object_sha256(core) != declared)):
        raise CampaignError("earliest-full-sync anchor artifact identity changed")
    if (artifact.get("scope") != "development_only" or
            artifact.get("validation_data_accessed") is not False or
            artifact.get("placeholder_anchors_allowed") is not False or
            artifact.get("live_fallback_allowed") is not False or
            artifact.get("reference_phase_a_campaign_identity_sha256") !=
            reference_plan.get("identity_sha256") or
            artifact.get("session_count") != len(session_ids) or
            artifact.get("session_ids") != list(session_ids) or
            artifact.get("extraction_contract") !=
            config.get("anchor_extraction")):
        raise CampaignError("anchor artifact provenance/scope contract mismatch")
    raw_sessions = artifact.get("sessions")
    if not isinstance(raw_sessions, list) or len(raw_sessions) != len(session_ids):
        raise CampaignError("anchor artifact does not cover the exact dev grid")
    reference_by_id = {
        str(row["id"]): row for row in reference_plan["sessions"]
    }
    result: Dict[str, Dict[str, Any]] = {}
    for row in raw_sessions:
        if not isinstance(row, Mapping):
            raise CampaignError("malformed anchor session row")
        session_id = str(row.get("session_id", ""))
        if (session_id not in session_ids or session_id in result or
                row.get("split") != "development"):
            raise CampaignError(f"unexpected/duplicate anchor session: {session_id!r}")
        reference = reference_by_id[session_id]
        input_row = row.get("input")
        if (not isinstance(input_row, Mapping) or
                input_row.get("path") != reference["input_bag"] or
                input_row.get("declared_sha256") !=
                reference["input_declared_sha256"] or
                input_row.get("verified_sha256") !=
                reference["input_declared_sha256"] or
                input_row.get("full_file_sha256_verified") is not True or
                input_row.get("input_provenance_sha256") !=
                reference["input_provenance_sha256"]):
            raise CampaignError(f"anchor input identity mismatch: {session_id}")
        crop = row.get("crop")
        if not isinstance(crop, Mapping):
            raise CampaignError(f"anchor crop missing: {session_id}")
        for field, expected in reference["crop"].items():
            if crop.get(field) != expected:
                raise CampaignError(
                    f"anchor frozen crop mismatch {session_id}/{field}")
        anchor = row.get("anchor")
        if not isinstance(anchor, Mapping):
            raise CampaignError(f"anchor data missing: {session_id}")
        anchor_ns = _positive_decimal_ns(
            anchor.get("anchor_stamp_ns"), f"{session_id} anchor_stamp_ns")
        image_ns = _positive_decimal_ns(
            anchor.get("image_epoch_ns"), f"{session_id} image_epoch_ns")
        if (anchor_ns != image_ns or
                anchor.get("anchor_definition") !=
                "earliest_explicit_eligible_full_sync_sensor_epoch" or
                anchor.get("anchor_mode_required") != "explicit"):
            raise CampaignError(f"anchor/image epoch contract failed: {session_id}")
        lidar = anchor.get("lidar_watermark")
        imu = anchor.get("imu_watermark_at_coverage")
        predecessor = anchor.get("predecessor_imu")
        successor = anchor.get("successor_imu")
        if not all(isinstance(value, Mapping)
                   for value in (lidar, imu, predecessor, successor)):
            raise CampaignError(f"anchor bracketing data missing: {session_id}")
        lidar_ns = _positive_decimal_ns(
            lidar.get("watermark_ns"), f"{session_id} lidar watermark")
        imu_ns = _positive_decimal_ns(
            imu.get("watermark_ns"), f"{session_id} imu watermark")
        predecessor_ns = _positive_decimal_ns(
            predecessor.get("watermark_ns"), f"{session_id} predecessor")
        successor_ns = _positive_decimal_ns(
            successor.get("watermark_ns"), f"{session_id} successor")
        if (lidar_ns < anchor_ns or imu_ns < anchor_ns or
                predecessor_ns > anchor_ns or successor_ns <= anchor_ns or
                anchor.get("predecessor_gap_ns") != str(
                    anchor_ns - predecessor_ns) or
                anchor.get("successor_gap_ns") != str(successor_ns - anchor_ns)):
            raise CampaignError(f"anchor coverage/bracketing failed: {session_id}")
        maximum_gap_ns = int(round(float(
            config["anchor_extraction"]["synchronization"][
                "init_anchor_max_predecessor_gap_s"]) * 1e9))
        if (anchor_ns - predecessor_ns > maximum_gap_ns or
                anchor.get("init_anchor_max_predecessor_gap_s") != 0.02 or
                anchor.get("init_anchor_max_predecessor_gap_ns") !=
                str(maximum_gap_ns)):
            raise CampaignError(
                f"anchor predecessor exceeds frozen limit: {session_id}")
        vector = anchor.get("expected_first_30_strict_post_anchor_stamp_seq")
        if not isinstance(vector, list) or len(vector) != 30:
            raise CampaignError(f"anchor lacks expected 30-IMU vector: {session_id}")
        previous = anchor_ns
        for index, sample in enumerate(vector):
            if not isinstance(sample, Mapping):
                raise CampaignError(f"malformed expected IMU {session_id}/{index}")
            stamp = _positive_decimal_ns(
                sample.get("stamp_ns"), f"{session_id} expected IMU {index}")
            if stamp <= previous or not isinstance(sample.get("seq"), int):
                raise CampaignError(
                    f"expected IMUs are not strict post-anchor: {session_id}")
            previous = stamp
        if (anchor.get("expected_first_30_stamp_seq_sha256") !=
                _stamp_seq_sha256(vector) or
                anchor.get("expected_first_30_stamp_seq_hash_encoding") !=
                "utf8_lines_stamp_ns_comma_seq_newline" or
                anchor.get("expected_first_used_stamp_ns") != vector[0]["stamp_ns"] or
                anchor.get("expected_first_used_seq") != vector[0]["seq"] or
                anchor.get("expected_last_used_stamp_ns") != vector[-1]["stamp_ns"] or
                anchor.get("expected_last_used_seq") != vector[-1]["seq"]):
            raise CampaignError(f"expected IMU vector identity mismatch: {session_id}")
        result[session_id] = copy.deepcopy(dict(anchor))
    if list(result) != list(session_ids):
        raise CampaignError("anchor artifact session order differs from dev grid")
    return declared, result


def generate_plan(
        config: Mapping[str, Any], reference_plan: Mapping[str, Any],
        expected_arms: Sequence[Mapping[str, Any]],
        anchors_artifact: Mapping[str, Any], *,
        config_identity: Optional[Mapping[str, Any]] = None,
        anchors_identity: Optional[Mapping[str, Any]] = None,
        verify_identity: bool = True) -> Dict[str, Any]:
    if config.get("schema") != CONFIG_SCHEMA:
        raise CampaignError("wrong post-fix qualification config schema")
    reference_sessions = verify_phase_a_plan(
        reference_plan, expected_arms, verify_identity=verify_identity)
    reference_identity = str(reference_plan.get("identity_sha256", ""))
    if config.get(
            "reference_phase_a_campaign_identity_sha256") != reference_identity:
        raise CampaignError("qualification config/reference campaign mismatch")
    anchor_document_identity, anchors_by_session = _validated_anchors(
        anchors_artifact, config, reference_plan, reference_sessions,
        verify_identity=verify_identity)

    raw_sentinels = config.get("sentinels")
    if not isinstance(raw_sentinels, list) or len(raw_sentinels) != 2:
        raise CampaignError("qualification requires exactly two sentinels")
    by_arm = {str(arm["id"]): copy.deepcopy(dict(arm["overrides"]))
              for arm in expected_arms}
    session_records = {
        str(row["id"]): copy.deepcopy(dict(row))
        for row in reference_plan["sessions"]
    }
    sentinels: List[Dict[str, Any]] = []
    sentinel_ids = set()
    pairs = set()
    for raw in raw_sentinels:
        if not isinstance(raw, Mapping):
            raise CampaignError("malformed qualification sentinel")
        sentinel_id = str(raw.get("id", ""))
        arm_id = str(raw.get("arm_id", ""))
        session_id = str(raw.get("session_id", ""))
        if not sentinel_id or sentinel_id in sentinel_ids:
            raise CampaignError("empty/duplicate qualification sentinel id")
        if arm_id not in by_arm:
            raise CampaignError(f"unknown sentinel arm: {arm_id}")
        if (session_id not in session_records or
                session_id not in DEVELOPMENT_SESSION_IDS or
                session_id not in reference_sessions):
            raise CampaignError(
                f"refusing validation/non-development sentinel: {session_id}")
        pair = (arm_id, session_id)
        if pair in pairs:
            raise CampaignError("duplicate qualification arm/session pair")
        sentinel_ids.add(sentinel_id)
        pairs.add(pair)
        session = session_records[session_id]
        anchor = anchors_by_session[session_id]
        arm_overrides = copy.deepcopy(by_arm[arm_id])
        imu_overrides = arm_overrides.setdefault("imu", {})
        if not isinstance(imu_overrides, dict):
            raise CampaignError(f"sentinel arm has scalar imu override: {arm_id}")
        if ("init_anchor_stamp_ns" in imu_overrides or
                "init_anchor_max_predecessor_gap_s" in imu_overrides):
            raise CampaignError(
                f"sentinel arm already overrides init-anchor contract: {arm_id}")
        imu_overrides["init_anchor_stamp_ns"] = anchor["anchor_stamp_ns"]
        imu_overrides["init_anchor_max_predecessor_gap_s"] = 0.02
        sentinels.append({
            "id": sentinel_id,
            "arm_id": arm_id,
            "phase_a_arm_overrides": by_arm[arm_id],
            "runtime_overrides": arm_overrides,
            "session_id": session_id,
            "session_split": "development",
            "input_bag": session["input_bag"],
            "input_declared_sha256": session["input_declared_sha256"],
            "input_provenance_sha256": session["input_provenance_sha256"],
            "crop": session["crop"],
            "explicit_anchor": anchor,
        })

    rates = config.get("rates")
    if (not isinstance(rates, list) or len(rates) != 2 or
            sorted(float(rate) for rate in rates) != [0.5, 1.0] or
            any(not math.isfinite(float(rate)) for rate in rates)):
        raise CampaignError("qualification rates must be exactly [0.5, 1.0]")
    repeats = config.get("fresh_process_repeats_per_rate")
    if not isinstance(repeats, int) or isinstance(repeats, bool) or repeats != 3:
        raise CampaignError("qualification requires exactly three repeats/rate")
    semantics = config.get("required_semantics")
    if not isinstance(semantics, Mapping):
        raise CampaignError("qualification config lacks required semantics")
    exact_semantics = {
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
        "canonicalization_schema": "fastlivo_sensor_stamped_state_canonical/v1",
        "compare_rosbag_record_time": False,
        "require_exact_low_rate_payload_hashes_across_rates": True,
        "high_rate_payload_hashes_are_diagnostic_only": True,
    }
    if dict(semantics) != exact_semantics:
        raise CampaignError("qualification semantics differ from frozen contract")

    runs: List[Dict[str, Any]] = []
    for sentinel in sentinels:
        for rate in (0.5, 1.0):
            for repeat in range(1, 4):
                run_id = (
                    f"{sentinel['id']}__rate{str(rate).replace('.', 'p')}__r{repeat}")
                runs.append({
                    "run_id": run_id,
                    "sentinel_id": sentinel["id"],
                    "arm_id": sentinel["arm_id"],
                    "session_id": sentinel["session_id"],
                    "rate": rate,
                    "repeat": repeat,
                    "fresh_process_required": True,
                    "expected_receipt": f"receipts/{run_id}.json",
                })
    if len(runs) != 12 or len({run["run_id"] for run in runs}) != 12:
        raise CampaignError("qualification plan must contain exactly 12 unique runs")

    rebaseline = config.get("postfix_phase_a_rebaseline")
    if not isinstance(rebaseline, Mapping):
        raise CampaignError("missing post-fix Phase-A rebaseline contract")
    if (float(rebaseline.get("rate", math.nan)) != 1.0 or
            rebaseline.get("repeats_per_arm_session") != 1 or
            rebaseline.get("expected_arm_count") != 8 or
            rebaseline.get("expected_run_count") != 40 or
            list(rebaseline.get("expected_session_ids", [])) != reference_sessions or
            rebaseline.get("old_scores_may_be_pooled") is not False or
            rebaseline.get("reuse_old_completion_pointers") is not False or
            rebaseline.get("validation_access_allowed") is not False):
        raise CampaignError("malformed post-fix Phase-A rebaseline contract")

    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "execution_neutral": True,
        "reference_phase_a_campaign_id": reference_plan.get("campaign_id"),
        "reference_phase_a_campaign_identity_sha256": reference_identity,
        "reference_phase_a_build": reference_plan.get("build"),
        "config_identity": dict(config_identity or {
            "object_sha256": object_sha256(config),
        }),
        "frozen_phase_a_arms_sha256": object_sha256(expected_arms),
        "anchors_identity": dict(anchors_identity or {
            "identity_sha256": anchor_document_identity,
        }),
        "anchors_artifact_identity_sha256": anchor_document_identity,
        "sentinels": sentinels,
        "rates": [0.5, 1.0],
        "fresh_process_repeats_per_rate": 3,
        "expected_run_count": 12,
        "required_semantics": exact_semantics,
        "runs": runs,
        "postfix_phase_a_rebaseline": copy.deepcopy(dict(rebaseline)),
        "postfix_phase_a_explicit_anchor_overrides": {
            session_id: {
                "imu": {
                    "init_anchor_stamp_ns":
                        anchors_by_session[session_id]["anchor_stamp_ns"],
                    "init_anchor_max_predecessor_gap_s": 0.02,
                },
            }
            for session_id in reference_sessions
        },
    }
    return {**core, "identity_sha256": object_sha256(core)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--reference-campaign", type=Path,
                        default=DEFAULT_REFERENCE)
    parser.add_argument("--anchors", type=Path, required=True,
                        help="self-hashed five-session earliest-full-sync artifact")
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args(argv)
    try:
        config_path = arguments.config.resolve()
        reference = arguments.reference_campaign.resolve()
        config = _load_yaml(config_path)
        reference_plan = load_json(reference / "campaign.json")
        anchors_path = arguments.anchors.resolve()
        anchors = load_json(anchors_path)
        validate_plan_identity(reference_plan)
        expected_arms = load_arms(PHASE_A_ARMS)
        plan = generate_plan(config, reference_plan, expected_arms, anchors,
                             config_identity={
                                 "path": str(config_path),
                                 "sha256": sha256(config_path),
                                 "object_sha256": object_sha256(config),
                             }, anchors_identity={
                                 "path": str(anchors_path),
                                 "sha256": sha256(anchors_path),
                                 "identity_sha256": anchors.get("identity_sha256"),
                             })
        if arguments.output:
            _write_json_exclusive(arguments.output, plan)
        else:
            print(json.dumps(plan, indent=2, sort_keys=True,
                             ensure_ascii=False))
        return 0
    except (CampaignError, FileExistsError, OSError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

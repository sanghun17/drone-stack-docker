#!/usr/bin/env python3
"""Freeze development-only ground/pre-takeoff explicit IMU-init anchors.

Selection is deliberately blind to GT and estimator output: for each of the
five frozen development bags it takes the earliest explicit-eligible
full-synchronization image epoch reachable from bag record start.  The exact
30 strict post-anchor IMUs are then checked with the same sequential Welford
sample-standard-deviation math used by FAST-LIVO.  Cached GT takeoff/hover
epochs are consulted only after selection as an auditable pre-takeoff check.

This tool reads bags and writes one append-only, self-hashed JSON artifact.  It
never starts ROS, launches the estimator, or accesses the locked validation
split.
"""

from __future__ import annotations

import argparse
import copy
from decimal import Decimal, ROUND_HALF_UP
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import rosbag
import yaml

from extract_vio_earliest_full_sync_anchors import (
    _validated_extraction_config,
    extract_session_anchor,
    load_frozen_effective_parameters,
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


SCHEMA = "fastlivo_ground_init_anchors/v1"
CONFIG_SCHEMA = "fastlivo_ground_init_qualification_config/v1"
DEFAULT_CONFIG = Path(__file__).with_name(
    "vio_ground_init_qualification_config.yaml")
MEASUREMENT_HASH_ENCODING = (
    "concatenated_big_endian_uint64_stamp_ns_uint32_seq_"
    "6x_ieee754_binary64_accxyz_gyrxyz")
ESTIMATOR_ROOT = Path(__file__).resolve().parents[2] / \
    "ws/fast-livo/src/FAST-LIVO2"


def _load_config(path: Path) -> Dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read ground-init config {path}: {error}") \
            from error
    if not isinstance(document, dict) or document.get("schema") != CONFIG_SCHEMA:
        raise CampaignError("wrong ground-init config schema")
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


def _seconds_to_ns(value: Any) -> int:
    decimal = Decimal(str(value)) * Decimal(1_000_000_000)
    return int(decimal.to_integral_value(rounding=ROUND_HALF_UP))


def _full_record_crop(path: Path) -> Tuple[Dict[str, Any], int, int]:
    with rosbag.Bag(str(path), "r") as bag:
        try:
            first = int(next(bag.read_messages(raw=True)).timestamp.to_nsec())
        except StopIteration as error:
            raise CampaignError(f"input bag is empty: {path}") from error
        last = first
        for _, _, stamp in bag.read_messages(raw=True):
            last = int(stamp.to_nsec())
    # extract_session_anchor uses an exclusive end boundary.
    duration_ns = last - first + 1
    return ({
        "basis": "full_bag_record_start_to_record_end_for_anchor_selection",
        "start_s": 0.0,
        "duration_s": duration_ns / 1e9,
        "full_duration_s": duration_ns / 1e9,
        "window_method": "full_bag_record_bounds",
        "smoke_truncated": False,
    }, first, last)


def _welford(samples: Sequence[Sequence[float]]) -> Tuple[List[float], List[float]]:
    if len(samples) < 2:
        raise CampaignError("sample standard deviation requires at least 2 samples")
    mean = [0.0, 0.0, 0.0]
    m2 = [0.0, 0.0, 0.0]
    count = 0
    for raw in samples:
        values = [float(value) for value in raw]
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise CampaignError("non-finite/malformed IMU vector")
        count += 1
        for axis in range(3):
            delta = values[axis] - mean[axis]
            mean[axis] += delta / float(count)
            m2[axis] += delta * (values[axis] - mean[axis])
    std = [math.sqrt(max(value / float(count - 1), 0.0)) for value in m2]
    return mean, std


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in vector))


def measurement_vector_sha256(samples: Sequence[Mapping[str, Any]]) -> str:
    digest = hashlib.sha256()
    for sample in samples:
        stamp_ns = int(sample["stamp_ns"])
        seq = int(sample["seq"])
        acc = [float.fromhex(value) for value in sample["acc_hex"]]
        gyr = [float.fromhex(value) for value in sample["gyr_hex"]]
        if not 0 <= seq <= (1 << 32) - 1:
            raise CampaignError(f"IMU seq outside uint32: {seq}")
        digest.update(struct.pack(">QI6d", stamp_ns, seq, *(acc + gyr)))
    return digest.hexdigest()


def _read_selected_measurements(
        bag_path: Path, topic: str, imu_header_subtract_s: float,
        expected: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    wanted = [(int(row["stamp_ns"]), int(row["seq"])) for row in expected]
    wanted_set = set(wanted)
    found: Dict[Tuple[int, int], Dict[str, Any]] = {}
    offset_ns = _seconds_to_ns(imu_header_subtract_s)
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, message, record_stamp in bag.read_messages(topics=[topic]):
            effective_stamp = int(message.header.stamp.to_nsec()) - offset_ns
            key = (effective_stamp, int(message.header.seq))
            if key not in wanted_set:
                continue
            if key in found:
                raise CampaignError(f"duplicate selected IMU identity {key} in {bag_path}")
            acc = [float(message.linear_acceleration.x),
                   float(message.linear_acceleration.y),
                   float(message.linear_acceleration.z)]
            gyr = [float(message.angular_velocity.x),
                   float(message.angular_velocity.y),
                   float(message.angular_velocity.z)]
            if not all(math.isfinite(value) for value in acc + gyr):
                raise CampaignError(f"non-finite selected IMU {key} in {bag_path}")
            found[key] = {
                "stamp_ns": str(effective_stamp),
                "original_header_stamp_ns": str(message.header.stamp.to_nsec()),
                "record_stamp_ns": str(record_stamp.to_nsec()),
                "seq": key[1],
                "acc_hex": [value.hex() for value in acc],
                "gyr_hex": [value.hex() for value in gyr],
            }
    if set(found) != wanted_set:
        missing = [key for key in wanted if key not in found]
        raise CampaignError(f"selected IMU measurements missing from {bag_path}: {missing}")
    return [found[key] for key in wanted]


def _validated_gate(config: Mapping[str, Any]) -> Dict[str, Any]:
    gate = config.get("stationarity_gate")
    expected = {
        "gravity_m_s2": 9.81,
        "max_abs_mean_acc_norm_error_m_s2": 0.10,
        "max_acc_sample_std_vector_norm_m_s2": 0.25,
        "max_mean_gyr_vector_norm_rad_s": 0.01,
        "max_gyr_sample_std_vector_norm_rad_s": 0.04,
        "variance_denominator": "N_minus_1",
        "accumulator": "sequential_welford_binary64_in_stamp_seq_order",
        "vector_norm": "euclidean_l2",
        "comparison": "inclusive",
        "measurement_vector_hash_encoding": MEASUREMENT_HASH_ENCODING,
    }
    if not isinstance(gate, Mapping) or dict(gate) != expected:
        raise CampaignError("ground-init stationarity gate differs from frozen contract")
    return expected


def _runtime_constant_binding(config: Mapping[str, Any]) -> Dict[str, Any]:
    binding = config.get("runtime_constant_binding")
    expected = {
        "gravity_macro": "G_m_s2",
        "expected_literal": 9.81,
        "source_relative_to_estimator_root": "include/common_lib.h",
        "source_sha256":
            "e89d7b8a80c9f9bb88663a6eaa7e4921c222b21a7ad7f898262158d307679796",
    }
    if not isinstance(binding, Mapping) or dict(binding) != expected:
        raise CampaignError("runtime gravity constant binding changed")
    source = (ESTIMATOR_ROOT /
              str(binding["source_relative_to_estimator_root"])).resolve()
    if sha256(source) != binding["source_sha256"]:
        raise CampaignError("runtime gravity constant source identity changed")
    text = source.read_text()
    if "#define G_m_s2 (9.81)" not in text:
        raise CampaignError("runtime G_m_s2 literal is not the frozen 9.81")
    return {
        **dict(binding),
        "source_path": str(source),
        "verified_against_current_estimator_source": True,
    }


def _base_extraction_config() -> Dict[str, Any]:
    # Reuse the already reviewed D435/L515 synchronization contract verbatim.
    legacy = yaml.safe_load(Path(__file__).with_name(
        "vio_postfix_init_qualification_config.yaml").read_text())
    return _validated_extraction_config(legacy)


def extract_ground_anchors(
        config: Mapping[str, Any], reference_plan: Mapping[str, Any],
        expected_arms: Sequence[Mapping[str, Any]], *,
        effective_parameters_by_session: Mapping[str, Mapping[str, Any]],
        config_identity: Optional[Mapping[str, Any]] = None,
        verify_bag_hash: bool = True) -> Dict[str, Any]:
    if config.get("schema") != CONFIG_SCHEMA or config.get("scope") != \
            "development_only" or config.get("validation_data_accessed") is not False:
        raise CampaignError("ground-init scope/config contract mismatch")
    session_ids = verify_phase_a_plan(reference_plan, expected_arms)
    if config.get("reference_phase_a_campaign_identity_sha256") != \
            reference_plan.get("identity_sha256"):
        raise CampaignError("ground-init reference campaign identity mismatch")
    gate = _validated_gate(config)
    runtime_constant = _runtime_constant_binding(config)
    selection = config.get("selection")
    expected_selection = {
        "rule": "earliest_explicit_eligible_full_sync_sensor_epoch_from_full_bag_start",
        "event_order_basis": "rosbag_record_order",
        "uses_ground_truth": False,
        "uses_estimator_output": False,
        "live_fallback_allowed": False,
        "placeholder_anchor_allowed": False,
        "init_anchor_max_predecessor_gap_s": 0.02,
        "init_sample_count": 30,
        "samples_strictly_after_anchor": True,
    }
    if not isinstance(selection, Mapping) or dict(selection) != expected_selection:
        raise CampaignError("ground-init selection differs from frozen contract")
    extraction = _base_extraction_config()
    rows: List[Dict[str, Any]] = []
    for reference_session in reference_plan["sessions"]:
        session_id = str(reference_session["id"])
        bag_path = Path(str(reference_session["input_bag"])).resolve()
        crop, first_record_ns, last_record_ns = _full_record_crop(bag_path)
        full_session = {**copy.deepcopy(dict(reference_session)), "crop": crop}
        base = extract_session_anchor(
            full_session, extraction, effective_parameters_by_session[session_id],
            verify_bag_hash=verify_bag_hash)
        anchor = copy.deepcopy(base["anchor"])
        vector = anchor["expected_first_30_strict_post_anchor_stamp_seq"]
        measurements = _read_selected_measurements(
            bag_path, base["topics"]["imu"],
            float(base["effective_clock_transforms"]["imu_header_subtract_s"]),
            vector)
        acc = [[float.fromhex(value) for value in row["acc_hex"]]
               for row in measurements]
        gyr = [[float.fromhex(value) for value in row["gyr_hex"]]
               for row in measurements]
        mean_acc, std_acc = _welford(acc)
        mean_gyr, std_gyr = _welford(gyr)
        mean_acc_norm = _norm(mean_acc)
        mean_gyr_norm = _norm(mean_gyr)
        std_acc_norm = _norm(std_acc)
        std_gyr_norm = _norm(std_gyr)
        checks = {
            "mean_acc_norm": abs(mean_acc_norm - gate["gravity_m_s2"]) <=
                gate["max_abs_mean_acc_norm_error_m_s2"],
            "acc_sample_std_vector_norm": std_acc_norm <=
                gate["max_acc_sample_std_vector_norm_m_s2"],
            "mean_gyr_vector_norm": mean_gyr_norm <=
                gate["max_mean_gyr_vector_norm_rad_s"],
            "gyr_sample_std_vector_norm": std_gyr_norm <=
                gate["max_gyr_sample_std_vector_norm_rad_s"],
        }
        if not all(checks.values()):
            raise CampaignError(
                f"{session_id}: earliest ground anchor fails stationarity: {checks}")

        cache_path = Path(str(reference_session["window_cache"])).resolve()
        cache = load_json(cache_path)
        provenance_path = Path(str(reference_session["input_provenance"])).resolve()
        provenance = load_json(provenance_path)
        source_signature = cache.get("source_signature")
        mavros_source = provenance.get("mavros_source")
        hybrid_source = provenance.get("source")
        if (cache.get("cache_version") != 4 or
                cache.get("flight_id") != session_id or
                cache.get("split") != "development" or
                sha256(cache_path) != reference_session[
                    "window_cache_sha256"] or
                sha256(provenance_path) != reference_session[
                    "input_provenance_sha256"] or
                not isinstance(source_signature, Mapping) or
                not isinstance(mavros_source, Mapping) or
                dict(source_signature) != dict(mavros_source) or
                not Path(str(source_signature.get("path", ""))).is_file() or
                Path(str(source_signature["path"])).stat().st_size !=
                source_signature["size_bytes"] or
                Path(str(source_signature["path"])).stat().st_mtime_ns !=
                source_signature["mtime_ns"] or
                provenance.get("schema") != "fastlivo_hybrid_imu_sidecar/v1" or
                provenance.get("copy_all_source_topics") is not True or
                provenance.get("topics", {}).get("output") !=
                "/camera/imu_hybrid" or
                provenance.get("output", {}).get("path") != str(bag_path) or
                provenance.get("output", {}).get("sha256") !=
                reference_session["input_declared_sha256"] or
                provenance.get("output", {}).get("size_bytes") !=
                bag_path.stat().st_size or
                not isinstance(hybrid_source, Mapping) or
                not Path(str(hybrid_source.get("path", ""))).is_file() or
                Path(str(hybrid_source["path"])).stat().st_size !=
                hybrid_source.get("size_bytes") or
                Path(str(hybrid_source["path"])).stat().st_mtime_ns !=
                hybrid_source.get("mtime_ns")):
            raise CampaignError(
                f"{session_id}: cache/hybrid source provenance mismatch")
        windows = cache.get("windows", {})
        takeoff_ns = _seconds_to_ns(windows["events"]["takeoff"])
        hover_start_ns = _seconds_to_ns(windows["hover"]["start"])
        landing_ns = _seconds_to_ns(windows["hover"]["end"])
        full_start_ns = _seconds_to_ns(windows["full"]["start"])
        last_init_ns = int(vector[-1]["stamp_ns"])
        takeoff_lead_ns = takeoff_ns - last_init_ns
        hover_lead_ns = hover_start_ns - int(anchor["anchor_stamp_ns"])
        audit_config = config["post_selection_ground_hard_gate"]
        audit_checks = {
            "30th_sample_to_takeoff_lead": takeoff_lead_ns >= _seconds_to_ns(
                audit_config["minimum_30th_sample_to_takeoff_lead_s"]),
            "anchor_to_hover_score_start_lead": hover_lead_ns >= _seconds_to_ns(
                audit_config["minimum_anchor_to_hover_score_start_lead_s"]),
        }
        if not all(audit_checks.values()):
            raise CampaignError(
                f"{session_id}: post-selection ground audit failed: {audit_checks}")
        # rosbag play -s/-u is relative to the derived bag record-time origin,
        # not the raw cache's nominal full.start epoch.
        replay_end_offset_ns = landing_ns - first_record_ns
        if replay_end_offset_ns <= 0:
            raise CampaignError(f"{session_id}: invalid full-start-to-landing window")
        rows.append({
            "session_id": session_id,
            "condition": reference_session.get("condition"),
            "split": "development",
            "input": {
                "path": str(bag_path),
                "size_bytes": bag_path.stat().st_size,
                "declared_sha256": reference_session["input_declared_sha256"],
                "verified_sha256": base["input"]["verified_sha256"],
                "full_file_sha256_verified": verify_bag_hash,
                "input_provenance_sha256": reference_session[
                    "input_provenance_sha256"],
            },
            "full_bag_record_bounds": {
                "first_record_stamp_ns": str(first_record_ns),
                "last_record_stamp_ns": str(last_record_ns),
            },
            "topics": base["topics"],
            "effective_clock_transforms": base["effective_clock_transforms"],
            "effective_lidar_validation": base["effective_lidar_validation"],
            "effective_parameter_source": base["effective_parameter_source"],
            "anchor": anchor,
            "stationarity": {
                "sample_count": len(measurements),
                "sample_order": "expected_strict_post_anchor_stamp_seq_order",
                "variance_denominator": "N_minus_1",
                "accumulator": "sequential_welford_binary64_in_stamp_seq_order",
                "measurement_vector": measurements,
                "measurement_vector_sha256": measurement_vector_sha256(measurements),
                "measurement_vector_hash_encoding": MEASUREMENT_HASH_ENCODING,
                "mean_acc": mean_acc,
                "mean_acc_norm_m_s2": mean_acc_norm,
                "acc_sample_std": std_acc,
                "acc_sample_std_vector_norm_m_s2": std_acc_norm,
                "mean_gyr": mean_gyr,
                "mean_gyr_vector_norm_rad_s": mean_gyr_norm,
                "gyr_sample_std": std_gyr,
                "gyr_sample_std_vector_norm_rad_s": std_gyr_norm,
                "checks": checks,
                "accepted": True,
            },
            "post_selection_ground_hard_gate": {
                "uses_ground_truth_only_after_anchor_selection": True,
                "affects_anchor_selection": False,
                "window_cache": str(cache_path),
                "window_cache_sha256": sha256(cache_path),
                "cache_version": cache["cache_version"],
                "cache_source_signature": copy.deepcopy(dict(source_signature)),
                "hybrid_provenance": {
                    "path": str(provenance_path),
                    "sha256": sha256(provenance_path),
                    "schema": provenance["schema"],
                    "copy_all_source_topics": True,
                    "record_time_origin": "copied_from_canonical_source_bag",
                    "limitation": (
                        "hybrid bag carries copied canonical record times; raw "
                        "MAVROS identity is bound through the signed sidecar and "
                        "frozen cache, not re-derived from a MAVROS topic in the "
                        "hybrid bag"),
                    "canonical_source_signature": copy.deepcopy(
                        dict(hybrid_source)),
                    "mavros_source_signature": copy.deepcopy(
                        dict(mavros_source)),
                    "generator": {
                        "path": str(Path(__file__).with_name(
                            "make_hybrid_imu_bag.py").resolve()),
                        "sha256": sha256(Path(__file__).with_name(
                            "make_hybrid_imu_bag.py")),
                    },
                },
                "takeoff_record_epoch_ns": str(takeoff_ns),
                "takeoff_epoch_origin": "mavros_record_time_from_frozen_cache",
                "hover_score_start_epoch_ns": str(hover_start_ns),
                "hover_score_start_epoch_origin":
                    "gt_pose_header_time_from_frozen_cache",
                "landing_record_epoch_ns": str(landing_ns),
                "landing_epoch_origin": "mavros_record_time_from_frozen_cache",
                "30th_sample_to_takeoff_lead_ns": str(takeoff_lead_ns),
                "anchor_to_hover_score_start_lead_ns": str(hover_lead_ns),
                "checks": audit_checks,
                "passed": True,
            },
            "windows": {
                "replay": {
                    "start_offset_s": 0.0,
                    "duration_s": replay_end_offset_ns / 1e9,
                    "end_definition": "frozen_cached_landing",
                },
                "primary_evaluation": {
                    "interval": "complete_recorded_ground_to_landing_result",
                },
                "secondary_hover_evaluation": {
                    "start_absolute_ros_epoch_ns": str(hover_start_ns),
                    "start_epoch_origin":
                        "gt_pose_header_time_from_frozen_cache",
                    "end_absolute_ros_epoch_ns": str(landing_ns),
                    "end_epoch_origin":
                        "mavros_landing_record_time_from_frozen_cache",
                    "mask_basis": "result_stream_sensor_header_epoch",
                    "interpretation":
                        "absolute_ros_epoch_numeric_mask_with_mixed_frozen_sources",
                    "boundary": "start_inclusive_end_inclusive",
                    "purpose": "phase_a_ranking_compatibility_only",
                },
            },
        })
    if [row["session_id"] for row in rows] != session_ids or len(rows) != 5:
        raise CampaignError("ground anchor artifact does not cover exact dev grid")
    predecessor = config["predecessor_qualification"]
    predecessor_path = Path(__file__).resolve().parents[2] / predecessor["path"]
    predecessor_report = load_json(predecessor_path)
    if (sha256(predecessor_path) != predecessor["sha256"] or
            predecessor_report.get("status") != predecessor["required_status"]):
        raise CampaignError("predecessor qualification FAIL artifact changed")
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "selection_uses_ground_truth": False,
        "selection_uses_estimator_output": False,
        "post_selection_ground_truth_audit_only": True,
        "reference_phase_a_campaign_id": reference_plan["campaign_id"],
        "reference_phase_a_campaign_identity_sha256": reference_plan[
            "identity_sha256"],
        "config_identity": dict(config_identity or {
            "object_sha256": object_sha256(config)}),
        "frozen_phase_a_arms_sha256": object_sha256(expected_arms),
        "predecessor_qualification": {
            "path": str(predecessor_path.resolve()),
            "sha256": predecessor["sha256"],
            "status": predecessor_report["status"],
            "identity_sha256": predecessor_report.get("identity_sha256"),
            "remains_authoritative_for_high_rate_interface": True,
            "superseded": False,
        },
        "selection_contract": copy.deepcopy(dict(selection)),
        "stationarity_gate": gate,
        "runtime_constant_binding": runtime_constant,
        "post_selection_ground_hard_gate_contract": copy.deepcopy(
            dict(config["post_selection_ground_hard_gate"])),
        "window_contract": copy.deepcopy(dict(config["windows"])),
        "session_count": len(rows),
        "session_ids": session_ids,
        "sessions": rows,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--reference-campaign", type=Path,
                        default=DEFAULT_REFERENCE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--skip-bag-sha", action="store_true",
                        help="tests only; artifact records that full SHA was not verified")
    arguments = parser.parse_args(argv)
    try:
        config_path = arguments.config.resolve()
        config = _load_config(config_path)
        reference_dir = arguments.reference_campaign.resolve()
        reference_plan = load_json(reference_dir / "campaign.json")
        validate_plan_identity(reference_plan)
        expected_arms = load_arms(PHASE_A_ARMS)
        session_ids = verify_phase_a_plan(reference_plan, expected_arms)
        effective_parameters = load_frozen_effective_parameters(
            reference_dir, reference_plan, session_ids,
            _base_extraction_config())
        artifact = extract_ground_anchors(
            config, reference_plan, expected_arms,
            effective_parameters_by_session=effective_parameters,
            config_identity={
                "path": str(config_path),
                "size_bytes": config_path.stat().st_size,
                "sha256": sha256(config_path),
                "object_sha256": object_sha256(config),
            },
            verify_bag_hash=not arguments.skip_bag_sha)
        _write_json_exclusive(arguments.output, artifact)
        print(json.dumps({
            "output": str(arguments.output.resolve()),
            "identity_sha256": artifact["identity_sha256"],
            "session_count": artifact["session_count"],
            "sessions": [{
                "session_id": row["session_id"],
                "anchor_stamp_ns": row["anchor"]["anchor_stamp_ns"],
                "mean_acc_norm_m_s2": row["stationarity"][
                    "mean_acc_norm_m_s2"],
                "acc_sample_std_vector_norm_m_s2": row["stationarity"][
                    "acc_sample_std_vector_norm_m_s2"],
                "mean_gyr_vector_norm_rad_s": row["stationarity"][
                    "mean_gyr_vector_norm_rad_s"],
                "gyr_sample_std_vector_norm_rad_s": row["stationarity"][
                    "gyr_sample_std_vector_norm_rad_s"],
                "30th_sample_to_takeoff_lead_ns": row[
                    "post_selection_ground_hard_gate"][
                        "30th_sample_to_takeoff_lead_ns"],
            } for row in artifact["sessions"]],
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, rosbag.ROSBagException,
            yaml.YAMLError, KeyError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

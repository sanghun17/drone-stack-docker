#!/usr/bin/env python3
"""Extract explicit IMU-init anchors from frozen Phase-A input bags.

The extractor is read-only and models the first LIVO image epoch covered by
both LiDAR and IMU sensor-time watermarks in rosbag record order.  It applies
the effective FAST-LIVO clock transforms before comparing epochs.  All five
frozen development sessions are emitted into one self-hashed artifact; live
fallback and placeholder anchors are not representable by the schema.

For the D435/L515 input used here, every retained depth point has zero relative
scan time, so the LiDAR watermark is its transformed header epoch.  The
configuration names and freezes that rule explicitly rather than guessing it
from the message at runtime.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import genpy
import rosbag
from sensor_msgs import point_cloud2
import yaml

from generate_vio_postfix_init_qualification_plan import (
    CONFIG_SCHEMA,
    DEFAULT_CONFIG,
    DEFAULT_REFERENCE,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    COMPLETION_SCHEMA,
    RUN_SCHEMA,
    load_arms,
    load_json,
    object_sha256,
    sha256,
    validate_plan_identity,
)
from select_vio_flight_tuning_phase_a import PHASE_A_ARMS
from select_vio_flight_tuning_phase_b import verify_phase_a_plan


SCHEMA = "fastlivo_earliest_full_sync_anchors/v1"


def stamp_seq_sha256(vector: Sequence[Mapping[str, Any]]) -> str:
    payload = "".join(
        f"{sample['stamp_ns']},{int(sample['seq'])}\n" for sample in vector)
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


def _time(ns: int) -> genpy.Time:
    return genpy.Time(int(ns // 1_000_000_000), int(ns % 1_000_000_000))


def _duration_ns(seconds: float) -> int:
    if not math.isfinite(seconds) or seconds < 0.0:
        raise CampaignError(f"invalid duration: {seconds!r}")
    whole = math.floor(seconds)
    # ros::Duration::fromSec ultimately uses round-to-nearest for nanoseconds.
    nanos = math.floor((seconds - whole) * 1e9 + 0.5)
    return int(whole) * 1_000_000_000 + int(nanos)


def _ros_from_double_ns(seconds: float) -> int:
    """Mirror positive ros::Time::fromSec(double), including epoch rounding."""
    if not math.isfinite(seconds) or seconds <= 0.0:
        raise CampaignError(f"invalid transformed sensor epoch: {seconds!r}")
    whole = math.floor(seconds)
    nanos = math.floor((seconds - whole) * 1e9 + 0.5)
    whole += nanos // 1_000_000_000
    nanos %= 1_000_000_000
    return int(whole) * 1_000_000_000 + int(nanos)


def _header_to_sec(stamp: Any) -> float:
    return float(stamp.secs) + 1e-9 * float(stamp.nsecs)


def _load_config(path: Path) -> Dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read anchor config {path}: {error}") from error
    if not isinstance(document, dict) or document.get("schema") != CONFIG_SCHEMA:
        raise CampaignError("wrong anchor-extraction config schema")
    return document


def _get_nested(document: Mapping[str, Any], dotted: str) -> Any:
    value: Any = document
    for key in dotted.split("."):
        if not isinstance(value, Mapping) or key not in value:
            raise CampaignError(f"effective parameter snapshot lacks {dotted}")
        value = value[key]
    return value


def _derived_effective_parameters(
        snapshot: Mapping[str, Any], extraction: Mapping[str, Any]) -> Dict[str, Any]:
    paths = extraction["effective_parameter_source"]["paths"]
    values = {name: _get_nested(snapshot, dotted)
              for name, dotted in paths.items()}
    topics = {
        "image": values["image_topic"],
        "lidar": values["lidar_topic"],
        "imu": values["imu_topic"],
    }
    if any(not isinstance(value, str) or not value.startswith("/")
           for value in topics.values()):
        raise CampaignError("effective sensor topics must be absolute ROS names")
    numeric_names = (
        "image_header_add_s", "image_exposure_add_s",
        "lidar_header_add_s", "imu_header_subtract_s", "blind_m",
    )
    if any(not isinstance(values[name], (int, float)) or
           isinstance(values[name], bool) or
           not math.isfinite(float(values[name])) for name in numeric_names):
        raise CampaignError("effective clock/blind parameters must be finite")
    if values["ros_driver_bug_fix"] is not False:
        raise CampaignError(
            "anchor extractor refuses the stateful ros_driver_bug_fix transform")
    lidar_type = values["lidar_type"]
    point_filter_num = values["point_filter_num"]
    if (not isinstance(lidar_type, int) or isinstance(lidar_type, bool) or
            lidar_type != extraction["lidar_validation"]["required_lidar_type"] or
            not isinstance(point_filter_num, int) or
            isinstance(point_filter_num, bool) or point_filter_num <= 0 or
            float(values["blind_m"]) < 0.0):
        raise CampaignError("effective L515 preprocessing parameters are invalid")
    return {
        "topics": topics,
        "effective_clock_transforms": {
            "image_header_add_s": float(values["image_header_add_s"]),
            "image_exposure_add_s": float(values["image_exposure_add_s"]),
            "lidar_header_add_s": float(values["lidar_header_add_s"]),
            "lidar_scan_end_rule": extraction["lidar_validation"][
                "scan_end_rule"],
            "imu_header_subtract_s": float(values["imu_header_subtract_s"]),
            "ros_driver_bug_fix": False,
        },
        "lidar_validation": {
            "lidar_type": lidar_type,
            "point_filter_num": point_filter_num,
            "blind_m": float(values["blind_m"]),
            "minimum_retained_points": extraction["lidar_validation"][
                "minimum_retained_points"],
        },
    }


def load_frozen_effective_parameters(
        campaign_dir: Path, plan: Mapping[str, Any],
        session_ids: Sequence[str], extraction: Mapping[str, Any]
        ) -> Dict[str, Dict[str, Any]]:
    """Derive clock/topic/preprocessing inputs from hashed baseline snapshots."""
    campaign_dir = campaign_dir.resolve()
    arm_id = extraction["effective_parameter_source"]["arm_id"]
    artifact_name = extraction["effective_parameter_source"]["artifact"]
    result: Dict[str, Dict[str, Any]] = {}
    for session_id in session_ids:
        pointer_path = campaign_dir / "completed" / arm_id / f"{session_id}.json"
        pointer = load_json(pointer_path)
        exact_pointer = {
            "schema": COMPLETION_SCHEMA,
            "campaign_identity_sha256": plan["identity_sha256"],
            "arm_id": arm_id,
            "session_id": session_id,
        }
        for field, expected in exact_pointer.items():
            if pointer.get(field) != expected:
                raise CampaignError(
                    f"effective-parameter completion mismatch {session_id}/{field}")
        relative = Path(str(pointer.get("attempt", "")))
        if relative.is_absolute() or ".." in relative.parts:
            raise CampaignError(f"unsafe baseline attempt path: {relative}")
        attempt = (campaign_dir / relative).resolve()
        try:
            attempt.relative_to(campaign_dir)
        except ValueError as error:
            raise CampaignError(f"baseline attempt escapes campaign: {attempt}") \
                from error
        manifest_path = attempt / "manifest.json"
        if sha256(manifest_path) != pointer.get("manifest_sha256"):
            raise CampaignError(f"baseline manifest hash changed: {manifest_path}")
        manifest = load_json(manifest_path)
        if (manifest.get("schema") != RUN_SCHEMA or
                manifest.get("state") != "complete" or
                manifest.get("campaign_identity_sha256") !=
                plan["identity_sha256"] or
                manifest.get("arm_id") != arm_id or
                manifest.get("session_id") != session_id):
            raise CampaignError(f"baseline manifest identity mismatch: {session_id}")
        identity = manifest.get("artifacts", {}).get(artifact_name)
        snapshot_path = attempt / artifact_name
        if (not isinstance(identity, Mapping) or not snapshot_path.is_file() or
                snapshot_path.stat().st_size != identity.get("size_bytes") or
                sha256(snapshot_path) != identity.get("sha256")):
            raise CampaignError(
                f"hashed effective parameter snapshot changed: {snapshot_path}")
        try:
            snapshot = yaml.safe_load(snapshot_path.read_text())
        except (OSError, yaml.YAMLError) as error:
            raise CampaignError(
                f"cannot read effective parameter snapshot {snapshot_path}: {error}") \
                from error
        if not isinstance(snapshot, Mapping):
            raise CampaignError(f"malformed effective parameter snapshot: {snapshot_path}")
        result[session_id] = {
            "parameters": _derived_effective_parameters(snapshot, extraction),
            "source": {
                "arm_id": arm_id,
                "completion_pointer": str(pointer_path),
                "completion_manifest_sha256": pointer["manifest_sha256"],
                "attempt": str(attempt),
                "artifact": str(snapshot_path),
                "artifact_size_bytes": snapshot_path.stat().st_size,
                "artifact_sha256": identity["sha256"],
            },
        }
    return result


def _validated_extraction_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    raw = config.get("anchor_extraction")
    if not isinstance(raw, Mapping):
        raise CampaignError("config has no anchor_extraction mapping")
    expected = {
        "effective_parameter_source": {
            "arm_id": "baseline_acc10_img1000_out1000",
            "artifact": "result_params.yaml",
            "paths": {
                "image_topic": "common.img_topic",
                "lidar_topic": "common.lid_topic",
                "imu_topic": "common.imu_topic",
                "image_header_add_s": "time_offset.img_time_offset",
                "image_exposure_add_s": "time_offset.exposure_time_init",
                "lidar_header_add_s": "time_offset.lidar_time_offset",
                "imu_header_subtract_s": "time_offset.imu_time_offset",
                "ros_driver_bug_fix": "common.ros_driver_bug_fix",
                "lidar_type": "preprocess.lidar_type",
                "point_filter_num": "preprocess.point_filter_num",
                "blind_m": "preprocess.blind",
            },
        },
        "expected_types": {
            "image": "sensor_msgs/Image",
            "lidar": "sensor_msgs/PointCloud2",
            "imu": "sensor_msgs/Imu",
        },
        "image_filters": {
            "hilti_decimation_enabled": False,
            "duplicate_epsilon_s": 0.001,
            "minimum_interframe_s": 0.02,
        },
        "lidar_validation": {
            "required_lidar_type": 4,
            "scan_end_rule": "l515_zero_point_offset",
            "minimum_retained_points": 2,
        },
        "synchronization": {
            "mode": "LIVO",
            "initial_image_before_first_lidar_epsilon_s": 0.00001,
            "anchor_definition":
                "earliest_explicit_eligible_full_sync_sensor_epoch",
            "event_order_basis": "rosbag_record_order_within_frozen_crop",
            "require_predecessor_imu_at_or_before_anchor": True,
            "init_anchor_max_predecessor_gap_s": 0.02,
            "require_successor_imu_strictly_after_anchor": True,
        },
    }
    if dict(raw) != expected:
        raise CampaignError(
            "anchor extraction config differs from the frozen D435/L515 contract")
    return expected


def _pointcloud_has_minimum_retained(
        message: Any, point_filter_num: int, blind_m: float,
        minimum: int) -> bool:
    if int(message.width) * int(message.height) <= 1:
        return False
    retained = 0
    try:
        for index, coordinates in enumerate(point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=False)):
            if index % point_filter_num:
                continue
            x, y, z = (float(value) for value in coordinates)
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            if x * x + y * y + z * z < blind_m * blind_m:
                continue
            retained += 1
            if retained >= minimum:
                return True
    except (KeyError, ValueError, TypeError) as error:
        raise CampaignError(f"cannot validate L515 point cloud: {error}") from error
    return False


def _finite_imu(message: Any) -> bool:
    values = (
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
    )
    return all(math.isfinite(float(value)) for value in values)


def _first_record_stamp_ns(bag: rosbag.Bag) -> int:
    try:
        return int(next(bag.read_messages(raw=True)).timestamp.to_nsec())
    except StopIteration as error:
        raise CampaignError("input bag is empty") from error


def extract_session_anchor(
        session: Mapping[str, Any], extraction: Mapping[str, Any],
        effective_parameters: Mapping[str, Any],
        *, verify_bag_hash: bool = True) -> Dict[str, Any]:
    bag_path = Path(str(session["input_bag"])).resolve()
    if not bag_path.is_file():
        raise CampaignError(f"missing input bag: {bag_path}")
    declared_hash = str(session.get("input_declared_sha256", ""))
    actual_hash = sha256(bag_path) if verify_bag_hash else declared_hash
    if actual_hash != declared_hash:
        raise CampaignError(f"input bag SHA-256 mismatch: {bag_path}")

    parameters = effective_parameters.get("parameters")
    source = effective_parameters.get("source")
    if not isinstance(parameters, Mapping) or not isinstance(source, Mapping):
        raise CampaignError(f"{session['id']}: missing effective parameters/provenance")
    topics = parameters["topics"]
    topic_names = [topics["image"], topics["lidar"], topics["imu"]]
    transforms = parameters["effective_clock_transforms"]
    image_filters = extraction["image_filters"]
    lidar_validation = parameters["lidar_validation"]
    sync = extraction["synchronization"]

    with rosbag.Bag(str(bag_path), "r") as bag:
        inventory = bag.get_type_and_topic_info().topics
        for kind, topic in topics.items():
            info = inventory.get(topic)
            expected_type = extraction["expected_types"][kind]
            if info is None or info.msg_type != expected_type:
                raise CampaignError(
                    f"{session['id']}: {topic} has type "
                    f"{None if info is None else info.msg_type!r}, expected {expected_type!r}")

        bag_start_ns = _first_record_stamp_ns(bag)
        crop = session.get("crop")
        if not isinstance(crop, Mapping):
            raise CampaignError(f"{session['id']}: missing frozen crop")
        crop_start_offset_ns = _duration_ns(float(crop["start_s"]))
        crop_duration_ns = _duration_ns(float(crop["duration_s"]))
        crop_start_ns = bag_start_ns + crop_start_offset_ns
        crop_end_ns = crop_start_ns + crop_duration_ns

        per_topic_index = {topic: 0 for topic in topic_names}
        relevant_event_index = 0
        last_image_header_time = -1.0
        first_lidar_effective_time: Optional[float] = None
        latest_lidar: Optional[Dict[str, Any]] = None
        latest_imu: Optional[Dict[str, Any]] = None
        images: List[Dict[str, Any]] = []
        imu_entries: List[Dict[str, Any]] = []
        anchor: Optional[Dict[str, Any]] = None
        expected_init_vector: Optional[List[Dict[str, Any]]] = None
        ignored = {
            "duplicate_or_too_close_image": 0,
            "backward_image": 0,
            "backward_imu": 0,
            "degenerate_lidar": 0,
            "image_before_initial_lidar_epoch": 0,
        }
        skipped_explicit_ineligible_images: List[Dict[str, Any]] = []

        for topic, message, record_stamp in bag.read_messages(
                topics=topic_names, start_time=_time(crop_start_ns),
                end_time=_time(crop_end_ns)):
            record_ns = int(record_stamp.to_nsec())
            topic_index = per_topic_index[topic]
            per_topic_index[topic] += 1
            event_index = relevant_event_index
            relevant_event_index += 1

            if topic == topics["image"]:
                header_ns = int(message.header.stamp.to_nsec())
                header_effective = (
                    _header_to_sec(message.header.stamp) +
                    float(transforms["image_header_add_s"]))
                if abs(header_effective - last_image_header_time) < float(
                        image_filters["duplicate_epsilon_s"]):
                    ignored["duplicate_or_too_close_image"] += 1
                    continue
                if header_effective < last_image_header_time:
                    ignored["backward_image"] += 1
                    continue
                if (header_effective - last_image_header_time <
                        float(image_filters["minimum_interframe_s"])):
                    ignored["duplicate_or_too_close_image"] += 1
                    continue
                last_image_header_time = header_effective
                capture_time = (
                    header_effective +
                    float(transforms["image_exposure_add_s"]))
                images.append({
                    "header_stamp_ns": str(header_ns),
                    "effective_header_stamp_ns": str(
                        _ros_from_double_ns(header_effective)),
                    "capture_time": capture_time,
                    "image_epoch_ns": str(_ros_from_double_ns(capture_time)),
                    "record_stamp_ns": str(record_ns),
                    "header_seq": int(message.header.seq),
                    "topic_index_within_crop": topic_index,
                    "relevant_event_index_within_crop": event_index,
                })

            elif topic == topics["lidar"]:
                if not _pointcloud_has_minimum_retained(
                        message,
                        int(lidar_validation["point_filter_num"]),
                        float(lidar_validation["blind_m"]),
                        int(lidar_validation["minimum_retained_points"])):
                    ignored["degenerate_lidar"] += 1
                    continue
                effective = (
                    _header_to_sec(message.header.stamp) +
                    float(transforms["lidar_header_add_s"]))
                latest_lidar = {
                    "header_stamp_ns": str(message.header.stamp.to_nsec()),
                    "watermark_time": effective,
                    "watermark_ns": str(_ros_from_double_ns(effective)),
                    "record_stamp_ns": str(record_ns),
                    "header_seq": int(message.header.seq),
                    "topic_index_within_crop": topic_index,
                    "relevant_event_index_within_crop": event_index,
                }
                if first_lidar_effective_time is None:
                    first_lidar_effective_time = effective

            else:
                effective = (
                    _header_to_sec(message.header.stamp) -
                    float(transforms["imu_header_subtract_s"]))
                effective_ns = _ros_from_double_ns(effective)
                if (latest_imu is not None and
                        effective <= latest_imu["watermark_time"]):
                    relation = ("duplicate" if
                                effective == latest_imu["watermark_time"] else
                                "backward")
                    raise CampaignError(
                        f"{session['id']}: {relation} IMU sensor stamp in "
                        "initialization prefix; runtime fails closed")
                latest_imu = {
                    "original_header_stamp_ns": str(message.header.stamp.to_nsec()),
                    "watermark_time": effective,
                    "watermark_ns": str(effective_ns),
                    "record_stamp_ns": str(record_ns),
                    "header_seq": int(message.header.seq),
                    "topic_index_within_crop": topic_index,
                    "relevant_event_index_within_crop": event_index,
                    "finite_measurement": _finite_imu(message),
                }
                imu_entries.append(latest_imu)

            if anchor is None and first_lidar_effective_time is not None:
                epsilon = float(
                    sync["initial_image_before_first_lidar_epsilon_s"])
                while (images and
                       images[0]["capture_time"] <
                       first_lidar_effective_time + epsilon):
                    images.pop(0)
                    ignored["image_before_initial_lidar_epoch"] += 1
                while (images and latest_lidar is not None and
                       latest_imu is not None and
                       images[0]["capture_time"] <=
                       latest_lidar["watermark_time"] and
                       images[0]["capture_time"] <=
                       latest_imu["watermark_time"] and anchor is None):
                    selected_image = images[0]
                    anchor_ns = int(selected_image["image_epoch_ns"])
                    predecessors = [entry for entry in imu_entries
                                    if int(entry["watermark_ns"]) <= anchor_ns]
                    predecessor = predecessors[-1] if predecessors else None
                    predecessor_ns = (
                        int(predecessor["watermark_ns"])
                        if predecessor is not None else None)
                    predecessor_gap_ns = (
                        anchor_ns - predecessor_ns
                        if predecessor_ns is not None else None)
                    maximum_gap_ns = _duration_ns(float(
                        sync["init_anchor_max_predecessor_gap_s"]))
                    reason = None
                    if predecessor is None:
                        reason = "no_received_imu_at_or_before_image_epoch"
                    elif predecessor_gap_ns is None or \
                            predecessor_gap_ns > maximum_gap_ns:
                        reason = "imu_predecessor_gap_exceeds_frozen_limit"
                    if reason is not None:
                        skipped_explicit_ineligible_images.append({
                            "reason": reason,
                            "image_epoch_ns": str(anchor_ns),
                            "image_header_stamp_ns":
                                selected_image["header_stamp_ns"],
                            "image_record_stamp_ns":
                                selected_image["record_stamp_ns"],
                            "lidar_watermark_ns":
                                latest_lidar["watermark_ns"],
                            "imu_watermark_ns": latest_imu["watermark_ns"],
                            "predecessor_imu_stamp_ns": (
                                predecessor["watermark_ns"]
                                if predecessor is not None else None),
                            "predecessor_gap_ns": (
                                str(predecessor_gap_ns)
                                if predecessor_gap_ns is not None else None),
                            "frozen_maximum_predecessor_gap_ns":
                                str(maximum_gap_ns),
                            "coverage_relevant_event_index_within_crop":
                                event_index,
                        })
                        images.pop(0)
                        continue
                    anchor = {
                        "anchor_definition":
                            "earliest_explicit_eligible_full_sync_sensor_epoch",
                        "anchor_mode_required": "explicit",
                        "anchor_stamp_ns": str(anchor_ns),
                        "image_epoch_ns": str(anchor_ns),
                        "image": {key: value for key, value in selected_image.items()
                                  if key != "capture_time"},
                        "lidar_watermark": {key: value for key, value in latest_lidar.items()
                                            if key != "watermark_time"},
                        "imu_watermark_at_coverage": {
                            key: value for key, value in latest_imu.items()
                            if key not in {"watermark_time", "finite_measurement"}
                        },
                        "predecessor_imu": {
                            key: value for key, value in predecessor.items()
                            if key not in {"watermark_time", "finite_measurement"}
                        },
                        "predecessor_gap_ns": str(predecessor_gap_ns),
                        "init_anchor_max_predecessor_gap_s": float(
                            sync["init_anchor_max_predecessor_gap_s"]),
                        "init_anchor_max_predecessor_gap_ns": str(
                            maximum_gap_ns),
                        "coverage_relevant_event_index_within_crop": event_index,
                        "messages_seen_through_coverage": dict(per_topic_index),
                    }

            if anchor is not None:
                anchor_ns = int(anchor["anchor_stamp_ns"])
                post_anchor = []
                last_stamp = None
                for entry in imu_entries:
                    stamp_ns = int(entry["watermark_ns"])
                    if stamp_ns <= anchor_ns or not entry["finite_measurement"]:
                        continue
                    if last_stamp is not None and stamp_ns <= last_stamp:
                        raise CampaignError(
                            f"{session['id']}: non-increasing post-anchor IMU "
                            "stamp; runtime fails closed")
                    post_anchor.append({
                        "stamp_ns": str(stamp_ns),
                        "seq": entry["header_seq"],
                    })
                    last_stamp = stamp_ns
                    if len(post_anchor) == 30:
                        break
                if len(post_anchor) == 30:
                    expected_init_vector = post_anchor
                    break

        if anchor is None:
            raise CampaignError(
                f"{session['id']}: no full-sync anchor in frozen crop")
        if expected_init_vector is None:
            raise CampaignError(
                f"{session['id']}: fewer than 30 valid strict post-anchor IMUs")
        anchor_ns = int(anchor["anchor_stamp_ns"])
        successor = next(
            entry for entry in imu_entries
            if int(entry["watermark_ns"]) > anchor_ns)
        anchor["successor_imu"] = {
            key: value for key, value in successor.items()
            if key not in {"watermark_time", "finite_measurement"}
        }
        anchor["successor_gap_ns"] = str(
            int(successor["watermark_ns"]) - anchor_ns)
        anchor["expected_first_30_strict_post_anchor_stamp_seq"] = \
            expected_init_vector
        anchor["expected_first_30_stamp_seq_sha256"] = stamp_seq_sha256(
            expected_init_vector)
        anchor["expected_first_30_stamp_seq_hash_encoding"] = \
            "utf8_lines_stamp_ns_comma_seq_newline"
        anchor["expected_first_used_stamp_ns"] = expected_init_vector[0]["stamp_ns"]
        anchor["expected_first_used_seq"] = expected_init_vector[0]["seq"]
        anchor["expected_last_used_stamp_ns"] = expected_init_vector[-1]["stamp_ns"]
        anchor["expected_last_used_seq"] = expected_init_vector[-1]["seq"]
        anchor["messages_seen_through_30th_post_anchor_imu"] = dict(
            per_topic_index)

    return {
        "session_id": session["id"],
        "condition": session.get("condition"),
        "split": "development",
        "input": {
            "path": str(bag_path),
            "size_bytes": bag_path.stat().st_size,
            "declared_sha256": declared_hash,
            "verified_sha256": actual_hash,
            "full_file_sha256_verified": verify_bag_hash,
            "input_provenance_sha256": session["input_provenance_sha256"],
        },
        "crop": {
            **dict(crop),
            "bag_begin_record_stamp_ns": str(bag_start_ns),
            "crop_start_record_stamp_ns": str(crop_start_ns),
            "crop_end_record_stamp_ns": str(crop_end_ns),
            "boundary_semantics": "start_inclusive_end_exclusive",
        },
        "topics": dict(topics),
        "effective_clock_transforms": dict(transforms),
        "effective_lidar_validation": dict(lidar_validation),
        "effective_parameter_source": dict(source),
        "synchronization": dict(sync),
        "anchor": anchor,
        "ignored_before_selection": ignored,
        "skipped_explicit_ineligible_image_count": len(
            skipped_explicit_ineligible_images),
        "skipped_explicit_ineligible_images":
            skipped_explicit_ineligible_images,
    }


def extract_anchors(
        config: Mapping[str, Any], reference_plan: Mapping[str, Any],
        expected_arms: Sequence[Mapping[str, Any]], *,
        effective_parameters_by_session: Mapping[str, Mapping[str, Any]],
        config_identity: Optional[Mapping[str, Any]] = None,
        verify_plan_identity: bool = True,
        verify_bag_hash: bool = True) -> Dict[str, Any]:
    extraction = _validated_extraction_config(config)
    session_ids = verify_phase_a_plan(
        reference_plan, expected_arms, verify_identity=verify_plan_identity)
    if config.get(
            "reference_phase_a_campaign_identity_sha256") != reference_plan.get(
                "identity_sha256"):
        raise CampaignError("anchor config/reference campaign mismatch")
    sessions = reference_plan.get("sessions")
    if not isinstance(sessions, list) or [row.get("id") for row in sessions] != session_ids:
        raise CampaignError("reference session order changed")
    extracted = [
        extract_session_anchor(
            session, extraction,
            effective_parameters_by_session[str(session["id"])],
            verify_bag_hash=verify_bag_hash)
        for session in sessions
    ]
    if len(extracted) != 5 or len({row["session_id"] for row in extracted}) != 5:
        raise CampaignError("anchor artifact must contain all five dev sessions")
    core: Dict[str, Any] = {
        "schema": SCHEMA,
        "scope": "development_only",
        "validation_data_accessed": False,
        "placeholder_anchors_allowed": False,
        "live_fallback_allowed": False,
        "reference_phase_a_campaign_id": reference_plan.get("campaign_id"),
        "reference_phase_a_campaign_identity_sha256": reference_plan.get(
            "identity_sha256"),
        "config_identity": dict(config_identity or {
            "object_sha256": object_sha256(config),
        }),
        "frozen_phase_a_arms_sha256": object_sha256(expected_arms),
        "session_count": len(extracted),
        "session_ids": session_ids,
        "extraction_contract": extraction,
        "sessions": extracted,
    }
    return {**core, "identity_sha256": object_sha256(core)}


def _write_new(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2, sort_keys=True,
                          ensure_ascii=False) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--reference-campaign", type=Path,
                        default=DEFAULT_REFERENCE)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        config_path = arguments.config.resolve()
        config = _load_config(config_path)
        reference = arguments.reference_campaign.resolve()
        plan = load_json(reference / "campaign.json")
        validate_plan_identity(plan)
        expected_arms = load_arms(PHASE_A_ARMS)
        extraction = _validated_extraction_config(config)
        session_ids = verify_phase_a_plan(plan, expected_arms)
        effective_parameters = load_frozen_effective_parameters(
            reference, plan, session_ids, extraction)
        artifact = extract_anchors(
            config, plan, expected_arms,
            effective_parameters_by_session=effective_parameters,
            config_identity={
                "path": str(config_path),
                "sha256": sha256(config_path),
                "object_sha256": object_sha256(config),
            })
        _write_new(arguments.output, artifact)
        print(json.dumps({
            "output": str(arguments.output.resolve()),
            "identity_sha256": artifact["identity_sha256"],
            "session_count": artifact["session_count"],
            "anchors": {
                row["session_id"]: row["anchor"]["anchor_stamp_ns"]
                for row in artifact["sessions"]
            },
        }, indent=2, sort_keys=True))
        return 0
    except (CampaignError, FileExistsError, OSError, rosbag.ROSBagException,
            yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

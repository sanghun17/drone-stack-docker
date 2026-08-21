#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest

import genpy
import rosbag
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import String
import yaml

from check_vio_postfix_init_qualification import (
    RECEIPT_SCHEMA,
    check_qualification,
)
from extract_vio_earliest_full_sync_anchors import (
    extract_anchors,
    stamp_seq_sha256,
)
from generate_vio_postfix_init_qualification_plan import (
    generate_plan,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    SCHEMA as CAMPAIGN_SCHEMA,
    object_sha256,
    sha256,
)


DEV_SESSIONS = [
    "pw1_20260804_052639",
    "pm2_20260805_020515",
    "p0_20260804_211027",
    "n0_20260805_021950",
    "pw3_20260804_053018",
]


def stamp(seconds: float) -> genpy.Time:
    return genpy.Time.from_sec(seconds)


def write_fixture_bag(path: Path, *, duplicate_imu: bool = False,
                      pre_crop_header_imu: bool = False) -> None:
    def imu_message(seconds: float, seq: int) -> Imu:
        message = Imu()
        message.header.stamp = stamp(seconds)
        message.header.seq = seq
        message.linear_acceleration.z = 9.81
        return message

    def image_message(seconds: float, seq: int) -> Image:
        message = Image()
        message.header.stamp = stamp(seconds)
        message.header.seq = seq
        message.height = 1
        message.width = 1
        message.encoding = "mono8"
        message.step = 1
        message.data = b"\0"
        return message

    def cloud_message(seconds: float, seq: int,
                      valid: bool = True) -> PointCloud2:
        header = Image().header
        header.stamp = stamp(seconds)
        header.seq = seq
        points = (
            [(1.0, 0.0, 1.0), (1.1, 0.0, 1.0),
             (1.2, 0.0, 1.0), (1.3, 0.0, 1.0)]
            if valid else
            [(0.01, 0.0, 0.01), (0.02, 0.0, 0.01),
             (0.03, 0.0, 0.01), (0.04, 0.0, 0.01)])
        return point_cloud2.create_cloud_xyz32(
            header, points)

    events = []
    if pre_crop_header_imu:
        # Establish a bag-record epoch before the frozen crop.  The first IMU
        # delivered inside that crop deliberately has a sensor header 10 ms
        # before the crop boundary; header time, not record time, must survive.
        events.extend([
            (99.900, "/unrelated", String(data="bag epoch")),
            (100.010, "/camera/imu_hybrid", imu_message(99.990, 0)),
        ])
    # First image epoch is 100.100 s.  The first LiDAR transforms to 100.095
    # and does not cover it; the second transforms to 100.195 and does.
    events.extend([
        (100.050, "/camera/imu_hybrid", imu_message(100.050, 1)),
        (100.080, "/camera/depth/color/points_10hz",
         cloud_message(100.075, 19, valid=False)),
        (100.100, "/camera/color/image_raw_10hz", image_message(100.100, 10)),
        (100.101, "/camera/depth/color/points_10hz", cloud_message(100.100, 20)),
    ])
    for index in range(1, 46):
        seconds = 100.050 + index * 0.005
        events.append((seconds + 0.0005, "/camera/imu_hybrid",
                       imu_message(seconds, 1 + index)))
    if duplicate_imu:
        events.append((100.05025, "/camera/imu_hybrid",
                       imu_message(100.050, 999)))
    events.append((100.201, "/camera/depth/color/points_10hz",
                   cloud_message(100.200, 21)))
    events.sort(key=lambda row: row[0])
    with rosbag.Bag(str(path), "w") as bag:
        for record_time, topic, message in events:
            bag.write(topic, message, t=stamp(record_time))


def arms_fixture():
    return [
        {"id": "baseline_acc10_img1000_out1000", "overrides": {}},
        {"id": "acc5", "overrides": {"imu": {"acc_cov": 5.0}}},
        {"id": "acc20", "overrides": {"imu": {"acc_cov": 20.0}}},
        {"id": "img300", "overrides": {
            "vio": {"img_point_cov": 300.0}}},
        {"id": "img3000", "overrides": {
            "vio": {"img_point_cov": 3000.0}}},
        {"id": "outlier100", "overrides": {
            "vio": {"outlier_threshold": 100.0}}},
        {"id": "outlier300", "overrides": {
            "vio": {"outlier_threshold": 300.0}}},
        {"id": "outlier600", "overrides": {
            "vio": {"outlier_threshold": 600.0}}},
    ]


def plan_fixture(path: Path, digest: str):
    arms = arms_fixture()
    identity = {
        "schema": CAMPAIGN_SCHEMA,
        "campaign_id": "synthetic_clean_phase_a",
        "mode": "full",
        "build": {
            "executable_sha256": "a" * 64,
            "dynamic_libraries": {"libimu_proc.so": {"sha256": "b" * 64}},
        },
        "arms": copy.deepcopy(arms),
        "sessions": [
            {
                "id": session_id,
                "condition": "test",
                "split": "development",
                "input_bag": str(path.resolve()),
                "input_declared_sha256": digest,
                "input_provenance_sha256": "c" * 64,
                "crop": {
                    "start_s": 0.0,
                    "duration_s": 1.0,
                    "full_duration_s": 1.0,
                    "basis": "synthetic",
                    "window_method": "synthetic",
                    "smoke_truncated": False,
                },
            }
            for session_id in DEV_SESSIONS
        ],
    }
    return arms, {**identity, "identity_sha256": object_sha256(identity)}


def config_fixture(plan):
    path = Path(__file__).with_name("vio_postfix_init_qualification_config.yaml")
    config = yaml.safe_load(path.read_text())
    config["reference_phase_a_campaign_identity_sha256"] = plan[
        "identity_sha256"]
    return config


def effective_parameters_fixture():
    return {
        session_id: {
            "parameters": {
                "topics": {
                    "image": "/camera/color/image_raw_10hz",
                    "lidar": "/camera/depth/color/points_10hz",
                    "imu": "/camera/imu_hybrid",
                },
                "effective_clock_transforms": {
                    "image_header_add_s": 0.0,
                    "image_exposure_add_s": 0.0,
                    "lidar_header_add_s": -0.005,
                    "lidar_scan_end_rule": "l515_zero_point_offset",
                    "imu_header_subtract_s": 0.0,
                    "ros_driver_bug_fix": False,
                },
                "lidar_validation": {
                    "lidar_type": 4,
                    "point_filter_num": 3,
                    "blind_m": 0.2,
                    "minimum_retained_points": 2,
                },
            },
            "source": {
                "arm_id": "baseline_acc10_img1000_out1000",
                "artifact": "/tmp/synthetic/result_params.yaml",
                "artifact_sha256": "a" * 64,
            },
        }
        for session_id in DEV_SESSIONS
    }


def with_identity(document, field="identity_sha256"):
    core = dict(document)
    core.pop(field, None)
    return {**core, field: object_sha256(core)}


def build_fixture():
    return with_identity({
        "executable_sha256": "d" * 64,
        "dynamic_libraries": {"libimu_proc.so": "e" * 64},
        "source_tree_sha256": "f" * 64,
        "reviewed_init_anchor_patch_sha256": "1" * 64,
    })


def receipt_fixture(plan, run):
    sentinel = next(row for row in plan["sentinels"]
                    if row["id"] == run["sentinel_id"])
    expected = sentinel["explicit_anchor"]
    vector = expected["expected_first_30_strict_post_anchor_stamp_seq"]
    anchor_ns = int(expected["anchor_stamp_ns"])
    stats = {"mean_acc": [0.0, 0.0, 9.81], "mean_gyr": [0.0, 0.0, 0.0]}
    hashes = {
        "low_rate_pose": ("2" * 64, "3" * 64),
        "low_rate_init": ("2" * 64, "a" * 64),
        "correction": ("4" * 64, "5" * 64),
        "propagated_odom": ("6" * 64, "7" * 64),
        "world_twist": ("6" * 64, "8" * 64),
    }
    streams = {
        name: {
            "message_count": 10 if "propagated" not in name and
                name != "world_twist" else 100,
            "sensor_stamp_vector_sha256": values[0],
            "canonical_state_sha256": values[1],
            "first_message_binary64_be_sha256": "b" * 64,
            "all_values_finite": True,
            "sensor_stamps_monotonic_non_decreasing": True,
        }
        for name, values in hashes.items()
    }
    document = {
        "schema": RECEIPT_SCHEMA,
        "plan_identity_sha256": plan["identity_sha256"],
        "run_id": run["run_id"],
        "sentinel_id": run["sentinel_id"],
        "arm_id": run["arm_id"],
        "session_id": run["session_id"],
        "rate": run["rate"],
        "repeat": run["repeat"],
        "fresh_process": True,
        "process_instance_uuid": "process-" + run["run_id"],
        "build": build_fixture(),
        "runtime_parameters": {
            "imu/init_anchor_stamp_ns": expected["anchor_stamp_ns"],
            "imu/init_anchor_max_predecessor_gap_s": 0.02,
        },
        "initialization": {
            "anchor_definition":
                "earliest_explicit_eligible_full_sync_sensor_epoch",
            "anchor_mode": "explicit",
            "anchor_covered": True,
            "anchor_stamp_ns": expected["anchor_stamp_ns"],
            "image_epoch_ns": expected["anchor_stamp_ns"],
            "lidar_watermark_ns": expected["lidar_watermark"]["watermark_ns"],
            "imu_watermark_ns": expected[
                "imu_watermark_at_coverage"]["watermark_ns"],
            "predecessor_imu_stamp_ns": expected[
                "predecessor_imu"]["watermark_ns"],
            "predecessor_gap_ns": expected["predecessor_gap_ns"],
            "init_anchor_max_predecessor_gap_s": 0.02,
            "has_pre_anchor_imu": True,
            "state_epoch_ns": str(int(vector[-1]["stamp_ns"]) + 10_000_000),
            "state_epoch_rule": "legacy_later_acceptance_sync_epoch",
            "suffix_rule": "legacy_skip_through_acceptance_state_epoch",
            "sample_sensor_stamp_seq_vector": vector,
            "sample_sensor_stamp_seq_vector_sha256": stamp_seq_sha256(vector),
            "sample_sensor_stamp_seq_hash_encoding":
                "utf8_lines_stamp_ns_comma_seq_newline",
            "valid_count": 30,
            "first_used_stamp_ns": vector[0]["stamp_ns"],
            "first_used_seq": vector[0]["seq"],
            "last_used_stamp_ns": vector[-1]["stamp_ns"],
            "last_used_seq": vector[-1]["seq"],
            "invalid_count": 0,
            "rejected_window_count": 0,
            "queue_drop_count": 0,
            "statistics": stats,
            "statistics_sha256": object_sha256(stats),
            "initial_state_binary64_be_sha256": "9" * 64,
        },
        "canonicalization_schema": "fastlivo_sensor_stamped_state_canonical/v1",
        "quaternion_sign_canonicalized": True,
        "rosbag_record_time_used": False,
        "streams": streams,
        "first_correction": {
            "correction_epoch_ns": str(int(vector[-1]["stamp_ns"]) + 10_000_000),
            "state_binary64_be_sha256": "0" * 64,
            "trajectory_index": 0,
            "all_values_finite": True,
            "qualification_gate_ready": True,
            "trajectory_sensor_stamp_vector_sha256": streams[
                "correction"]["sensor_stamp_vector_sha256"],
            "trajectory_binary64_be_sha256": streams[
                "correction"]["canonical_state_sha256"],
            "trajectory_message_binary64_be_sha256": streams[
                "correction"]["first_message_binary64_be_sha256"],
        },
        "accuracy": {
            "local_objective_normalized_max": 1.25,
            "full_report_normalized_max": 2.0,
        },
    }
    return with_identity(document)


class PostfixInitQualificationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.bag = self.root / "fixture.bag"
        write_fixture_bag(self.bag)
        digest = sha256(self.bag)
        self.arms, self.reference_plan = plan_fixture(self.bag, digest)
        self.config = config_fixture(self.reference_plan)

    def tearDown(self):
        self.temporary.cleanup()

    def _artifacts(self):
        anchors = extract_anchors(
            self.config, self.reference_plan, self.arms,
            effective_parameters_by_session=effective_parameters_fixture(),
            verify_bag_hash=True)
        plan = generate_plan(
            self.config, self.reference_plan, self.arms, anchors)
        return anchors, plan

    def test_extracts_all_five_explicit_anchors_and_exact_30_vectors(self):
        anchors, _ = self._artifacts()
        self.assertEqual(anchors["session_count"], 5)
        self.assertEqual(anchors["session_ids"], DEV_SESSIONS)
        for row in anchors["sessions"]:
            anchor = row["anchor"]
            self.assertEqual(
                anchor["anchor_stamp_ns"], str(stamp(100.100).to_nsec()))
            self.assertEqual(anchor["image_epoch_ns"], anchor["anchor_stamp_ns"])
            self.assertLessEqual(
                int(anchor["predecessor_imu"]["watermark_ns"]),
                int(anchor["anchor_stamp_ns"]))
            self.assertGreater(
                int(anchor["successor_imu"]["watermark_ns"]),
                int(anchor["anchor_stamp_ns"]))
            vector = anchor[
                "expected_first_30_strict_post_anchor_stamp_seq"]
            self.assertEqual(len(vector), 30)
            self.assertTrue(all(
                int(sample["stamp_ns"]) > int(anchor["anchor_stamp_ns"])
                for sample in vector))
            self.assertEqual(
                anchor["expected_first_30_stamp_seq_sha256"],
                stamp_seq_sha256(vector))
            self.assertEqual(row["ignored_before_selection"][
                "degenerate_lidar"], 1)

    def test_plan_is_exactly_12_and_injects_quoted_explicit_anchors(self):
        anchors, plan = self._artifacts()
        self.assertEqual(plan["expected_run_count"], 12)
        self.assertEqual(len(plan["runs"]), 12)
        self.assertEqual(len({row["run_id"] for row in plan["runs"]}), 12)
        self.assertEqual(
            {(row["sentinel_id"], row["rate"], row["repeat"])
             for row in plan["runs"]},
            {(sentinel, rate, repeat)
             for sentinel in ("acc5_n0", "baseline_pw1")
             for rate in (0.5, 1.0) for repeat in (1, 2, 3)})
        for sentinel in plan["sentinels"]:
            imu = sentinel["runtime_overrides"]["imu"]
            value = imu["init_anchor_stamp_ns"]
            self.assertIsInstance(value, str)
            self.assertEqual(value, sentinel["explicit_anchor"]["anchor_stamp_ns"])
            self.assertEqual(imu["init_anchor_max_predecessor_gap_s"], 0.02)
        self.assertEqual(
            set(plan["postfix_phase_a_explicit_anchor_overrides"]),
            set(DEV_SESSIONS))
        self.assertEqual(
            plan["anchors_artifact_identity_sha256"],
            anchors["identity_sha256"])

    def test_extractor_refuses_nonincreasing_imu_prefix_like_runtime(self):
        duplicate_bag = self.root / "duplicate.bag"
        write_fixture_bag(duplicate_bag, duplicate_imu=True)
        digest = sha256(duplicate_bag)
        arms, reference = plan_fixture(duplicate_bag, digest)
        config = config_fixture(reference)
        with self.assertRaisesRegex(CampaignError, "duplicate IMU sensor stamp"):
            extract_anchors(
                config, reference, arms,
                effective_parameters_by_session=effective_parameters_fixture(),
                verify_bag_hash=True)

    def test_crop_keeps_first_delivered_imu_whose_header_predates_crop(self):
        boundary_bag = self.root / "boundary.bag"
        write_fixture_bag(boundary_bag, pre_crop_header_imu=True)
        digest = sha256(boundary_bag)
        arms, reference = plan_fixture(boundary_bag, digest)
        for session in reference["sessions"]:
            session["crop"]["start_s"] = 0.1
            session["crop"]["duration_s"] = 1.0
            session["crop"]["full_duration_s"] = 1.1
        core = dict(reference)
        core.pop("identity_sha256")
        reference["identity_sha256"] = object_sha256(core)
        config = config_fixture(reference)
        anchors = extract_anchors(
            config, reference, arms,
            effective_parameters_by_session=effective_parameters_fixture(),
            verify_bag_hash=True)
        for row in anchors["sessions"]:
            self.assertEqual(row["crop"]["crop_start_record_stamp_ns"],
                             str(stamp(100.0).to_nsec()))
            self.assertEqual(row["anchor"]["anchor_stamp_ns"],
                             str(stamp(100.1).to_nsec()))

    def test_generator_refuses_placeholder_or_tampered_anchor(self):
        anchors, _ = self._artifacts()
        anchors["live_fallback_allowed"] = True
        core = dict(anchors)
        core.pop("identity_sha256")
        anchors["identity_sha256"] = object_sha256(core)
        with self.assertRaisesRegex(CampaignError, "provenance/scope"):
            generate_plan(
                self.config, self.reference_plan, self.arms, anchors)

        anchors = extract_anchors(
            self.config, self.reference_plan, self.arms,
            effective_parameters_by_session=effective_parameters_fixture(),
            verify_bag_hash=True)
        anchors["sessions"][0]["anchor"]["anchor_stamp_ns"] = "placeholder"
        core = dict(anchors)
        core.pop("identity_sha256")
        anchors["identity_sha256"] = object_sha256(core)
        with self.assertRaisesRegex(CampaignError, "quoted positive"):
            generate_plan(
                self.config, self.reference_plan, self.arms, anchors)

    def _write_receipts(self, plan):
        plan_path = self.root / "qualification_plan.json"
        plan_path.write_text(json.dumps(plan))
        receipt_dir = self.root / "receipts"
        receipt_dir.mkdir()
        for run in plan["runs"]:
            receipt = receipt_fixture(plan, run)
            (receipt_dir / f"{run['run_id']}.json").write_text(
                json.dumps(receipt))
        return plan_path

    def test_checker_accepts_exact_anchor_vector_state_and_trajectory(self):
        _, plan = self._artifacts()
        plan_path = self._write_receipts(plan)
        report = check_qualification(plan_path, self.root)
        self.assertEqual(report["status"], "pass")
        self.assertTrue(report["go_for_postfix_phase_a_rebaseline"])
        self.assertEqual(report["receipt_count"], 12)
        self.assertEqual(report["fresh_process_instance_count"], 12)
        self.assertTrue(all(row["canonical_init_state_agreement"]
                            for row in report["sentinels"]))
        self.assertTrue(all(row["first_correction_agreement"]
                            for row in report["sentinels"]))

    def test_checker_refuses_vector_or_init_state_or_correction_drift(self):
        _, plan = self._artifacts()
        plan_path = self._write_receipts(plan)
        target = self.root / plan["runs"][0]["expected_receipt"]

        receipt = json.loads(target.read_text())
        receipt["initialization"]["sample_sensor_stamp_seq_vector"][0]["seq"] += 1
        receipt = with_identity(receipt)
        target.write_text(json.dumps(receipt))
        with self.assertRaisesRegex(CampaignError, "differs from expected"):
            check_qualification(plan_path, self.root)

        # Restore vector, then create an internally valid but cross-repeat init
        # state fingerprint mismatch; this must be a failed qualification.
        run = plan["runs"][0]
        receipt = receipt_fixture(plan, run)
        receipt["initialization"]["initial_state_binary64_be_sha256"] = "8" * 64
        receipt = with_identity(receipt)
        target.write_text(json.dumps(receipt))
        report = check_qualification(plan_path, self.root)
        self.assertEqual(report["status"], "fail")
        self.assertTrue(any(
            failure["gate"].startswith("identical_init_anchor_vector")
            for failure in report["failures"]))

        receipt = receipt_fixture(plan, run)
        receipt["first_correction"]["state_binary64_be_sha256"] = "7" * 64
        receipt = with_identity(receipt)
        target.write_text(json.dumps(receipt))
        report = check_qualification(plan_path, self.root)
        self.assertEqual(report["status"], "fail")
        self.assertTrue(any(
            failure["gate"] ==
            "identical_first_correction_across_repeats_and_rates"
            for failure in report["failures"]))

    def test_checker_refuses_unavailable_legacy_startup_counters(self):
        _, plan = self._artifacts()
        plan_path = self._write_receipts(plan)
        target = self.root / plan["runs"][0]["expected_receipt"]
        receipt = json.loads(target.read_text())
        receipt["initialization"]["startup_duplicate_imu_count"] = 0
        target.write_text(json.dumps(with_identity(receipt)))
        with self.assertRaisesRegex(
                CampaignError, "unavailable startup/reinitialization counters"):
            check_qualification(plan_path, self.root)

    def test_checker_reports_high_rate_payload_and_full_score_drift_only(self):
        _, plan = self._artifacts()
        plan_path = self._write_receipts(plan)
        target = self.root / plan["runs"][0]["expected_receipt"]
        receipt = json.loads(target.read_text())
        receipt["streams"]["propagated_odom"][
            "canonical_state_sha256"] = "a" * 64
        receipt["streams"]["world_twist"][
            "canonical_state_sha256"] = "b" * 64
        receipt["accuracy"]["full_report_normalized_max"] = 9.0
        target.write_text(json.dumps(with_identity(receipt)))
        report = check_qualification(plan_path, self.root)
        self.assertEqual(report["status"], "pass")
        self.assertEqual(len(report["warnings"]), 2)
        self.assertTrue(all(
            warning["known_separate_interface_blocker"]
            for warning in report["warnings"]))

        receipt["accuracy"]["local_objective_normalized_max"] = 1.5
        target.write_text(json.dumps(with_identity(receipt)))
        report = check_qualification(plan_path, self.root)
        self.assertEqual(report["status"], "fail")
        self.assertTrue(any(
            failure["gate"] ==
            "identical_local_objective_accuracy_across_repeats_and_rates"
            for failure in report["failures"]))


if __name__ == "__main__":
    unittest.main()

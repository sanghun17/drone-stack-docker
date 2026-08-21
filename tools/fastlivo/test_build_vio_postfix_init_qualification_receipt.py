#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import genpy
import rosbag
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
import yaml

import test_vio_postfix_init_qualification as qualification_fixture
from build_vio_postfix_init_qualification_receipt import (
    ORCHESTRATION_SCHEMA,
    REVIEWED_INIT_FILES,
    build_receipt,
)
from prepare_vio_postfix_init_qualification import (
    derive_build_manifest,
    derive_execution_receipt,
    prepare_orchestration,
)
from run_vio_flight_tuning_campaign import (
    COMPLETION_SCHEMA,
    INPUT_SCHEMA,
    QUALIFICATION_BINDING_SCHEMA,
    REQUIRED_OUTPUT_TOPICS,
    RUN_SCHEMA,
    SCHEMA as CAMPAIGN_SCHEMA,
    bag_topic_inventory,
    file_identity,
    object_sha256,
    sha256,
)


def identified(document):
    return {**document, "identity_sha256": object_sha256(document)}


def sensor_time(stamp):
    return genpy.Time(stamp // 1_000_000_000, stamp % 1_000_000_000)


def pose(stamp, seq, *, negative=False):
    message = PoseStamped()
    message.header.stamp = sensor_time(stamp)
    message.header.seq = seq
    message.header.frame_id = "odom"
    message.pose.position.x = 1.0
    message.pose.orientation.w = -1.0 if negative else 1.0
    return message


def odometry(stamp, seq, *, negative=False):
    message = Odometry()
    message.header.stamp = sensor_time(stamp)
    message.header.seq = seq
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x = 1.0
    message.pose.pose.orientation.w = -1.0 if negative else 1.0
    return message


def correction(stamp, seq, *, negative=False):
    message = PoseWithCovarianceStamped()
    message.header.stamp = sensor_time(stamp)
    message.header.seq = seq
    message.header.frame_id = "odom"
    message.pose.pose.position.x = 1.0
    message.pose.pose.orientation.w = -1.0 if negative else 1.0
    message.pose.covariance[0] = 0.1
    return message


def twist(stamp, seq):
    message = TwistStamped()
    message.header.stamp = sensor_time(stamp)
    message.header.seq = seq
    message.header.frame_id = "odom"
    message.twist.linear.x = 0.1
    return message


class ReceiptBuilderTests(unittest.TestCase):
    """Synthetic standard-harness campaign through execution/receipt binding."""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        fixture = qualification_fixture.PostfixInitQualificationTests("runTest")
        fixture.setUp()
        try:
            anchors, plan = fixture._artifacts()
            bag_copy = self.root / "input.bag"
            bag_copy.write_bytes(fixture.bag.read_bytes())
        finally:
            fixture.tearDown()
        # Rebind the generated synthetic artifact identities to real files.
        self.anchors_path = self.root / "anchors.json"
        self.anchors_path.write_text(json.dumps(anchors, sort_keys=True))
        self.config_path = self.root / "config.yaml"
        self.config_path.write_text("qualification: synthetic\n")
        provenance = self.root / "input.bag.provenance.json"
        provenance.write_text(json.dumps({"synthetic": True}))
        for sentinel in plan["sentinels"]:
            sentinel["input_bag"] = str(bag_copy)
            sentinel["input_declared_sha256"] = sha256(bag_copy)
            sentinel["input_provenance_sha256"] = sha256(provenance)
        plan["config_identity"] = {
            "path": str(self.config_path), "sha256": sha256(self.config_path),
            "object_sha256": object_sha256({"qualification": "synthetic"}),
        }
        plan["anchors_identity"] = {
            "path": str(self.anchors_path),
            "sha256": sha256(self.anchors_path),
            "identity_sha256": anchors["identity_sha256"],
        }
        plan["anchors_artifact_identity_sha256"] = anchors["identity_sha256"]
        core = dict(plan)
        core.pop("identity_sha256", None)
        plan["identity_sha256"] = object_sha256(core)
        self.plan = plan
        self.plan_path = self.root / "qualification_plan.json"
        self.plan_path.write_text(json.dumps(plan, sort_keys=True))
        self.input_bag = bag_copy
        self.input_provenance = provenance
        self.run = plan["runs"][0]
        self.sentinel = next(row for row in plan["sentinels"]
                             if row["id"] == self.run["sentinel_id"])
        self.build = self._build_fixture()
        self.build_path = self.root / "build_manifest.json"
        self.build_path.write_text(json.dumps(self.build, sort_keys=True))
        self.dependencies = self._dependency_fixture()
        self.orchestration, self.campaign_dir, self.attempt = \
            self._campaign_fixture()
        self.orchestration_path = self.root / "orchestration.json"
        self.orchestration_path.write_text(
            json.dumps(self.orchestration, sort_keys=True))

    def tearDown(self):
        self.temporary.cleanup()

    def _build_fixture(self):
        source_root = self.root / "source"
        files = {}
        for relative in REVIEWED_INIT_FILES:
            path = source_root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(relative + "\n")
            files[relative] = {
                "size_bytes": path.stat().st_size, "sha256": sha256(path)}
        source = {"root": str(source_root), "files": files,
                  "tree_sha256": object_sha256(files)}
        binary = {
            "container": "synthetic-container",
            "replay_devel": "/isolated/devel",
            "setup_sha256": "1" * 64,
            "executable": "/isolated/devel/lib/fast_livo/fastlivo_mapping",
            "executable_sha256": "2" * 64,
            "dynamic_libraries": {
                "libimu_proc.so": {
                    "path": "/isolated/devel/lib/libimu_proc.so",
                    "sha256": "3" * 64}},
            "container_identity": {"id": "synthetic"},
        }
        git = {"revision": "4" * 40, "dirty": False,
               "status_porcelain": "", "status_porcelain_sha256": "5" * 64,
               "dirty_diff_sha256": "6" * 64,
               "dirty_diff_size_bytes": 0, "dirty_diff": ""}
        return derive_build_manifest(binary, source, git)

    def _dependency_fixture(self):
        labels = (
            "preparer", "campaign_harness", "receipt_builder", "checker",
            "anchor_extractor", "plan_generator", "replay_wrapper",
            "replay_launch", "strict_evaluator", "thresholds",
            "base_overlay", "session_spec", "fastlivo_base_config")
        result = {}
        for label in labels:
            path = self.root / "dependencies" / label
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(label + "\n")
            result[label] = file_identity(path)
        result["qualification_config"] = file_identity(self.config_path)
        result["qualification_plan"] = file_identity(self.plan_path)
        result["anchors_artifact"] = file_identity(self.anchors_path)
        return result

    def _write_result_artifacts(self, attempt):
        attempt.mkdir(parents=True)
        anchor = self.sentinel["explicit_anchor"]
        vector = anchor["expected_first_30_strict_post_anchor_stamp_seq"]
        state_epoch = int(vector[-1]["stamp_ns"]) + 10_000_000
        result_bag = attempt / "result.bag"
        with rosbag.Bag(str(result_bag), "w", compression="lz4") as bag:
            for index, stamp_ns in enumerate((state_epoch,
                                              state_epoch + 5_000_000)):
                rows = {
                    "/aft_mapped_to_body": pose(stamp_ns, index,
                                                 negative=index == 0),
                    "/aft_mapped_to_init": odometry(stamp_ns, index,
                                                     negative=index == 0),
                    "/aft_mapped_to_body_correction_pose_cov": correction(
                        stamp_ns, index, negative=index == 0),
                    "/aft_mapped_to_body_imu_propagated": odometry(
                        stamp_ns, index, negative=index == 0),
                    "/aft_mapped_to_body_imu_propagated_world_twist": twist(
                        stamp_ns, index),
                    "/vrpn_client_node/pure/pose": pose(stamp_ns, index),
                }
                for topic, message in rows.items():
                    bag.write(topic, message, t=message.header.stamp)
        events = [
            {"schema": "fast_livo/imu_init/v1", "status": "configured",
             "anchor_mode": "explicit", "anchor_stamp_ns": anchor[
                 "anchor_stamp_ns"],
             "anchor_max_predecessor_gap_ns": "20000000"},
            {"schema": "fast_livo/imu_init/v1", "status": "anchor_covered",
             "anchor_mode": "explicit", "anchor_stamp_ns": anchor[
                 "anchor_stamp_ns"], "sync_epoch_ns": anchor["anchor_stamp_ns"],
             "image_epoch_ns": anchor["anchor_stamp_ns"],
             "lidar_watermark_ns": anchor["lidar_watermark"]["watermark_ns"],
             "imu_watermark_ns": anchor["imu_watermark_at_coverage"][
                 "watermark_ns"], "has_pre_anchor_imu": True,
             "anchor_predecessor_stamp_ns": anchor["predecessor_imu"][
                 "watermark_ns"],
             "anchor_predecessor_gap_ns": anchor["predecessor_gap_ns"]},
            {"schema": "fast_livo/imu_init/v1", "status": "accepted",
             "anchor_mode": "explicit", "anchor_stamp_ns": anchor[
                 "anchor_stamp_ns"], "state_epoch_ns": str(state_epoch),
             "first_used_stamp_ns": vector[0]["stamp_ns"],
             "first_used_seq": vector[0]["seq"],
             "last_used_stamp_ns": vector[-1]["stamp_ns"],
             "last_used_seq": vector[-1]["seq"], "valid_count": 30,
             "invalid_count": 0, "rejected_window_count": 0,
             "queue_drop_count": 0,
             "selected_stamp_seq_sha256": anchor[
                 "expected_first_30_stamp_seq_sha256"],
             "selected_stamp_seq": vector,
             "initial_state_fingerprint_schema":
                 "fast_livo/initial_state_ieee754_be/v1",
             "initial_state_binary64_be_sha256": "7" * 64,
             "mean_acc": [0.0, 0.0, 9.81], "mean_gyr": [0.0, 0.0, 0.0],
             "initialization_gate_ready": True},
            {"schema": "fast_livo/imu_init/v1",
             "status": "first_correction_received",
             "correction_epoch_ns": str(state_epoch),
             "initial_state_binary64_be_sha256": "7" * 64,
             "state_fingerprint_schema":
                 "fast_livo/initial_state_ieee754_be/v1",
             "state_binary64_be_sha256": "8" * 64,
             "qualification_gate_ready": True},
        ]
        # Model the real shutdown-buffering case: early diagnostics are in the
        # screen log, while the first-correction evidence survives only on the
        # recorder-flushed /rosout topic.
        first_correction = events.pop()
        rosout = Log()
        rosout.header.stamp = sensor_time(state_epoch)
        rosout.name = "/laserMapping"
        rosout.msg = "[imu_init_diag] " + json.dumps(first_correction)
        with rosbag.Bag(str(result_bag), "a") as bag:
            bag.write("/rosout", rosout, t=rosout.header.stamp)
        (attempt / "result_node.log").write_text("".join(
            "[imu_init_diag] " + json.dumps(event) + "\n" for event in events))
        params = copy.deepcopy(self.sentinel["runtime_overrides"])
        params.setdefault("uav", {})["runtime_reinit_enable"] = False
        (attempt / "result_params.yaml").write_text(yaml.safe_dump(params))
        (attempt / "result.flight_readiness.json").write_text(json.dumps({
            "local": {"translation_ape_rmse_m": 0.1,
                      "translation_rpe_1p0s_rmse_m": 0.05,
                      "orientation_rmse_deg": 2.0, "path_ratio": 1.02}}))

    def _campaign_fixture(self):
        run, sentinel = self.run, self.sentinel
        campaign_dir = self.root / "campaigns" / ("postfixq_" + run["run_id"])
        attempt_id = "q_12345678123442349234123456789abc"
        attempt = (campaign_dir / "attempts" / run["arm_id"] /
                   run["session_id"] / attempt_id)
        replay = ["synthetic", "replay", str(attempt / "result.bag")]
        evaluator = ["synthetic", "evaluate", str(attempt / "result.bag")]
        campaign_dependencies = {
            "harness": self.dependencies["campaign_harness"],
            "replay_wrapper": self.dependencies["replay_wrapper"],
            "strict_evaluator": self.dependencies["strict_evaluator"],
            "thresholds": self.dependencies["thresholds"],
            "session_spec": self.dependencies["session_spec"],
            "base_overlay": self.dependencies["base_overlay"],
            "arms": self.dependencies["base_overlay"],
            "fastlivo_base_config": self.dependencies["fastlivo_base_config"],
            "replay_launch": self.dependencies["replay_launch"],
            "fastlivo_source_tree": self.build["source_tree_identity"],
            "fastlivo_git": self.build["git_source_identity"],
        }
        effective_sha = "9" * 64
        session = {
            "id": run["session_id"], "condition": "synthetic",
            "split": "development", "input_bag": sentinel["input_bag"],
            "input_size_bytes": self.input_bag.stat().st_size,
            "input_mtime_ns": self.input_bag.stat().st_mtime_ns,
            "input_provenance": str(self.input_provenance),
            "input_provenance_sha256": sentinel["input_provenance_sha256"],
            "input_declared_sha256": sentinel["input_declared_sha256"],
            "window_cache": str(self.root / "window.json"),
            "window_cache_sha256": "a" * 64, "crop": sentinel["crop"],
        }
        campaign = identified({
            "schema": CAMPAIGN_SCHEMA,
            "campaign_id": "postfixq_" + run["run_id"], "mode": "full",
            "single_worker": True, "host": {"synthetic": True},
            "replay": {"rate": run["rate"], "no_gt_anchor": True,
                       "with_propagated": True,
                       "fixed_zero_time_offset_evaluator": True,
                       "ros_master_port": 11341},
            "build": self.build["binary_identity"],
            "dependencies": campaign_dependencies,
            "immutable_overlay": {},
            "arms": [{"id": run["arm_id"],
                      "overrides": sentinel["runtime_overrides"],
                      "effective_overlay_sha256": effective_sha}],
            "sessions": [session],
        })
        campaign_dir.mkdir(parents=True)
        (campaign_dir / "campaign.json").write_text(json.dumps(campaign))
        input_dir = campaign_dir / "inputs"
        input_dir.mkdir()
        (input_dir / (run["session_id"] + ".json")).write_text(json.dumps({
            "schema": INPUT_SCHEMA, "session_id": run["session_id"],
            "path": sentinel["input_bag"],
            "size_bytes": self.input_bag.stat().st_size,
            "mtime_ns": self.input_bag.stat().st_mtime_ns,
            "declared_sha256": sentinel["input_declared_sha256"],
            "actual_sha256": sentinel["input_declared_sha256"],
            "provenance_path": str(self.input_provenance),
            "provenance_sha256": sentinel["input_provenance_sha256"],
            "validation": "passed"}))
        process_uuid = "12345678-1234-4234-9234-123456789abc"
        binding = identified({
            "schema": QUALIFICATION_BINDING_SCHEMA,
            "qualification_plan_identity_sha256": self.plan["identity_sha256"],
            "qualification_run_id": run["run_id"],
            "build_manifest_identity_sha256": self.build["identity_sha256"],
            "binary_identity_sha256": object_sha256(
                self.build["binary_identity"]),
            "process_instance_uuid": process_uuid, "attempt_id": attempt_id,
            "repeat": run["repeat"], "campaign_id": campaign["campaign_id"],
            "expected_campaign_identity_sha256": campaign["identity_sha256"],
            "arm_id": run["arm_id"], "session_id": run["session_id"],
            "rate": run["rate"], "input_bag": sentinel["input_bag"],
            "input_declared_sha256": sentinel["input_declared_sha256"],
            "input_provenance_sha256": sentinel["input_provenance_sha256"],
            "crop": sentinel["crop"],
            "runtime_overrides": sentinel["runtime_overrides"],
            "runtime_overrides_sha256": object_sha256(
                sentinel["runtime_overrides"]),
            "effective_overlay_sha256": effective_sha,
            "replay_command": replay, "evaluator_command": evaluator,
        })
        binding_path = self.root / "run_binding.json"
        binding_path.write_text(json.dumps(binding))
        self._write_result_artifacts(attempt)
        artifacts = {}
        for name in ("result_node.log", "result.bag",
                     "result.flight_readiness.json", "result_params.yaml"):
            path = attempt / name
            artifacts[name] = {"size_bytes": path.stat().st_size,
                               "sha256": sha256(path)}
        manifest = {
            "schema": RUN_SCHEMA, "state": "complete",
            "campaign_identity_sha256": campaign["identity_sha256"],
            "arm_id": run["arm_id"], "session_id": run["session_id"],
            "input_bag": sentinel["input_bag"],
            "input_sha256": sentinel["input_declared_sha256"],
            "crop": sentinel["crop"], "rate": run["rate"],
            "replay_flags": ["--no-gt-anchor", "--with-propagated"],
            "replay_command": replay, "evaluator_command": evaluator,
            "output_topic_inventory": bag_topic_inventory(attempt / "result.bag"),
            "artifacts": artifacts, "qualification_run_binding": binding,
        }
        manifest_path = attempt / "manifest.json"
        manifest_path.write_text(json.dumps(manifest))
        pointer = campaign_dir / "completed" / run["arm_id"] / (
            run["session_id"] + ".json")
        pointer.parent.mkdir(parents=True)
        pointer.write_text(json.dumps({
            "schema": COMPLETION_SCHEMA,
            "campaign_identity_sha256": campaign["identity_sha256"],
            "arm_id": run["arm_id"], "session_id": run["session_id"],
            "attempt": attempt.relative_to(campaign_dir).as_posix(),
            "manifest_sha256": sha256(manifest_path)}))
        arm_identity = self.dependencies["base_overlay"]
        run_specs = []
        for plan_run in self.plan["runs"]:
            row = {"run_id": plan_run["run_id"],
                   "sentinel_id": plan_run["sentinel_id"],
                   "arm_id": plan_run["arm_id"],
                   "session_id": plan_run["session_id"],
                   "rate": plan_run["rate"], "repeat": plan_run["repeat"]}
            if plan_run["run_id"] == run["run_id"]:
                row.update({
                    "fresh_process_uuid": process_uuid,
                    "input_bag": sentinel["input_bag"],
                    "input_declared_sha256": sentinel["input_declared_sha256"],
                    "input_provenance_sha256": sentinel[
                        "input_provenance_sha256"], "crop": sentinel["crop"],
                    "runtime_overrides": sentinel["runtime_overrides"],
                    "runtime_overrides_sha256": object_sha256(
                        sentinel["runtime_overrides"]),
                    "arm_yaml": arm_identity, "run_binding": file_identity(
                        binding_path), "effective_overlay_sha256": effective_sha,
                    "campaign_id": campaign["campaign_id"],
                    "campaign_dir": str(campaign_dir), "port": 11341,
                    "expected_campaign_identity_sha256": campaign[
                        "identity_sha256"], "attempt_id": attempt_id,
                    "replay_command": replay, "evaluator_command": evaluator})
            run_specs.append(row)
        orchestration = identified({
            "schema": ORCHESTRATION_SCHEMA, "scope": "development_only",
            "validation_data_accessed": False,
            "replay_executed_by_generator": False,
            "qualification_plan_identity_sha256": self.plan["identity_sha256"],
            "build_manifest_identity_sha256": self.build["identity_sha256"],
            "build_manifest": file_identity(self.build_path),
            "dependencies": self.dependencies, "rates": [0.5, 1.0],
            "fresh_process_repeats_per_rate": 3,
            "expected_run_count": 12, "runs": run_specs,
        })
        return orchestration, campaign_dir, attempt

    def _execution(self):
        return derive_execution_receipt(
            self.plan, self.orchestration, self.build, self.run["run_id"],
            self.campaign_dir, orchestration_path=self.orchestration_path,
            verify_actual_build=False)

    def test_standard_campaign_builds_sensor_time_receipt(self):
        execution = self._execution()
        receipt = build_receipt(
            self.plan, self.run["run_id"], self.attempt, execution,
            self.build, verify_actual_build=False)
        self.assertEqual(receipt["initialization"]["valid_count"], 30)
        self.assertNotIn("startup_dropped_imu_count",
                         receipt["initialization"])
        self.assertEqual(receipt["source"]["campaign_identity_sha256"],
                         execution["campaign_identity_sha256"])

    def test_prepare_emits_exact_twelve_bound_commands_without_replay(self):
        output = self.root / "prepared"
        output.mkdir()
        (output / "build_manifest.json").write_text(json.dumps(self.build))
        sentinel_by_session = {
            row["session_id"]: row for row in self.plan["sentinels"]}

        def selected_session(_spec, selected):
            return [{"id": selected[0], "condition": "synthetic",
                     "split": "development"}]

        def session_record(selected, _hybrid, _cache, _smoke, _duration):
            sentinel = sentinel_by_session[selected["id"]]
            return {**selected, "input_bag": sentinel["input_bag"],
                    "input_declared_sha256": sentinel[
                        "input_declared_sha256"],
                    "input_provenance_sha256": sentinel[
                        "input_provenance_sha256"],
                    "crop": sentinel["crop"]}

        def campaign_plan(arguments, _base, arms, sessions, build):
            return identified({
                "schema": CAMPAIGN_SCHEMA,
                "campaign_id": arguments.campaign_id,
                "build": build, "arms": list(arms),
                "sessions": list(sessions),
                "dependencies": {
                    "fastlivo_source_tree": self.build[
                        "source_tree_identity"],
                    "fastlivo_git": self.build["git_source_identity"]},
            })

        with mock.patch(
                "prepare_vio_postfix_init_qualification.load_sessions",
                side_effect=selected_session), mock.patch(
                "prepare_vio_postfix_init_qualification.input_and_window",
                side_effect=session_record), mock.patch(
                "prepare_vio_postfix_init_qualification.make_plan",
                side_effect=campaign_plan), mock.patch(
                "prepare_vio_postfix_init_qualification.container_path",
                side_effect=lambda path: "/work/synthetic/" + path.name):
            orchestration = prepare_orchestration(
                self.plan, self.build, {}, output, self.dependencies,
                container="synthetic-container",
                replay_devel="/isolated/devel", port=11341,
                spec_path=self.dependencies["session_spec"]["path"],
                hybrid_dir=self.root, window_cache=self.root,
                python_executable="/usr/bin/python3")
        self.assertEqual(len(orchestration["runs"]), 12)
        self.assertFalse(orchestration["replay_executed_by_generator"])
        self.assertEqual(len({row["campaign_id"]
                              for row in orchestration["runs"]}), 12)
        for row in orchestration["runs"]:
            binding = json.loads(Path(row["run_binding"]["path"]).read_text())
            self.assertEqual(binding["schema"], QUALIFICATION_BINDING_SCHEMA)
            self.assertEqual(binding["replay_command"], row["replay_command"])
            self.assertEqual(binding["evaluator_command"],
                             row["evaluator_command"])
            self.assertIn("FASTLIVO_QUALIFICATION_RUN_BINDING=",
                          " ".join(row["harness_command"]))

    def test_execution_refuses_tampered_campaign_plan(self):
        campaign_path = self.campaign_dir / "campaign.json"
        campaign = json.loads(campaign_path.read_text())
        campaign["replay"]["rate"] = 9.0
        campaign_path.write_text(json.dumps(campaign))
        with self.assertRaisesRegex(Exception, "identity changed"):
            self._execution()

    def test_builder_refuses_tampered_hashed_node_log(self):
        execution = self._execution()
        with (self.attempt / "result_node.log").open("a") as stream:
            stream.write("tamper\n")
        with self.assertRaisesRegex(Exception, "artifact"):
            build_receipt(
                self.plan, self.run["run_id"], self.attempt, execution,
                self.build, verify_actual_build=False)


if __name__ == "__main__":
    unittest.main()

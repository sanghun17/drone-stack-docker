#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import yaml

from summarize_vio_flight_tuning_campaign import (
    METRICS,
    csv_payload,
    screening_assessment,
    worst_metric,
)
from select_vio_flight_tuning_phase_a import (
    interaction_arms,
    rank_phase_a,
)

from run_vio_flight_tuning_campaign import (
    CampaignError,
    IMMUTABLE_OVERLAY,
    REQUIRED_ESTIMATOR_LIBRARIES,
    REQUIRED_OUTPUT_TOPICS,
    QUALIFICATION_BINDING_SCHEMA,
    binary_identity,
    deep_merge,
    effective_overlay,
    estimator_source_identity,
    expand_dotted_overrides,
    load_sessions,
    load_qualification_binding,
    object_sha256,
    validate_output_topic_inventory,
    validate_plan_identity,
    validate_parameter_snapshot,
    validate_effective_overlay,
    write_json_exclusive,
)


class TuningCampaignTests(unittest.TestCase):
    @staticmethod
    def _phase_a_fixture():
        arms = [
            {"id": "baseline", "overrides": {}},
            {"id": "acc", "overrides": {"imu": {"acc_cov": 5.0}}},
            {"id": "img", "overrides": {"vio": {"img_point_cov": 300.0}}},
            {"id": "blocked", "overrides": {
                "vio": {"outlier_threshold": 100.0}}},
        ]
        sessions = ["pw1_20260804_052639", "p0_20260804_211027"]
        plan = {
            "identity_sha256": "phase-a-fixture",
            "mode": "full",
            "arms": arms,
            "sessions": [
                {"id": session, "split": "development"}
                for session in sessions
            ],
        }
        arm_scores = {
            "baseline": [0.8, 0.8],
            "acc": [0.5, 0.6],
            "img": [0.55, 0.65],
            # Numerically attractive but one hard integration failure: it must
            # not be promoted or leak that run into the accuracy score.
            "blocked": [0.01, 0.01],
        }
        runs = []
        for arm in arms:
            for session, score in zip(sessions, arm_scores[arm["id"]]):
                blocked = arm["id"] == "blocked" and session == sessions[0]
                runs.append({
                    "arm_id": arm["id"],
                    "session_id": session,
                    "split": "development",
                    "accuracy_screen_eligible": not blocked,
                    "accuracy_rankable": True,
                    "accuracy_screen_blockers": (
                        ["propagated_rate"] if blocked else []),
                    "translation_ape_rmse_m": 0.25 * score,
                    "translation_rpe_1p0s_rmse_m": 0.05,
                    "orientation_rmse_deg": 1.0,
                    "path_ratio": 1.0,
                })
        summary = {
            "campaign": "/tmp/phase-a-fixture",
            "campaign_identity_sha256": "phase-a-fixture",
            "scope": "development_only",
            "ranking_validation_forbidden": True,
            "runs": runs,
        }
        return arms, plan, summary

    def test_dotted_override_and_immutable_sensors(self) -> None:
        arm = {"id": "candidate", "overrides": expand_dotted_overrides({
            "imu.acc_cov": 5.0,
            "vio.img_point_cov": 300,
            "common.img_en": 0,
        })}
        result = effective_overlay({
            "common": {"img_en": 1, "lidar_en": 1},
            "imu": {"imu_en": True, "acc_cov": 10.0},
        }, arm)
        validate_effective_overlay(result)
        self.assertEqual(result["imu"]["acc_cov"], 5.0)
        self.assertEqual(result["vio"]["img_point_cov"], 300)
        self.assertEqual(result["common"]["img_en"], 1)
        self.assertEqual(result["common"]["imu_topic"], "/camera/imu_hybrid")
        self.assertFalse(result["uav"]["runtime_reinit_enable"])
        self.assertTrue(result["uav"]["imu_rate_odom"])
        self.assertFalse(result["imu"]["init_estimate_gyr_bias"])
        self.assertFalse(result["mocap"]["anchor_enable"])
        self.assertFalse(result["debug"]["fusion_log"])
        self.assertFalse(result["debug"]["visual_quality_log"])

    def test_validation_split_is_locked(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "spec.json"
            path.write_text(json.dumps({"sessions": [
                {"id": "dev", "split": "development"},
                {"id": "held", "split": "validation"},
            ]}))
            self.assertEqual(load_sessions(path, ["dev"])[0]["id"], "dev")
            with self.assertRaisesRegex(CampaignError, "locked"):
                load_sessions(path, ["held"])

    def test_append_only_json_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "record.json"
            write_json_exclusive(path, {"first": True})
            with self.assertRaises(FileExistsError):
                write_json_exclusive(path, {"second": True})
            self.assertEqual(json.loads(path.read_text()), {"first": True})

    def test_canonical_identity_ignores_mapping_order(self) -> None:
        self.assertEqual(
            object_sha256({"b": 2, "a": 1}),
            object_sha256({"a": 1, "b": 2}))

    def test_campaign_plan_self_hash_detects_edit(self) -> None:
        identity = {"schema": "test", "sessions": ["development"]}
        plan = {**identity, "identity_sha256": object_sha256(identity)}
        self.assertEqual(
            validate_plan_identity(plan), plan["identity_sha256"])
        plan["sessions"] = ["validation"]
        with self.assertRaisesRegex(CampaignError, "identity changed"):
            validate_plan_identity(plan)

    def test_opt_in_qualification_binding_is_self_hashed_and_plan_bound(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "binding.json"
            arm = {
                "id": "acc5",
                "overrides": {"imu": {"acc_cov": 5.0}},
                "effective_overlay_sha256": "a" * 64,
            }
            session = {
                "id": "n0", "input_bag": "/tmp/input.bag",
                "input_declared_sha256": "b" * 64,
                "input_provenance_sha256": "c" * 64,
                "crop": {"start_s": 1.0, "duration_s": 2.0},
            }
            plan_core = {
                "schema": "fastlivo_vio_tuning_campaign/v1",
                "campaign_id": "qualification_test", "arms": [arm],
                "sessions": [session], "replay": {"rate": 0.5},
                "build": {"executable_sha256": "d" * 64},
            }
            plan = {**plan_core,
                    "identity_sha256": object_sha256(plan_core)}
            core = {
                "schema": QUALIFICATION_BINDING_SCHEMA,
                "qualification_plan_identity_sha256": "e" * 64,
                "qualification_run_id": "acc5_n0_rate0p5_r1",
                "build_manifest_identity_sha256": "f" * 64,
                "process_instance_uuid":
                    "12345678-1234-4234-9234-123456789abc",
                "attempt_id": "q_12345678123442349234123456789abc",
                "repeat": 1,
                "campaign_id": plan["campaign_id"],
                "expected_campaign_identity_sha256": plan["identity_sha256"],
                "arm_id": arm["id"], "session_id": session["id"],
                "rate": 0.5, "input_bag": session["input_bag"],
                "input_declared_sha256": session["input_declared_sha256"],
                "input_provenance_sha256": session[
                    "input_provenance_sha256"],
                "crop": session["crop"],
                "runtime_overrides": arm["overrides"],
                "runtime_overrides_sha256": object_sha256(arm["overrides"]),
                "effective_overlay_sha256": arm[
                    "effective_overlay_sha256"],
                "binary_identity_sha256": object_sha256(plan["build"]),
                "replay_command": ["synthetic replay"],
                "evaluator_command": ["synthetic evaluator"],
            }
            binding = {**core, "identity_sha256": object_sha256(core)}
            path.write_text(json.dumps(binding))
            self.assertEqual(
                load_qualification_binding(path, plan)["identity_sha256"],
                binding["identity_sha256"])
            binding["rate"] = 1.0
            altered = dict(binding)
            altered.pop("identity_sha256")
            binding["identity_sha256"] = object_sha256(altered)
            path.write_text(json.dumps(binding))
            with self.assertRaisesRegex(CampaignError, "mismatch: rate"):
                load_qualification_binding(path, plan)

    def test_estimator_source_identity_is_nonempty_and_content_addressed(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            (root / "config").mkdir()
            (root / "src").mkdir()
            (root / "CMakeLists.txt").write_text("project(test)\n")
            (root / "package.xml").write_text("<package/>\n")
            (root / "config/d435i.yaml").write_text("common: {}\n")
            (root / "src/mapper.cpp").write_text("int mapper = 1;\n")
            identity = estimator_source_identity(root)
            self.assertEqual(identity["root"], str(root.resolve()))
            self.assertEqual(
                set(identity["files"]),
                {"CMakeLists.txt", "package.xml", "config/d435i.yaml",
                 "src/mapper.cpp"})
            self.assertEqual(
                identity["tree_sha256"], object_sha256(identity["files"]))

    def test_nested_merge_rejects_scalar_mapping_collision(self) -> None:
        target = {"imu": 1}
        with self.assertRaisesRegex(CampaignError, "mapping into scalar"):
            deep_merge(target, {"imu": {"acc_cov": 5.0}})

    def test_immutable_overlay_declares_all_safety_keys(self) -> None:
        self.assertEqual(IMMUTABLE_OVERLAY["common"]["img_en"], 1)
        self.assertEqual(IMMUTABLE_OVERLAY["common"]["lidar_en"], 1)
        self.assertTrue(IMMUTABLE_OVERLAY["imu"]["imu_en"])
        self.assertTrue(IMMUTABLE_OVERLAY["uav"]["imu_rate_odom"])
        self.assertFalse(IMMUTABLE_OVERLAY["mocap"]["anchor_enable"])

    def test_parameter_snapshot_checks_arm_and_gt_isolation(self) -> None:
        with tempfile.TemporaryDirectory() as raw:
            path = Path(raw) / "params.yaml"
            snapshot = {
                "common": {"img_en": 1, "lidar_en": 1,
                           "imu_topic": "/camera/imu_hybrid"},
                "imu": {"imu_en": True, "acc_cov": 5.0,
                        "init_estimate_gyr_bias": False},
                "uav": {"runtime_reinit_enable": False,
                        "imu_rate_odom": True},
                "mocap": {"anchor_enable": False},
                "debug": {"fusion_log": False,
                          "visual_quality_log": False},
                "use_sim_time": True,
            }
            path.write_text(yaml.safe_dump(snapshot))
            validate_parameter_snapshot(path, {
                "common": snapshot["common"],
                "imu": snapshot["imu"],
                "uav": snapshot["uav"],
                "mocap": snapshot["mocap"],
                "debug": snapshot["debug"],
            })
            snapshot["mocap"]["anchor_enable"] = True
            path.write_text(yaml.safe_dump(snapshot))
            with self.assertRaisesRegex(CampaignError, "GT anchor"):
                validate_parameter_snapshot(path, {
                    "common": snapshot["common"],
                    "imu": snapshot["imu"],
                    "uav": snapshot["uav"],
                    "mocap": snapshot["mocap"],
                    "debug": snapshot["debug"],
                })

    def test_summary_csv_is_stable(self) -> None:
        payload = csv_payload([
            {"arm_id": "a", "translation_ape_rmse_m": 0.2},
            {"arm_id": "b", "translation_ape_rmse_m": 0.3},
        ])
        self.assertEqual(
            payload,
            "arm_id,translation_ape_rmse_m\na,0.2\nb,0.3\n")

    def test_summary_worst_handles_path_shrinkage_and_coverage(self) -> None:
        self.assertEqual(worst_metric([0.55, 1.2], "path_ratio"), 0.55)
        self.assertEqual(
            worst_metric([0.97, 0.91], "post_initialization_coverage"),
            0.91)
        self.assertEqual(
            worst_metric([0.2, 0.4], "translation_ape_rmse_m"), 0.4)

    def test_phase_a_selection_excludes_blocked_runs_and_is_minimax(self) -> None:
        arms, plan, summary = self._phase_a_fixture()
        selection = rank_phase_a(summary, plan, expected_arms=arms)
        self.assertEqual(selection["selected_top_two"], ["acc", "img"])
        by_id = {row["arm_id"]: row for row in selection["ranking"]}
        self.assertFalse(by_id["blocked"]["eligible_for_promotion"])
        self.assertEqual(
            by_id["blocked"]["hard_integration_failure_session_count"], 1)
        self.assertTrue(by_id["blocked"]["accuracy_numeric_complete"])
        self.assertAlmostEqual(
            by_id["blocked"]["worst_session_normalized_max"], 0.5)
        self.assertAlmostEqual(
            by_id["acc"]["worst_session_normalized_max"], 0.6)
        self.assertAlmostEqual(
            by_id["acc"]["mean_session_normalized_max"], 0.55)
        self.assertFalse(selection["validation_data_accessed"])

    def test_phase_a_shared_blockers_still_yield_tuning_top_two(self) -> None:
        arms, plan, summary = self._phase_a_fixture()
        for run in summary["runs"]:
            run["accuracy_screen_eligible"] = False
            run["accuracy_rankable"] = True
            run["accuracy_screen_blockers"] = ["propagated_sensor_age_p99"]
        selection = rank_phase_a(summary, plan, expected_arms=arms)
        self.assertEqual(selection["selected_top_two"], ["blocked", "acc"])
        self.assertEqual(selection["promotion_eligible_arms"], [])
        self.assertTrue(selection["selection_complete"])
        self.assertTrue(
            selection[
                "selected_top_two_are_development_tuning_choices_not_promotion"])

    def test_phase_a_interaction_export_is_four_unique_factorial_cells(self) -> None:
        arms, plan, summary = self._phase_a_fixture()
        selection = rank_phase_a(summary, plan, expected_arms=arms)
        document = interaction_arms(selection, plan)
        generated = {row["id"]: row["overrides"]
                     for row in document["arms"]}
        self.assertEqual(set(generated), {
            "phaseb_baseline", "phaseb_main1", "phaseb_main2",
            "phaseb_interaction",
        })
        self.assertEqual(generated["phaseb_baseline"], {})
        self.assertEqual(
            generated["phaseb_interaction"],
            {"imu": {"acc_cov": 5.0},
             "vio": {"img_point_cov": 300.0}})

    def test_phase_a_selection_refuses_non_whitelisted_session(self) -> None:
        arms, plan, summary = self._phase_a_fixture()
        plan["sessions"][0]["id"] = "p1_20260804_212926"
        with self.assertRaisesRegex(CampaignError, "non-development"):
            rank_phase_a(summary, plan, expected_arms=arms)

    def test_short_stationary_crop_remains_usable_only_for_accuracy_screen(self) -> None:
        local = {metric: 1.0 for metric in METRICS}
        report = {
            "status": "incomplete",
            "checks": [
                {"id": "local_finite", "status": "pass"},
                {"id": "stationary_translation_drift", "status": "unavailable"},
                {"id": "stationary_yaw_drift", "status": "unavailable"},
            ],
        }
        assessment = screening_assessment(report, local)
        self.assertTrue(assessment["accuracy_screen_eligible"])
        self.assertEqual(assessment["accuracy_screen_blockers"], [])

        report["checks"].append({
            "id": "propagated_rate", "status": "fail",
        })
        assessment = screening_assessment(report, local)
        self.assertFalse(assessment["accuracy_screen_eligible"])
        self.assertTrue(assessment["accuracy_rankable"])
        self.assertEqual(
            assessment["accuracy_screen_blockers"], ["propagated_rate"])

    def test_failed_accuracy_objectives_remain_rankable_but_interface_does_not(self) -> None:
        local = {metric: 1.0 for metric in METRICS}
        objective_failures = [
            "translation_ape_rmse", "translation_ape_max",
            "translation_rpe_1s", "orientation_rmse", "orientation_p90",
            "path_ratio_upper", "direction_cosine",
            "propagated_translation_ape_rmse",
            "propagated_translation_ape_max",
            "propagated_orientation_rmse",
            "propagated_orientation_p90",
        ]
        report = {
            "status": "fail",
            "checks": [
                *({"id": identifier, "status": "fail"}
                  for identifier in objective_failures),
                {"id": "stationary_translation_drift", "status": "unavailable"},
                {"id": "stationary_yaw_drift", "status": "unavailable"},
                {"id": "local_finite", "status": "pass"},
            ],
        }
        assessment = screening_assessment(report, local)
        self.assertTrue(assessment["accuracy_screen_eligible"])
        self.assertEqual(assessment["accuracy_screen_blockers"], [])
        self.assertEqual(
            assessment["objective_failures_ignored_for_screening"],
            sorted(objective_failures))

        report["checks"].append({
            "id": "propagated_twist_pose_consistency", "status": "fail",
        })
        assessment = screening_assessment(report, local)
        self.assertFalse(assessment["accuracy_screen_eligible"])
        self.assertTrue(assessment["accuracy_rankable"])
        self.assertEqual(
            assessment["accuracy_screen_blockers"],
            ["propagated_twist_pose_consistency"])

    def test_actual_smoke_pattern_is_rankable_but_not_promotable(self) -> None:
        local = {metric: 1.0 for metric in METRICS}
        objective = [
            "local_pose_coverage",  # boundary coverage is interface-only
            "translation_ape_rmse", "translation_ape_max",
            "translation_rpe_1s", "orientation_rmse", "orientation_p90",
            "path_ratio_upper", "direction_cosine",
            "propagated_translation_ape_rmse",
            "propagated_translation_ape_max",
            "propagated_orientation_rmse",
            "propagated_orientation_p90",
            "propagated_sensor_age_p99",
            "propagated_nonnegative_sensor_age",
            "propagated_position_jump",
            "propagated_twist_pose_consistency",
        ]
        report = {
            "status": "fail",
            "checks": [
                *({"id": identifier, "status": "fail"}
                  for identifier in objective),
                {"id": "stationary_translation_drift", "status": "unavailable"},
                {"id": "stationary_yaw_drift", "status": "unavailable"},
            ],
        }
        assessment = screening_assessment(report, local)
        self.assertTrue(assessment["accuracy_rankable"])
        self.assertFalse(assessment["accuracy_screen_eligible"])
        self.assertEqual(assessment["accuracy_screen_blockers"], sorted({
            "local_pose_coverage", "propagated_sensor_age_p99",
            "propagated_nonnegative_sensor_age", "propagated_position_jump",
            "propagated_twist_pose_consistency",
        }))

    def test_binary_identity_includes_every_dynamic_library(self) -> None:
        devel = "/tmp/isolated_devel"
        paths = [f"{devel}/setup.bash",
                 f"{devel}/lib/fast_livo/fastlivo_mapping"] + [
                     f"{devel}/lib/{name}"
                     for name in REQUIRED_ESTIMATOR_LIBRARIES]
        output = "\n".join(f"{'a' * 64}  {path}" for path in paths)
        with mock.patch(
                "run_vio_flight_tuning_campaign.docker_output",
                return_value=output), mock.patch(
                    "run_vio_flight_tuning_campaign.subprocess.run",
                    return_value=SimpleNamespace(
                        returncode=0, stdout="[{}]", stderr="")):
            identity = binary_identity("replay-container", devel)
        self.assertEqual(
            set(identity["dynamic_libraries"]),
            set(REQUIRED_ESTIMATOR_LIBRARIES))
        self.assertTrue(all(
            row["sha256"] == "a" * 64
            for row in identity["dynamic_libraries"].values()))

    def test_binary_identity_rejects_missing_dynamic_library(self) -> None:
        devel = "/tmp/isolated_devel"
        output = "\n".join([
            f"{'a' * 64}  {devel}/setup.bash",
            f"{'a' * 64}  {devel}/lib/fast_livo/fastlivo_mapping",
        ])
        with mock.patch(
                "run_vio_flight_tuning_campaign.docker_output",
                return_value=output), mock.patch(
                    "run_vio_flight_tuning_campaign.subprocess.run",
                    return_value=SimpleNamespace(
                        returncode=0, stdout="[{}]", stderr="")):
            with self.assertRaisesRegex(CampaignError, "fingerprint"):
                binary_identity("replay-container", devel)

    def test_output_inventory_enforces_fifo_contract(self) -> None:
        inventory = {
            topic: {
                "message_type": message_type,
                "message_count": 20 if "imu_propagated" in topic else 2,
                "connection_count": 1,
            }
            for topic, message_type in REQUIRED_OUTPUT_TOPICS.items()
        }
        # The paired propagated odometry and world-twist streams are emitted
        # for the same FIFO sample and therefore have an exact count contract.
        inventory[
            "/aft_mapped_to_body_imu_propagated_world_twist"
        ]["message_count"] = inventory[
            "/aft_mapped_to_body_imu_propagated"
        ]["message_count"]
        validate_output_topic_inventory(inventory)

        wrong_frame_contract = dict(inventory)
        wrong_frame_contract["/aft_mapped_to_optitrack"] = {
            "message_type": "nav_msgs/Odometry",
            "message_count": 1,
            "connection_count": 1,
        }
        with self.assertRaisesRegex(CampaignError, "GT-anchored"):
            validate_output_topic_inventory(wrong_frame_contract)

        mismatched = {topic: dict(row) for topic, row in inventory.items()}
        mismatched[
            "/aft_mapped_to_body_imu_propagated_world_twist"
        ]["message_count"] -= 1
        with self.assertRaisesRegex(CampaignError, "counts differ"):
            validate_output_topic_inventory(mismatched)


if __name__ == "__main__":
    unittest.main()

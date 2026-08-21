#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

import generate_vio_ground_phase_a_rebaseline as generator  # noqa: E402
import run_vio_ground_phase_a_rebaseline as runner  # noqa: E402
import build_vio_ground_phase_a_rebaseline_report as reporter  # noqa: E402
from run_vio_flight_tuning_campaign import (  # noqa: E402
    CampaignError,
    load_arms,
    load_json,
    object_sha256,
)


TOOLS = Path(__file__).resolve().parent
GROUND_ROOT = (
    TOOLS / "_campaign_vio_flight_20260814" /
    "ground_init_qualification_v1"
)
REFERENCE = (
    TOOLS / "_campaign_vio_flight_20260814" /
    "tuning_campaigns/phase_a_ofat_clean_v2"
)
BUILD = (
    TOOLS / "_campaign_vio_flight_20260814" /
    "postfix_init_qualification_v2/build_manifest.json"
)
BASE = TOOLS / "mock_candidate3_full_livo_hybrid_imu.yaml"
THRESHOLDS = TOOLS / "vio_flight_readiness_thresholds.yaml"
ARMS = TOOLS / "vio_flight_tuning_arms_phase_a.yaml"


def self_hashed(core):
    return {**core, "identity_sha256": object_sha256(core)}


class GroundPhaseARebaselineTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(
            dir=TOOLS, prefix="_test_ground_phase_a_workflow_")
        self.root = Path(self.temp.name)
        self.plan_path = GROUND_ROOT / "qualification_plan.json"
        self.anchors_path = GROUND_ROOT / "ground_init_anchors.json"
        self.plan = load_json(self.plan_path)
        self.anchors = load_json(self.anchors_path)
        self.build = load_json(BUILD)
        self.reference = load_json(REFERENCE / "campaign.json")
        self.arms = load_arms(ARMS)
        self.base = generator._load_yaml(BASE, "base overlay")
        self.report = self_hashed({
            "schema": generator.QUALIFICATION_REPORT_SCHEMA,
            "qualification_variant": generator.QUALIFICATION_VARIANT,
            "scope": "development_only",
            "validation_data_accessed": False,
            "status": "pass",
            "failures": [],
            "low_rate_estimator_rebaseline_go": True,
            "go_for_ground_init_low_rate_phase_a_rebaseline": True,
            "high_rate_interface_remains_no_go": True,
            "high_rate_interface_status": "NO_GO",
            "primary_strict_flight_status": "NO_GO",
            "secondary_can_override_primary_failure": False,
            "predecessor_qualification_status": "fail",
            "predecessor_qualification_superseded": False,
            "plan_identity_sha256": self.plan["identity_sha256"],
            "build_manifest_identity_sha256": self.build["identity_sha256"],
            "postfix_build_identity_sha256": self.build["identity_sha256"],
            "postfix_executable_sha256": self.build["executable_sha256"],
            "receipt_count": 12,
            "fresh_process_instance_count": 12,
            "not_a_flight_readiness_decision": True,
            "flight_ready": False,
        })

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _write_report(self, report=None, name="qualification_report.json"):
        path = self.root / name
        path.write_text(json.dumps(report or self.report))
        return path

    def _prepare(self, report=None, name="prepared"):
        report = report or self.report
        report_path = self._write_report(report, name=f"{name}_report.json")
        output = self.root / name
        with mock.patch("subprocess.run", side_effect=AssertionError(
                "preparer must not run Docker, replay, build, or evaluator")):
            orchestration = generator.prepare_orchestration(
                self.plan, report, self.build, self.anchors,
                self.reference, self.arms, self.base,
                qualification_plan_path=self.plan_path,
                qualification_report_path=report_path,
                build_path=BUILD,
                anchors_path=self.anchors_path,
                reference_campaign=REFERENCE,
                arms_path=ARMS,
                base_overlay_path=BASE,
                thresholds_path=THRESHOLDS,
                output_root=output,
                port=11431,
                python_executable=sys.executable,
                verify_actual_build=False,
                verify_actual_inputs=False,
            )
        return output, orchestration

    def test_prepares_exact_8x5_full_start_dual_evaluation_grid(self):
        output, orchestration = self._prepare()
        self.assertEqual(orchestration["schema"], generator.SCHEMA)
        self.assertEqual(orchestration["design"]["expected_run_count"], 40)
        self.assertEqual(orchestration["design"]["arm_ids"],
                         list(generator.ARM_IDS))
        self.assertEqual(orchestration["design"]["session_ids"],
                         list(generator.SESSION_IDS))
        expected_grid = [
            (arm_id, session_id)
            for arm_id in generator.ARM_IDS
            for session_id in generator.SESSION_IDS
        ]
        observed_grid = [
            (row["arm_id"], row["session_id"])
            for row in orchestration["cells"]
        ]
        self.assertEqual(observed_grid, expected_grid)
        self.assertTrue(orchestration["qualification_gate"]
                        ["low_rate_estimator_rebaseline_go"])
        self.assertTrue(orchestration["decision_contract"]
                        ["high_rate_interface_remains_no_go"])
        self.assertFalse(orchestration["decision_contract"]
                         ["old_scores_may_be_pooled"])
        self.assertFalse(orchestration["decision_contract"]
                         ["candidate_promotion_allowed"])
        self.assertEqual(orchestration["runtime_constant_binding"], {
            "gravity_macro": "G_m_s2",
            "expected_literal": 9.81,
            "source_relative_to_estimator_root": "include/common_lib.h",
            "source_sha256":
                "e89d7b8a80c9f9bb88663a6eaa7e4921c222b21a7ad7f898262158d307679796",
        })

        for row in orchestration["cells"]:
            cell = load_json(Path(row["cell"]["path"]))
            generator._self_hash(cell, generator.CELL_SCHEMA, "test cell")
            replay = cell["session"]["replay"]
            self.assertEqual(replay["start_s"], 0.0)
            self.assertEqual(replay["rate"], 1.0)
            self.assertGreater(replay["duration_s"], 0.0)
            self.assertEqual(replay["end_definition"],
                             "frozen_cached_landing")
            self.assertTrue(replay["no_gt_anchor"])
            self.assertTrue(replay["with_propagated"])
            primary = cell["session"]["primary_evaluation"]
            self.assertIsNone(primary["score_window_sensor_stamp_ns"])
            secondary = cell["session"]["secondary_evaluation"]
            self.assertEqual(secondary["alignment"],
                             "reuse_primary_without_refit")
            self.assertEqual(secondary["mask_basis"],
                             "result_stream_sensor_header_epoch")
            self.assertEqual(
                secondary["interpretation"],
                "absolute_ros_epoch_numeric_mask_with_mixed_frozen_sources")
            self.assertEqual(secondary["start_epoch_origin"],
                             "gt_pose_header_time_from_frozen_cache")
            self.assertEqual(secondary["end_epoch_origin"],
                             "mavros_landing_record_time_from_frozen_cache")
            self.assertLess(int(secondary["score_start_ns"]),
                            int(secondary["score_end_ns"]))
            self.assertFalse(secondary["can_override_primary_failure"])
            self.assertFalse(secondary["can_clear_high_rate_interface"])
            imu = cell["runtime_overrides"]["imu"]
            for key, expected in generator.TIGHT_INIT_PARAMETERS.items():
                self.assertEqual(imu[key], expected)
            self.assertEqual(
                imu["init_anchor_stamp_ns"],
                cell["session"]["explicit_anchor"]["anchor_stamp_ns"])

        commands = load_json(output / "commands.json")
        generator._self_hash(
            commands, generator.COMMANDS_SCHEMA, "test commands")
        self.assertEqual(commands["command_count"], 40)
        self.assertTrue(commands["strictly_sequential"])
        self.assertEqual(commands["commands"], orchestration["commands"])
        self.assertIn("phase_a_reporter", orchestration["dependencies"])
        self.assertEqual(
            orchestration["execution"]["report_command"], [
                sys.executable, str(Path(reporter.__file__).resolve()),
                str((output / "orchestration.json").resolve()),
                "--output", str((output / "phase_a_report.json").resolve()),
            ])

    def test_refuses_missing_or_overbroad_go(self):
        for index, changes in enumerate((
                {"low_rate_estimator_rebaseline_go": False},
                {"high_rate_interface_remains_no_go": False},
                {"flight_ready": True},
                {"validation_data_accessed": True},
                {"failures": [{"gate": "fixture"}]},
        )):
            core = copy.deepcopy(self.report)
            core.pop("identity_sha256")
            core.update(changes)
            report = self_hashed(core)
            report_path = self._write_report(report, f"refuse_{index}.json")
            with self.assertRaisesRegex(CampaignError, "not an explicit"):
                generator.prepare_orchestration(
                    self.plan, report, self.build, self.anchors,
                    self.reference, self.arms, self.base,
                    qualification_plan_path=self.plan_path,
                    qualification_report_path=report_path,
                    build_path=BUILD,
                    anchors_path=self.anchors_path,
                    reference_campaign=REFERENCE,
                    arms_path=ARMS,
                    base_overlay_path=BASE,
                    thresholds_path=THRESHOLDS,
                    output_root=self.root / f"never_created_{index}",
                    port=11431,
                    verify_actual_build=False,
                    verify_actual_inputs=False,
                )

    def test_runtime_constant_must_match_qualified_source_tree(self):
        anchors = copy.deepcopy(self.anchors)
        plan = copy.deepcopy(self.plan)
        anchors["runtime_constant_binding"]["source_sha256"] = "0" * 64
        plan["runtime_constant_binding"]["source_sha256"] = "0" * 64
        with self.assertRaisesRegex(CampaignError, "9.81 constant file"):
            generator.validate_runtime_constant_binding(
                plan, anchors, self.build)

    def test_runner_loads_grid_and_rejects_self_valid_command_rewrite(self):
        output, orchestration = self._prepare()
        loaded, identity = runner._load_orchestration(
            output / "orchestration.json")
        self.assertEqual(identity, orchestration["identity_sha256"])
        paths = runner._validate_dependencies(loaded)
        self.assertEqual(paths["runner"], Path(runner.__file__).resolve())
        first = loaded["cells"][0]
        cell, _ = runner._load_cell(loaded, first["run_id"])
        self.assertEqual(cell["run_id"], first["run_id"])
        commands_path = output / "commands.json"
        commands = runner._validate_commands(
            output / "orchestration.json", loaded, commands_path)
        self.assertEqual(len(commands), 40)

        rewritten = load_json(commands_path)
        rewritten["commands"][0][-1] = "different_cell"
        rewritten.pop("identity_sha256")
        rewritten = self_hashed(rewritten)
        tampered = self.root / "tampered_commands.json"
        tampered.write_text(json.dumps(rewritten))
        with self.assertRaisesRegex(CampaignError, "differs"):
            runner._validate_commands(
                output / "orchestration.json", loaded, tampered)

    def test_direct_cell_call_cannot_skip_predecessor(self):
        output, orchestration = self._prepare()
        first, _ = runner._load_cell(
            orchestration, orchestration["cells"][0]["run_id"])
        runner._require_predecessor_completions(
            orchestration, orchestration["identity_sha256"], first)
        second, _ = runner._load_cell(
            orchestration, orchestration["cells"][1]["run_id"])
        with self.assertRaisesRegex(CampaignError, "cannot precede incomplete"):
            runner._require_predecessor_completions(
                orchestration, orchestration["identity_sha256"], second)

    def test_reporter_ranks_hover_accuracy_but_blocks_promotion(self):
        orchestration = {"identity_sha256": "a" * 64}
        plan = reporter._selection_plan(orchestration, self.arms)
        runs = []
        for arm_index, arm_id in enumerate(generator.ARM_IDS):
            for session_id in generator.SESSION_IDS:
                cell = {
                    "arm_id": arm_id,
                    "session": {
                        "session_id": session_id,
                        "condition": "fixture",
                    },
                }
                # acc5 is the best numeric fixture; every cell still carries
                # the independent high-rate promotion blocker.
                scale = 1.0 if arm_id == "acc5" else 2.0 + arm_index
                primary = {
                    "status": "pass",
                    "flight_ready": True,
                    "checks": [{"id": "fixture", "status": "pass"}],
                }
                secondary = {"local_accuracy": {
                    "translation_ape_rmse_m": 0.01 * scale,
                    "translation_ape_max_m": 0.02 * scale,
                    "translation_rpe_1p0s_rmse_m": 0.005 * scale,
                    "orientation_rmse_deg": 0.1 * scale,
                    "path_ratio": 1.0 + 0.001 * scale,
                    "post_initialization_coverage": 0.99,
                }}
                row = reporter._run_row(
                    cell, self.root / "fixture_attempt", primary, secondary)
                self.assertTrue(row["accuracy_rankable"])
                self.assertFalse(row["accuracy_screen_eligible"])
                self.assertIn("known_high_rate_interface_no_go",
                              row["accuracy_screen_blockers"])
                runs.append(row)
        summary = {
            "scope": "development_only",
            "ranking_validation_forbidden": True,
            "campaign_identity_sha256": plan["identity_sha256"],
            "runs": runs,
        }
        selection = reporter.rank_phase_a(summary, plan, self.arms)
        self.assertEqual(selection["selected_top_two"][0], "acc5")
        self.assertEqual(selection["promotion_eligible_arms"], [])
        self.assertTrue(all(not row["eligible_for_promotion"]
                            for row in selection["ranking"]))

    def test_evaluator_commands_force_same_bag_and_primary_alignment(self):
        result = self.root / "result.bag"
        primary = self.root / "primary.json"
        secondary = self.root / "secondary.json"
        primary_command, secondary_command = runner._evaluation_commands(
            result, THRESHOLDS, primary, secondary, "100", "200")
        self.assertEqual(primary_command[2], str(result))
        self.assertEqual(secondary_command[2], str(result))
        self.assertNotIn("--score-start-ns", primary_command)
        self.assertEqual(
            secondary_command[secondary_command.index("--score-start-ns") + 1],
            "100")
        self.assertEqual(
            secondary_command[secondary_command.index("--score-end-ns") + 1],
            "200")
        self.assertEqual(
            secondary_command[
                secondary_command.index("--fixed-alignment-report") + 1],
            str(primary))

    def test_secondary_validator_proves_no_refit_and_inherits_primary(self):
        result = self.root / "fixture_result.bag"
        thresholds = self.root / "fixture_thresholds.yaml"
        result.write_bytes(b"fixture bag bytes")
        thresholds.write_text("schema: fixture\n")
        result_identity = runner._file_identity(result)
        threshold_identity = runner._file_identity(thresholds)
        alignment = {
            "method": "initialization_window_yaw_and_translation_only",
            "yaw_deg": 1.25,
            "translation_m": [1.0, 2.0, 3.0],
            "scale": 1.0,
        }
        primary = {
            "schema": runner.PRIMARY_SCHEMA,
            "result_bag": str(result.resolve()),
            "evaluation_semantics": {
                "time_offset_used_for_scoring_s": 0.0,
                "whole_trajectory_alignment_used": False,
                "per_session_time_optimization_used": False,
                "score_window_sensor_stamp_ns": None,
                "fixed_alignment_supplied": False,
            },
            "gt_independence": {"gt_anchor_free": True},
            "local": {"alignment": alignment, "integrity": {"count": 9}},
            "propagated": {"message_count": 90},
            "correction_covariance": {"count": 9},
            "checks": [{"id": "fixture", "status": "fail"}],
            "status": "fail",
            "flight_ready": False,
            "artifact_bindings": {
                "result_bag": result_identity,
                "thresholds": threshold_identity,
            },
        }
        primary_path = self.root / "fixture_primary.json"
        primary_path.write_text(json.dumps(primary))
        primary_identity = runner._file_identity(primary_path)
        reused = {
            **alignment,
            "reused_from_primary_full_result": True,
            "primary_report": str(primary_path.resolve()),
            "primary_report_identity": primary_identity,
        }
        secondary = {
            "schema": runner.SECONDARY_SCHEMA,
            "result_bag": str(result.resolve()),
            "role": "phase_a_ranking_compatibility_only",
            "flight_ready": False,
            "status": "ranking_only",
            "can_override_primary_failure": False,
            "primary_report_identity": primary_identity,
            "primary_status": "fail",
            "primary_flight_ready": False,
            "evaluation_semantics": {
                "score_window_sensor_stamp_ns": {
                    "start": "100", "end": "200",
                    "boundary": "start_inclusive_end_inclusive",
                },
                "fixed_alignment_supplied": True,
                "primary_alignment_reused_without_refit": True,
            },
            "local_accuracy": {"alignment": reused, "ape_rmse_m": 1.0},
            "propagated_accuracy": {},
            "full_result_interface_inherited": {
                "gt_independence": primary["gt_independence"],
                "local_integrity": primary["local"]["integrity"],
                "propagated": primary["propagated"],
                "correction_covariance": primary["correction_covariance"],
                "checks": primary["checks"],
                "status": primary["status"],
                "flight_ready": primary["flight_ready"],
            },
            "artifact_bindings": {
                "result_bag": result_identity,
                "thresholds": threshold_identity,
            },
        }
        secondary_path = self.root / "fixture_secondary.json"
        secondary_path.write_text(json.dumps(secondary))
        accepted = runner._validate_secondary_report(
            secondary_path, result, thresholds, primary_path, primary,
            "100", "200")
        self.assertEqual(accepted["status"], "ranking_only")

        bad = copy.deepcopy(secondary)
        bad["local_accuracy"]["alignment"]["translation_m"] = [9.0, 9.0, 9.0]
        bad_path = self.root / "fixture_secondary_refit.json"
        bad_path.write_text(json.dumps(bad))
        with self.assertRaisesRegex(CampaignError, "did not reuse"):
            runner._validate_secondary_report(
                bad_path, result, thresholds, primary_path, primary,
                "100", "200")


if __name__ == "__main__":
    unittest.main()

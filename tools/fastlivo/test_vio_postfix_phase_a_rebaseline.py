#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from compare_vio_postfix_phase_a_to_prefix_v2 import (
    _campaign_build_matches,
    build_sensitivity_report,
    per_run_csv,
)
from generate_vio_postfix_phase_a_rebaseline import (
    ANCHORS_SCHEMA,
    ARM_IDS,
    BUILD_SCHEMA,
    PHASE_A_ARMS,
    QUALIFICATION_PLAN_SCHEMA,
    QUALIFICATION_REPORT_SCHEMA,
    SCHEMA,
    SESSION_IDS,
    prepare_orchestration,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    EVALUATOR,
    REPLAY,
    REPLAY_LAUNCH,
    REQUIRED_ESTIMATOR_LIBRARIES,
    SCHEMA as CAMPAIGN_SCHEMA,
    load_arms,
    object_sha256,
    sha256,
)


TOOLS = Path(__file__).resolve().parent
HASHES = {
    "old_exe": "1" * 64,
    "new_exe": "2" * 64,
    "old_lib": "3" * 64,
    "new_lib": "4" * 64,
    "source": "5" * 64,
    "bag": "6" * 64,
    "provenance": "7" * 64,
}


def self_hashed(core):
    return {**core, "identity_sha256": object_sha256(core)}


class PostfixPhaseAWorkflowTests(unittest.TestCase):
    def setUp(self) -> None:
        self.canonical = load_arms(PHASE_A_ARMS)
        self.temp = tempfile.TemporaryDirectory(
            dir=TOOLS, prefix="_test_postfix_phase_a_")
        self.root = Path(self.temp.name)
        self.reference_campaign = self.root / "prefix_v2"
        self.reference_campaign.mkdir()
        self.spec = self.root / "sessions.json"
        self.base = self.root / "base.yaml"
        self.thresholds = self.root / "thresholds.yaml"
        self.spec.write_text("{}\n")
        self.base.write_text("{}\n")
        self.thresholds.write_text("{}\n")

        self.reference_sessions = []
        for index, session_id in enumerate(SESSION_IDS):
            crop = {
                "basis": "cached stable-hover start through cached landing start",
                "start_s": float(index + 1),
                "duration_s": 30.0 + index,
                "full_duration_s": 30.0 + index,
                "smoke_truncated": False,
                "window_method": "fixture",
            }
            self.reference_sessions.append({
                "id": session_id,
                "condition": "fixture",
                "split": "development",
                "input_bag": str(self.root / "hybrid" / f"{session_id}.bag"),
                "input_declared_sha256": HASHES["bag"],
                "input_provenance_sha256": HASHES["provenance"],
                "window_cache": str(self.root / "cache" / f"{session_id}.json"),
                "crop": crop,
            })
        old_build = {
            "executable_sha256": HASHES["old_exe"],
            "dynamic_libraries": {
                name: {"sha256": HASHES["old_lib"]}
                for name in REQUIRED_ESTIMATOR_LIBRARIES
            },
        }
        reference_core = {
            "schema": CAMPAIGN_SCHEMA,
            "campaign_id": "phase_a_ofat_clean_v2",
            "mode": "full",
            "replay": {
                "rate": 1.0,
                "no_gt_anchor": True,
                "with_propagated": True,
            },
            "build": old_build,
            "dependencies": {
                "session_spec": {"path": str(self.spec)},
                "arms": {"path": str(PHASE_A_ARMS.resolve()),
                         "sha256": sha256(PHASE_A_ARMS.resolve())},
                "base_overlay": {"path": str(self.base.resolve()),
                                 "sha256": sha256(self.base.resolve())},
                "thresholds": {"path": str(self.thresholds.resolve()),
                               "sha256": sha256(self.thresholds.resolve())},
                "strict_evaluator": {"path": str(EVALUATOR.resolve()),
                                     "sha256": sha256(EVALUATOR.resolve())},
                "replay_wrapper": {"path": str(REPLAY.resolve()),
                                   "sha256": sha256(REPLAY.resolve())},
                "replay_launch": {"path": str(REPLAY_LAUNCH.resolve()),
                                  "sha256": sha256(REPLAY_LAUNCH.resolve())},
            },
            "arms": copy.deepcopy(self.canonical),
            "sessions": copy.deepcopy(self.reference_sessions),
        }
        self.reference = self_hashed(reference_core)
        self.reference_path = self.reference_campaign / "campaign.json"
        self.reference_path.write_text(json.dumps(self.reference))

        anchor_rows = []
        self.anchor_stamps = {}
        for index, session in enumerate(self.reference_sessions):
            stamp = str(1_700_000_000_000_000_001 + index)
            self.anchor_stamps[session["id"]] = stamp
            anchor_rows.append({
                "session_id": session["id"],
                "split": "development",
                "input": {
                    "path": session["input_bag"],
                    "declared_sha256": session["input_declared_sha256"],
                    "input_provenance_sha256":
                        session["input_provenance_sha256"],
                    "full_file_sha256_verified": True,
                },
                "crop": copy.deepcopy(session["crop"]),
                "anchor": {
                    "anchor_stamp_ns": stamp,
                    "anchor_mode_required": "explicit",
                    "anchor_definition":
                        "earliest_explicit_eligible_full_sync_sensor_epoch",
                    "init_anchor_max_predecessor_gap_s": 0.02,
                },
            })
        anchors_core = {
            "schema": ANCHORS_SCHEMA,
            "scope": "development_only",
            "validation_data_accessed": False,
            "reference_phase_a_campaign_identity_sha256":
                self.reference["identity_sha256"],
            "session_count": 5,
            "session_ids": list(SESSION_IDS),
            "sessions": anchor_rows,
        }
        self.anchors = self_hashed(anchors_core)
        self.anchors_path = self.root / "anchors.json"
        self.anchors_path.write_text(json.dumps(self.anchors))

        build_core = {
            "schema": BUILD_SCHEMA,
            "container": "fixture-container",
            "replay_devel": "/tmp/fresh-postfix-devel",
            "binary_identity": {
                "container": "fixture-container",
                "replay_devel": "/tmp/fresh-postfix-devel",
                "executable_sha256": HASHES["new_exe"],
            },
            "executable_sha256": HASHES["new_exe"],
            "dynamic_libraries": {
                name: HASHES["new_lib"]
                for name in REQUIRED_ESTIMATOR_LIBRARIES
            },
            "source_tree_sha256": HASHES["source"],
            "derived_from_actual_isolated_devel_and_source": True,
        }
        self.build = self_hashed(build_core)
        self.build_path = self.root / "build.json"
        self.build_path.write_text(json.dumps(self.build))

        explicit = {
            session_id: {"imu": {
                "init_anchor_stamp_ns": self.anchor_stamps[session_id],
                "init_anchor_max_predecessor_gap_s": 0.02,
            }} for session_id in SESSION_IDS
        }
        plan_core = {
            "schema": QUALIFICATION_PLAN_SCHEMA,
            "scope": "development_only",
            "validation_data_accessed": False,
            "reference_phase_a_campaign_identity_sha256":
                self.reference["identity_sha256"],
            "anchors_artifact_identity_sha256": self.anchors["identity_sha256"],
            "frozen_phase_a_arms_sha256": object_sha256(self.canonical),
            "postfix_phase_a_rebaseline": {
                "rate": 1.0,
                "repeats_per_arm_session": 1,
                "expected_arm_count": 8,
                "expected_run_count": 40,
                "expected_session_ids": list(SESSION_IDS),
                "old_scores_may_be_pooled": False,
                "reuse_old_completion_pointers": False,
                "validation_access_allowed": False,
            },
            "postfix_phase_a_explicit_anchor_overrides": explicit,
        }
        self.qualification_plan = self_hashed(plan_core)
        self.plan_path = self.root / "qualification_plan.json"
        self.plan_path.write_text(json.dumps(self.qualification_plan))

        report_core = {
            "schema": QUALIFICATION_REPORT_SCHEMA,
            "scope": "development_only",
            "validation_data_accessed": False,
            "status": "pass",
            "go_for_postfix_phase_a_rebaseline": True,
            "failures": [],
            "plan_identity_sha256": self.qualification_plan["identity_sha256"],
            "postfix_build_identity_sha256": self.build["identity_sha256"],
            "postfix_executable_sha256": HASHES["new_exe"],
        }
        self.report = self_hashed(report_core)
        self.report_path = self.root / "qualification_report.json"
        self.report_path.write_text(json.dumps(self.report))

    def tearDown(self) -> None:
        self.temp.cleanup()

    def prepare(self):
        output = self.root / "prepared"
        with mock.patch("rosbag.Bag", side_effect=AssertionError(
                "generator must not open a bag")):
            result = prepare_orchestration(
                self.qualification_plan, self.report, self.build,
                self.anchors, self.reference, self.canonical, {},
                qualification_plan_path=self.plan_path,
                qualification_report_path=self.report_path,
                build_path=self.build_path,
                anchors_path=self.anchors_path,
                reference_campaign=self.reference_campaign,
                arms_source_path=PHASE_A_ARMS,
                base_overlay_path=self.base,
                thresholds_path=self.thresholds,
                output_root=output,
                container="fixture-container",
                replay_devel="/tmp/fresh-postfix-devel",
                port=11421,
                python_executable="/usr/bin/python3",
            )
        return output, result

    def test_prepare_exact_8x5_rate1_anchor_grid_without_execution(self) -> None:
        output, result = self.prepare()
        self.assertEqual(result["schema"], SCHEMA)
        self.assertEqual(result["expected_run_count"], 40)
        self.assertEqual(result["arm_ids"], list(ARM_IDS))
        self.assertEqual(result["session_ids"], list(SESSION_IDS))
        self.assertFalse(result["replay_executed_by_generator"])
        self.assertFalse(result["build_executed_by_generator"])
        commands = json.loads((output / "commands.json").read_text())
        self.assertEqual(len(commands["commands"]), 5)
        for row, command in zip(result["sessions"], commands["commands"]):
            session_id = row["session_id"]
            self.assertEqual(command[command.index("--session") + 1], session_id)
            self.assertEqual(command[command.index("--rate") + 1], "1")
            self.assertEqual(
                command[:3],
                ["env", "-u", "FASTLIVO_QUALIFICATION_RUN_BINDING"])
            self.assertNotIn("--smoke", command)
            arms = load_arms(output / "arms" / f"{session_id}.yaml")
            self.assertEqual([arm["id"] for arm in arms], list(ARM_IDS))
            self.assertEqual(
                arms[0]["overrides"]["imu"]["init_anchor_stamp_ns"],
                self.anchor_stamps[session_id])
            self.assertEqual(
                arms[1]["overrides"]["imu"]["acc_cov"], 5.0)
            self.assertEqual(row["expected_cell_count"], 8)
        with self.assertRaises(FileExistsError):
            self.prepare()

    def test_prepare_refuses_failed_qualification(self) -> None:
        report = copy.deepcopy(self.report)
        report["status"] = "fail"
        report["go_for_postfix_phase_a_rebaseline"] = False
        core = dict(report)
        core.pop("identity_sha256")
        report["identity_sha256"] = object_sha256(core)
        with self.assertRaisesRegex(CampaignError, "not a bound GO"):
            prepare_orchestration(
                self.qualification_plan, report, self.build,
                self.anchors, self.reference, self.canonical, {},
                qualification_plan_path=self.plan_path,
                qualification_report_path=self.report_path,
                build_path=self.build_path,
                anchors_path=self.anchors_path,
                reference_campaign=self.reference_campaign,
                arms_source_path=PHASE_A_ARMS,
                base_overlay_path=self.base,
                thresholds_path=self.thresholds,
                output_root=self.root / "must_not_exist",
                container="fixture-container",
                replay_devel="/tmp/fresh-postfix-devel", port=11421)
        self.assertFalse((self.root / "must_not_exist").exists())

    def test_prepare_refuses_pre_fix_build_reuse(self) -> None:
        build = copy.deepcopy(self.build)
        build["executable_sha256"] = HASHES["old_exe"]
        build["binary_identity"]["executable_sha256"] = HASHES["old_exe"]
        build["dynamic_libraries"] = {
            name: HASHES["old_lib"] for name in REQUIRED_ESTIMATOR_LIBRARIES
        }
        build_core = dict(build)
        build_core.pop("identity_sha256")
        build = self_hashed(build_core)
        report = copy.deepcopy(self.report)
        report["postfix_build_identity_sha256"] = build["identity_sha256"]
        report["postfix_executable_sha256"] = HASHES["old_exe"]
        report_core = dict(report)
        report_core.pop("identity_sha256")
        report = self_hashed(report_core)
        with self.assertRaisesRegex(CampaignError, "identical to the pre-fix"):
            prepare_orchestration(
                self.qualification_plan, report, build,
                self.anchors, self.reference, self.canonical, {},
                qualification_plan_path=self.plan_path,
                qualification_report_path=self.report_path,
                build_path=self.build_path,
                anchors_path=self.anchors_path,
                reference_campaign=self.reference_campaign,
                arms_source_path=PHASE_A_ARMS,
                base_overlay_path=self.base,
                thresholds_path=self.thresholds,
                output_root=self.root / "must_not_exist",
                container="fixture-container",
                replay_devel="/tmp/fresh-postfix-devel", port=11421)

    def test_prepare_refuses_comparison_dependency_drift(self) -> None:
        self.thresholds.write_text("changed: true\n")
        with self.assertRaisesRegex(
                CampaignError, "dependency differs from pre-fix v2: thresholds"):
            prepare_orchestration(
                self.qualification_plan, self.report, self.build,
                self.anchors, self.reference, self.canonical, {},
                qualification_plan_path=self.plan_path,
                qualification_report_path=self.report_path,
                build_path=self.build_path,
                anchors_path=self.anchors_path,
                reference_campaign=self.reference_campaign,
                arms_source_path=PHASE_A_ARMS,
                base_overlay_path=self.base,
                thresholds_path=self.thresholds,
                output_root=self.root / "must_not_exist",
                container="fixture-container",
                replay_devel="/tmp/fresh-postfix-devel", port=11421)
        self.assertFalse((self.root / "must_not_exist").exists())

    @staticmethod
    def _summary(plan, *, postfix: bool):
        rows = []
        for arm_index, arm_id in enumerate(ARM_IDS):
            for session_index, session_id in enumerate(SESSION_IDS):
                # The post-fix ranking deliberately reverses the leading
                # ordering so the comparison proves independent ranking.
                base = ((len(ARM_IDS) - arm_index) if postfix else
                        (arm_index + 1)) * 0.01 + session_index * 0.001
                rows.append({
                    "arm_id": arm_id,
                    "session_id": session_id,
                    "split": "development",
                    "status": "fail",
                    "accuracy_rankable": True,
                    "accuracy_screen_eligible": True,
                    "accuracy_screen_blockers": [],
                    "translation_ape_rmse_m": 0.10 + base,
                    "translation_rpe_1p0s_rmse_m": 0.04 + base / 2,
                    "orientation_rmse_deg": 2.0 + base,
                    "path_ratio": 1.0 + base / 10,
                })
        return {
            "campaign": "/fixture",
            "campaign_identity_sha256": plan["identity_sha256"],
            "scope": "development_only",
            "ranking_validation_forbidden": True,
            "runs": rows,
        }

    def test_sensitivity_report_keeps_epoch_scores_separate(self) -> None:
        prefix_plan = {
            "mode": "full", "identity_sha256": "prefix",
            "arms": copy.deepcopy(self.canonical),
            "sessions": [{"id": session, "split": "development"}
                         for session in SESSION_IDS],
        }
        postfix_plan = copy.deepcopy(prefix_plan)
        postfix_plan["identity_sha256"] = "postfix"
        orchestration = {
            "identity_sha256": "orchestration",
            "qualified_build": {"identity_sha256": "build"},
        }
        report = build_sensitivity_report(
            orchestration, prefix_plan, self._summary(prefix_plan, postfix=False),
            postfix_plan, self._summary(postfix_plan, postfix=True),
            self.canonical, [f"child-{i}" for i in range(5)])
        self.assertFalse(report["old_scores_may_be_pooled"])
        self.assertFalse(report["promotion_decision_allowed"])
        self.assertFalse(report["phase_b_generation_allowed"])
        self.assertEqual(report["exact_grid"]["paired_cell_count"], 40)
        self.assertNotEqual(
            report["prefix_only_selection"]["selected_top_two"],
            report["postfix_only_selection"]["selected_top_two"])
        self.assertIn("delta_semantics", report)
        self.assertEqual(len(per_run_csv(report).splitlines()), 41)

    def test_sensitivity_report_rejects_incomplete_matrix(self) -> None:
        plan = {
            "mode": "full", "identity_sha256": "epoch",
            "arms": copy.deepcopy(self.canonical),
            "sessions": [{"id": session, "split": "development"}
                         for session in SESSION_IDS],
        }
        incomplete = self._summary(plan, postfix=True)
        incomplete["runs"].pop()
        with self.assertRaisesRegex(CampaignError, "complete exact 8x5"):
            build_sensitivity_report(
                {"identity_sha256": "o",
                 "qualified_build": {"identity_sha256": "b"}},
                plan, self._summary(plan, postfix=False), plan, incomplete,
                self.canonical, ["a"] * 5)

    def test_child_build_match_requires_source_and_all_libraries(self) -> None:
        qualified = self.build
        plan = {
            "build": {
                "container": qualified["container"],
                "replay_devel": qualified["replay_devel"],
                "executable_sha256": qualified["executable_sha256"],
                "dynamic_libraries": {
                    name: {"sha256": value}
                    for name, value in qualified["dynamic_libraries"].items()
                },
            },
            "dependencies": {"fastlivo_source_tree": {
                "tree_sha256": qualified["source_tree_sha256"]}},
        }
        self.assertTrue(_campaign_build_matches(plan, qualified))
        plan["dependencies"]["fastlivo_source_tree"]["tree_sha256"] = "0" * 64
        self.assertFalse(_campaign_build_matches(plan, qualified))


if __name__ == "__main__":
    unittest.main()

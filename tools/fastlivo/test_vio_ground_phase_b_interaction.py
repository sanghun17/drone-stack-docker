#!/usr/bin/env python3

from __future__ import annotations

import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

import build_vio_ground_phase_b_interaction_report as reporter  # noqa: E402
import generate_vio_ground_phase_b_interaction as generator  # noqa: E402
import run_vio_ground_phase_b_interaction as runner  # noqa: E402
from run_vio_flight_tuning_campaign import (  # noqa: E402
    CampaignError,
    load_json,
)


TOOLS = Path(__file__).resolve().parent
PHASE_A_ROOT = (
    TOOLS / "_campaign_vio_flight_20260814" /
    "ground_phase_a_rebaseline_v1"
)
PHASE_A_ORCHESTRATION = PHASE_A_ROOT / "orchestration.json"
PHASE_A_REPORT = PHASE_A_ROOT / "phase_a_report.json"
BASE = TOOLS / "mock_candidate3_full_livo_hybrid_imu.yaml"
THRESHOLDS = TOOLS / "vio_flight_readiness_thresholds.yaml"
FAILED_ATTEMPT = generator.DEFAULT_FAILED_ATTEMPT


class GroundPhaseBInteractionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(
            dir=TOOLS, prefix="_test_ground_phase_b_workflow_")
        self.root = Path(self.temp.name)

    def tearDown(self) -> None:
        self.temp.cleanup()

    def _prepare(self):
        output = self.root / "prepared"
        with mock.patch("subprocess.run", side_effect=AssertionError(
                "Phase-B generator must not replay/build/evaluate")):
            orchestration = generator.prepare_orchestration(
                PHASE_A_ORCHESTRATION,
                PHASE_A_REPORT,
                BASE,
                THRESHOLDS,
                output,
                FAILED_ATTEMPT,
                port=11432,
                python_executable=sys.executable,
                verify_actual_build=False,
            )
        return output, orchestration

    def test_frozen_schedule_is_exact_balanced_interleaved_60(self):
        schedule = generator.frozen_schedule()
        self.assertEqual(len(schedule), 60)
        self.assertEqual(len(set(schedule)), 60)
        self.assertEqual(set(schedule), {
            (repeat_id, session_id, config_id)
            for repeat_id in generator.REPEAT_IDS
            for session_id in generator.SESSION_IDS
            for config_id in generator.CONFIG_IDS
        })
        positions = {config_id: [0, 0, 0, 0]
                     for config_id in generator.CONFIG_IDS}
        per_session = {
            (session_id, config_id): [0, 0, 0, 0]
            for session_id in generator.SESSION_IDS
            for config_id in generator.CONFIG_IDS
        }
        block_strata = []
        for start in range(0, 60, 4):
            block = schedule[start:start + 4]
            self.assertEqual({row[2] for row in block},
                             set(generator.CONFIG_IDS))
            self.assertEqual(len({row[:2] for row in block}), 1)
            block_strata.append(block[0][:2])
            for position, row in enumerate(block):
                positions[row[2]][position] += 1
                per_session[(row[1], row[2])][position] += 1
        self.assertEqual(len(set(block_strata)), 15)
        for counts in positions.values():
            self.assertLessEqual(max(counts) - min(counts), 1)
        for counts in per_session.values():
            self.assertLessEqual(max(counts) - min(counts), 1)

    def test_prepares_exact_grid_and_binds_transitive_producers(self):
        output, orchestration = self._prepare()
        phase_a = load_json(PHASE_A_ORCHESTRATION)
        self.assertEqual(orchestration["design"]["expected_run_count"], 60)
        self.assertEqual(orchestration["design"]
                         ["repeats_per_configuration_session"], 3)
        observed = [(row["repeat_id"], row["session_id"],
                     row["configuration_id"])
                    for row in orchestration["cells"]]
        self.assertEqual(observed, generator.frozen_schedule())
        self.assertEqual(orchestration["qualified_build"],
                         phase_a["qualified_build"])
        self.assertEqual(orchestration["runtime_constant_binding"],
                         phase_a["runtime_constant_binding"])
        self.assertIn("phase_a_selector", orchestration["dependencies"])
        self.assertIn("phase_a_summarizer", orchestration["dependencies"])
        self.assertEqual(
            Path(orchestration["dependencies"]["phase_a_selector"]["path"]),
            generator.PHASE_A_SELECTOR.resolve())
        self.assertEqual(
            Path(orchestration["dependencies"]["phase_a_summarizer"]["path"]),
            generator.PHASE_A_SUMMARIZER.resolve())
        self.assertFalse(orchestration["validation_data_accessed"])
        self.assertFalse(orchestration["decision_contract"]
                         ["candidate_promotion_allowed"])
        self.assertFalse(orchestration["decision_contract"]
                         ["flight_ready_can_be_declared"])
        self.assertTrue(orchestration["decision_contract"]
                        ["global_high_rate_interface_remains_no_go"])
        retry = orchestration["infrastructure_retry_contract"]
        self.assertEqual(retry["maximum_retries_per_cell"], 1)
        self.assertTrue(retry["unapproved_failure_stops_sequence"])
        self.assertTrue(retry["second_occurrence_stops_sequence"])
        self.assertFalse(retry["approved_retry_enters_configuration_tuning_rank"])
        self.assertTrue(orchestration["execution"]
                        ["sequence_stops_on_unapproved_or_second_failure"])
        commands = load_json(output / "commands.json")
        self.assertEqual(commands["command_count"], 60)
        self.assertTrue(commands["stop_on_unapproved_or_second_failure"])
        self.assertNotIn("continue_after_cell_failure", commands)

        expected_by_id = {row["id"]: row
                          for row in generator.CONFIGURATIONS}
        for row in orchestration["cells"]:
            cell = load_json(Path(row["cell"]["path"]))
            config = expected_by_id[row["configuration_id"]]
            self.assertEqual(cell["runtime_overrides"]["imu"]["acc_cov"],
                             config["acc_cov"])
            self.assertEqual(cell["runtime_overrides"]["vio"], {
                "img_point_cov": 1000.0,
                "outlier_threshold": config["outlier_threshold"],
            })
            self.assertEqual(
                cell["runtime_overrides"]["imu"]["init_anchor_stamp_ns"],
                cell["session"]["explicit_anchor"]["anchor_stamp_ns"])
            self.assertTrue(cell["evaluation_contract"]
                            ["secondary_reuses_primary_alignment_without_refit"])

    def test_historical_failed_attempt_is_bound_by_structured_zero_output_evidence(self):
        row = generator._validate_failed_attempt(FAILED_ATTEMPT, PHASE_A_ROOT)
        self.assertEqual(row["structured_rosout_failure_event"]["status"],
                         "failed")
        self.assertEqual(
            row["structured_rosout_failure_event"]["reason"],
            runner.RETRY_ERROR_TEXT)
        self.assertTrue(row["zero_estimator_outputs"])
        self.assertEqual(set(row["estimator_output_message_counts"]),
                         set(runner.ESTIMATOR_OUTPUT_TOPICS))
        self.assertFalse(any(row["estimator_output_message_counts"].values()))
        self.assertTrue(row["excluded_from_phase_b_numeric_pool"])

    def test_retry_classifier_needs_exact_event_and_zero_all_outputs(self):
        attempt = self.root / "retry_attempt"
        attempt.mkdir()
        (attempt / "result.bag").write_bytes(b"")
        (attempt / "result_params.yaml").write_text(
            "imu:\n  init_anchor_stamp_ns: '100'\n")
        exact_event = {
            "schema": "fast_livo/imu_init/v1",
            "status": "failed",
            "anchor_mode": "explicit",
            "anchor_stamp_ns": "100",
            "sync_epoch_ns": "200",
            "reason": runner.RETRY_ERROR_TEXT,
        }
        inventory = {topic: {"message_count": 0}
                     for topic in runner.ESTIMATOR_OUTPUT_TOPICS}
        with mock.patch.object(runner, "_structured_failed_events",
                               return_value=[exact_event]), mock.patch.object(
                                   runner, "bag_topic_inventory",
                                   return_value=inventory):
            eligible, evidence = runner._eligible_startup_retry(
                attempt)
        self.assertTrue(eligible)
        self.assertTrue(evidence["zero_estimator_outputs"])

        nonzero = copy.deepcopy(inventory)
        nonzero[runner.ESTIMATOR_OUTPUT_TOPICS[0]]["message_count"] = 1
        with mock.patch.object(runner, "_structured_failed_events",
                               return_value=[exact_event]), mock.patch.object(
                                   runner, "bag_topic_inventory",
                                   return_value=nonzero):
            eligible, _ = runner._eligible_startup_retry(
                attempt)
        self.assertFalse(eligible)

        wrong = {**exact_event, "reason": "anything else"}
        with mock.patch.object(runner, "_structured_failed_events",
                               return_value=[wrong]), mock.patch.object(
                                   runner, "bag_topic_inventory",
                                   return_value=inventory):
            eligible, _ = runner._eligible_startup_retry(
                attempt)
        self.assertFalse(eligible)

    def test_orphan_attempt_directory_is_fail_closed(self):
        campaign = self.root / "campaign"
        orphan = campaign / "attempts" / "killed_without_receipt"
        orphan.mkdir(parents=True)
        (orphan / "replay.stdout.log").write_text("interrupted")
        with self.assertRaisesRegex(CampaignError, "orphan/incomplete"):
            runner._inventory_attempt_directories(campaign)

    def test_determinism_is_hard_gate_and_retry_never_enters_rank(self):
        rows = []
        for config_order, config_id in enumerate(generator.CONFIG_IDS):
            for repeat_id in generator.REPEAT_IDS:
                for session_id in generator.SESSION_IDS:
                    rows.append({
                        "configuration_id": config_id,
                        "repeat_id": repeat_id,
                        "session_id": session_id,
                        "normalized_accuracy_max": float(config_order + 1),
                        "accuracy_metrics": {
                            "translation_ape_rmse_m": float(config_order + 1),
                            "translation_rpe_1p0s_rmse_m":
                                float(config_order + 1),
                            "orientation_rmse_deg": float(config_order + 1),
                            "path_ratio": float(config_order + 1),
                        },
                        "startup_retry_count": (
                            1 if config_id == generator.CONFIG_IDS[0] and
                            repeat_id == "r1" and
                            session_id == generator.SESSION_IDS[0] else 0),
                        "low_rate_output_reliability": {
                            "failed_check_count": 0,
                            "passed": True,
                        },
                    })
        gates = [{
            "configuration_id": config_id,
            "session_id": session_id,
            "all_required_exact": True,
        } for config_id in generator.CONFIG_IDS
            for session_id in generator.SESSION_IDS]
        selection = reporter.rank_configurations(rows, gates)
        self.assertTrue(selection["selection_complete"])
        # The best config deliberately contains an approved retry.  It remains
        # best because approved infrastructure retries are operational-only.
        self.assertEqual(selection["best_development_configuration"],
                         generator.CONFIG_IDS[0])
        self.assertNotIn("startup_retry_count",
                         selection["ranking"][0]["lexicographic_key"])

        broken = copy.deepcopy(gates)
        broken[0]["all_required_exact"] = False
        refused = reporter.rank_configurations(rows, broken)
        self.assertFalse(refused["selection_complete"])
        self.assertEqual(refused["selection_failure"],
                         "low_rate_repeat_determinism_hard_gate")
        self.assertEqual(refused["ranking"], [])

        decomposition = reporter.factorial_decomposition(rows)
        self.assertEqual(decomposition["stratum_count"], 15)
        self.assertEqual(
            decomposition["role"],
            "diagnostic_only_does_not_change_frozen_selection_rule")
        first = decomposition["strata"][0]["effects"]
        for outcome in reporter.ACCURACY_METRICS + (
                "normalized_accuracy_max",):
            self.assertEqual(
                first[outcome]["acc_cov_main_effect_5_minus_10"], 1.0)
            self.assertEqual(
                first[outcome]["outlier_main_effect_600_minus_1000"], 2.0)
            self.assertEqual(
                first[outcome]["interaction_delta_delta"], 0.0)

    def test_low_rate_exactness_is_gate_high_rate_is_diagnostic(self):
        base_streams = {
            "low_rate_pose": "pose",
            "low_rate_init": "init",
            "correction": "correction",
            "propagated_odom": "high-0",
            "world_twist": "twist-0",
        }
        rows = []
        for index, repeat_id in enumerate(generator.REPEAT_IDS):
            streams = dict(base_streams)
            streams["propagated_odom"] = f"high-{index}"
            streams["world_twist"] = f"twist-{index}"
            rows.append({
                "repeat_id": repeat_id,
                "stream_signatures": streams,
                "initialization_signature_sha256": "same-init",
                "first_correction_signature_sha256": "same-correction",
                "all_local_metrics_signature_sha256": "same-metrics",
            })
        gate = reporter._repeat_determinism(rows)
        self.assertTrue(gate["all_required_exact"])
        self.assertEqual(
            gate["high_rate_diagnostic_signature_counts"]
                ["propagated_odom"], 3)
        self.assertTrue(gate["high_rate_payload_is_not_determinism_gate"])

        broken = copy.deepcopy(rows)
        broken[2]["stream_signatures"]["correction"] = "different"
        gate = reporter._repeat_determinism(broken)
        self.assertFalse(gate["all_required_exact"])
        self.assertFalse(gate["exact"]["correction"])

    def test_rosbag_record_time_age_failures_do_not_change_rank_key(self):
        def primary(age_status):
            checks = [
                {"id": check_id, "status": "pass"}
                for check_id in reporter.LOW_RATE_RELIABILITY_CHECKS
            ]
            checks.extend({"id": check_id, "status": age_status}
                          for check_id in
                          reporter.RECORD_TIME_AGE_DIAGNOSTIC_CHECKS)
            return {"checks": checks}

        passed = reporter._low_rate_reliability(primary("pass"))
        failed = reporter._low_rate_reliability(primary("fail"))
        self.assertEqual(passed["failed_check_count"], 0)
        self.assertEqual(failed["failed_check_count"], 0)
        self.assertTrue(failed["passed"])
        self.assertNotEqual(
            passed["rosbag_callback_record_time_age_diagnostic_status"],
            failed["rosbag_callback_record_time_age_diagnostic_status"])

    def test_preflight_is_read_only_and_does_not_replay(self):
        output, orchestration = self._prepare()
        with mock.patch.object(runner, "_validate_gate_and_build"), \
                mock.patch.object(runner, "_validate_input",
                                  return_value="0" * 64), \
                mock.patch.object(runner, "check_no_live_worker") as worker, \
                mock.patch("subprocess.run", side_effect=AssertionError(
                    "preflight must not launch replay/build/evaluator")):
            receipt = runner.preflight(
                output / "orchestration.json", output / "commands.json",
                verify_actual_build=False, verify_inputs=True)
        self.assertEqual(receipt["status"], "go")
        self.assertEqual(receipt["command_count"], 60)
        self.assertEqual(receipt["unique_development_inputs_verified"], 5)
        self.assertFalse(receipt["replay_executed"])
        self.assertFalse(receipt["build_executed"])
        self.assertTrue(receipt["no_active_worker_verified"])
        worker.assert_called_once_with(
            orchestration["qualified_build"]["container"],
            orchestration["ros_master_port"])
        self.assertFalse(receipt["candidate_promotion_allowed"])
        self.assertFalse(receipt["flight_ready"])
        self.assertTrue(receipt["global_high_rate_interface_remains_no_go"])
        self.assertEqual(orchestration["identity_sha256"],
                         receipt["orchestration_identity_sha256"])


if __name__ == "__main__":
    unittest.main()

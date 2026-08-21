#!/usr/bin/env python3

from __future__ import annotations

import copy
import unittest

from compare_vio_flight_tuning_phase_a_replicates import (
    compare_phase_a_replicates,
    consensus_phase_b_documents,
    per_run_csv,
)
from run_vio_flight_tuning_campaign import (
    CampaignError,
    SCHEMA as CAMPAIGN_SCHEMA,
    object_sha256,
)
from summarize_vio_flight_tuning_campaign import SUMMARY_SCHEMA


def fixture():
    arms = [
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
    sessions = ["pw1_20260804_052639", "p0_20260804_211027"]

    def plan(name):
        identity = {
            "schema": CAMPAIGN_SCHEMA,
            "campaign_id": name,
            "mode": "full",
            "arms": copy.deepcopy(arms),
            "sessions": [
                {"id": session, "split": "development"}
                for session in sessions
            ],
        }
        return {**identity, "identity_sha256": object_sha256(identity)}

    scores = {
        "baseline_acc10_img1000_out1000": [0.5, 0.5],
        "acc5": [0.4, 0.45],
        "acc20": [0.8, 0.9],
        "img300": [0.7, 0.65],
        "img3000": [0.5, 0.55],
        "outlier100": [0.7, 0.8],
        "outlier300": [0.4, 0.45],
        "outlier600": [0.6, 0.65],
    }

    def summary(which_plan, campaign, offset=0.0):
        runs = []
        for arm in arms:
            for session_id, normalized in zip(sessions, scores[arm["id"]]):
                value = normalized + offset
                runs.append({
                    "arm_id": arm["id"],
                    "session_id": session_id,
                    "split": "development",
                    "status": "fail",
                    "accuracy_rankable": True,
                    "accuracy_screen_eligible": True,
                    "accuracy_screen_blockers": [],
                    "translation_ape_rmse_m": 0.25 * value,
                    "translation_rpe_1p0s_rmse_m": 0.005,
                    "orientation_rmse_deg": 0.5,
                    "path_ratio": 1.0,
                })
        return {
            "schema": SUMMARY_SCHEMA,
            "campaign": campaign,
            "campaign_identity_sha256": which_plan["identity_sha256"],
            "scope": "development_only",
            "ranking_validation_forbidden": True,
            "runs": runs,
        }

    plan_a = plan("replicate_a")
    plan_b = plan("replicate_b")
    return (arms, plan_a, summary(plan_a, "/tmp/a"),
            plan_b, summary(plan_b, "/tmp/b", offset=0.02))


def rehash_plan(plan):
    identity = dict(plan)
    identity.pop("identity_sha256", None)
    plan["identity_sha256"] = object_sha256(identity)


class PhaseAReplicateComparisonTests(unittest.TestCase):
    def test_reports_per_run_deltas_and_rank_agreement(self):
        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=arms)
        self.assertEqual(report["exact_grid"]["expected_run_count_per_replicate"], 16)
        self.assertEqual(len(report["per_run"]), 16)
        self.assertAlmostEqual(
            report["per_run"][0]["raw_delta_b_minus_a"][
                "translation_ape_rmse_m"],
            0.005)
        self.assertTrue(report["global_rank_agreement"]["exact_order_agreement"])
        self.assertAlmostEqual(report["global_rank_agreement"]["spearman_rho"], 1.0)
        self.assertAlmostEqual(report["global_rank_agreement"]["kendall_tau"], 1.0)
        self.assertTrue(report["all_three_family_selections_agree"])
        self.assertTrue(report["consensus_phase_b_generation_allowed"])
        self.assertFalse(report["validation_data_accessed"])
        self.assertEqual(
            report["replicate_a"]["campaign_identity_sha256"],
            plan_a["identity_sha256"])
        self.assertEqual(
            report["replicate_b"]["campaign_identity_sha256"],
            plan_b["identity_sha256"])

    def test_detects_family_selection_and_rank_disagreement(self):
        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        # Reverse acc ordering only in replicate B.
        for run in summary_b["runs"]:
            if run["arm_id"] == "acc5":
                run["translation_ape_rmse_m"] = 0.24
            elif run["arm_id"] == "acc20":
                run["translation_ape_rmse_m"] = 0.025
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=arms)
        acc = next(row for row in report["family_comparisons"]
                   if row["family"] == "acc_cov")
        self.assertFalse(acc["selected_level_agreement"])
        self.assertIsNone(acc["consensus_nonbaseline_arm"])
        self.assertFalse(acc["rank_agreement"]["exact_order_agreement"])
        self.assertFalse(report["all_three_family_selections_agree"])
        self.assertFalse(report["consensus_phase_b_generation_allowed"])

    def test_refuses_different_or_incomplete_grid(self):
        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        plan_b["sessions"].reverse()
        rehash_plan(plan_b)
        summary_b["campaign_identity_sha256"] = plan_b["identity_sha256"]
        with self.assertRaisesRegex(CampaignError, "exact same ordered"):
            compare_phase_a_replicates(
                summary_a, plan_a, summary_b, plan_b, expected_arms=arms)

        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        summary_b["runs"].pop()
        with self.assertRaisesRegex(CampaignError, "matrix is incomplete"):
            compare_phase_a_replicates(
                summary_a, plan_a, summary_b, plan_b, expected_arms=arms)

    def test_refuses_same_campaign_identity(self):
        arms, plan_a, summary_a, _, _ = fixture()
        with self.assertRaisesRegex(CampaignError, "with itself"):
            compare_phase_a_replicates(
                summary_a, plan_a, copy.deepcopy(summary_a),
                copy.deepcopy(plan_a), expected_arms=arms)

    def test_csv_contains_one_row_per_exact_grid_cell(self):
        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=arms)
        payload = per_run_csv(report)
        self.assertEqual(len(payload.strip().splitlines()), 17)
        self.assertIn("delta_translation_ape_rmse_m", payload.splitlines()[0])

    def test_consensus_arms_bind_both_campaigns_and_are_unique(self):
        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=arms)
        document, provenance = consensus_phase_b_documents(report)
        self.assertEqual(len(document["arms"]), 8)
        self.assertEqual(
            len({object_sha256(row["overrides"])
                 for row in document["arms"]}), 8)
        self.assertEqual(
            provenance["source_campaign_identity_sha256"],
            [plan_a["identity_sha256"], plan_b["identity_sha256"]])
        self.assertTrue(document["phase_b_provenance"]["replicate_consensus"])
        self.assertEqual(
            document["phase_b_provenance"]["consensus_identity_sha256"],
            provenance["consensus_identity_sha256"])

    def test_consensus_arms_refuse_selection_disagreement_or_tampering(self):
        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        for run in summary_b["runs"]:
            if run["arm_id"] == "acc5":
                run["translation_ape_rmse_m"] = 0.24
            elif run["arm_id"] == "acc20":
                run["translation_ape_rmse_m"] = 0.025
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=arms)
        with self.assertRaisesRegex(CampaignError, "replicates disagree"):
            consensus_phase_b_documents(report)

        arms, plan_a, summary_a, plan_b, summary_b = fixture()
        report = compare_phase_a_replicates(
            summary_a, plan_a, summary_b, plan_b, expected_arms=arms)
        report["per_run"][0]["normalized_max_delta_b_minus_a"] = 999.0
        with self.assertRaisesRegex(CampaignError, "identity changed"):
            consensus_phase_b_documents(report)


if __name__ == "__main__":
    unittest.main()

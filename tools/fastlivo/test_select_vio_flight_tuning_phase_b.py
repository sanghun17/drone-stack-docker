#!/usr/bin/env python3

from __future__ import annotations

import copy
import math
import unittest

from run_vio_flight_tuning_campaign import (
    CampaignError,
    SCHEMA as CAMPAIGN_SCHEMA,
    object_sha256,
)
from select_vio_flight_tuning_phase_b import (
    ARMS_SCHEMA,
    FAMILIES,
    SCHEMA,
    select_phase_b,
)
from summarize_vio_flight_tuning_campaign import SUMMARY_SCHEMA


class PhaseBSelectorTests(unittest.TestCase):
    @staticmethod
    def _fixture():
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
        identity = {
            "schema": CAMPAIGN_SCHEMA,
            "campaign_id": "synthetic_phase_a",
            "mode": "full",
            "arms": copy.deepcopy(arms),
            "sessions": [
                {"id": session, "split": "development"}
                for session in sessions
            ],
        }
        plan = {
            **identity,
            "identity_sha256": object_sha256(identity),
        }
        scores = {
            "baseline_acc10_img1000_out1000": [0.5, 0.5],
            # Better numeric score, but one hard blocker: acc20 must win.
            "acc5": [0.1, 0.1],
            "acc20": [0.9, 0.9],
            # Equal worst score; img3000 wins on mean.
            "img300": [0.7, 0.7],
            "img3000": [0.7, 0.6],
            # Equal worst score; outlier300 wins on mean.  outlier600 loses
            # on worst score despite a low score in its second session.
            "outlier100": [0.6, 0.6],
            "outlier300": [0.6, 0.4],
            "outlier600": [0.8, 0.2],
        }
        runs = []
        for arm in arms:
            arm_id = arm["id"]
            for session_id, normalized in zip(sessions, scores[arm_id]):
                blockers = (
                    ["propagated_rate"]
                    if arm_id == "acc5" and session_id == sessions[0]
                    else [])
                runs.append({
                    "arm_id": arm_id,
                    "session_id": session_id,
                    "split": "development",
                    "status": "fail",
                    "accuracy_rankable": True,
                    "accuracy_screen_eligible": not blockers,
                    "accuracy_screen_blockers": blockers,
                    "translation_ape_rmse_m": 0.25 * normalized,
                    "translation_rpe_1p0s_rmse_m": 0.005,
                    "orientation_rmse_deg": 0.5,
                    "path_ratio": 1.0,
                })
        summary = {
            "schema": SUMMARY_SCHEMA,
            "campaign": "/tmp/synthetic_phase_a",
            "campaign_identity_sha256": plan["identity_sha256"],
            "scope": "development_only",
            "ranking_validation_forbidden": True,
            "runs": runs,
        }
        return arms, plan, summary

    @staticmethod
    def _rehash_plan(plan):
        identity = dict(plan)
        identity.pop("identity_sha256", None)
        plan["identity_sha256"] = object_sha256(identity)

    def test_selects_each_family_independently_and_emits_eight_cells(self):
        arms, plan, summary = self._fixture()
        document, provenance = select_phase_b(
            summary, plan, expected_arms=arms)
        self.assertEqual(document["schema"], ARMS_SCHEMA)
        self.assertEqual(provenance["schema"], SCHEMA)
        selected = {
            row["family"]: row["selected_nonbaseline_arm"]
            for row in provenance["family_selections"]
        }
        self.assertEqual(selected, {
            "acc_cov": "acc20",
            "img_point_cov": "img3000",
            "outlier_threshold": "outlier300",
        })

        generated = document["arms"]
        self.assertEqual(len(generated), 8)
        self.assertEqual(len({row["id"] for row in generated}), 8)
        self.assertEqual(
            len({object_sha256(row["overrides"]) for row in generated}), 8)
        self.assertEqual(
            {row["overrides"]["imu"]["acc_cov"] for row in generated},
            {10.0, 20.0})
        self.assertEqual(
            {row["overrides"]["vio"]["img_point_cov"] for row in generated},
            {1000.0, 3000.0})
        self.assertEqual(
            {row["overrides"]["vio"]["outlier_threshold"]
             for row in generated},
            {1000.0, 300.0})

        self.assertEqual(
            document["phase_b_provenance"][
                "source_campaign_identity_sha256"],
            plan["identity_sha256"])
        self.assertEqual(
            document["phase_b_provenance"]["selection_identity_sha256"],
            provenance["selection_identity_sha256"])
        self.assertFalse(provenance["validation_data_accessed"])
        self.assertTrue(
            provenance["replicate_policy"][
                "do_not_execute_before_clean_phase_a_replicate_comparison"])
        self.assertTrue(
            document["phase_b_provenance"][
                "provisional_single_phase_a_campaign"])
        self.assertEqual(
            provenance["source_run_matrix_shape"]["run_count"], 16)

    def test_blocker_count_precedes_numeric_accuracy(self):
        arms, plan, summary = self._fixture()
        _, provenance = select_phase_b(summary, plan, expected_arms=arms)
        acc = next(row for row in provenance["family_selections"]
                   if row["family"] == "acc_cov")
        self.assertEqual(acc["ranking"][0]["arm_id"], "acc20")
        self.assertEqual(
            acc["ranking"][0]["hard_integration_failure_session_count"], 0)
        self.assertEqual(
            acc["ranking"][1]["hard_integration_failure_session_count"], 1)
        self.assertGreater(
            acc["ranking"][0]["worst_session_normalized_max"],
            acc["ranking"][1]["worst_session_normalized_max"])

    def test_refuses_incomplete_run_matrix(self):
        arms, plan, summary = self._fixture()
        summary["runs"].pop()
        with self.assertRaisesRegex(CampaignError, "matrix is incomplete"):
            select_phase_b(summary, plan, expected_arms=arms)

    def test_refuses_unrankable_or_nonfinite_run(self):
        arms, plan, summary = self._fixture()
        summary["runs"][0]["accuracy_rankable"] = False
        with self.assertRaisesRegex(CampaignError, "incomplete/unrankable"):
            select_phase_b(summary, plan, expected_arms=arms)

        arms, plan, summary = self._fixture()
        summary["runs"][0]["orientation_rmse_deg"] = math.nan
        with self.assertRaisesRegex(CampaignError, "missing/non-finite"):
            select_phase_b(summary, plan, expected_arms=arms)

    def test_refuses_validation_id_before_selection(self):
        arms, plan, summary = self._fixture()
        old_id = plan["sessions"][0]["id"]
        held_out = "p1_20260804_212926"
        plan["sessions"][0]["id"] = held_out
        self._rehash_plan(plan)
        summary["campaign_identity_sha256"] = plan["identity_sha256"]
        for run in summary["runs"]:
            if run["session_id"] == old_id:
                run["session_id"] = held_out
        with self.assertRaisesRegex(CampaignError, "validation/non-development"):
            select_phase_b(summary, plan, expected_arms=arms)

    def test_refuses_campaign_identity_mismatch(self):
        arms, plan, summary = self._fixture()
        summary["campaign_identity_sha256"] = "0" * 64
        with self.assertRaisesRegex(CampaignError, "identity mismatch"):
            select_phase_b(summary, plan, expected_arms=arms)

    def test_factor_order_is_frozen(self):
        self.assertEqual(
            [family["id"] for family in FAMILIES],
            ["acc_cov", "img_point_cov", "outlier_threshold"])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Decision-scope tests for the ground-init qualification checker."""

from __future__ import annotations

import copy
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_vio_ground_init_qualification as checker  # noqa: E402


class GroundCheckerDecisionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.sentinels = [{"id": "s0"}, {"id": "s1"}]
        self.runs = []
        for sentinel in self.sentinels:
            for rate in (0.5, 1.0):
                for repeat in (1, 2, 3):
                    run_id = f"{sentinel['id']}_{rate}_{repeat}"
                    self.runs.append({
                        "run_id": run_id,
                        "sentinel_id": sentinel["id"],
                        "expected_receipt": f"receipts/{run_id}.json",
                        "rate": rate,
                        "repeat": repeat,
                    })
        self.plan = {"sentinels": self.sentinels, "runs": self.runs}

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def _stream(name: str, variant: str = "stable"):
        return {
            "message_count": 10,
            "sensor_stamp_vector_sha256": f"{name}-stamps-{variant}",
            "canonical_state_sha256": f"{name}-state-{variant}",
            "first_message_binary64_be_sha256": f"{name}-first-{variant}",
        }

    def _row(self, run, *, mutate_low: bool = False):
        low_variant = "changed" if mutate_low else "stable"
        high_variant = str(run["repeat"])
        return {
            "run_id": run["run_id"],
            "sentinel_id": run["sentinel_id"],
            "rate": run["rate"],
            "repeat": run["repeat"],
            "process_instance_uuid": "uuid-" + run["run_id"],
            "receipt_identity_sha256": "receipt-" + run["run_id"],
            "build_manifest_identity_sha256": "full-build",
            "base_build_identity_sha256": "base-build",
            "build": {"executable_sha256": "estimator-executable"},
            "initialization": {"exact": "same"},
            "streams": {
                "low_rate_pose": self._stream("pose", low_variant),
                "low_rate_init": self._stream("init"),
                "correction": self._stream("correction"),
                "first_correction": {"state": "same"},
                # Deliberately nondeterministic: this must remain a warning,
                # never a low-rate rebaseline failure or a flight GO.
                "propagated_odom": self._stream("prop", high_variant),
                "world_twist": self._stream("twist", high_variant),
            },
            "accuracy_diagnostic": {"local": 1.0},
            "primary_status": "fail",
            "primary_flight_ready": False,
            "alignment_takeoff_diagnostic": {
                "first_output_to_detected_takeoff_lead_s": 0.2,
                "alignment_overlaps_detected_takeoff": True,
            },
        }

    def _check(self, mutate_run: Optional[str] = None):
        def validated(_receipt, run, *_args):
            return self._row(run, mutate_low=run["run_id"] == mutate_run)

        with mock.patch.object(
                checker, "_load_ground_plan",
                return_value=(copy.deepcopy(self.plan), "plan-identity")), \
                mock.patch.object(checker, "_receipt_path",
                                  side_effect=lambda root, value: root / value), \
                mock.patch.object(checker, "load_json", return_value={}), \
                mock.patch.object(checker, "_validate_ground_receipt",
                                  side_effect=validated):
            return checker.check_qualification(
                self.root / "plan.json", self.root)

    def test_high_rate_and_primary_failures_cannot_block_or_clear_low_rate_gate(self):
        report = self._check()
        self.assertEqual(report["status"], "pass")
        self.assertTrue(report["low_rate_estimator_rebaseline_go"])
        self.assertTrue(report["high_rate_interface_remains_no_go"])
        self.assertEqual(report["primary_strict_flight_status"], "NO_GO")
        self.assertFalse(report["flight_ready"])
        self.assertGreater(len(report["warnings"]), 0)

    def test_one_low_rate_payload_difference_fails_rebaseline_gate(self):
        report = self._check(self.runs[0]["run_id"])
        self.assertEqual(report["status"], "fail")
        self.assertFalse(report["low_rate_estimator_rebaseline_go"])
        self.assertTrue(any(
            failure["gate"] ==
            "exact_low_rate_stream_across_rates_and_repeats"
            for failure in report["failures"]))
        self.assertTrue(report["high_rate_interface_remains_no_go"])
        self.assertFalse(report["flight_ready"])


if __name__ == "__main__":
    unittest.main()

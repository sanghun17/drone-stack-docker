#!/usr/bin/env python3
"""Synthetic fail-closed tests for ground qualification postprocessing."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_vio_flight_tuning_campaign import (  # noqa: E402
    CampaignError,
    file_identity,
    object_sha256,
)
from run_vio_ground_init_qualification_cell import (  # noqa: E402
    POSTPROCESS_SCHEMA,
    _append_or_validate,
    _validate_secondary,
)


class GroundPostprocessContractTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.attempt = Path(self.temp.name) / "attempt"
        self.attempt.mkdir()
        self.result_bag = self.attempt / "result.bag"
        self.result_bag.write_bytes(b"bound result bag")
        self.thresholds = Path(self.temp.name) / "thresholds.yaml"
        self.thresholds.write_text("schema: test-thresholds/v1\nchecks: {}\n")
        self.primary_path = self.attempt / "result.flight_readiness.json"
        self.alignment = {
            "method": "initialization_window_yaw_and_translation_only",
            "scale": 1.0,
            "yaw_deg": 12.5,
            "translation_m": [1.0, 2.0, 3.0],
        }
        self.primary = {
            "schema": "fastlivo_vio_flight_readiness/v1",
            "result_bag": str(self.result_bag.resolve()),
            "status": "fail",
            "flight_ready": False,
            "evaluation_semantics": {
                "score_window_sensor_stamp_ns": None,
                "fixed_alignment_supplied": False,
            },
            "artifact_bindings": {
                "result_bag": file_identity(self.result_bag),
                "thresholds": file_identity(self.thresholds),
            },
            "local": {"alignment": self.alignment},
        }
        self.primary_path.write_text(json.dumps(self.primary))
        self.report_path = self.attempt / "result.hover_ranking.json"
        self.log_path = self.attempt / "evaluate.hover.stdout.log"
        self.log_path.write_text("complete\n")
        self.report = {
            "schema": "fastlivo_vio_ground_hover_ranking/v1",
            "result_bag": str(self.result_bag.resolve()),
            "role": "phase_a_ranking_compatibility_only",
            "flight_ready": False,
            "status": "ranking_only",
            "can_override_primary_failure": False,
            "primary_report_identity": file_identity(self.primary_path),
            "primary_status": "fail",
            "primary_flight_ready": False,
            "evaluation_semantics": {
                "primary_alignment_reused_without_refit": True,
                "fixed_alignment_supplied": True,
                "score_window_sensor_stamp_ns": {
                    "start": "101", "end": "202",
                    "boundary": "start_inclusive_end_inclusive",
                },
            },
            "artifact_bindings": {
                "result_bag": file_identity(self.result_bag),
                "thresholds": file_identity(self.thresholds),
            },
            "local_accuracy": {"alignment": {
                **self.alignment,
                "reused_from_primary_full_result": True,
            }},
        }
        self.report_path.write_text(json.dumps(self.report))
        self.postprocess_path = self.attempt / "ground_postprocess.json"
        core = {
            "schema": POSTPROCESS_SCHEMA,
            "plan_identity_sha256": "plan",
            "orchestration_identity_sha256": "orchestration",
            "build_manifest_identity_sha256": "build",
            "run_id": "run",
            "attempt": str(self.attempt),
            "primary_report": file_identity(self.primary_path),
            "secondary_report": file_identity(self.report_path),
            "secondary_log": file_identity(self.log_path),
            "secondary_cannot_override_primary": True,
        }
        self.postprocess = {
            **core, "identity_sha256": object_sha256(core)}
        self.postprocess_path.write_text(json.dumps(self.postprocess))
        self.spec = {"secondary_evaluator_command": [
            "python3", "eval.py", str(self.result_bag),
            "--thresholds", str(self.thresholds),
            "--score-start-ns", "101", "--score-end-ns", "202",
        ]}

    def tearDown(self):
        self.temp.cleanup()

    def validate(self):
        _validate_secondary(
            self.report_path, self.postprocess_path, self.log_path,
            self.attempt, "plan", "orchestration", "build", "run",
            self.spec)

    def test_exact_primary_secondary_contract_passes(self):
        self.validate()

    def test_threshold_or_transform_rebinding_is_rejected(self):
        report = json.loads(self.report_path.read_text())
        report["artifact_bindings"]["thresholds"]["sha256"] = "0" * 64
        self.report_path.write_text(json.dumps(report))
        with self.assertRaisesRegex(CampaignError, "window/alignment"):
            self.validate()
        self.report_path.write_text(json.dumps(self.report))
        report = json.loads(self.report_path.read_text())
        report["local_accuracy"]["alignment"]["yaw_deg"] = 12.6
        self.report_path.write_text(json.dumps(report))
        with self.assertRaisesRegex(CampaignError, "window/alignment"):
            self.validate()

    def test_resume_rejects_mutated_self_hashed_postprocess(self):
        postprocess = json.loads(self.postprocess_path.read_text())
        postprocess["run_id"] = "other"
        self.postprocess_path.write_text(json.dumps(postprocess))
        with self.assertRaisesRegex(CampaignError, "self hash"):
            self.validate()

    def test_append_only_document_resume_is_exact(self):
        path = Path(self.temp.name) / "immutable.json"
        _append_or_validate(path, {"value": 1}, "fixture")
        _append_or_validate(path, {"value": 1}, "fixture")
        with self.assertRaisesRegex(CampaignError, "differs"):
            _append_or_validate(path, {"value": 2}, "fixture")


if __name__ == "__main__":
    unittest.main()

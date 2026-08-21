#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from build_vio_post_v2_scope_analysis import (
    ANALYSIS_SCHEMA,
    AnalysisError,
    common_payload_prefix,
    object_sha256,
    require_self_hash,
    tail_inventory_scope,
    write_append_only,
)


class ScopeAnalysisTest(unittest.TestCase):
    def test_self_hash(self) -> None:
        document = {"schema": ANALYSIS_SCHEMA, "value": 7}
        document["identity_sha256"] = object_sha256(document)
        self.assertEqual(
            require_self_hash(document, ANALYSIS_SCHEMA, "fixture"),
            document["identity_sha256"])
        document["value"] = 8
        with self.assertRaises(AnalysisError):
            require_self_hash(document, ANALYSIS_SCHEMA, "fixture")

    def test_three_message_tail_isolated(self) -> None:
        short = [10, 20, 30]
        long = [10, 20, 30, 40, 50, 60]
        result = tail_inventory_scope([long, short, long])
        self.assertTrue(result["shortest_tail_is_prefix_of_every_run"])
        self.assertEqual(result["tail_message_count_delta"], 3)
        self.assertEqual(result["variable_suffix_message_count"], 3)
        self.assertEqual(
            result["variable_suffix_sensor_stamps_ns"], ["40", "50", "60"])

    def test_payload_divergence_index(self) -> None:
        left = [(1, b"same"), (2, b"left")]
        right = [(1, b"same"), (2, b"right")]
        result = common_payload_prefix([left, right])
        self.assertEqual(result["common_message_count"], 1)
        self.assertEqual(
            result["first_divergence"]["sensor_stamp_ns_values"], ["2"])

    def test_append_only_output_refuses_overwrite(self) -> None:
        document = {
            "identity_sha256": "a" * 64,
            "pw1_current_anchor_assessment": {
                "local_objective_normalized_max": 2.0,
                "selected_local_metrics_exact_across_runs": {
                    "signatures": [{"value": {
                        "translation_ape_rmse_m": 1.0,
                        "orientation_rmse_deg": 2.0,
                    }}],
                },
            },
        }
        with tempfile.TemporaryDirectory() as temporary:
            output_dir = Path(temporary) / "new"
            output = write_append_only(output_dir, document)
            self.assertTrue(output.is_file())
            self.assertTrue((output_dir / "README.md").is_file())
            with self.assertRaises(AnalysisError):
                write_append_only(output_dir, document)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Static/unit tests for the Phase-C generator; these never replay a bag."""

from __future__ import annotations

import tempfile
from pathlib import Path
import sys
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_vio_flight_tuning_phase_c_arms as phase_c


class PhaseCGeneratorTest(unittest.TestCase):
    def test_family_sizes_and_controls(self) -> None:
        expected = {
            "gyr_cov": (3, "gyr010"),
            "bias_pair": (2, "bias1e4"),
            "lidar_offset": (4, "lidar_m005"),
            "image_offset": (3, "image_000"),
            "geometry": (6, "geom_fs150_vox030"),
            "depth_noise": (5, "depth_a020_r010"),
            "vio_gate": (3, "gate_always"),
        }
        self.assertEqual(set(expected), set(phase_c.FAMILIES))
        for family_id, (count, control) in expected.items():
            family = phase_c.FAMILIES[family_id]
            self.assertEqual(count, len(family.levels))
            self.assertEqual(control, family.control_level)

    def test_bias_is_a_paired_factor(self) -> None:
        document = phase_c.build_document("bias_pair", {})
        self.assertEqual(2, len(document["arms"]))
        for arm in document["arms"]:
            values = arm["overrides"]
            self.assertEqual(values["imu.b_acc_cov"],
                             values["imu.b_gyr_cov"])

    def test_geometry_is_exact_two_by_three_grid(self) -> None:
        document = phase_c.build_document("geometry", {})
        pairs = {
            (arm["overrides"]["preprocess.filter_size_surf"],
             arm["overrides"]["lio.voxel_size"])
            for arm in document["arms"]
        }
        self.assertEqual(
            {(fs, voxel) for fs in (0.15, 0.20)
             for voxel in (0.25, 0.30, 0.40)}, pairs)

    def test_depth_is_historical_five_point_star(self) -> None:
        document = phase_c.build_document("depth_noise", {})
        pairs = {
            (arm["overrides"]["lio.dept_err"],
             arm["overrides"]["lio.dept_err_rel"])
            for arm in document["arms"]
        }
        self.assertEqual({
            (0.01, 0.01), (0.02, 0.01), (0.04, 0.01),
            (0.02, 0.00), (0.02, 0.02),
        }, pairs)

    def test_image_requires_selected_lidar_lock(self) -> None:
        with self.assertRaises(phase_c.DesignError):
            phase_c.build_document("image_offset", {})
        document = phase_c.build_document(
            "image_offset", {"time_offset.lidar_time_offset": -0.010})
        self.assertTrue(all(
            arm["overrides"]["time_offset.lidar_time_offset"] == -0.010
            for arm in document["arms"]))

    def test_gate_requires_explicit_final_geometry_and_depth(self) -> None:
        with self.assertRaises(phase_c.DesignError):
            phase_c.build_document("vio_gate", {})
        locks = {
            "preprocess.filter_size_surf": 0.20,
            "lio.voxel_size": 0.25,
            "lio.dept_err": 0.02,
            "lio.dept_err_rel": 0.01,
        }
        document = phase_c.build_document("vio_gate", locks)
        values = [arm["overrides"]["vio.max_lio_features_for_fusion"]
                  for arm in document["arms"]]
        self.assertEqual([-1, 500, 800], values)
        self.assertTrue(all(type(value) is int for value in values))

    def test_survivors_preserve_requested_order_and_locks(self) -> None:
        locks = {"imu.acc_cov": 5.0}
        document = phase_c.build_document(
            "gyr_cov", locks, ["gyr020", "gyr005"])
        self.assertEqual(["gyr020", "gyr005"],
                         [arm["id"] for arm in document["arms"]])
        self.assertTrue(all(arm["overrides"]["imu.acc_cov"] == 5.0
                            for arm in document["arms"]))

    def test_factor_lock_collision_and_unapproved_lock_fail(self) -> None:
        with self.assertRaises(phase_c.DesignError):
            phase_c.build_document("gyr_cov", {"imu.gyr_cov": 0.1})
        with self.assertRaises(phase_c.DesignError):
            phase_c.parse_locks(["mocap.anchor_enable=true"])

    def test_yaml_matches_campaign_arm_contract(self) -> None:
        document = phase_c.build_document("lidar_offset", {})
        decoded = yaml.safe_load(phase_c.dump_yaml(document))
        self.assertEqual(phase_c.ARMS_SCHEMA, decoded["schema"])
        self.assertTrue(decoded["arms"])
        for arm in decoded["arms"]:
            self.assertRegex(arm["id"], phase_c.SAFE_ID)
            self.assertIsInstance(arm["overrides"], dict)

    def test_output_is_exclusive(self) -> None:
        document = phase_c.build_document("gyr_cov", {})
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "arms.yaml"
            phase_c.write_exclusive(target, phase_c.dump_yaml(document))
            with self.assertRaises(FileExistsError):
                phase_c.write_exclusive(target, phase_c.dump_yaml(document))

    def test_every_parameter_has_one_loader_and_a_use(self) -> None:
        report = phase_c.audit_repository()
        self.assertTrue(report["ok"], report["failures"])
        for row in report["parameters"]:
            self.assertEqual(1, len(row["loader_lines"]), row)
            self.assertTrue(row["uses"], row)
            self.assertTrue(all(use["lines"] for use in row["uses"]), row)


if __name__ == "__main__":
    unittest.main()

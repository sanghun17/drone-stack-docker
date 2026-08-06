#!/usr/bin/env python3
"""Unit tests for the pure math/policy in make_hybrid_imu_bag.py."""

import unittest
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from make_hybrid_imu_bag import (
    R_BASE_FROM_CAMERA,
    interpolate_sample,
    rotate_base_to_camera,
    rotate_covariance_base_to_camera,
)


class TransformTest(unittest.TestCase):
    def test_rotation_is_proper_with_calibration_precision(self):
        np.testing.assert_allclose(
            R_BASE_FROM_CAMERA.T @ R_BASE_FROM_CAMERA,
            np.eye(3), atol=1e-6)
        self.assertAlmostEqual(float(np.linalg.det(R_BASE_FROM_CAMERA)), 1.0, places=5)

    def test_row_vector_convention(self):
        basis_x_base = np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(
            rotate_base_to_camera(basis_x_base),
            R_BASE_FROM_CAMERA[0], atol=1e-12)

    def test_covariance_rotation_preserves_isotropic_covariance(self):
        covariance = 2.5 * np.eye(3)
        np.testing.assert_allclose(
            rotate_covariance_base_to_camera(covariance.reshape(-1)),
            covariance, atol=5e-6)


class InterpolationTest(unittest.TestCase):
    def setUp(self):
        self.times = np.array([1.0, 1.02, 1.04])
        self.values = np.array([[0.0, 0.0, 0.0],
                                [2.0, 4.0, 6.0],
                                [4.0, 8.0, 12.0]])
        self.covariances = np.stack([np.eye(3).reshape(-1) * x
                                     for x in (1.0, 2.0, 3.0)])

    def test_linear_interpolation(self):
        value, covariance, status, gap = interpolate_sample(
            1.01, self.times, self.values, self.covariances, 0.05)
        self.assertEqual(status, "interpolated")
        self.assertAlmostEqual(gap, 0.02)
        np.testing.assert_allclose(value, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(covariance.reshape(3, 3), 1.5 * np.eye(3))

    def test_exact_sample(self):
        value, _, status, gap = interpolate_sample(
            1.02, self.times, self.values, self.covariances, 0.05)
        self.assertEqual(status, "exact")
        self.assertEqual(gap, 0.0)
        np.testing.assert_allclose(value, self.values[1])

    def test_default_drops_outside_range(self):
        value, _, status, _ = interpolate_sample(
            0.99, self.times, self.values, self.covariances, 0.05)
        self.assertIsNone(value)
        self.assertEqual(status, "outside")

    def test_nearest_edge_is_bounded(self):
        value, _, status, gap = interpolate_sample(
            0.99, self.times, self.values, self.covariances, 0.05, "nearest")
        self.assertEqual(status, "nearest_edge")
        self.assertAlmostEqual(gap, 0.01)
        np.testing.assert_allclose(value, self.values[0])

    def test_large_internal_gap_is_rejected(self):
        value, _, status, gap = interpolate_sample(
            1.01, self.times, self.values, self.covariances, 0.005)
        self.assertIsNone(value)
        self.assertEqual(status, "gap_too_large")
        self.assertAlmostEqual(gap, 0.02)


if __name__ == "__main__":
    unittest.main()

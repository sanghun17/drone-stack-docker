#!/usr/bin/env python3
"""Unit and optional real-bag parity tests for the causal hybrid IMU bridge."""

from __future__ import annotations

import copy
import os
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "modules/sensor/realsense-d435i"
sys.path.insert(0, str(MODULE))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from hybrid_imu_core import (  # noqa: E402
    CausalHybridMatcher,
    D435Sample,
    FcuSample,
    R_BASE_FROM_CAMERA,
    rotate_base_to_camera,
    rotate_covariance_base_to_camera,
)
from make_hybrid_imu_bag import interpolate_sample  # noqa: E402


class CausalMatcherTest(unittest.TestCase):
    def setUp(self):
        self.matcher = CausalHybridMatcher(
            max_bracket_gap_s=0.05, max_wait_s=0.06)
        self.cov0 = np.eye(3).reshape(-1)

    def fcu(self, stamp, values, arrival=None, covariance=None):
        return FcuSample(
            stamp, np.asarray(values, dtype=np.float64),
            np.asarray(covariance if covariance is not None else self.cov0,
                       dtype=np.float64),
            stamp if arrival is None else arrival)

    def test_rotation_is_the_calibrated_proper_rotation(self):
        np.testing.assert_allclose(
            R_BASE_FROM_CAMERA.T @ R_BASE_FROM_CAMERA, np.eye(3), atol=1e-6)
        self.assertAlmostEqual(float(np.linalg.det(R_BASE_FROM_CAMERA)), 1.0,
                               places=5)
        covariance = 2.5 * np.eye(3)
        np.testing.assert_allclose(
            rotate_covariance_base_to_camera(covariance.reshape(-1)),
            covariance, atol=5e-6)

    def test_waits_for_actual_future_sample_then_matches_offline(self):
        self.matcher.add_fcu(self.fcu(1.0, [0, 0, 0], 10.000), 10.000)
        matches, drops, error = self.matcher.add_d435(
            D435Sample(1.01, "camera", 10.002), 10.002)
        self.assertEqual((matches, drops, error), ([], [], None))
        matches, drops, error = self.matcher.add_fcu(
            self.fcu(1.02, [2, 4, 6], 10.012,
                     2.0 * np.eye(3).reshape(-1)), 10.012)
        self.assertIsNone(error)
        self.assertFalse(drops)
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].d435.payload, "camera")
        self.assertEqual(matches[0].right_arrival, 10.012)

        expected, covariance, status, gap = interpolate_sample(
            1.01, np.asarray([1.0, 1.02]),
            np.asarray([[0, 0, 0], [2, 4, 6]], dtype=np.float64),
            np.asarray([self.cov0, 2.0 * self.cov0]), 0.05)
        np.testing.assert_allclose(matches[0].acceleration_base, expected)
        np.testing.assert_allclose(matches[0].covariance_base, covariance)
        self.assertEqual(matches[0].status, status)
        self.assertEqual(matches[0].support_gap_s, gap)

    def test_timeout_never_fabricates_hold_or_extrapolated_output(self):
        self.matcher.add_fcu(self.fcu(1.0, [0, 0, 0], 10.000), 10.000)
        self.matcher.add_d435(D435Sample(1.01, "camera", 10.002), 10.002)
        matches, drops = self.matcher.drain(10.063)
        self.assertFalse(matches)
        self.assertEqual([drop.reason for drop in drops], ["wait_timeout"])

    def test_large_gap_and_stamp_rollback_are_rejected(self):
        self.matcher.add_fcu(self.fcu(1.0, [0, 0, 0]), 1.0)
        self.matcher.add_d435(D435Sample(1.01, "camera", 1.01), 1.01)
        matches, drops, error = self.matcher.add_fcu(
            self.fcu(1.08, [1, 1, 1], arrival=1.02), 1.02)
        self.assertFalse(matches)
        self.assertIsNone(error)
        self.assertEqual([drop.reason for drop in drops], ["gap_too_large"])
        _, _, error = self.matcher.add_fcu(
            self.fcu(0.9, [1, 1, 1]), 1.09)
        self.assertEqual(error, "fcu_stamp_rollback")

    def test_output_transform_matches_offline_equations(self):
        vector = np.asarray([1.2, -3.4, 9.0])
        np.testing.assert_allclose(
            rotate_base_to_camera(vector), vector @ R_BASE_FROM_CAMERA)

    def test_bounded_history_survives_fcu_ahead_of_delayed_d435(self):
        # At startup and under scheduling jitter, several prompt FCU callbacks
        # can precede an older D435 callback.  Those already-seen samples remain
        # causal support and must not be trimmed to only the newest two.
        for stamp in (1.00, 1.02, 1.04, 1.06):
            self.matcher.add_fcu(self.fcu(stamp, [stamp, 0, 0], 10.0), 10.0)
        matches, drops, error = self.matcher.add_d435(
            D435Sample(1.01, "delayed_camera", 10.01), 10.01)
        self.assertIsNone(error)
        self.assertFalse(drops)
        self.assertEqual(len(matches), 1)
        self.assertAlmostEqual(matches[0].acceleration_base[0], 1.01)

    def test_nan_fcu_is_rejected_without_output(self):
        matches, drops, error = self.matcher.add_fcu(
            self.fcu(1.0, [float("nan"), 0, 0]), 1.0)
        self.assertFalse(matches)
        self.assertFalse(drops)
        self.assertEqual(error, "invalid_fcu")


@unittest.skipUnless(os.environ.get("HYBRID_IMU_REAL_BAG_TEST") == "1",
                     "set HYBRID_IMU_REAL_BAG_TEST=1 for the 1.5GB parity test")
class RealBagParityTest(unittest.TestCase):
    """Replay callback order and compare every output to the saved sidecar."""

    SOURCE = Path(os.environ.get(
        "HYBRID_IMU_SOURCE_BAG",
        "/home/ml/webcam_recorder/recordings/pure_flight_2026-08-04_21-10-27_0/"
        "flight_2026-08-04_21-10-27.bag"))
    EXPECTED = Path(os.environ.get(
        "HYBRID_IMU_EXPECTED_BAG",
        str(ROOT / "tools/fastlivo/_campaign_20260805/derived_hybrid_imu/"
            "campaign21/p0_20260804_211027_full_with_hybrid.bag")))

    def test_callback_order_is_causal_and_numerically_identical(self):
        import rosbag

        self.assertTrue(self.SOURCE.is_file(), self.SOURCE)
        self.assertTrue(self.EXPECTED.is_file(), self.EXPECTED)
        expected = {}
        with rosbag.Bag(str(self.EXPECTED), "r") as bag:
            for _, msg, _ in bag.read_messages(topics=["/camera/imu_hybrid"]):
                expected[msg.header.stamp.to_nsec()] = msg

        matcher = CausalHybridMatcher(0.05, 0.06, 512, 512)
        actual = {}
        with rosbag.Bag(str(self.SOURCE), "r") as bag:
            first_bag_time = bag.get_start_time()
            for topic, msg, bag_time in bag.read_messages(
                    topics=["/camera/imu", "/mavros/imu/data_raw"]):
                now = bag_time.to_sec() - first_bag_time
                stamp = msg.header.stamp.to_sec()
                if topic == "/camera/imu":
                    matches, drops, error = matcher.add_d435(
                        D435Sample(stamp, msg, now), now)
                else:
                    matches, drops, error = matcher.add_fcu(FcuSample(
                        stamp,
                        np.asarray([msg.linear_acceleration.x,
                                    msg.linear_acceleration.y,
                                    msg.linear_acceleration.z]),
                        np.asarray(msg.linear_acceleration_covariance), now), now)
                self.assertIsNone(error)
                # Drops are allowed only when the offline edge/gap policy also
                # omitted that D435 stamp; they are checked by the final key set.
                for match in matches:
                    out = copy.deepcopy(match.d435.payload)
                    accel = rotate_base_to_camera(match.acceleration_base)
                    cov = rotate_covariance_base_to_camera(match.covariance_base)
                    out.header.frame_id = "camera_depth_optical_frame"
                    out.linear_acceleration.x = float(accel[0])
                    out.linear_acceleration.y = float(accel[1])
                    out.linear_acceleration.z = float(accel[2])
                    out.linear_acceleration_covariance = cov.reshape(-1).tolist()
                    # The output is created only after its right support callback.
                    self.assertLessEqual(match.right_arrival, now + 1e-12)
                    actual[out.header.stamp.to_nsec()] = out

        self.assertEqual(set(actual), set(expected))
        for stamp in expected:
            lhs, rhs = actual[stamp], expected[stamp]
            self.assertEqual(lhs.header.stamp, rhs.header.stamp)
            self.assertEqual(lhs.header.frame_id, rhs.header.frame_id)
            np.testing.assert_allclose(
                [lhs.angular_velocity.x, lhs.angular_velocity.y,
                 lhs.angular_velocity.z],
                [rhs.angular_velocity.x, rhs.angular_velocity.y,
                 rhs.angular_velocity.z], rtol=0.0, atol=0.0)
            np.testing.assert_allclose(
                [lhs.linear_acceleration.x, lhs.linear_acceleration.y,
                 lhs.linear_acceleration.z],
                [rhs.linear_acceleration.x, rhs.linear_acceleration.y,
                 rhs.linear_acceleration.z], rtol=0.0, atol=1e-12)
            np.testing.assert_allclose(
                lhs.linear_acceleration_covariance,
                rhs.linear_acceleration_covariance, rtol=0.0, atol=1e-12)


if __name__ == "__main__":
    unittest.main()

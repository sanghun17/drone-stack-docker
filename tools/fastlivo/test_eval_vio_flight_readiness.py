#!/usr/bin/env python3
"""Focused synthetic tests for the strict FAST-LIVO flight evaluator."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_vio_flight_readiness import (  # noqa: E402
    AssociatedTrajectory,
    DEFAULT_GT_TOPIC,
    DEFAULT_LOCAL_TOPIC,
    DEFAULT_PROP_TOPIC,
    DEFAULT_CORRECTION_COV_TOPIC,
    GT_ANCHORED_TOPIC,
    PoseStream,
    apply_alignment,
    associate_fixed,
    evaluate_bag,
    fixed_alignment_from_primary,
    initialization_alignment,
    load_thresholds,
    read_pose_stream,
    _file_identity,
    _score_window_mask,
    _slice_stream,
)


def _quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _pose_message(stamp: float, position, yaw: float, frame: str = "world"):
    message = PoseStamped()
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.header.frame_id = frame
    message.pose.position.x, message.pose.position.y, message.pose.position.z = position
    (message.pose.orientation.x, message.pose.orientation.y,
     message.pose.orientation.z, message.pose.orientation.w) = _quaternion(yaw)
    return message


def _odom_message(stamp: float, position, yaw: float, velocity,
                  yaw_rate: float, covariance: bool = True):
    message = Odometry()
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
    (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
     message.pose.pose.orientation.z, message.pose.pose.orientation.w) = _quaternion(yaw)
    (message.twist.twist.linear.x, message.twist.twist.linear.y,
     message.twist.twist.linear.z) = velocity
    message.twist.twist.angular.z = yaw_rate
    if covariance:
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[14] = 0.01
        message.twist.covariance[0] = 0.04
        message.twist.covariance[7] = 0.04
        message.twist.covariance[14] = 0.04
    return message


def _pose_cov_message(stamp: float, position, yaw: float,
                      covariance: bool = True):
    message = PoseWithCovarianceStamped()
    message.header.stamp = rospy.Time.from_sec(stamp)
    message.header.frame_id = "odom"
    (message.pose.pose.position.x, message.pose.pose.position.y,
     message.pose.pose.position.z) = position
    (message.pose.pose.orientation.x, message.pose.pose.orientation.y,
     message.pose.pose.orientation.z,
     message.pose.pose.orientation.w) = _quaternion(yaw)
    if covariance:
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.01
    return message


class AlignmentMathTest(unittest.TestCase):
    def test_initial_yaw_translation_alignment_is_frozen_and_exact(self):
        time = np.arange(0.0, 3.0, 0.1)
        yaw_offset = 0.4
        rotation = np.asarray([
            [math.cos(yaw_offset), -math.sin(yaw_offset), 0.0],
            [math.sin(yaw_offset), math.cos(yaw_offset), 0.0],
            [0.0, 0.0, 1.0],
        ])
        translation = np.asarray([1.2, -0.7, 0.3])
        estimate = np.column_stack([time, 0.2 * time, 0.1 * time])
        truth = (rotation @ estimate.T).T + translation
        estimate_rotation = np.repeat(np.eye(3)[None, :, :], len(time), axis=0)
        truth_rotation = np.repeat(rotation[None, :, :], len(time), axis=0)
        trajectory = AssociatedTrajectory(
            time, estimate, estimate_rotation, truth, truth_rotation)
        fitted_rotation, fitted_translation, details = initialization_alignment(
            trajectory, duration_s=1.0, minimum_samples=5)
        aligned = apply_alignment(trajectory, fitted_rotation, fitted_translation)
        np.testing.assert_allclose(fitted_rotation, rotation, atol=1e-12)
        np.testing.assert_allclose(fitted_translation, translation, atol=1e-12)
        np.testing.assert_allclose(aligned.estimate_position, truth, atol=1e-12)
        self.assertEqual(details["scale"], 1.0)


class SyntheticBagTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.thresholds = load_thresholds(
            Path(__file__).with_name("vio_flight_readiness_thresholds.yaml"))

    def tearDown(self):
        self.temp.cleanup()

    def _write_bag(self, anchored: bool = False, jump: bool = False,
                   zero_covariance: bool = False,
                   takeoff: bool = False) -> Path:
        path = self.root / "synthetic.bag"
        yaw_offset = 0.25
        cosine, sine = math.cos(yaw_offset), math.sin(yaw_offset)
        rotation = np.asarray([[cosine, -sine, 0.0],
                               [sine, cosine, 0.0], [0.0, 0.0, 1.0]])
        translation = np.asarray([1.0, -0.5, 0.2])
        with rosbag.Bag(str(path), "w") as bag:
            for index in range(1201):
                stamp = 100.0 + index * 0.005
                relative = stamp - 100.0
                z = (min(1.0, max(0.0, 0.5 * (relative - 2.0)))
                     if takeoff else 1.0)
                gt_position = np.asarray([0.15 * relative,
                                          0.1 * math.sin(0.5 * relative), z])
                gt_yaw = 0.1 * (stamp - 100.0)
                bag.write(DEFAULT_GT_TOPIC,
                          _pose_message(stamp, gt_position, gt_yaw, "optitrack"),
                          rospy.Time.from_sec(stamp + 0.002))

                local_position = rotation.T @ (gt_position - translation)
                local_yaw = gt_yaw - yaw_offset
                if index % 20 == 0:
                    bag.write(DEFAULT_LOCAL_TOPIC,
                              _pose_message(stamp, local_position, local_yaw, "odom"),
                              rospy.Time.from_sec(stamp + 0.015))
                    bag.write(
                        DEFAULT_CORRECTION_COV_TOPIC,
                        _pose_cov_message(stamp, local_position, local_yaw,
                                          covariance=not zero_covariance),
                        rospy.Time.from_sec(stamp + 0.016))
                prop_position = local_position.copy()
                if jump and index >= 300:
                    prop_position[0] += 2.0
                local_world_velocity = rotation.T @ np.asarray([
                    0.15, 0.05 * math.cos(0.5 * (stamp - 100.0)), 0.0])
                body_rotation = np.asarray([
                    [math.cos(local_yaw), -math.sin(local_yaw), 0.0],
                    [math.sin(local_yaw), math.cos(local_yaw), 0.0],
                    [0.0, 0.0, 1.0],
                ])
                local_velocity = body_rotation.T @ local_world_velocity
                bag.write(DEFAULT_PROP_TOPIC,
                          _odom_message(stamp, prop_position, local_yaw,
                                        local_velocity, 0.1,
                                        covariance=not zero_covariance),
                          rospy.Time.from_sec(stamp + 0.015))
            if anchored:
                bag.write(GT_ANCHORED_TOPIC,
                          _odom_message(103.0, [0.0, 0.0, 1.0], 0.0,
                                        [0.0, 0.0, 0.0], 0.0),
                          rospy.Time.from_sec(103.01))
        return path

    def test_fixed_time_association_and_healthy_metrics(self):
        path = self._write_bag()
        propagated = read_pose_stream(path, DEFAULT_PROP_TOPIC)
        self.assertIsNotNone(propagated)
        self.assertEqual(
            len(propagated.header_time), len(propagated.linear_velocity))
        self.assertEqual(
            len(propagated.header_time), len(propagated.angular_velocity))
        # Use a varying velocity so accidental duplication/misalignment cannot
        # pass as it would for a constant synthetic twist.
        expected_first_world = np.asarray([0.15, 0.05, 0.0])
        first_rotation = np.asarray([
            [math.cos(-0.25), -math.sin(-0.25), 0.0],
            [math.sin(-0.25), math.cos(-0.25), 0.0],
            [0.0, 0.0, 1.0],
        ])
        local_world_rotation = np.asarray([
            [math.cos(0.25), -math.sin(0.25), 0.0],
            [math.sin(0.25), math.cos(0.25), 0.0],
            [0.0, 0.0, 1.0],
        ])
        np.testing.assert_allclose(
            propagated.linear_velocity[0],
            first_rotation.T @ local_world_rotation.T @ expected_first_world,
            atol=1e-12)
        document = evaluate_bag(path, self.thresholds)
        self.assertAlmostEqual(document["evaluation_semantics"]
                               ["time_offset_used_for_scoring_s"], 0.0)
        self.assertLess(document["local"]["translation_ape_rmse_m"], 1e-8)
        self.assertLess(document["local"]["orientation_rmse_deg"], 1e-5)
        self.assertTrue(document["gt_independence"]["gt_anchor_free"])
        self.assertGreater(document["propagated"]["rate_hz"], 190.0)
        self.assertLess(
            document["propagated"]["angular_twist_pose_p95_rad_s"], 1e-7)
        # Six seconds cannot evaluate the required 30-second stationary gate,
        # so an otherwise healthy short bag must never claim PASS.
        self.assertEqual(document["status"], "incomplete")
        nonpassing = {
            row["id"] for row in document["checks"]
            if row["status"] != "pass"
        }
        self.assertEqual(nonpassing, {
            "stationary_translation_drift", "stationary_yaw_drift",
        })

    def test_anchor_jump_and_zero_covariance_are_hard_failures(self):
        path = self._write_bag(anchored=True, jump=True, zero_covariance=True)
        document = evaluate_bag(path, self.thresholds)
        self.assertFalse(document["gt_independence"]["gt_anchor_free"])
        self.assertGreater(document["propagated"]["position_step_max_m"], 1.0)
        self.assertEqual(
            document["correction_covariance"]["nonzero_fraction"], 0.0)
        self.assertEqual(document["status"], "fail")
        failed = {row["id"] for row in document["checks"]
                  if row["status"] == "fail"}
        self.assertIn("gt_was_not_visible_to_estimator", failed)
        self.assertIn("propagated_position_jump", failed)
        self.assertIn("correction_covariance_nonzero", failed)

    def test_takeoff_window_and_vertical_scale(self):
        path = self._write_bag(takeoff=True)
        document = evaluate_bag(path, self.thresholds)
        takeoff = document["local"]["takeoff"]
        self.assertTrue(takeoff["available"])
        self.assertTrue(takeoff["stable_hover_detected"])
        self.assertAlmostEqual(takeoff["vertical_displacement_scale"],
                               1.0, places=5)
        self.assertGreater(takeoff["output_ready_lead_s"], 1.0)

    def test_association_never_applies_diagnostic_time_offset(self):
        path = self._write_bag()
        stream = read_pose_stream(path, DEFAULT_LOCAL_TOPIC).sorted_valid()
        gt = read_pose_stream(path, DEFAULT_GT_TOPIC).sorted_valid()
        gt_valid = np.ones(len(gt.header_time), dtype=bool)
        shifted = replace(stream, header_time=stream.header_time + 0.037)
        associated = associate_fixed(
            shifted, gt, gt_valid, 0.02)
        # The shifted samples can still interpolate GT at their declared time,
        # but no code estimates or subtracts a per-session clock shift here.
        self.assertAlmostEqual(
            associated.time[0], stream.header_time[0] + 0.037, places=6)
        self.assertAlmostEqual(
            associated.gt_position[0, 0], 0.15 * 0.037, places=6)

    def test_secondary_window_reuses_primary_alignment_without_refit(self):
        path = self._write_bag()
        primary = evaluate_bag(path, self.thresholds)
        details = primary["local"]["alignment"]
        yaw = math.radians(details["yaw_deg"])
        fixed = (
            np.asarray([[math.cos(yaw), -math.sin(yaw), 0.0],
                        [math.sin(yaw), math.cos(yaw), 0.0],
                        [0.0, 0.0, 1.0]]),
            np.asarray(details["translation_m"]),
            details,
        )
        secondary = evaluate_bag(
            path, self.thresholds,
            score_window_ns=(102_000_000_000, 105_000_000_000),
            fixed_alignment=fixed)
        self.assertTrue(secondary["evaluation_semantics"]
                        ["fixed_alignment_supplied"])
        self.assertEqual(secondary["evaluation_semantics"]
                         ["score_window_sensor_stamp_ns"], {
                             "start": "102000000000",
                             "end": "105000000000",
                             "boundary": "start_inclusive_end_inclusive",
                         })
        secondary_alignment = secondary["local"]["alignment"]
        self.assertTrue(secondary_alignment[
            "reused_from_primary_full_result"])
        self.assertEqual(secondary_alignment["yaw_deg"], details["yaw_deg"])
        self.assertEqual(secondary_alignment["translation_m"],
                         details["translation_m"])

    def test_score_window_uses_exact_integer_header_nanoseconds(self):
        # At this epoch adjacent ns cannot be represented by binary64 seconds;
        # the mask must therefore operate on preserved integer stamps.
        stamps = np.asarray([
            1_785_788_800_000_000_000,
            1_785_788_800_000_000_001,
            1_785_788_800_000_000_002,
        ], dtype=np.int64)
        np.testing.assert_array_equal(
            _score_window_mask(stamps, (
                1_785_788_800_000_000_001,
                1_785_788_800_000_000_001)),
            [False, True, False])

    def test_sorted_pipeline_preserves_adjacent_epoch_nanoseconds(self):
        stamps = np.asarray([
            1_785_788_800_000_000_002,
            1_785_788_800_000_000_000,
            1_785_788_800_000_000_001,
        ], dtype=np.int64)
        # All three intentionally collapse to the same binary64 seconds.
        seconds = stamps.astype(float) * 1e-9
        stream = PoseStream(
            topic="/synthetic", header_time=seconds,
            header_stamp_ns=stamps, bag_time=seconds.copy(),
            position=np.zeros((3, 3)),
            quaternion=np.tile([0.0, 0.0, 0.0, 1.0], (3, 1)),
            linear_velocity=None, angular_velocity=None,
            pose_covariance_nonzero=None, twist_covariance_nonzero=None,
            frame_id=["world"] * 3, child_frame_id=[""] * 3)
        sorted_stream = stream.sorted_valid()
        np.testing.assert_array_equal(
            sorted_stream.header_stamp_ns, np.sort(stamps))
        sliced = _slice_stream(
            sorted_stream,
            _score_window_mask(sorted_stream.header_stamp_ns, (
                1_785_788_800_000_000_001,
                1_785_788_800_000_000_001)))
        self.assertEqual(sliced.header_stamp_ns.tolist(), [
            1_785_788_800_000_000_001])

    def test_primary_alignment_binding_rejects_wrong_bag_and_thresholds(self):
        path = self._write_bag()
        primary = evaluate_bag(path, self.thresholds)
        thresholds_path = Path(__file__).with_name(
            "vio_flight_readiness_thresholds.yaml")
        primary["artifact_bindings"] = {
            "result_bag": _file_identity(path),
            "thresholds": _file_identity(thresholds_path),
        }
        primary_path = self.root / "primary.json"
        import json
        primary_path.write_text(json.dumps(primary))
        fixed, loaded = fixed_alignment_from_primary(
            primary_path, path, thresholds_path)
        self.assertEqual(loaded["result_bag"], str(path.resolve()))
        self.assertEqual(len(fixed[1]), 3)

        wrong_bag = self.root / "wrong.bag"
        wrong_bag.write_bytes(path.read_bytes())
        with self.assertRaisesRegex(ValueError, "not bound"):
            fixed_alignment_from_primary(
                primary_path, wrong_bag, thresholds_path)
        wrong_thresholds = self.root / "thresholds.yaml"
        wrong_thresholds.write_text(thresholds_path.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "not bound"):
            fixed_alignment_from_primary(
                primary_path, path, wrong_thresholds)


if __name__ == "__main__":
    unittest.main()

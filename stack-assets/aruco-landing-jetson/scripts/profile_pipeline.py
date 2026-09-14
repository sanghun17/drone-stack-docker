#!/usr/bin/env python3
"""Measure the FC-independent ArUco landing pipeline while recording a bag."""

import argparse
import datetime
import json
import math
import os
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Int32MultiArray
from std_srvs.srv import SetBool


def atomic_json(path, document):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def rate(receipts):
    if len(receipts) < 2 or receipts[-1] <= receipts[0]:
        return None
    return (len(receipts) - 1) / (receipts[-1] - receipts[0])


def stats(values):
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    data = np.asarray(values, dtype=float)
    return {
        "count": int(data.size),
        "mean": float(np.mean(data)),
        "median": float(np.median(data)),
        "p95": float(np.percentile(data, 95.0)),
        "max": float(np.max(data)),
    }


class PipelineProfile:
    def __init__(self):
        self.lock = threading.Lock()
        self.receipts = {
            "camera": [], "detection": [], "fusion": [], "vehicle_pose": [],
            "command": [], "processing": [],
        }
        self.image = None
        self.marker_ids = []
        self.marker_frames_any = []
        self.inlier_counts = []
        self.visible = []
        self.processing_ms = []
        self.nonzero_commands = 0
        self.pose_samples = []

        rospy.Subscriber("/landing/camera/image_raw", Image, self.camera, queue_size=2)
        rospy.Subscriber("/landing/markers/ids", Int32MultiArray, self.detection, queue_size=20)
        rospy.Subscriber("/landing/estimator/inlier_ids", Int32MultiArray, self.inliers, queue_size=20)
        rospy.Subscriber("/landing/estimator/processing_ms", Float32, self.processing, queue_size=50)
        rospy.Subscriber("/landing/target_visible", Bool, self.visibility, queue_size=50)
        rospy.Subscriber(
            "/landing/target_pose_camera", PoseWithCovarianceStamped,
            self.fusion, queue_size=20, tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/landing/vehicle_pose_pad", PoseWithCovarianceStamped,
            self.vehicle_pose, queue_size=20, tcp_nodelay=True,
        )
        rospy.Subscriber("/landing/cmd_vel_pad", TwistStamped, self.command, queue_size=50)

    def now(self):
        return time.monotonic()

    def camera(self, message):
        with self.lock:
            self.receipts["camera"].append(self.now())
            if self.image is None:
                self.image = {
                    "width": int(message.width), "height": int(message.height),
                    "encoding": message.encoding, "step_bytes": int(message.step),
                    "frame_id": message.header.frame_id,
                }

    def detection(self, message):
        with self.lock:
            self.receipts["detection"].append(self.now())
            self.marker_frames_any.append(bool(message.data))
            self.marker_ids.extend(int(value) for value in message.data)

    def inliers(self, message):
        with self.lock:
            self.inlier_counts.append(len(message.data))

    def processing(self, message):
        with self.lock:
            self.receipts["processing"].append(self.now())
            self.processing_ms.append(float(message.data))

    def visibility(self, message):
        with self.lock:
            self.visible.append(bool(message.data))

    def fusion(self, message):
        position = message.pose.pose.position
        stamp = message.header.stamp.to_sec() or rospy.get_time()
        with self.lock:
            self.receipts["fusion"].append(self.now())
            self.pose_samples.append((stamp, position.x, position.y, position.z))

    def vehicle_pose(self, _message):
        with self.lock:
            self.receipts["vehicle_pose"].append(self.now())

    def command(self, message):
        values = (
            message.twist.linear.x, message.twist.linear.y, message.twist.linear.z,
            message.twist.angular.x, message.twist.angular.y, message.twist.angular.z,
        )
        with self.lock:
            self.receipts["command"].append(self.now())
            if any(abs(value) > 1e-6 for value in values):
                self.nonzero_commands += 1

    def result(self, duration, budget_ms, layout, dictionary, pad_size_m):
        with self.lock:
            velocities = []
            for first, second in zip(self.pose_samples, self.pose_samples[1:]):
                dt = second[0] - first[0]
                if dt > 1e-6:
                    velocities.append(math.sqrt(sum(
                        ((second[index] - first[index]) / dt) ** 2
                        for index in (1, 2, 3)
                    )))
            detections = len(self.receipts["detection"])
            fusions = len(self.receipts["fusion"])
            commands = len(self.receipts["command"])
            unique_ids = sorted(set(self.marker_ids))
            return {
                "schema_version": 1,
                "duration_s": duration,
                "hardware_scope": {
                    "flight_controller": "excluded_powered_off",
                    "optitrack": "excluded_powered_off",
                    "actuator_adapter": "not_started",
                },
                "layout": {
                    "path": layout,
                    "dictionary": dictionary,
                    "pad_size_m": pad_size_m,
                    "scale_anchor": "center marker side length = 0.03 m",
                    "geometry_status": "unverified_arbitrary_physical_layout",
                    "metric_pose_accuracy_valid": False,
                    "note": "Detection/rate/load results are valid; fused pose and derived velocity are provisional until marker geometry is surveyed.",
                },
                "image": self.image,
                "estimator_crop": {
                    "center_crop": rospy.get_param(
                        "/paper_pad_estimator/center_crop", None
                    ),
                    "processing_width": rospy.get_param(
                        "/paper_pad_estimator/processing_width", None
                    ),
                    "processing_height": rospy.get_param(
                        "/paper_pad_estimator/processing_height", None
                    ),
                },
                "rates_hz": {name: rate(values) for name, values in self.receipts.items()},
                "counts": {name: len(values) for name, values in self.receipts.items()},
                "detection": {
                    "frames_with_any_marker": sum(self.marker_frames_any),
                    "frame_results": len(self.marker_frames_any),
                    "marker_detection_availability": (
                        sum(self.marker_frames_any) / len(self.marker_frames_any)
                        if self.marker_frames_any else None
                    ),
                    "frames_with_valid_fusion": sum(self.visible),
                    "fusion_frame_results": len(self.visible),
                    "fusion_availability": (
                        sum(self.visible) / len(self.visible)
                        if self.visible else None
                    ),
                    "unique_marker_ids": unique_ids,
                    "marker_observations": len(self.marker_ids),
                    "inlier_count": stats(self.inlier_counts),
                },
                "processing_ms": dict(
                    stats(self.processing_ms),
                    budget=budget_ms,
                    over_budget_count=sum(value > budget_ms for value in self.processing_ms),
                    over_budget_fraction=(
                        sum(value > budget_ms for value in self.processing_ms)
                        / float(len(self.processing_ms)) if self.processing_ms else None
                    ),
                ),
                "provisional_fused_pose_speed_mps": stats(velocities),
                "control": {
                    "command_count": commands,
                    "nonzero_command_count": self.nonzero_commands,
                    "nonzero_command_fraction": (
                        self.nonzero_commands / float(commands) if commands else None
                    ),
                },
                "detector_result_per_camera_frame": (
                    detections / float(len(self.receipts["camera"]))
                    if self.receipts["camera"] else None
                ),
                "fusion_per_detector_result": (
                    fusions / float(detections) if detections else None
                ),
            }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--output-dir", default="/work/experiments/aruco-landing/hardware-profiles")
    parser.add_argument("--label", default="bench")
    parser.add_argument("--layout", required=True)
    parser.add_argument("--dictionary", required=True)
    parser.add_argument("--pad-size-m", required=True, type=float)
    parser.add_argument("--processing-budget-ms", type=float, default=16.6667)
    parser.add_argument("--enable-controller", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")

    rospy.init_node("aruco_hardware_pipeline_profile", anonymous=True)
    profile = PipelineProfile()
    rospy.wait_for_service("/session_recorder/set_recording", timeout=20.0)
    recorder = rospy.ServiceProxy("/session_recorder/set_recording", SetBool)
    controller = None
    if args.enable_controller:
        rospy.wait_for_service("/landing_controller/enable", timeout=20.0)
        controller = rospy.ServiceProxy("/landing_controller/enable", SetBool)

    stamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    label = "%s_%s" % (args.label, stamp)
    rospy.set_param("/session_recorder/session_label", label)
    response = recorder(True)
    if not response.success:
        raise RuntimeError("recording start failed: " + response.message)
    bag_path = response.message
    started = time.time()
    try:
        if controller is not None:
            response = controller(True)
            if not response.success:
                raise RuntimeError("controller enable failed: " + response.message)
        rospy.sleep(args.duration)
    finally:
        if controller is not None:
            try:
                controller(False)
            except rospy.ServiceException as error:
                rospy.logerr("controller disable failed: %s", error)
        try:
            response = recorder(False)
            if not response.success:
                rospy.logerr("recording stop failed: %s", response.message)
        except rospy.ServiceException as error:
            rospy.logerr("recording stop failed: %s", error)

    os.makedirs(args.output_dir, exist_ok=True)
    result = profile.result(
        time.time() - started,
        args.processing_budget_ms,
        args.layout,
        args.dictionary,
        args.pad_size_m,
    )
    result["bag_path"] = bag_path
    result["controller_enabled_for_profile"] = bool(args.enable_controller)
    result["created_at"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    output_path = os.path.join(args.output_dir, label + ".json")
    atomic_json(output_path, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    print("profile: " + output_path)


if __name__ == "__main__":
    main()

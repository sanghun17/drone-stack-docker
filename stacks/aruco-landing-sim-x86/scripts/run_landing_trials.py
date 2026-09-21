#!/usr/bin/env python3
"""Run matched randomized AirSim landing trials and calculate paper metrics."""

import argparse
import csv
import glob
import json
import math
import os
import random
import signal
import subprocess
import threading
import time

import airsim
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


DEFAULT_BAG_TOPICS = [
    "/landing/camera/image_raw",
    "/landing/camera/camera_info",
    "/landing/markers/ids",
    "/landing/markers/poses_camera",
    "/landing/target_pose_camera",
    "/landing/target_visible",
    "/landing/estimator/inlier_ids",
    "/landing/estimator/processing_ms",
    "/landing/vehicle_pose_pad",
    "/landing/camera_pose_pad",
    "/landing/cmd_vel_pad",
    "/landing/yaw_cmd",
    "/landing/controller/state",
    "/landing/controller/active",
    "/landing/controller/abort",
    "/landing/controller/touchdown",
    "/landing/safety_intervention",
    "/landing/ground_truth/airsim_local_ned",
    "/tf",
    "/tf_static",
]


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def atomic_json(path, document):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def generate_or_load_manifest(path, count, seed, h_max, radius_min, radius_max):
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as stream:
            manifest = json.load(stream)
        if len(manifest.get("staging_positions", [])) != count:
            raise ValueError(
                "existing manifest has %d positions, requested %d"
                % (len(manifest.get("staging_positions", [])), count)
            )
        return manifest
    if radius_min <= 0.0 or radius_max < radius_min:
        raise ValueError("invalid staging radius range")
    generator = random.Random(seed)
    positions = []
    for index in range(count):
        # Uniform by area within the annulus, not uniformly by radius.
        radius = math.sqrt(
            generator.uniform(radius_min * radius_min, radius_max * radius_max)
        )
        angle = generator.uniform(-math.pi, math.pi)
        positions.append(
            {
                "trial": index + 1,
                "x_pad_m": radius * math.cos(angle),
                "y_pad_m": radius * math.sin(angle),
                "height_m": h_max,
                "yaw_rad": 0.0,
            }
        )
    manifest = {
        "schema_version": 1,
        "seed": seed,
        "distribution": "uniform_area_annulus",
        "radius_min_m": radius_min,
        "radius_max_m": radius_max,
        "staging_positions": positions,
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    atomic_json(path, manifest)
    return manifest


class TrialMeasurements:
    def __init__(self, pad_local_ned, pad_from_ned, lateral_bound_m,
                 lateral_eval_height_min_m, lateral_eval_height_max_m):
        self.lock = threading.Lock()
        self.pad_local_ned = np.asarray(pad_local_ned, dtype=float)
        self.pad_from_ned = np.asarray(pad_from_ned, dtype=float)
        self.lateral_bound_m = float(lateral_bound_m)
        self.lateral_eval_height_min_m = float(lateral_eval_height_min_m)
        self.lateral_eval_height_max_m = float(lateral_eval_height_max_m)
        self.running = False
        self.activated = False
        self.total_frames = 0
        self.valid_frames = 0
        self.squared_errors = []
        self.latest_ground_truth_pad = None
        self.latest_camera_pose_pad = None
        self.image_source_samples = 0
        self.first_image_stamp = None
        self.last_image_stamp = None
        self.first_image_receipt = None
        self.last_image_receipt = None
        self.state = ""
        self.state_event = threading.Event()
        self.lateral_linf_samples = []
        self.pose_age_samples_s = []
        self.ground_truth_touchdown_pad = None
        self.previous_ground_truth_pad = None
        self.ground_truth_touchdown_event = threading.Event()

        rospy.Subscriber(
            "/landing/target_visible", Bool, self.visible_callback, queue_size=200
        )
        rospy.Subscriber(
            "/landing/vehicle_pose_pad", PoseWithCovarianceStamped,
            self.estimate_callback, queue_size=20, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/landing/camera_pose_pad", PoseWithCovarianceStamped,
            self.camera_pose_callback, queue_size=20, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/landing/ground_truth/airsim_local_ned", Odometry,
            self.ground_truth_callback, queue_size=20, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/landing/controller/state", String, self.state_callback, queue_size=20
        )
        rospy.Subscriber(
            "/landing/safety_intervention", Bool, self.safety_callback, queue_size=20
        )

    def reset(self):
        with self.lock:
            self.total_frames = 0
            self.valid_frames = 0
            self.squared_errors = []
            self.latest_ground_truth_pad = None
            self.latest_camera_pose_pad = None
            self.image_source_samples = 0
            self.first_image_stamp = None
            self.last_image_stamp = None
            self.first_image_receipt = None
            self.last_image_receipt = None
            self.state = ""
            self.running = True
            self.activated = False
            self.state_event.clear()
            self.lateral_linf_samples = []
            self.pose_age_samples_s = []
            self.ground_truth_touchdown_pad = None
            self.previous_ground_truth_pad = None
            self.ground_truth_touchdown_event.clear()

    def stop(self):
        with self.lock:
            self.running = False

    def visible_callback(self, message):
        with self.lock:
            if not self.running or not self.activated:
                return
            self.total_frames += 1
            if message.data:
                self.valid_frames += 1

    def ground_truth_callback(self, message):
        position = message.pose.pose.position
        local_ned = np.array([position.x, position.y, position.z], dtype=float)
        relative_ned = local_ned - self.pad_local_ned
        ground_truth_pad = np.matmul(self.pad_from_ned, relative_ned)
        stamp = message.header.stamp.to_sec()
        receipt = time.monotonic()
        with self.lock:
            self.latest_ground_truth_pad = ground_truth_pad
            if self.running and self.activated:
                previous = self.previous_ground_truth_pad
                if (self.ground_truth_touchdown_pad is None
                        and previous is not None
                        and previous[2] > self.lateral_eval_height_min_m
                        and ground_truth_pad[2] <= self.lateral_eval_height_min_m):
                    dz = previous[2] - ground_truth_pad[2]
                    fraction = (
                        (previous[2] - self.lateral_eval_height_min_m) / dz
                        if dz > 1e-9 else 1.0
                    )
                    self.ground_truth_touchdown_pad = (
                        previous + fraction * (ground_truth_pad - previous)
                    )
                    self.ground_truth_touchdown_event.set()
                if (self.lateral_eval_height_min_m <= ground_truth_pad[2]
                        <= self.lateral_eval_height_max_m):
                    self.lateral_linf_samples.append(
                        float(np.max(np.abs(ground_truth_pad[0:2])))
                    )
                if self.first_image_stamp is None:
                    self.first_image_stamp = stamp
                    self.first_image_receipt = receipt
                self.last_image_stamp = stamp
                self.last_image_receipt = receipt
                self.image_source_samples += 1
                self.previous_ground_truth_pad = ground_truth_pad.copy()

    def estimate_callback(self, message):
        position = message.pose.pose.position
        estimate = np.array([position.x, position.y, position.z], dtype=float)
        with self.lock:
            if (not self.running or not self.activated
                    or self.latest_ground_truth_pad is None):
                return
            delta = estimate - self.latest_ground_truth_pad
            self.squared_errors.append(float(np.dot(delta, delta)))

    def camera_pose_callback(self, message):
        position = message.pose.pose.position
        pose_age_s = max(0.0, rospy.get_time() - message.header.stamp.to_sec())
        with self.lock:
            if self.running and self.activated:
                self.latest_camera_pose_pad = np.array(
                    [position.x, position.y, position.z], dtype=float
                )
                self.pose_age_samples_s.append(pose_age_s)

    def state_callback(self, message):
        with self.lock:
            if not self.running:
                return
            self.state = message.data
            if self.running and message.data == "DESCENDING":
                self.activated = True
            if self.running and message.data in ("TOUCHDOWN", "ABORTED_MARKER_LOSS"):
                self.state_event.set()

    def safety_callback(self, message):
        if not message.data:
            return
        with self.lock:
            if self.running:
                self.state = "ABORTED_SAFETY_INTERVENTION"
                self.state_event.set()

    def result(self, timeout_abort=False, terminal_override=None):
        with self.lock:
            total = self.total_frames
            valid = self.valid_frames
            state = terminal_override or ("ABORTED_TIMEOUT" if timeout_abort else self.state)
            rmse = (
                math.sqrt(sum(self.squared_errors) / len(self.squared_errors))
                if self.squared_errors else None
            )
            touchdown_position = self.ground_truth_touchdown_pad
            touchdown_error = (
                float(np.linalg.norm(touchdown_position[0:2]))
                if touchdown_position is not None else None
            )
            source_span = (
                self.last_image_stamp - self.first_image_stamp
                if self.first_image_stamp is not None and self.last_image_stamp is not None
                else 0.0
            )
            receipt_span = (
                self.last_image_receipt - self.first_image_receipt
                if self.first_image_receipt is not None and self.last_image_receipt is not None
                else 0.0
            )
            lateral_linf_max = (
                max(self.lateral_linf_samples)
                if self.lateral_linf_samples else None
            )
            pose_age_median_ms = (
                float(np.median(self.pose_age_samples_s)) * 1000.0
                if self.pose_age_samples_s else None
            )
            pose_age_p95_ms = (
                float(np.percentile(self.pose_age_samples_s, 95.0)) * 1000.0
                if self.pose_age_samples_s else None
            )
            return {
                "terminal_state": state,
                "success": state == "TOUCHDOWN",
                "total_image_frames": total,
                "valid_pose_frames": valid,
                "marker_pose_availability": float(valid) / total if total else None,
                "localization_samples": len(self.squared_errors),
                "localization_position_rmse_m": rmse,
                "horizontal_touchdown_error_m": touchdown_error,
                "ground_truth_touchdown_position_pad_m": (
                    touchdown_position.tolist()
                    if touchdown_position is not None else None
                ),
                "image_source_samples": self.image_source_samples,
                "camera_source_rate_hz": (
                    (self.image_source_samples - 1) / source_span
                    if self.image_source_samples > 1 and source_span > 0.0 else None
                ),
                "camera_delivery_rate_hz": (
                    (self.image_source_samples - 1) / receipt_span
                    if self.image_source_samples > 1 and receipt_span > 0.0 else None
                ),
                "final_ground_truth_position_pad_m": (
                    self.latest_ground_truth_pad.tolist()
                    if self.latest_ground_truth_pad is not None else None
                ),
                "final_estimated_camera_position_pad_m": (
                    self.latest_camera_pose_pad.tolist()
                    if self.latest_camera_pose_pad is not None else None
                ),
                "estimated_camera_height_at_touchdown_m": (
                    float(self.latest_camera_pose_pad[2])
                    if state == "TOUCHDOWN" and self.latest_camera_pose_pad is not None
                    else None
                ),
                "lateral_eval_height_min_m": self.lateral_eval_height_min_m,
                "lateral_eval_height_max_m": self.lateral_eval_height_max_m,
                "lateral_linf_max_m": lateral_linf_max,
                "lateral_bound_m": self.lateral_bound_m,
                "lateral_bound_met": (
                    lateral_linf_max is not None
                    and lateral_linf_max <= self.lateral_bound_m
                ),
                "marker_pose_age_median_ms": pose_age_median_ms,
                "marker_pose_age_p95_ms": pose_age_p95_ms,
            }


class BagRecorder:
    def __init__(self, output_path, topics):
        self.output_path = output_path
        self.topics = topics
        self.process = None

    def start(self):
        command = ["rosbag", "record", "--lz4", "-O", self.output_path] + self.topics
        self.process = subprocess.Popen(
            command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(0.35)
        if self.process.poll() is not None:
            raise RuntimeError("rosbag record failed to start for " + self.output_path)

    def stop(self):
        if self.process is None:
            return
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait(timeout=5.0)
        self.process = None


class SessionServiceRecorder:
    """Adapter from a trial to the shared simulation/hardware recorder API."""

    def __init__(self, output_dir, label):
        self.output_dir = output_dir
        self.label = label
        self.output_path = None
        self.set_recording = rospy.ServiceProxy(
            "/session_recorder/set_recording", SetBool
        )

    def start(self):
        rospy.set_param("/session_recorder/bag_dir", self.output_dir)
        rospy.set_param("/session_recorder/session_label", self.label)
        response = self.set_recording(True)
        if not response.success:
            raise RuntimeError("session recorder failed to start: " + response.message)
        self.output_path = response.message

    def stop(self):
        response = self.set_recording(False)
        if not response.success:
            raise RuntimeError("session recorder failed to stop: " + response.message)

    def archive(self, destination):
        os.makedirs(destination, exist_ok=True)
        if not self.output_path:
            return
        stem = os.path.splitext(self.output_path)[0]
        # rosbag finalizes through a temporary .active name. Its process has
        # exited when the service returns, but allow the mounted filesystem to
        # expose the rename before collecting every sidecar for a rejected run.
        time.sleep(0.25)
        for path in glob.glob(stem + ".*"):
            os.replace(path, os.path.join(destination, os.path.basename(path)))

class TopicRateMonitor:
    def __init__(self, topic):
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.count = 0
        self.receipts = []
        self.visible = []
        self.subscriber = rospy.Subscriber(topic, Bool, self.callback, queue_size=500)

    def callback(self, message):
        with self.condition:
            self.count += 1
            self.receipts.append(time.monotonic())
            self.visible.append(bool(message.data))
            self.condition.notify_all()

    def mark(self):
        with self.lock:
            return len(self.receipts)

    def rate_since(self, mark):
        with self.lock:
            samples = self.receipts[mark:]
        if len(samples) < 2 or samples[-1] <= samples[0]:
            return None
        return (len(samples) - 1) / (samples[-1] - samples[0])

    def wait_for_visible_after(self, mark, timeout):
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if any(self.visible[mark:]):
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.condition.wait(timeout=remaining)

    def measure(self, duration):
        mark = self.mark()
        rospy.sleep(duration)
        return self.rate_since(mark) or 0.0


class PoseAcquisitionMonitor:
    """Wait for an actual estimator pose, not a possibly stale visibility flag."""

    def __init__(self, topic):
        self.condition = threading.Condition()
        self.count = 0
        self.subscriber = rospy.Subscriber(
            topic, PoseWithCovarianceStamped, self.callback,
            queue_size=20, tcp_nodelay=True,
        )

    def callback(self, _message):
        with self.condition:
            self.count += 1
            self.condition.notify_all()

    def mark(self):
        with self.condition:
            return self.count

    def wait_after(self, mark, timeout):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.count <= mark:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False
                self.condition.wait(timeout=remaining)
            return True

def write_trial_csv(path, results):
    columns = [
        "pad_name", "trial", "x_pad_m", "y_pad_m", "height_m", "yaw_rad",
        "terminal_state", "success", "total_image_frames", "valid_pose_frames",
        "marker_pose_availability", "localization_samples",
        "localization_position_rmse_m", "horizontal_touchdown_error_m", "bag",
        "ground_truth_touchdown_position_pad_m",
        "touchdown_source", "image_source_samples", "camera_source_rate_hz",
        "camera_delivery_rate_hz",
        "estimator_frame_result_rate_hz", "collection_attempts",
        "camera_rate_requirement_met",
        "estimated_camera_height_at_touchdown_m",
        "lateral_eval_height_min_m", "lateral_eval_height_max_m",
        "lateral_linf_max_m", "lateral_bound_m", "lateral_bound_met",
        "marker_pose_age_median_ms", "marker_pose_age_p95_ms",
    ]
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)


def summarize(pad_name, results):
    available = [row["marker_pose_availability"] for row in results
                 if row["marker_pose_availability"] is not None]
    rmses = [row["localization_position_rmse_m"] for row in results
             if row["localization_position_rmse_m"] is not None]
    touchdown = [row["horizontal_touchdown_error_m"] for row in results
                 if row["success"] and row["horizontal_touchdown_error_m"] is not None]
    source_rates = [row["camera_source_rate_hz"] for row in results
                    if row.get("camera_source_rate_hz") is not None]
    delivery_rates = [row["camera_delivery_rate_hz"] for row in results
                      if row.get("camera_delivery_rate_hz") is not None]
    lateral_maxima = [row["lateral_linf_max_m"] for row in results
                      if row.get("lateral_linf_max_m") is not None]
    pose_age_medians = [row["marker_pose_age_median_ms"] for row in results
                        if row.get("marker_pose_age_median_ms") is not None]
    pose_age_p95s = [row["marker_pose_age_p95_ms"] for row in results
                     if row.get("marker_pose_age_p95_ms") is not None]
    total_frames = sum(row["total_image_frames"] for row in results)
    valid_frames = sum(row["valid_pose_frames"] for row in results)
    localization_samples = sum(row["localization_samples"] for row in results)
    localization_squared_error = sum(
        row["localization_samples"] * row["localization_position_rmse_m"] ** 2
        for row in results if row["localization_position_rmse_m"] is not None
    )
    return {
        "pad_name": pad_name,
        "trial_count": len(results),
        "success_count": sum(1 for row in results if row["success"]),
        "landing_success_rate": (
            sum(1 for row in results if row["success"]) / float(len(results))
            if results else None
        ),
        "pooled_marker_pose_availability": (
            valid_frames / float(total_frames) if total_frames else None
        ),
        "mean_trial_marker_pose_availability": (
            sum(available) / len(available) if available else None
        ),
        "mean_trial_localization_rmse_m": (
            sum(rmses) / len(rmses) if rmses else None
        ),
        "pooled_localization_position_rmse_m": (
            math.sqrt(localization_squared_error / localization_samples)
            if localization_samples else None
        ),
        "mean_successful_touchdown_error_m": (
            sum(touchdown) / len(touchdown) if touchdown else None
        ),
        "minimum_trial_camera_source_rate_hz": min(source_rates) if source_rates else None,
        "mean_trial_camera_source_rate_hz": (
            sum(source_rates) / len(source_rates) if source_rates else None
        ),
        "minimum_trial_camera_delivery_rate_hz": (
            min(delivery_rates) if delivery_rates else None
        ),
        "camera_rate_requirement_met_count": sum(
            1 for row in results if row.get("camera_rate_requirement_met")
        ),
        "lateral_bound_met_count": sum(
            1 for row in results if row.get("lateral_bound_met")
        ),
        "maximum_low_altitude_lateral_linf_m": (
            max(lateral_maxima) if lateral_maxima else None
        ),
        "all_trials_meet_lateral_bound": (
            bool(results) and all(row.get("lateral_bound_met") for row in results)
        ),
        "mean_trial_marker_pose_age_median_ms": (
            sum(pose_age_medians) / len(pose_age_medians)
            if pose_age_medians else None
        ),
        "maximum_trial_marker_pose_age_p95_ms": (
            max(pose_age_p95s) if pose_age_p95s else None
        ),
        "trials": results,
    }


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    module_dir = os.path.dirname(script_dir)
    default_root = "/work" if os.path.isdir("/work/modules") else os.path.abspath(
        os.path.join(module_dir, "../../..")
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("--pad-name", required=True, help="baseline or proposed run label")
    parser.add_argument(
        "--manifest",
        default=os.path.join(default_root, "flight_logs/aruco-landing/staging.json"),
        help="shared matched-position manifest; created once and then reused",
    )
    parser.add_argument(
        "--output-dir",
        default=os.path.join(default_root, "flight_logs/aruco-landing"),
    )
    parser.add_argument("--trials", type=int, default=None)
    parser.add_argument("--seed", type=int, default=1701)
    parser.add_argument("--radius-min", type=float, default=0.20)
    parser.add_argument("--radius-max", type=float, default=0.35)
    parser.add_argument("--trial-timeout", type=float, default=25.0)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--coarse-approach-speed", type=float, default=0.35)
    parser.add_argument("--coarse-approach-timeout", type=float, default=5.0)
    parser.add_argument("--maximum-technical-retries", type=int, default=2)
    parser.add_argument("--minimum-camera-rate", type=float, default=59.5)
    parser.add_argument("--preflight-attempts", type=int, default=5)
    parser.add_argument("--allow-rate-shortfall", action="store_true")
    parser.add_argument("--lateral-bound", type=float, default=0.05)
    parser.add_argument("--lateral-eval-height-max", type=float, default=0.50)
    parser.add_argument(
        "--resume", action="store_true",
        help="preserve completed rows in an interrupted run and collect the remainder",
    )
    parser.add_argument("--no-bag", action="store_true")
    parser.add_argument(
        "--session-recorder", action="store_true",
        help="record through the common /session_recorder service instead of spawning rosbag",
    )
    parser.add_argument(
        "--camera-config",
        default=os.path.join(module_dir, "config", "landing_camera.yaml"),
    )
    parser.add_argument(
        "--environment-config",
        default=os.path.join(module_dir, "config", "baseline_environment.yaml"),
    )
    parser.add_argument(
        "--experiment-config",
        default=os.path.join(
            default_root, "ws/aruco-landing/src/aruco_landing/config/common_experiment.yaml"
        ),
    )
    args = parser.parse_args(rospy.myargv()[1:])

    camera_config = load_yaml(args.camera_config)
    environment_config = load_yaml(args.environment_config)
    experiment_config = load_yaml(args.experiment_config)
    simulation_environment = environment_config["environment"]
    terminate_on_collision = bool(
        simulation_environment.get("terminate_trial_on_collision", True)
    )
    post_touchdown_behavior = str(
        simulation_environment.get("post_touchdown_behavior", "stop")
    ).lower()
    if post_touchdown_behavior not in ("stop", "drop"):
        raise ValueError("post_touchdown_behavior must be stop or drop")
    if args.coarse_approach_speed <= 0.0 or args.coarse_approach_timeout <= 0.0:
        raise ValueError("coarse approach speed and timeout must be positive")
    if args.maximum_technical_retries < 0:
        raise ValueError("maximum technical retries cannot be negative")
    if args.preflight_attempts <= 0:
        raise ValueError("preflight attempts must be positive")
    trial_count = args.trials or int(experiment_config["trials_per_pad"])
    h_max = float(experiment_config["landing_altitude_max_m"])
    h_min = float(experiment_config["landing_altitude_min_m"])
    if args.lateral_bound <= 0.0 or args.lateral_eval_height_max < h_min:
        raise ValueError("invalid lateral-bound evaluation parameters")
    manifest = generate_or_load_manifest(
        args.manifest, trial_count, args.seed, h_max, args.radius_min, args.radius_max
    )

    rospy.init_node("airsim_landing_trial_runner", anonymous=True)
    rate_monitor = TopicRateMonitor("/landing/target_visible")
    pose_monitor = PoseAcquisitionMonitor("/landing/vehicle_pose_pad")
    spawn = environment_config["spawn_ned"]
    explicit_pad_ned = environment_config["ground_truth"].get("pad_local_ned_m")
    pad_local_ned = np.array(
        explicit_pad_ned if explicit_pad_ned is not None else
        [-float(spawn["x_m"]), -float(spawn["y_m"]), -float(spawn["z_m"])],
        dtype=float,
    )
    pad_from_ned = np.asarray(
        environment_config["ground_truth"]["pad_from_local_ned_rotation"],
        dtype=float,
    )
    measurements = TrialMeasurements(
        pad_local_ned=pad_local_ned,
        pad_from_ned=pad_from_ned,
        lateral_bound_m=args.lateral_bound,
        lateral_eval_height_min_m=h_min,
        lateral_eval_height_max_m=args.lateral_eval_height_max,
    )
    airsim_config = camera_config["airsim"]
    client = airsim.MultirotorClient(
        ip=str(airsim_config.get("ip", "127.0.0.1")),
        port=int(airsim_config.get("port", 41451)),
    )
    client.confirmConnection()
    vehicle_name = str(airsim_config["vehicle_name"])

    rospy.wait_for_service("/landing_controller/enable", timeout=20.0)
    rospy.wait_for_service("/landing_controller/reset", timeout=20.0)
    if args.session_recorder and not args.no_bag:
        rospy.wait_for_service("/session_recorder/set_recording", timeout=20.0)
    enable_controller = rospy.ServiceProxy("/landing_controller/enable", SetBool)
    reset_controller = rospy.ServiceProxy("/landing_controller/reset", Trigger)
    observed_rate = 0.0
    preflight_attempt = 0
    for preflight_attempt in range(1, args.preflight_attempts + 1):
        observed_rate = rate_monitor.measure(3.0)
        rospy.loginfo(
            "preflight estimator frame-result rate: %.2f Hz (attempt %d/%d)",
            observed_rate, preflight_attempt, args.preflight_attempts,
        )
        if observed_rate >= args.minimum_camera_rate or args.allow_rate_shortfall:
            break
        if preflight_attempt < args.preflight_attempts:
            rospy.logwarn("AirSim render stream is still warming up; retrying preflight")
            rospy.sleep(2.0)
    if observed_rate < args.minimum_camera_rate and not args.allow_rate_shortfall:
        raise RuntimeError(
            "observed %.2f Hz is below %.2f Hz; fix simulator throughput or pass "
            "--allow-rate-shortfall for non-paper smoke tests"
            % (observed_rate, args.minimum_camera_rate)
        )

    run_dir = os.path.join(args.output_dir, args.pad_name)
    bag_dir = os.path.join(run_dir, "bags")
    os.makedirs(bag_dir, exist_ok=True)
    results = []
    existing_summary_path = os.path.join(run_dir, "summary.json")
    if args.resume and os.path.isfile(existing_summary_path):
        with open(existing_summary_path, "r", encoding="utf-8") as stream:
            existing_summary = json.load(stream)
        if existing_summary.get("pad_name") != args.pad_name:
            raise ValueError("resume summary pad name does not match")
        results = list(existing_summary.get("trials", []))
        rospy.loginfo("resuming %s after %d accepted trials", args.pad_name, len(results))
    atomic_json(
        os.path.join(run_dir, "run_config.json"),
        {
            "pad_name": args.pad_name,
            "camera_config_path": os.path.abspath(args.camera_config),
            "environment_config_path": os.path.abspath(args.environment_config),
            "experiment_config_path": os.path.abspath(args.experiment_config),
            "experiment_parameters": experiment_config,
            "simulation_touchdown_parameters": {
                "terminate_trial_on_collision": terminate_on_collision,
                "post_touchdown_behavior": post_touchdown_behavior,
            },
            "camera_parameters": camera_config["camera"],
            "matched_staging_manifest_path": os.path.abspath(args.manifest),
            "matched_staging_manifest": manifest,
            "bag_topics": [] if args.no_bag else DEFAULT_BAG_TOPICS,
            "recording_adapter": (
                "disabled" if args.no_bag else
                "session_recorder_service" if args.session_recorder else
                "direct_rosbag"
            ),
            "minimum_camera_rate_hz": args.minimum_camera_rate,
            "preflight_observed_estimator_rate_hz": observed_rate,
            "preflight_attempts_used": preflight_attempt,
            "coarse_approach": {
                "known_pad_location_only": True,
                "speed_mps": args.coarse_approach_speed,
                "timeout_s": args.coarse_approach_timeout,
                "ends_at_first_valid_marker_pose": True,
            },
            "landing_activation": "first_valid_marker_pose",
            "descent_activation": "simultaneous_with_landing_activation",
            "maximum_technical_retries": args.maximum_technical_retries,
            "lateral_bound_evaluation": {
                "norm": "linf",
                "bound_m": args.lateral_bound,
                "height_min_m": h_min,
                "height_max_m": args.lateral_eval_height_max,
                "ground_truth_is_evaluation_only": True,
            },
        },
    )
    ned_from_pad = np.linalg.inv(pad_from_ned)
    completed_trials = {int(row["trial"]) for row in results}

    for staging in manifest["staging_positions"]:
        if rospy.is_shutdown():
            break
        trial_number = int(staging["trial"])
        if trial_number in completed_trials:
            continue
        collection_attempt = 0
        while not rospy.is_shutdown():
            collection_attempt += 1
            rospy.loginfo(
                "starting %s trial %d/%d (collection attempt %d)",
                args.pad_name, trial_number, trial_count, collection_attempt,
            )
            enable_controller(False)
            reset_controller()
            relative_pad = np.array(
                [staging["x_pad_m"], staging["y_pad_m"], staging["height_m"]],
                dtype=float,
            )
            local_ned = pad_local_ned + np.matmul(ned_from_pad, relative_pad)
            # AirSim reset is the supported way to clear residual linear and
            # angular velocity. Direct simSetKinematics calls can race UE4's
            # PhysX substep task and have caused an engine SIGSEGV in long runs.
            client.reset()
            client.enableApiControl(True, vehicle_name=vehicle_name)
            client.armDisarm(True, vehicle_name=vehicle_name)
            pose = airsim.Pose(
                airsim.Vector3r(*map(float, local_ned)),
                airsim.to_quaternion(0.0, 0.0, float(staging["yaw_rad"])),
            )
            client.simSetVehiclePose(
                pose, ignore_collision=True, vehicle_name=vehicle_name
            )

            # Command the hover immediately after teleport. Bag startup takes
            # about 0.35 s; delaying this command until afterward lets
            # SimpleFlight lose roughly 0.6 m before the trial even begins.
            client.moveByVelocityAsync(
                0.0, 0.0, 0.0,
                max(args.settle_time + args.coarse_approach_timeout + 1.0, 1.0),
                vehicle_name=vehicle_name,
            )

            bag_path = os.path.join(
                bag_dir, "%s_trial_%02d.bag" % (args.pad_name, trial_number)
            )
            active_bag_path = bag_path + ".active"
            if os.path.isfile(active_bag_path):
                interrupted_dir = os.path.join(run_dir, "interrupted_attempts")
                os.makedirs(interrupted_dir, exist_ok=True)
                interrupted_name = "%s_trial_%02d_interrupted_%d.bag.active" % (
                    args.pad_name, trial_number, int(time.time())
                )
                os.replace(
                    active_bag_path, os.path.join(interrupted_dir, interrupted_name)
                )
            if args.no_bag:
                recorder = None
            elif args.session_recorder:
                recorder = SessionServiceRecorder(
                    bag_dir, "%s_trial_%02d_attempt_%02d" % (
                        args.pad_name, trial_number, collection_attempt
                    )
                )
            else:
                recorder = BagRecorder(bag_path, DEFAULT_BAG_TOPICS)
            measurements.reset()
            if recorder is not None:
                recorder.start()
            rate_mark = rate_monitor.mark()

            # Flush any pose that was already in flight before simSetVehiclePose.
            # Only poses received after this fixed settle interval are allowed
            # to activate landing at the new 2 m staging position.
            time.sleep(args.settle_time)
            reset_controller()
            pose_mark = pose_monitor.mark()
            enable_controller(True)

            # The enabled controller stays in WAITING_FOR_MARKER and exerts no
            # control. A new pose activates horizontal feedback and descent at
            # once. If it is not visible here, approach using known pad position.
            marker_acquired = pose_monitor.wait_after(pose_mark, 0.10)
            if not marker_acquired:
                client.moveToPositionAsync(
                    float(pad_local_ned[0]),
                    float(pad_local_ned[1]),
                    float(local_ned[2]),
                    args.coarse_approach_speed,
                    timeout_sec=args.coarse_approach_timeout,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0.0),
                    vehicle_name=vehicle_name,
                )
                marker_acquired = pose_monitor.wait_after(
                    pose_mark, args.coarse_approach_timeout
                )
            client.cancelLastTask(vehicle_name=vehicle_name)
            if not marker_acquired:
                client.moveByVelocityAsync(
                    0.0, 0.0, 0.0, 0.25, vehicle_name=vehicle_name
                )

            collision_baseline = None
            if terminate_on_collision:
                collision_baseline = int(
                    client.simGetCollisionInfo(vehicle_name=vehicle_name).time_stamp
                )

            timed_out = False
            contact_touchdown = False
            approach_failed = not marker_acquired
            if marker_acquired:
                # This is the hard information boundary: from here through
                # touchdown, only marker-derived state drives the controller.
                started = time.monotonic()
                while not rospy.is_shutdown():
                    if measurements.state_event.wait(timeout=0.1):
                        break
                    if terminate_on_collision:
                        collision = client.simGetCollisionInfo(vehicle_name=vehicle_name)
                        if (collision.has_collided
                                and int(collision.time_stamp) > collision_baseline):
                            contact_touchdown = True
                            break
                    if time.monotonic() - started >= args.trial_timeout:
                        timed_out = True
                        break

            # A marker-only latency-compensated stop is intentionally issued
            # just before physical h_min. Keep logging GT briefly while the
            # zero-velocity command settles; GT does not affect that command.
            if measurements.state == "TOUCHDOWN":
                measurements.ground_truth_touchdown_event.wait(timeout=1.0)

            measurements.stop()
            enable_controller(False)
            if recorder is not None:
                recorder.stop()
                bag_path = recorder.output_path
            estimator_rate = rate_monitor.rate_since(rate_mark)
            terminal_override = (
                "ABORTED_APPROACH_NO_MARKER" if approach_failed
                else "TOUCHDOWN" if contact_touchdown else None
            )
            result = dict(staging)
            result.update(
                measurements.result(
                    timeout_abort=timed_out,
                    terminal_override=terminal_override,
                )
            )
            result["estimator_frame_result_rate_hz"] = estimator_rate
            result["collection_attempts"] = collection_attempt
            result["camera_rate_requirement_met"] = (
                estimator_rate is not None
                and estimator_rate >= args.minimum_camera_rate
            )
            if result["camera_rate_requirement_met"] or args.allow_rate_shortfall:
                break
            if collection_attempt > args.maximum_technical_retries:
                raise RuntimeError(
                    "trial %d remained below %.2f Hz after %d collection attempts"
                    % (trial_number, args.minimum_camera_rate, collection_attempt)
                )
            if isinstance(recorder, SessionServiceRecorder):
                recorder.archive(os.path.join(run_dir, "invalid_rate_attempts"))
            elif recorder is not None and os.path.isfile(bag_path):
                invalid_dir = os.path.join(run_dir, "invalid_rate_attempts")
                os.makedirs(invalid_dir, exist_ok=True)
                invalid_name = "%s_trial_%02d_attempt_%02d.bag" % (
                    args.pad_name, trial_number, collection_attempt
                )
                os.replace(bag_path, os.path.join(invalid_dir, invalid_name))
            rospy.logwarn(
                "trial %d estimator rate %.2f Hz is below %.2f Hz; recollecting same staging",
                trial_number, estimator_rate or 0.0, args.minimum_camera_rate,
            )

        result["touchdown_source"] = "airsim_contact" if contact_touchdown else (
            "estimated_%s_height" % experiment_config.get(
                "touchdown_height_reference", "camera"
            ) if result["terminal_state"] == "TOUCHDOWN" else None
        )
        result["pad_name"] = args.pad_name
        result["bag"] = None if args.no_bag else os.path.relpath(bag_path, run_dir)
        results.append(result)
        atomic_json(os.path.join(run_dir, "summary.json"), summarize(args.pad_name, results))
        write_trial_csv(os.path.join(run_dir, "trials.csv"), results)
        rospy.loginfo(
            "trial %d: %s, availability=%s, RMSE=%s m, touchdown=%s m",
            trial_number,
            result["terminal_state"],
            result["marker_pose_availability"],
            result["localization_position_rmse_m"],
            result["horizontal_touchdown_error_m"],
        )
        if not result["camera_rate_requirement_met"]:
            rospy.logwarn(
                "trial %d delivery rate %.2f Hz is below the %.2f Hz acceptance threshold",
                trial_number,
                result["camera_delivery_rate_hz"] or 0.0,
                args.minimum_camera_rate,
            )

    summary = summarize(args.pad_name, results)
    summary["preflight_observed_estimator_rate_hz"] = observed_rate
    atomic_json(os.path.join(run_dir, "summary.json"), summary)
    write_trial_csv(os.path.join(run_dir, "trials.csv"), results)
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

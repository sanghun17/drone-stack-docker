#!/usr/bin/env python3
"""Apply marker-derived pad-frame velocity commands to AirSim body-frame API."""

import math
import threading

import airsim
import numpy as np
import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion, quaternion_matrix


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AirSimControlAdapter:
    def __init__(self):
        rospy.init_node("airsim_landing_control")
        camera_config_path = rospy.get_param(
            "~camera_config_path",
            "/work/stack-assets/aruco-landing-sim-x86/config/landing_camera.yaml",
        )
        experiment_config_path = rospy.get_param(
            "~experiment_config_path",
            "/work/ws/aruco-landing/src/aruco_landing/config/common_experiment.yaml",
        )
        environment_config_path = rospy.get_param(
            "~environment_config_path",
            "/work/stack-assets/aruco-landing-sim-x86/config/baseline_environment.yaml",
        )
        with open(camera_config_path, "r", encoding="utf-8") as stream:
            camera_config = yaml.safe_load(stream)
        with open(experiment_config_path, "r", encoding="utf-8") as stream:
            experiment_config = yaml.safe_load(stream)
        with open(environment_config_path, "r", encoding="utf-8") as stream:
            environment_config = yaml.safe_load(stream)

        airsim_config = camera_config["airsim"]
        self.vehicle_name = str(airsim_config["vehicle_name"])
        self.rate_hz = float(experiment_config["update_rate_hz"])
        self.command_rate_hz = float(
            experiment_config.get("airsim_command_rate_hz", self.rate_hz)
        )
        if self.command_rate_hz <= 0.0 or self.command_rate_hz > self.rate_hz:
            raise ValueError("airsim_command_rate_hz must be in (0, update_rate_hz]")
        self.last_command_stamp = None
        self.yaw_kp = float(rospy.get_param("~yaw_kp", 2.0))
        self.yaw_rate_limit_deg_s = float(
            rospy.get_param("~yaw_rate_limit_deg_s", 45.0)
        )
        self.command_timeout = float(rospy.get_param("~command_timeout_s", 0.10))
        self.post_touchdown_behavior = str(
            environment_config["environment"].get(
                "post_touchdown_behavior", "stop"
            )
        ).lower()
        if self.post_touchdown_behavior not in ("stop", "drop"):
            raise ValueError("post_touchdown_behavior must be stop or drop")
        self.yaw_rate_sign = float(rospy.get_param("~yaw_rate_sign", -1.0))
        self.lock = threading.Lock()
        self.command = None
        self.command_receipt = None
        self.vehicle_pose_pad = None
        self.yaw_reference = float(experiment_config["yaw_reference_rad"])
        self.active = False
        self.terminal = False
        self.was_commanding = False
        self.drop_sent = False

        self.client = airsim.MultirotorClient(
            ip=str(airsim_config.get("ip", "127.0.0.1")),
            port=int(airsim_config.get("port", 41451)),
        )
        self.client.confirmConnection()
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)

        rospy.Subscriber(
            "/landing/cmd_vel_pad", TwistStamped, self.command_callback,
            queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/landing/vehicle_pose_pad", PoseWithCovarianceStamped,
            self.pose_callback, queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/landing/yaw_cmd", Float64, self.yaw_callback, queue_size=1
        )
        rospy.Subscriber(
            "/landing/controller/active", Bool, self.active_callback, queue_size=1
        )
        rospy.Subscriber(
            "/landing/controller/touchdown", Bool, self.terminal_callback, queue_size=1
        )
        rospy.Subscriber(
            "/landing/controller/abort", Bool, self.terminal_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.update)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "AirSim control adapter ready: vehicle=%s, %.1f Hz body-frame commands",
            self.vehicle_name, self.rate_hz
        )

    def command_callback(self, message):
        with self.lock:
            self.command = message
            self.command_receipt = rospy.get_time()

    def pose_callback(self, message):
        with self.lock:
            self.vehicle_pose_pad = message

    def yaw_callback(self, message):
        with self.lock:
            self.yaw_reference = float(message.data)

    def active_callback(self, message):
        with self.lock:
            if message.data and not self.active:
                self.terminal = False
                self.drop_sent = False
            self.active = bool(message.data)

    def terminal_callback(self, message):
        if message.data:
            with self.lock:
                self.terminal = True

    def update(self, event):
        with self.lock:
            command = self.command
            receipt = self.command_receipt
            pose = self.vehicle_pose_pad
            yaw_reference = self.yaw_reference
            active = self.active
            terminal = self.terminal
            drop_sent = self.drop_sent

        vx_frd = vy_frd = vz_frd = yaw_rate_deg = 0.0
        fresh = receipt is not None and rospy.get_time() - receipt <= self.command_timeout
        if active and not terminal and fresh and command is not None and pose is not None:
            orientation = pose.pose.pose.orientation
            rotation_pad_from_body_flu = quaternion_matrix(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )[0:3, 0:3]
            velocity_pad = np.array(
                [command.twist.linear.x, command.twist.linear.y,
                 command.twist.linear.z], dtype=float
            )
            velocity_body_flu = np.matmul(
                rotation_pad_from_body_flu.T, velocity_pad
            )
            vx_frd = float(velocity_body_flu[0])
            vy_frd = float(-velocity_body_flu[1])
            vz_frd = float(-velocity_body_flu[2])
            _, _, vehicle_yaw = euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            yaw_rate_ros = self.yaw_kp * wrap_angle(yaw_reference - vehicle_yaw)
            yaw_rate_deg = self.yaw_rate_sign * math.degrees(yaw_rate_ros)
            yaw_rate_deg = max(
                -self.yaw_rate_limit_deg_s,
                min(self.yaw_rate_limit_deg_s, yaw_rate_deg),
            )

        should_command = active and not terminal
        due = (
            self.last_command_stamp is None
            or (event.current_real - self.last_command_stamp).to_sec()
            >= 1.0 / self.command_rate_hz
        )
        if should_command and due:
            duration = max(2.0 / self.command_rate_hz, 0.05)
            self.client.moveByVelocityBodyFrameAsync(
                vx_frd,
                vy_frd,
                vz_frd,
                duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate_deg),
                vehicle_name=self.vehicle_name,
            )
            self.last_command_stamp = event.current_real
            self.was_commanding = True
        elif not should_command and self.was_commanding:
            # Send one stop at the active -> inactive transition, then leave the
            # shared AirSim RPC server idle until the next landing trial.
            self.client.moveByVelocityBodyFrameAsync(
                0.0,
                0.0,
                0.0,
                0.2,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=0.0),
                vehicle_name=self.vehicle_name,
            )
            self.was_commanding = False
            self.last_command_stamp = None
        if (terminal and self.post_touchdown_behavior == "drop"
                and not drop_sent):
            self.client.armDisarm(False, vehicle_name=self.vehicle_name)
            with self.lock:
                self.drop_sent = True

    def shutdown(self):
        try:
            self.client.moveByVelocityBodyFrameAsync(
                0.0, 0.0, 0.0, 0.2, vehicle_name=self.vehicle_name
            ).join()
            self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
        except Exception:
            pass


if __name__ == "__main__":
    AirSimControlAdapter()
    rospy.spin()

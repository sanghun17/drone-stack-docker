#!/usr/bin/env python3
"""Standard ROS trajectory to MAVROS body-velocity controller.

This node intentionally depends on no planner package. Any planner that publishes
trajectory_msgs/MultiDOFJointTrajectory can drive it. Output defaults to the
flight-safety normal-lane input and can be redirected with ``~setpoint_topic``.
"""

import math

import rospy
import tf.transformations as tft
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from trajectory_msgs.msg import MultiDOFJointTrajectory


def clamp(value, limit):
    return max(-limit, min(limit, value))


def yaw_of(rotation):
    return tft.euler_from_quaternion(
        [rotation.x, rotation.y, rotation.z, rotation.w]
    )[2]


class TrajectoryController:
    def __init__(self):
        rospy.init_node("local_controller")
        self.control_rate = float(rospy.get_param("~control_rate", 30.0))
        self.command_duration = float(rospy.get_param("~command_duration", 0.3))
        self.kp = float(rospy.get_param("~kp", 1.0))
        self.kd = float(rospy.get_param("~kd", 0.5))
        self.max_correction = float(rospy.get_param("~max_correction", 1.0))
        self.xy_vel_max = float(rospy.get_param("~xy_vel_max", 1.8))
        self.z_vel_max = float(rospy.get_param("~z_vel_max", 0.5))
        self.yaw_kp = float(rospy.get_param("~yaw_kp", 1.0))
        self.yaw_rate_max = math.radians(
            float(rospy.get_param("~yaw_rate_max_deg", 90.0))
        )
        self.trajectory_topic = rospy.get_param(
            "~trajectory_topic", "/jax/optimal_trajectory"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/robot/odom")
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/local_controller/setpoint_raw/local"
        )
        self.enabled = False
        self.odom = None
        self.points = []
        self.start_time = None

        self.publisher = rospy.Publisher(
            self.setpoint_topic, PositionTarget, queue_size=1
        )
        rospy.Subscriber(self.odom_topic, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(
            self.trajectory_topic,
            MultiDOFJointTrajectory,
            self._on_trajectory,
            queue_size=1,
        )
        rospy.Service("/control_bridge/toggle_running", SetBool, self._toggle)
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._tick)
        rospy.loginfo(
            "local_controller ready: %s + %s -> %s (disabled)",
            self.trajectory_topic,
            self.odom_topic,
            self.setpoint_topic,
        )

    def _toggle(self, request):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.points = []
            self.start_time = None
        return SetBoolResponse(
            success=True,
            message="trajectory controller %s" % ("enabled" if self.enabled else "disabled"),
        )

    def _on_odom(self, message):
        self.odom = message

    def _on_trajectory(self, message):
        if not self.enabled or len(message.points) < 2:
            return
        self.points = list(message.points)
        now = rospy.Time.now()
        self.start_time = message.header.stamp if message.header.stamp > now else now

    @staticmethod
    def _desired(point):
        if not point.transforms:
            return None
        transform = point.transforms[0]
        velocity = point.velocities[0].linear if point.velocities else None
        return {
            "x": transform.translation.x,
            "y": transform.translation.y,
            "z": transform.translation.z,
            "yaw": yaw_of(transform.rotation),
            "vx": velocity.x if velocity else 0.0,
            "vy": velocity.y if velocity else 0.0,
            "vz": velocity.z if velocity else 0.0,
        }

    def _tick(self, _event):
        if not self.enabled or self.odom is None or not self.points or self.start_time is None:
            return
        now = rospy.Time.now()
        if now < self.start_time:
            return
        index = int((now - self.start_time).to_sec() / self.command_duration) + 1
        if index >= len(self.points):
            self.points = []
            return
        desired = self._desired(self.points[index])
        if desired is None:
            return

        pose = self.odom.pose.pose
        twist = self.odom.twist.twist.linear
        current_yaw = yaw_of(pose.orientation)
        cosine, sine = math.cos(current_yaw), math.sin(current_yaw)

        # Position and odometry velocity are world ENU. Planner feed-forward is
        # body FLU, matching the historical JAX trajectory contract.
        error_x = desired["x"] - pose.position.x
        error_y = desired["y"] - pose.position.y
        error_z = desired["z"] - pose.position.z
        body_error_x = cosine * error_x + sine * error_y
        body_error_y = -sine * error_x + cosine * error_y
        body_velocity_x = cosine * twist.x + sine * twist.y
        body_velocity_y = -sine * twist.x + cosine * twist.y

        vx = desired["vx"] + clamp(
            self.kp * body_error_x + self.kd * (desired["vx"] - body_velocity_x),
            self.max_correction,
        )
        vy = desired["vy"] + clamp(
            self.kp * body_error_y + self.kd * (desired["vy"] - body_velocity_y),
            self.max_correction,
        )
        vz = desired["vz"] + clamp(
            self.kp * error_z + self.kd * (desired["vz"] - twist.z),
            self.max_correction,
        )
        horizontal = math.hypot(vx, vy)
        if horizontal > self.xy_vel_max > 0.0:
            scale = self.xy_vel_max / horizontal
            vx, vy = vx * scale, vy * scale
        vz = clamp(vz, self.z_vel_max)
        yaw_error = math.atan2(
            math.sin(desired["yaw"] - current_yaw),
            math.cos(desired["yaw"] - current_yaw),
        )

        setpoint = PositionTarget()
        setpoint.header.stamp = now
        setpoint.header.frame_id = "base_link"
        setpoint.coordinate_frame = PositionTarget.FRAME_BODY_NED
        setpoint.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW
        )
        setpoint.velocity.x = vx
        setpoint.velocity.y = vy
        setpoint.velocity.z = vz
        setpoint.yaw_rate = clamp(self.yaw_kp * yaw_error, self.yaw_rate_max)
        self.publisher.publish(setpoint)


if __name__ == "__main__":
    TrajectoryController()
    rospy.spin()

#!/usr/bin/env python3
"""Drive the generic RHEM planner through the sim stack's existing controller."""
import threading
import time

import numpy as np
import rospy
from bsp_planner.srv import bsp_srv
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, UInt32, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import euler_from_quaternion
from traj_utils.msg import MixTraj

from rhem_trajectory import parameterize


def waypoint(pose):
    p, q = pose.position, pose.orientation
    if not np.isfinite([q.x, q.y, q.z, q.w]).all() or np.linalg.norm([q.x, q.y, q.z, q.w]) < 1e-9:
        raise ValueError('Invalid path attitude')
    return [p.x, p.y, p.z, euler_from_quaternion([q.x, q.y, q.z, q.w])[2]]


class Adapter:
    def __init__(self):
        if rospy.get_param('/system/platform') != 'sim':
            raise RuntimeError('This adapter requires the sim profile')
        self.frame = rospy.get_param('/system/world_frame', 'odom')
        self.limits = rospy.get_param('/planning/shared')
        for short, full in [('max_a_xy', 'max_acc_xy'), ('max_a_z', 'max_acc_z'),
                            ('max_a_wz', 'max_yaw_acc')]:
            self.limits[full] = self.limits[short]
        bounds = rospy.get_param('/target_bounding_volume')
        self.lower = np.array([bounds[k + '_min'] for k in 'xyz'])
        self.upper = np.array([bounds[k + '_max'] for k in 'xyz'])
        self.lock = threading.Lock()
        self.odom = None
        self.enabled = rospy.get_param('~start', False)
        self.goal = None
        self.finish = 0.0
        self.traj_id = 0
        self.publisher = rospy.Publisher('/planning/trajectory', MixTraj, queue_size=1)
        self.accepted = rospy.Publisher('~belief_trajectories', UInt32, queue_size=1)
        self.improved = rospy.Publisher('~belief_improved', Bool, queue_size=1)
        self.subscriber = rospy.Subscriber(rospy.get_param('/system/odom_topic'), Odometry,
                                          self.receive, queue_size=1)
        self.toggle_service = rospy.Service('~toggle_running', SetBool, self.toggle)
        self.planner = rospy.ServiceProxy('/rhem/bsp_planner', bsp_srv)

    def receive(self, msg):
        with self.lock:
            self.odom = msg

    def toggle(self, req):
        with self.lock:
            if req.data and not self.enabled:
                self.goal = None
                self.finish = 0.0
            self.enabled = req.data
        return SetBoolResponse(True, 'RHEM planning enabled' if req.data else 'RHEM planning disabled')

    def current(self):
        with self.lock:
            odom = self.odom
        if odom is None or odom.header.frame_id != self.frame:
            raise ValueError('Waiting for common odometry in ' + self.frame)
        if abs((rospy.Time.now() - odom.header.stamp).to_sec()) > 0.5:
            raise ValueError('Common odometry is stale')
        point = np.array(waypoint(odom.pose.pose))
        vel = odom.twist.twist.linear
        if not np.isfinite(point).all():
            raise ValueError('Non-finite common odometry')
        return point, np.linalg.norm([vel.x, vel.y, vel.z])

    def run(self):
        rospy.wait_for_service('/rhem/bsp_planner')
        while not rospy.is_shutdown():
            time.sleep(0.1)
            try:
                if not self.enabled:
                    continue
                point, speed = self.current()
                if self.goal is not None and (rospy.Time.now().to_sec() < self.finish
                        or np.linalg.norm(point[:3] - self.goal) > 0.3):
                    continue
                if speed > 0.3:
                    continue
                response = self.planner(Header(stamp=rospy.Time.now(), frame_id=self.frame))
                if not self.enabled:
                    continue
                if not response.belief_space:
                    raise ValueError('Path has no valid belief evaluation; keeping hover')
                path = np.array([waypoint(p) for p in response.path])
                if len(path) < 2:
                    raise ValueError('RHEM has not produced a path')
                point, speed = self.current()
                if np.linalg.norm(path[0, :3] - point[:3]) > 0.3 or speed > 0.3:
                    raise ValueError('Vehicle moved during RHEM planning; waiting to replan')
                if np.any(path[:, :3] < self.lower) or np.any(path[:, :3] > self.upper):
                    raise ValueError('RHEM path is outside the common exploration bounds')
                knots, controls, durations, yaw = parameterize(path, self.limits)
                self.traj_id += 1
                msg = MixTraj(bspline_degree=5, traj_id=self.traj_id,
                              start_time=rospy.Time.now(), real_traj_duration=float(sum(durations)),
                              knots=knots.tolist(), pos_pts=[Point(*p) for p in controls],
                              minco_order=5, coef_yaw=yaw.ravel().tolist(), duration_yaw=durations.tolist())
                self.publisher.publish(msg)
                self.accepted.publish(self.traj_id)
                self.improved.publish(response.belief_improved)
                self.goal = path[-1, :3]
                self.finish = msg.start_time.to_sec() + msg.real_traj_duration
                rospy.loginfo('RHEM trajectory %d: %d vertices, %.2f seconds',
                              self.traj_id, len(path), msg.real_traj_duration)
            except (ValueError, rospy.ServiceException) as error:
                rospy.logwarn_throttle(5, 'RHEM: %s', error)
                time.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('rhem_control_adapter')
    Adapter().run()

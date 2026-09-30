#!/usr/bin/env python3
"""Drive the generic RHEM planner through the sim stack's existing controller."""
import threading
import time
import json

import numpy as np
import rospy
from bsp_planner.srv import bsp_srv, validate_trajectory
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, UInt32, Bool, String
from std_srvs.srv import SetBool, SetBoolResponse
from tf.transformations import euler_from_quaternion
from traj_utils.msg import MixTraj

from rhem_trajectory import parameterize, trajectory_envelopes


def waypoint(pose):
    p, q = pose.position, pose.orientation
    if not np.isfinite([q.x, q.y, q.z, q.w]).all() or np.linalg.norm([q.x, q.y, q.z, q.w]) < 1e-9:
        raise ValueError('Invalid path attitude')
    return [p.x, p.y, p.z, euler_from_quaternion([q.x, q.y, q.z, q.w])[2]]


class PlanningFailureWindow:
    """Allow recovery, but do not retry an unusable planner indefinitely."""
    def __init__(self, timeout_s=30.0):
        if not np.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Planning failure timeout must be finite and positive')
        self.timeout_s = timeout_s
        self.clear()

    def clear(self):
        self.started = None
        self.last = None
        self.attempts = 0

    def reject(self, now):
        if self.started is None or now < self.last:
            self.started = now
            self.attempts = 0
        self.last = now
        self.attempts += 1
        return self.attempts >= 3 and now - self.started >= self.timeout_s


class Adapter:
    def __init__(self):
        if rospy.get_param('/system/platform') != 'sim':
            raise RuntimeError('This adapter requires the sim profile')
        self.frame = rospy.get_param('/system/world_frame', 'odom')
        self.require_belief = rospy.get_param('/comparison/rhem_belief_mode', 'rovio') == 'rovio'
        if not self.require_belief and any(rospy.get_param('/comparison/sources/'+k) != 'gt'
                for k in ('planning_source', 'control_source')):
            raise RuntimeError('Belief isolation requires GT planning and control')
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
        self.status = rospy.Publisher('~status', String, queue_size=1)
        self.task_failure = rospy.Publisher('/planning/task_fail_reason', String, queue_size=1)
        timeout = float(rospy.get_param('~planning_failure_timeout_s', 30.0))
        self.planning_failures = PlanningFailureWindow(timeout)
        # Include the stopping policy in the recorder's parameter snapshot.
        rospy.set_param('~planning_failure_timeout_s', timeout)
        self.last_status = 0.0
        self.subscriber = rospy.Subscriber(rospy.get_param('/system/odom_topic'), Odometry,
                                          self.receive, queue_size=1)
        self.toggle_service = rospy.Service('~toggle_running', SetBool, self.toggle)
        self.planner = rospy.ServiceProxy('/rhem/bsp_planner', bsp_srv)
        self.validate = rospy.ServiceProxy('/rhem/validate_trajectory', validate_trajectory)
        self.envelope_extent = float(rospy.get_param('/system/voxel_size')) * 0.5
        rospy.set_param('~execution_profile', 'continuous-validated-v1')

    def receive(self, msg):
        with self.lock:
            self.odom = msg

    def toggle(self, req):
        with self.lock:
            if req.data and not self.enabled:
                self.goal = None
                self.finish = 0.0
                self.planning_failures.clear()
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

    def reject_plan(self, reason, detail):
        """Report an operational failure; no GT error or coverage threshold is used."""
        with self.lock:
            if not self.enabled:
                return
            if not self.planning_failures.reject(rospy.Time.now().to_sec()):
                return
            self.enabled = False
            self.task_failure.publish(String(data=reason))
            rospy.logerr('RHEM planning failed after %.1f s and %d retries: %s (%s)',
                         self.planning_failures.last - self.planning_failures.started,
                         self.planning_failures.attempts, reason, detail)

    def checked_trajectory(self, path):
        """Use the largest verified through-tangent; never restore stop-at-every-vertex."""
        last_reason = ''
        for scale in (1., .5, .25, .125, .0625):
            result = parameterize(path, self.limits, tangent_scale=scale)
            centers, sizes = trajectory_envelopes(*result[:3], self.envelope_extent)
            reply = self.validate(Header(stamp=rospy.Time.now(), frame_id=self.frame),
                [Point(*p) for p in centers], [Vector3(*p) for p in sizes])
            if reply.valid:
                rospy.loginfo('RHEM continuous trajectory: tangent_scale=%.4f, %d checked envelopes',
                              scale, reply.checked)
                return result
            last_reason = reply.reason
        raise ValueError('No collision-free continuous trajectory: ' + last_reason)

    def run(self):
        rospy.wait_for_service('/rhem/bsp_planner')
        while not rospy.is_shutdown():
            time.sleep(0.1)
            try:
                if not self.enabled:
                    continue
                point, speed = self.current()
                if time.monotonic() - self.last_status >= 1.0:
                    self.status.publish(json.dumps(dict(time=rospy.Time.now().to_sec(),
                        position=point.tolist(), speed=float(speed), trajectory=self.traj_id,
                        goal=None if self.goal is None else self.goal.tolist(), finish=self.finish,
                        goal_error=None if self.goal is None else float(np.linalg.norm(point[:3]-self.goal)))))
                    self.last_status = time.monotonic()
                # Finish the published trajectory, then request the next plan.
                # The shared traj_server holds its final pose while we compute;
                # there is no extra speed/arrival gate before the request.
                if self.goal is not None and rospy.Time.now().to_sec() < self.finish:
                    continue
                try:
                    response = self.planner(Header(stamp=rospy.Time.now(), frame_id=self.frame))
                except rospy.ServiceException as error:
                    self.reject_plan('PLANNER_SERVICE', str(error))
                    raise
                if not self.enabled:
                    continue
                if self.require_belief and not response.belief_space:
                    self.reject_plan('BELIEF_INVALID', 'Path has no valid belief evaluation')
                    raise ValueError('Path has no valid belief evaluation; keeping hover')
                path = np.array([waypoint(p) for p in response.path])
                if len(path) < 2:
                    self.reject_plan('NOVIEWPOINT', 'RHEM has not produced a path')
                    raise ValueError('RHEM has not produced a path')
                point, speed = self.current()
                if np.linalg.norm(path[0, :3] - point[:3]) > 0.3:
                    self.reject_plan('PATH_INVALID', 'Path start moved during planning')
                    raise ValueError('Vehicle moved during RHEM planning; waiting to replan')
                if np.any(path[:, :3] < self.lower) or np.any(path[:, :3] > self.upper):
                    self.reject_plan('PATH_INVALID', 'Path is outside exploration bounds')
                    raise ValueError('RHEM path is outside the common exploration bounds')
                try:
                    knots, controls, durations, yaw = self.checked_trajectory(path)
                except (ValueError, rospy.ServiceException) as error:
                    self.reject_plan('PATH_INVALID', str(error))
                    raise
                if not self.enabled:
                    continue
                # Validation can take time. Do not publish a path whose origin
                # no longer agrees with the current pose.
                point, speed = self.current()
                if np.linalg.norm(path[0, :3] - point[:3]) > 0.3:
                    self.reject_plan('PATH_INVALID', 'Path start moved during validation')
                    raise ValueError('Vehicle moved during trajectory validation')
                self.traj_id += 1
                msg = MixTraj(bspline_degree=5, traj_id=self.traj_id,
                              start_time=rospy.Time.now(), real_traj_duration=float(sum(durations)),
                              knots=knots.tolist(), pos_pts=[Point(*p) for p in controls],
                              minco_order=5, coef_yaw=yaw.ravel().tolist(), duration_yaw=durations.tolist())
                self.publisher.publish(msg)
                self.planning_failures.clear()
                self.accepted.publish(self.traj_id)
                self.improved.publish(response.belief_improved)
                self.goal = path[-1, :3]
                self.finish = msg.start_time.to_sec() + msg.real_traj_duration
                rospy.loginfo('RHEM accepted path start=%s goal=%s belief_improved=%s',
                              path[0].tolist(), path[-1].tolist(), response.belief_improved)
                rospy.loginfo('RHEM trajectory %d: %d input samples, %d motion segments, %.2f seconds',
                              self.traj_id, len(path), len(durations), msg.real_traj_duration)
            except (ValueError, rospy.ServiceException) as error:
                rospy.logwarn_throttle(5, 'RHEM: %s', error)
                time.sleep(0.5)


if __name__ == '__main__':
    rospy.init_node('rhem_control_adapter')
    Adapter().run()

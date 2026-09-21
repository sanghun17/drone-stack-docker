#!/usr/bin/env python3
"""Check real ROS messages and an optional short flight in the AirSim stack."""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_srvs.srv import SetBool
from std_msgs.msg import Bool, Float64, UInt32
from nav_msgs.msg import Path as NavPath
from trajectory_msgs.msg import MultiDOFJointTrajectory
from traj_utils.msg import MixTraj
from quadrotor_msgs.msg import PositionCommand


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', choices=('pure', 'la', 'rhem'), required=True)
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--timeout', type=float, default=45.0)
    parser.add_argument('--fly', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rospy.init_node('validate_planner_chain', anonymous=True)
    if rospy.get_param('/system/platform', None) != 'sim':
        raise SystemExit('This validator requires /system/platform=sim.')
    types = {
        '/robot/odom': Odometry, '/gt_odom': Odometry,
        '/camera/depth/image_raw': Image, '/camera/left/camera_info': CameraInfo,
        '/airsim_node/hmcl/imu/imu': Imu, '/fast_livo/visual_features': PointCloud2,
        '/planning/trajectory': MixTraj, '/planning/pos_cmd': PositionCommand,
    }
    if args.planner == 'pure':
        types.update({'/jax/optimal_trajectory': MultiDOFJointTrajectory,
                      '/planner/voxblox_node/tsdf_pointcloud': PointCloud2})
    events = {'/planning/trajectory', '/jax/optimal_trajectory'}
    if args.planner == 'rhem':
        types.update({'/rhem/belief/valid_landmarks': UInt32,
                      '/rhem/propagated_uncertainty': Float64,
                      '/rhem/belief_path_selected': Bool,
                      '/rhem/rhem_control_adapter/belief_trajectories': UInt32,
                      '/rhem/planner_path': NavPath})
        events.update(t for t in types if t.startswith('/rhem/'))
    counts = {topic: 0 for topic in types}
    last = {}
    gt_positions = []
    values = {}
    lock = threading.Lock()

    def receive(msg, topic):
        if isinstance(msg, PointCloud2) and not msg.width:
            return
        if isinstance(msg, MixTraj) and len(msg.pos_pts) < 2:
            return
        if isinstance(msg, MultiDOFJointTrajectory) and len(msg.points) < 2:
            return
        if isinstance(msg, (Float64, UInt32)) and (not math.isfinite(msg.data) or msg.data <= 0):
            return
        if isinstance(msg, Bool) and not msg.data:
            return
        with lock:
            now = time.monotonic()
            counts[topic] += 1
            last[topic] = now
            if isinstance(msg, (Float64, UInt32, Bool)):
                values[topic] = msg.data
            if topic == '/gt_odom':
                p = msg.pose.pose.position
                gt_positions.append((p.x, p.y, p.z))

    subscribers = [rospy.Subscriber(t, cls, receive, callback_args=t, queue_size=5)
                   for t, cls in types.items()]
    start = time.monotonic()
    ready = False
    def messages_ready(all_topics=True):
        return all(n >= (1 if t in events else 2) for t, n in counts.items()
                   if all_topics or (t not in events and t != '/planning/pos_cmd'))
    while not rospy.is_shutdown() and time.monotonic() - start < args.timeout:
        with lock:
            ready = messages_ready(all_topics=args.planner != 'rhem')
        if ready:
            break
        time.sleep(0.1)
    error = None
    started = time.monotonic()
    try:
        if ready and args.fly:
            target = rospy.get_param('/system/sim/teleport')
            with lock:
                start_pose = gt_positions[-1] if gt_positions else None
            if start_pose is None or math.dist(start_pose, [target[k] for k in 'xyz']) > 0.25:
                raise RuntimeError('GT start pose does not match the comparison profile; reset before flying')
            rospy.wait_for_service('/control_bridge/toggle_running', timeout=args.timeout)
            if args.planner == 'pure':
                rospy.ServiceProxy('/planner/planner_node/toggle_running', SetBool)(True)
            # The unchanged SO(3) bridge keeps initializer hover until a fresh
            # trajectory ID arrives, then releases it on successful takeover.
            response = rospy.ServiceProxy('/control_bridge/toggle_running', SetBool)(True)
            if not response.success:
                raise RuntimeError(response.message)
            with lock:
                gt_positions.clear()
        if ready and args.planner == 'rhem':
            rospy.wait_for_service('/rhem/rhem_control_adapter/toggle_running', timeout=args.timeout)
            response = rospy.ServiceProxy('/rhem/rhem_control_adapter/toggle_running', SetBool)(True)
            if not response.success:
                raise RuntimeError(response.message)
            deadline = time.monotonic() + args.timeout
            ready = False
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with lock:
                    ready = messages_ready()
                if ready:
                    break
                time.sleep(0.1)
        started = time.monotonic()
        if ready:
            while not rospy.is_shutdown() and time.monotonic() - started < args.seconds:
                time.sleep(0.1)
    except Exception as exc:
        error = str(exc)
    finally:
        if args.fly:
            for service, value in [('/control_bridge/toggle_running', False),
                                   ('/initialize_simulator/toggle_setpoint_publishing', True)]:
                try:
                    rospy.ServiceProxy(service, SetBool)(value)
                except rospy.ServiceException as exc:
                    error = error or str(exc)
        if args.planner == 'rhem':
            try:
                rospy.ServiceProxy('/rhem/rhem_control_adapter/toggle_running', SetBool)(False)
            except rospy.ServiceException as exc:
                error = error or str(exc)
    now = time.monotonic()
    with lock:
        origin = gt_positions[0] if gt_positions else (0, 0, 0)
        displacement = max((math.dist(origin, p) for p in gt_positions), default=0.0)
        result = {
            'validation_scope': 'runtime_message_chain_and_movement_only',
            'tracking_accuracy_assessed': False,
            'planner': args.planner, 'messages_ready': ready,
            'flight_requested': args.fly, 'max_gt_displacement_m': displacement,
            'counts': counts, 'last_message_age_s': {t: now - ts for t, ts in last.items()},
            'elapsed_s': now - start, 'system': rospy.get_param('/system'),
            'planning': rospy.get_param('/planning'),
            'start_gt_position': origin,
            'end_gt_position': gt_positions[-1] if gt_positions else None,
            'diagnostic_values': values, 'error': error,
        }
    result['passed'] = (ready and error is None and all(now - ts < 5 for t, ts in last.items() if t not in events)
                        and (not args.fly or displacement >= 0.25))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    for subscriber in subscribers:
        subscriber.unregister()
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()

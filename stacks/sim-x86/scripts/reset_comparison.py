#!/usr/bin/env python3
"""Reset the AirSim start pose before restarting estimator/planner/controller."""
import time
import math

import numpy as np
import airsim
import rosnode
import rospy
from dynamic_reconfigure.client import Client
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, Trigger


def main():
    rospy.init_node('reset_comparison', anonymous=True)
    if rospy.get_param('/system/platform') != 'sim':
        raise SystemExit('Only the simulation profile can be reset')
    active = [name for name in rosnode.get_node_names()
              if name in ('/laserMapping', '/so3_control_bridge', '/traj_server',
                          '/jax_mppi_controller', '/planner/planner_node', '/exploration_node')
              or name.startswith('/rhem/')]
    if active:
        raise SystemExit('Stop estimator/planner/control before resetting: ' + ', '.join(active))
    target = rospy.get_param('/system/sim/teleport')
    rospy.wait_for_service('/initialize_simulator/teleport_to_position', timeout=10)
    client = Client('/initialize_simulator', timeout=10)
    # The historical initializer leaves its movement flag set on completion.
    # Explicitly cancel before issuing another movement request.
    client.update_configuration({'move_to_xyz': False})
    time.sleep(0.6)
    rospy.ServiceProxy('/initialize_simulator/toggle_setpoint_publishing', SetBool)(False)
    response = rospy.ServiceProxy('/initialize_simulator/teleport_to_position', Trigger)()
    if not response.success:
        raise RuntimeError(response.message)
    # simSetVehiclePose preserves momentum. Clear the previous flight's motion
    # through the simulator API before restarting any estimator; otherwise the
    # same requested pose can immediately fall away after teleportation.
    simulation = airsim.MultirotorClient(ip=rospy.get_param('/system/sim/airsim_ip'),
                                        port=rospy.get_param('/system/sim/airsim_port'))
    state = simulation.simGetGroundTruthKinematics()
    state.position = airsim.Vector3r(target['y'], target['x'], -target['z'])
    state.orientation = airsim.to_quaternion(0, 0, math.pi / 2 - target['yaw'])
    state.linear_velocity = state.angular_velocity = airsim.Vector3r()
    state.linear_acceleration = state.angular_acceleration = airsim.Vector3r()
    simulation.simSetKinematics(state, True)
    simulation.enableApiControl(True)
    simulation.armDisarm(True)
    max_velocity = rospy.get_param('/system/sim/takeoff_velocity')
    deadline = time.monotonic() + 60
    stable_since = None
    while time.monotonic() < deadline:
        msg = rospy.wait_for_message('/gt_odom', Odometry, timeout=5)
        p = msg.pose.pose.position
        error = np.array([target[k] for k in 'xyz']) - [p.x, p.y, p.z]
        v, q = msg.twist.twist.linear, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        yaw_error = abs(math.atan2(math.sin(yaw-target['yaw']), math.cos(yaw-target['yaw'])))
        # Same world-velocity proportional positioning as the historical
        # initializer (gain 3), but always check settling. Its early return at
        # <0.2 m skips stabilization after a teleport.
        velocity = np.clip(3.0 * error, -max_velocity, max_velocity)
        simulation.moveByVelocityAsync(float(velocity[1]), float(velocity[0]), float(-velocity[2]),
                                       0.1, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                       yaw_mode=airsim.YawMode(False, math.degrees(math.pi/2-target['yaw'])))
        if (np.linalg.norm(error) < 0.08
                and np.linalg.norm([v.x, v.y, v.z]) < 0.08 and yaw_error < 0.1):
            if stable_since is None:
                stable_since = time.monotonic()
            if time.monotonic() - stable_since >= 3.0:
                rospy.ServiceProxy('/initialize_simulator/toggle_setpoint_publishing', SetBool)(True)
                print('Start pose reached. Start FAST-LIVO, then the selected planner and common control.')
                return
        else:
            stable_since = None
        time.sleep(0.05)
    rospy.ServiceProxy('/initialize_simulator/toggle_setpoint_publishing', SetBool)(True)
    raise RuntimeError('AirSim did not reach the comparison start pose')


if __name__ == '__main__':
    main()

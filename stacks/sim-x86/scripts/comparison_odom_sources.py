#!/usr/bin/env python3
"""Diagnostic odometry views; preserve pose, stamp and angular velocity."""
import copy
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix


def rotate_linear_twist(message, world_to_body):
    output = copy.deepcopy(message)
    q = message.pose.pose.orientation
    rotation = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
    if world_to_body:
        rotation = rotation.T
    v = message.twist.twist.linear
    result = rotation.dot([v.x, v.y, v.z])
    output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = map(float, result)
    transform = np.eye(6)
    transform[:3, :3] = rotation
    covariance = np.asarray(message.twist.covariance).reshape(6, 6)
    output.twist.covariance = (transform @ covariance @ transform.T).ravel().tolist()
    return output


def main():
    rospy.init_node('comparison_odom_sources')
    body = rospy.Publisher('/comparison/fast_livo/odom', Odometry, queue_size=10)
    world = rospy.Publisher('/comparison/gt/world_odom', Odometry, queue_size=10)
    # The world_odom view is for the model's world-linear-velocity context only;
    # it is not a body-twist control input (angular velocity remains body frame).
    rospy.Subscriber('/LIVO2/imu_propagate', Odometry,
                     lambda m: body.publish(rotate_linear_twist(m, True)), queue_size=10)
    rospy.Subscriber('/gt_odom', Odometry,
                     lambda m: world.publish(rotate_linear_twist(m, False)), queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    main()

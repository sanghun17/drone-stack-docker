import importlib.util
from pathlib import Path
import math
import unittest
import numpy as np
from nav_msgs.msg import Odometry

path = Path(__file__).resolve().parents[1] / 'scripts/comparison_odom_sources.py'
spec = importlib.util.spec_from_file_location('comparison_odom_sources', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class OdometrySourcesTest(unittest.TestCase):
    def test_world_body_conversion_preserves_pose_stamp_and_gyro(self):
        message = Odometry()
        message.header.stamp.secs = 123
        message.pose.pose.position.x = 7
        message.pose.pose.orientation.z = math.sin(math.pi / 4)
        message.pose.pose.orientation.w = math.cos(math.pi / 4)
        message.twist.twist.linear.x = 2
        message.twist.twist.angular.z = 0.3
        message.twist.covariance = np.diag([1, 4, 9, 2, 3, 5]).ravel().tolist()
        body = module.rotate_linear_twist(message, True)
        np.testing.assert_allclose([body.twist.twist.linear.x, body.twist.twist.linear.y], [0, -2], atol=1e-12)
        self.assertEqual(body.header, message.header)
        self.assertEqual(body.pose, message.pose)
        self.assertEqual(body.twist.twist.angular, message.twist.twist.angular)
        self.assertEqual(message.twist.twist.linear.x, 2)
        np.testing.assert_allclose(np.diag(np.array(body.twist.covariance).reshape(6, 6)), [4, 1, 9, 2, 3, 5])
        world = module.rotate_linear_twist(body, False)
        np.testing.assert_allclose([world.twist.twist.linear.x, world.twist.twist.linear.y], [2, 0], atol=1e-12)
        np.testing.assert_allclose(world.twist.covariance, message.twist.covariance, atol=1e-12)

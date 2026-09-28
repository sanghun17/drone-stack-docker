"""Operational checks for exact geometry and shared actuator limits."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.interpolate import BSpline

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rhem_trajectory import parameterize, collapse_collinear_samples


class TrajectoryTest(unittest.TestCase):
    def test_sample_density_does_not_add_stops(self):
        endpoints = np.array([[0., 0., 1., 0.], [1., 0., 1.2, 1.5]])
        dense = np.linspace(endpoints[0], endpoints[1], 41)
        limits = dict(max_vel_xy=2, max_vel_z=1, max_acc_xy=5, max_acc_z=2,
                      max_yaw_rate=2, max_yaw_acc=5)
        sparse_result = parameterize(endpoints, limits)
        dense_result = parameterize(dense, limits)
        for actual, expected in zip(dense_result, sparse_result):
            np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_collapse_preserves_turns_and_yaw_changes(self):
        points = np.array([[0.,0.,1.,0.], [1.,0.,1.,0.], [0.,0.,1.,0.],
                           [0.,1.,1.,0.], [0.,2.,1.,1.]])
        np.testing.assert_array_equal(collapse_collinear_samples(points), points)

    def test_corner_geometry_and_limits(self):
        points = np.array([[0, 0, 0, 3.0], [2, 0, 1, -3.0], [2, 3, 0.2, -1.0]])
        limits = dict(max_vel_xy=2, max_vel_z=1, max_acc_xy=5, max_acc_z=2,
                      max_yaw_rate=2, max_yaw_acc=5)
        knots, controls, durations, yaw = parameterize(points, limits)
        spline = BSpline(knots, controls, 5)
        times = np.r_[0, np.cumsum(durations)]
        for i, duration in enumerate(durations):
            sample = np.linspace(0, duration, 1001)
            value = spline(sample + times[i])
            s = sample / duration
            # Tests all samples against the independent expected straight edge.
            exact = points[i, :3] + (10*s**3-15*s**4+6*s**5)[:, None] * (points[i+1, :3]-points[i, :3])
            np.testing.assert_allclose(value, exact, atol=1e-10)
            vel, acc = spline(sample+times[i], nu=1), spline(sample+times[i], nu=2)
            self.assertLessEqual(np.linalg.norm(vel[:, :2], axis=1).max(), 2+1e-9)
            self.assertLessEqual(np.abs(vel[:, 2]).max(), 1+1e-9)
            self.assertLessEqual(np.linalg.norm(acc[:, :2], axis=1).max(), 5+1e-9)
            self.assertLessEqual(np.abs(acc[:, 2]).max(), 2+1e-9)
            self.assertLessEqual(np.abs(np.polyval(np.polyder(yaw[i]), sample)).max(), 2+1e-9)
            self.assertLessEqual(np.abs(np.polyval(np.polyder(yaw[i], 2), sample)).max(), 5+1e-9)
        np.testing.assert_allclose(spline(times, nu=1), 0, atol=1e-10)
        np.testing.assert_allclose(spline(times, nu=2), 0, atol=1e-10)
        self.assertTrue(np.isfinite(spline.derivative(3)(times)).all())
        # Yaw crosses pi by the short arc, not a full revolution.
        self.assertLess(abs(np.polyval(yaw[0], durations[0])-3.0), 0.3)

    def test_invalid_paths(self):
        for points in ([[0, 0, 0, 0]], [[0, 0, 0, 0], [float('nan'), 1, 1, 0]],
                       [[0, 0, 0, 0], [0, 0, 0, 0]]):
            with self.assertRaises(ValueError):
                parameterize(points, {})


if __name__ == '__main__':
    unittest.main()

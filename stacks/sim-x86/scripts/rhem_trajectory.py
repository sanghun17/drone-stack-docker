"""Time-parameterize RHEM's exact polyline for the existing MixTraj controller."""
import numpy as np
from scipy.interpolate import BSpline


def parameterize(waypoints, limits):
    """Return quintic B-spline positions and descending-power yaw polynomials.

    Each edge follows 10s^3-15s^4+6s^5. Stop at vertices so smoothing never
    cuts an unchecked corner. Bounds use the exact peak first/second derivatives.
    """
    points = np.asarray(waypoints, dtype=float).copy()
    if points.ndim != 2 or points.shape[1] != 4 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Expected at least two finite xyz/yaw waypoints')
    points[:, 3] = np.unwrap(points[:, 3])
    delta = np.diff(points, axis=0)
    # Remove duplicate endpoints shared by consecutive RHEM edges.
    points = points[np.r_[True, np.max(np.abs(delta), axis=1) > 1e-6]]
    if len(points) < 2:
        raise ValueError('Empty RHEM motion')
    required = ('max_vel_xy', 'max_vel_z', 'max_acc_xy', 'max_acc_z',
                'max_yaw_rate', 'max_yaw_acc')
    if any(not np.isfinite(limits[k]) or limits[k] <= 0 for k in required):
        raise ValueError('Motion limits must be positive and finite')
    delta = np.diff(points, axis=0)
    xy, z, yaw = np.linalg.norm(delta[:, :2], axis=1), np.abs(delta[:, 2]), np.abs(delta[:, 3])
    peak_acc = 10.0 / np.sqrt(3.0)
    durations = np.maximum.reduce([
        np.full(len(delta), 0.1),
        1.875 * xy / limits['max_vel_xy'], 1.875 * z / limits['max_vel_z'],
        1.875 * yaw / limits['max_yaw_rate'],
        np.sqrt(peak_acc * xy / limits['max_acc_xy']),
        np.sqrt(peak_acc * z / limits['max_acc_z']),
        np.sqrt(peak_acc * yaw / limits['max_yaw_acc']),
    ])
    times = np.r_[0.0, np.cumsum(durations)]
    knots = np.r_[np.repeat(times[0], 6), np.repeat(times[1:-1], 3), np.repeat(times[-1], 6)]
    basis = BSpline(knots, np.eye(len(knots) - 6), 5)
    matrix = np.concatenate([basis(times, nu=k) for k in range(3)])
    values = np.concatenate([points[:, :3], np.zeros((2 * len(points), 3))])
    controls = np.linalg.solve(matrix, values)
    yaw_coeffs = np.column_stack([
        6 * delta[:, 3] / durations**5, -15 * delta[:, 3] / durations**4,
        10 * delta[:, 3] / durations**3, np.zeros((len(delta), 2)), points[:-1, 3],
    ])
    return knots, controls, durations, yaw_coeffs

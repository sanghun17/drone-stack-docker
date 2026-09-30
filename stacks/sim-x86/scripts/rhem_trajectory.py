"""Continuous waypoint interpolation for the existing MixTraj controller.

Only the first and last velocities are clamped. The caller must validate the
complete curved trajectory with RHEM's map before publishing it.
"""
import numpy as np
from scipy.interpolate import BSpline


def collapse_collinear_samples(points, tolerance=1e-9):
    """Remove only redundant samples on the same xyz/unwrapped-yaw segment.

    RHEM returns densely sampled BSP edges. Those samples are not stops;
    retaining each as a quintic endpoint makes speed depend on sample density.
    Reversals, spatial corners and changes of yaw slope remain explicit.
    """
    kept = [points[0]]
    for i in range(1, len(points)-1):
        edge = points[i+1] - kept[-1]
        length2 = float(edge @ edge)
        if length2 > tolerance*tolerance:
            fraction = float((points[i]-kept[-1]) @ edge) / length2
            error = np.max(np.abs(points[i] - (kept[-1] + fraction*edge)))
            if 0 <= fraction <= 1 and error <= tolerance:
                continue
        kept.append(points[i])
    kept.append(points[-1])
    return np.asarray(kept)


def split_bezier(controls):
    """De Casteljau subdivision; each child stays inside its control hull."""
    levels = [np.asarray(controls)]
    while len(levels[-1]) > 1:
        levels.append((levels[-1][:-1] + levels[-1][1:]) * 0.5)
    return np.array([p[0] for p in levels]), np.array([p[-1] for p in levels[::-1]])


def derivative_hulls(controls, depth=4):
    pieces = [controls]
    for _ in range(depth):
        pieces = [child for piece in pieces for child in split_bezier(piece)]
    return np.concatenate(pieces)


def trajectory_envelopes(knots, controls, durations, max_extent):
    """Enclose every point of the curve, including points between samples.

    Boxes exclude the robot volume, which the native map checker adds using its
    own configuration. Small boxes reduce conservatism without skipping space.
    """
    if not np.isfinite(max_extent) or max_extent <= 0:
        raise ValueError('Envelope extent must be finite and positive')
    spline = BSpline(knots, controls, 5)
    times = np.r_[0., np.cumsum(durations)]
    boxes = []
    for start, end in zip(times[:-1], times[1:]):
        h = end - start
        p, q = spline(start), spline(end)
        v, w = spline(start, nu=1), spline(end, nu=1)
        a, b = spline(start, nu=2), spline(end, nu=2)
        pending = [np.array([p, p+h*v/5, p+2*h*v/5+h*h*a/20,
                            q-2*h*w/5+h*h*b/20, q-h*w/5, q])]
        while pending:
            piece = pending.pop()
            lower, upper = piece.min(axis=0), piece.max(axis=0)
            if np.max(upper-lower) > max_extent:
                pending.extend(split_bezier(piece))
            else:
                boxes.append(((lower+upper)*0.5, upper-lower+1e-9))
            if len(boxes)+len(pending) > 10000:
                raise ValueError('Trajectory needs too many collision envelopes')
    return np.array([p for p, _ in boxes]), np.array([s for _, s in boxes])


def parameterize(waypoints, limits, tangent_scale=1.0):
    """Return quintic B-spline positions and descending-power yaw polynomials.

    Interior tangents come from adjacent edges; there is no blanket zero-speed
    constraint at interior waypoints. Endpoint velocity and acceleration remain
    zero so the shared trajectory server can hold if no next path is available.
    Derivative control hulls bound speed/acceleration for the whole curve.
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
    points = collapse_collinear_samples(points)
    required = ('max_vel_xy', 'max_vel_z', 'max_acc_xy', 'max_acc_z',
                'max_yaw_rate', 'max_yaw_acc')
    if any(not np.isfinite(limits[k]) or limits[k] <= 0 for k in required):
        raise ValueError('Motion limits must be positive and finite')
    if not np.isfinite(tangent_scale) or not 0 < tangent_scale <= 1:
        raise ValueError('Interior tangent scale must be in (0, 1]')
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
    slopes = delta / durations[:, None]
    velocities = np.zeros_like(points)
    # Weighted adjacent secants give a common through-velocity on both sides.
    # A true reversal or a zero-motion pair can naturally have zero velocity.
    velocities[1:-1] = tangent_scale * (
        durations[1:, None] * slopes[:-1] + durations[:-1, None] * slopes[1:]
    ) / (durations[:-1] + durations[1:])[:, None]
    beziers = np.stack([points[:-1], points[:-1]+velocities[:-1]*durations[:, None]/5,
        points[:-1]+2*velocities[:-1]*durations[:, None]/5,
        points[1:]-2*velocities[1:]*durations[:, None]/5,
        points[1:]-velocities[1:]*durations[:, None]/5, points[1:]], axis=1)
    scale = 1.0
    for curve, duration in zip(beziers, durations):
        for order, prefix in ((1, 'max_vel'), (2, 'max_acc')):
            derivative = curve.copy()
            for k in range(order):
                derivative = (5-k)*np.diff(derivative, axis=0)/duration
            hull = derivative_hulls(derivative)
            ratios = [np.linalg.norm(hull[:, :2], axis=1).max()/limits[prefix+'_xy'],
                      np.abs(hull[:, 2]).max()/limits[prefix+'_z'],
                      np.abs(hull[:, 3]).max()/limits['max_yaw_rate' if order == 1 else 'max_yaw_acc']]
            scale = max(scale, max(ratios)**(1.0/order))
    durations *= scale
    velocities /= scale
    times = np.r_[0.0, np.cumsum(durations)]
    knots = np.r_[np.repeat(times[0], 6), np.repeat(times[1:-1], 3), np.repeat(times[-1], 6)]
    basis = BSpline(knots, np.eye(len(knots) - 6), 5)
    matrix = np.concatenate([basis(times, nu=k) for k in range(3)])
    values = np.concatenate([points[:, :3], velocities[:, :3], np.zeros((len(points), 3))])
    controls = np.linalg.solve(matrix, values)
    v, w = velocities[:-1, 3], velocities[1:, 3]
    yaw_coeffs = np.column_stack([
        6*delta[:, 3]/durations**5-3*(v+w)/durations**4,
        -15*delta[:, 3]/durations**4+(8*v+7*w)/durations**3,
        10*delta[:, 3]/durations**3-(6*v+4*w)/durations**2,
        np.zeros(len(delta)), v, points[:-1, 3]])
    return knots, controls, durations, yaw_coeffs

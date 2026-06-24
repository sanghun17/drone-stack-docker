import numpy as np
from scipy.spatial.transform import Rotation as Rot


def correct_est_offset(ref, est, search=0.4, hz=100.0):
    """est header.stamp = ros::Time::now() (LIVMapper.cpp:1436/1530) = processing-END clock, so est
    lags GT by the per-frame latency (~170ms on Jetson). Estimate via yaw-rate cross-correlation and
    return (time-shifted est, offset_seconds). eval_fastlivo.py removes this; the make_rrd_*/_eval_run
    paths historically did not. Roll/pitch metrics are insensitive (GT stays level); yaw and
    fast-motion APE are what this fixes."""
    def _yr(tr):
        return tr.timestamps, np.degrees(np.unwrap(np.radians(
            Rot.from_quat(np.c_[tr.orientations_quat_wxyz[:, 1:4], tr.orientations_quat_wxyz[:, 0]]).as_euler('xyz', degrees=True)[:, 2])))
    te, ye = _yr(est); tg, yg = _yr(ref)
    lo = max(te.min(), tg.min()); hi = min(te.max(), tg.max())
    if hi - lo < 1.0:
        return est, 0.0
    grid = np.arange(lo, hi, 1.0 / hz)
    a = np.gradient(np.interp(grid, te, ye)); b = np.gradient(np.interp(grid, tg, yg))
    a -= a.mean(); b -= b.mean(); n = int(search * hz); best, blag = -1e18, 0
    for lag in range(-n, n + 1):
        c = np.dot(a[-lag:], b[:lag]) if lag < 0 else (np.dot(a[:-lag], b[lag:]) if lag > 0 else np.dot(a, b))
        if c > best:
            best, blag = c, lag
    off = blag / hz
    return type(est)(positions_xyz=est.positions_xyz, orientations_quat_wxyz=est.orientations_quat_wxyz, timestamps=est.timestamps + off), off

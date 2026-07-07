"""SE3 / quaternion / projection helpers (host: numpy + cv2). Shared by calib_* and render."""
import numpy as np
import cv2


def quat_to_R(q):  # xyzw
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)]])


def R_to_quat(R):  # xyzw
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; x = 0.25 * s
        w = (R[2, 1] - R[1, 2]) / s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; y = 0.25 * s
        w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; z = 0.25 * s
        w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def se3(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def se3_inv(T):
    R = T[:3, :3]; Ti = np.eye(4)
    Ti[:3, :3] = R.T; Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def rt_to_se3(rvec, tvec):
    R, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    return se3(R, np.asarray(tvec, float).reshape(3))


def se3_to_rt(T):
    rvec, _ = cv2.Rodrigues(np.ascontiguousarray(T[:3, :3]))
    return rvec.reshape(3), T[:3, 3].reshape(3).copy()


def pose_to_se3(p, q):  # p=(x,y,z), q=(x,y,z,w)
    return se3(quat_to_R(q), p)


def project(pts_world, rvec, tvec, K, dist):
    """pts_world (N,3) -> (uv (N,2), in_front mask). Culls points behind the camera."""
    pts = np.asarray(pts_world, float).reshape(-1, 3)
    R, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    t = np.asarray(tvec, float).reshape(3)
    zc = (pts @ R.T)[:, 2] + t[2]
    front = zc > 1e-6
    uv = np.full((len(pts), 2), np.nan)
    if front.any():
        proj, _ = cv2.projectPoints(pts[front], np.asarray(rvec, float).reshape(3, 1),
                                    t.reshape(3, 1), np.asarray(K, float),
                                    np.asarray(dist, float).reshape(-1, 1))
        uv[front] = proj.reshape(-1, 2)
    return uv, front


def scale_K(K, sx, sy):
    K = np.asarray(K, float).copy()
    K[0, 0] *= sx; K[0, 2] *= sx; K[1, 1] *= sy; K[1, 2] *= sy
    return K


def color_ramp(f):
    """f in [0,1] -> BGR (blue->cyan->green->yellow->red)."""
    f = float(np.clip(f, 0, 1)) * 4
    if f < 1:   c = (255, int(255 * f), 0)
    elif f < 2: c = (int(255 * (2 - f)), 255, 0)
    elif f < 3: c = (0, 255, int(255 * (f - 2)))
    else:       c = (0, int(255 * (4 - f)), 255)
    return c

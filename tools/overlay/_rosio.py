"""rosbag readers (image / pose / camera_info) without cv_bridge, + calib json IO."""
import json
import re
import numpy as np
import cv2
import rosbag

POSE_TYPES = {
    "geometry_msgs/PoseStamped": lambda m: (m.pose.position, m.pose.orientation),
    "geometry_msgs/PoseWithCovarianceStamped": lambda m: (m.pose.pose.position, m.pose.pose.orientation),
    "geometry_msgs/TransformStamped": lambda m: (m.transform.translation, m.transform.rotation),
    "nav_msgs/Odometry": lambda m: (m.pose.pose.position, m.pose.pose.orientation),
}


def _stamp(msg, bagt):
    s = msg.header.stamp.to_sec()
    return s if s > 0 else bagt.to_sec()


def read_poses(bag_path, topic):
    """-> (t (N,), pos (N,3), quat (N,4 xyzw)). Auto-resolves topic msg type."""
    t, pos, quat = [], [], []
    with rosbag.Bag(bag_path) as bag:
        types = {tp: ti.msg_type for tp, ti in bag.get_type_and_topic_info().topics.items()}
        if topic not in types:
            raise KeyError("topic %s not in bag (have: %s)" % (topic, ", ".join(sorted(types))))
        acc = POSE_TYPES.get(types[topic])
        if acc is None:
            raise TypeError("topic %s is %s, not a pose type" % (topic, types[topic]))
        for _, m, bagt in bag.read_messages(topics=[topic]):
            p, q = acc(m)
            t.append(_stamp(m, bagt)); pos.append([p.x, p.y, p.z]); quat.append([q.x, q.y, q.z, q.w])
    return np.array(t), np.array(pos), np.array(quat)


def avg_pose(bag_path, topic):
    """Average a (static) pose stream -> (pos (3,), quat (4,) xyzw)."""
    _, pos, quat = read_poses(bag_path, topic)
    if len(pos) == 0:
        raise ValueError("no poses on %s" % topic)
    q = quat.copy()
    q[np.einsum("ij,j->i", q, q[0]) < 0] *= -1  # hemisphere align before averaging
    qm = q.mean(0); qm /= np.linalg.norm(qm)
    return pos.mean(0), qm


def decode_image(msg):
    """sensor_msgs/Image or CompressedImage -> BGR uint8 ndarray."""
    if hasattr(msg, "format"):  # CompressedImage
        return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(msg.data, np.uint8)
    if enc == "rgb8":
        return np.ascontiguousarray(buf.reshape(h, w, 3)[:, :, ::-1])
    if enc == "bgr8":
        return buf.reshape(h, w, 3).copy()
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(h, w, 4)[:, :, :3]
        return np.ascontiguousarray(img[:, :, ::-1] if enc == "rgba8" else img)
    if enc == "mono8":
        return cv2.cvtColor(buf.reshape(h, w), cv2.COLOR_GRAY2BGR)
    raise ValueError("unsupported image encoding %s" % enc)


def read_first_image(bag_path, topic):
    with rosbag.Bag(bag_path) as bag:
        for _, m, _ in bag.read_messages(topics=[topic]):
            return decode_image(m)
    raise KeyError("no image on %s" % topic)


def iter_images(bag_path, topic, stride=1):
    with rosbag.Bag(bag_path) as bag:
        i = 0
        for _, m, _ in bag.read_messages(topics=[topic]):
            if i % stride == 0:
                yield decode_image(m)
            i += 1


def read_camera_info(bag_path, topic):
    with rosbag.Bag(bag_path) as bag:
        for _, m, _ in bag.read_messages(topics=[topic]):
            K = np.array(m.K, float).reshape(3, 3)
            D = np.array(m.D, float) if len(m.D) else np.zeros(5)
            return K, D, m.width, m.height
    raise KeyError("no camera_info on %s" % topic)


def save_json(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)


def load_json(path):
    with open(path) as f:
        return json.load(f)


def load_handeye(path):
    """T_B_C (camera->body, p_body = R*p_cam + t) -> (t (3,), q (4,) xyzw).
    Accepts d435i.yaml `body_calib` (t_cam2body/q_cam2body_xyzw), handeye_calib.py yaml
    (translation_xyz_m/quaternion_xyzw), or a json with either key set."""
    txt = open(path).read()
    T_KEYS = ["t_cam2body", "translation_xyz_m"]
    Q_KEYS = ["q_cam2body_xyzw", "quaternion_xyzw"]
    try:
        d = json.loads(txt)
        for tk, qk in zip(T_KEYS, Q_KEYS):
            if tk in d and qk in d:
                return np.array(d[tk], float), np.array(d[qk], float)
    except Exception:
        pass

    def grab(keys):
        for k in keys:  # ^\s* anchors to a real yaml key, skipping '# comment' lines
            m = re.search(r"(?m)^\s*" + re.escape(k) + r"\s*:\s*\[([^\]]+)\]", txt)
            if m:
                return np.array([float(x) for x in m.group(1).split(",")])
        raise ValueError("none of %s found in %s" % (keys, path))
    return grab(T_KEYS), grab(Q_KEYS)

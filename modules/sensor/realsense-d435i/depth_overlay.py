#!/usr/bin/env python3
"""Debug overlay: the 10Hz point cloud projected onto the 10Hz color image,
colored by distance (red = near, blue = far) -> /camera/depth_overlay.

Purpose: tell depth NOISE from real structure by eye. rviz's Camera display
can't do this job: it projects EVERY enabled display (map cloud, markers,
grid) into the image, and the camera cloud arrives RGB-textured, so noise
points are camouflaged against the very pixels they came from. This node
draws ONLY the camera's own cloud, distance-colored, on the matching frame:
  * color speckle floating off object boundaries / hanging past edges
    = depth noise (flying pixels),
  * color hugging the image contours = real structure.

Zero cost when unwatched: a 1s timer polls image_out for subscribers and
(un)subscribes the heavy inputs accordingly — no subscriber, no decode, no
math, no bandwidth. View with rqt_image_view or an rviz Image display.

Inputs are the stamp-PAIRED 10Hz topics from paired_drop.py (same frame
subset by construction), so an ApproximateTimeSynchronizer with the same
25ms slop always finds the partner frame.
"""
import numpy as np

import cv2
import message_filters
import rospy
from realsense2_camera.msg import Extrinsics
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def cloud_xyz(msg, stride=1):
    """PointCloud2 -> (N,3) float32, no ros_numpy dependency.

    Zero-copy structured view + one strided field copy per axis. The naive
    reshape(n, point_step) + per-axis slice().copy().view() version costs
    ~75ms/frame on the Orin for a 307k-pt cloud — that alone pushed the
    callback past the 100ms frame budget and the output to ~4Hz. ::stride
    subsamples points (this is a debug VIEW; density beyond 1/2 adds nothing).
    """
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                   "offsets": [off["x"], off["y"], off["z"]],
                   "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)[::stride]
    return np.stack([a["x"], a["y"], a["z"]], axis=1)


class DepthOverlay:
    def __init__(self):
        self.range_min = rospy.get_param("~range_min", 0.3)  # color clamp [m]
        self.range_max = rospy.get_param("~range_max", 6.0)  # = clip_distance
        self.point_px = int(rospy.get_param("~point_px", 1))  # point size [px]
        self.alpha = float(rospy.get_param("~alpha", 0.45))   # 0=invisible 1=opaque
        self.stride = int(rospy.get_param("~point_stride", 2))  # draw every Nth point
        self.K = None            # (fx, fy, cx, cy) of the COLOR camera
        # depth->color extrinsics; default identity so projection works from
        # camera_info alone (D435i baseline ~14mm is negligible). on_extr refines
        # it when the latched /extrinsics msg arrives — but a -s-clipped replay that
        # skips that latched msg still overlays (just without the 14mm correction).
        self.R = np.eye(3, dtype=np.float32)
        self.t = np.zeros(3, dtype=np.float32)
        self.subs, self.sync = [], None

        # TURBO LUT, index 255 = near = red (more intuitive than far = red).
        g = np.arange(256, dtype=np.uint8).reshape(-1, 1)
        self.lut_bgr = cv2.applyColorMap(g, cv2.COLORMAP_TURBO).reshape(256, 3)
        self.lut_rgb = self.lut_bgr[:, ::-1].copy()

        self.pub = rospy.Publisher("image_out", Image, queue_size=1)
        # One-shot metadata: camera_info streams (tiny), extrinsics is latched;
        # both unregister themselves after the first message.
        self.info_sub = rospy.Subscriber("camera_info", CameraInfo, self.on_info)
        self.extr_sub = rospy.Subscriber("extrinsics", Extrinsics, self.on_extr)
        rospy.Timer(rospy.Duration(1.0), self.poll)

    def on_info(self, msg):
        self.K = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])
        self.info_sub.unregister()

    def on_extr(self, msg):
        # librealsense rs2_extrinsics.rotation is COLUMN-major -> transpose.
        # (Near-identity for the D435i depth->color pair, but do it right.)
        self.R = np.asarray(msg.rotation, dtype=np.float32).reshape(3, 3).T
        self.t = np.asarray(msg.translation, dtype=np.float32)
        self.extr_sub.unregister()

    # ---- lazy wiring: inputs exist only while someone watches the output ----
    def poll(self, _):
        want = self.pub.get_num_connections() > 0
        if want and self.sync is None:
            self.subs = [
                message_filters.Subscriber("image_in", Image,
                                           queue_size=2, buff_size=1 << 23),
                message_filters.Subscriber("cloud_in", PointCloud2,
                                           queue_size=2, buff_size=1 << 25),
            ]
            self.sync = message_filters.ApproximateTimeSynchronizer(
                self.subs, queue_size=4, slop=0.025)
            self.sync.registerCallback(self.on_pair)
            # One-shot liveness probes: if "projecting" appears but neither of
            # these does, the inputs themselves are not flowing to this node.
            self.subs[0].registerCallback(
                lambda m: rospy.loginfo_once("depth_overlay: image_in flowing"))
            self.subs[1].registerCallback(
                lambda m: rospy.loginfo_once("depth_overlay: cloud_in flowing"))
            rospy.loginfo("depth_overlay: viewer connected -> projecting")
        elif not want and self.sync is not None:
            for s in self.subs:
                s.sub.unregister()
            self.subs, self.sync = [], None
            rospy.loginfo("depth_overlay: no viewers -> idle (zero cost)")

    def on_pair(self, img, cloud):
        rospy.loginfo_once("depth_overlay: first image+cloud pair received")
        if self.K is None or self.R is None:
            rospy.logwarn_throttle(
                5, "depth_overlay: waiting for camera_info/extrinsics")
            return
        if img.encoding not in ("rgb8", "bgr8"):
            rospy.logerr_throttle(
                10, f"depth_overlay: unsupported encoding {img.encoding}")
            return

        # depth optical frame -> color optical frame -> pixels
        p = cloud_xyz(cloud, self.stride) @ self.R.T + self.t
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        with np.errstate(invalid="ignore", divide="ignore"):
            fx, fy, cx, cy = self.K
            u = (fx * x / z + cx).astype(np.int32)
            v = (fy * y / z + cy).astype(np.int32)
        h, w = img.height, img.width
        m = (np.isfinite(z) & (z > 0.05)
             & (u >= 0) & (u < w) & (v >= 0) & (v < h))
        u, v, z = u[m], v[m], z[m]
        order = np.argsort(-z)        # far first: near points paint LAST (on top)
        u, v, z = u[order], v[order], z[order]

        nrm = np.clip((z - self.range_min) / (self.range_max - self.range_min),
                      0.0, 1.0)
        lut = self.lut_rgb if img.encoding == "rgb8" else self.lut_bgr
        col = lut[(255 * (1.0 - nrm)).astype(np.uint8)]

        # Alpha-blend instead of overwrite, so the RGB content stays readable
        # under the (dense) cloud — the whole point is comparing the two.
        buf = np.frombuffer(img.data, dtype=np.uint8).reshape(h, w, 3).copy()
        a = self.alpha
        for dv in range(self.point_px):
            for du in range(self.point_px):
                vv = np.minimum(v + dv, h - 1)
                uu = np.minimum(u + du, w - 1)
                buf[vv, uu] = (buf[vv, uu] * (1.0 - a) + col * a).astype(np.uint8)

        out = Image(header=img.header, height=h, width=w,
                    encoding=img.encoding, is_bigendian=0, step=3 * w,
                    data=buf.tobytes())
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("depth_overlay")
    DepthOverlay()
    rospy.spin()

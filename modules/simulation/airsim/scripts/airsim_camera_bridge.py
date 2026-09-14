#!/usr/bin/env python3
"""AirSim Scene camera to simulator-agnostic ROS Image/CameraInfo topics."""

import math
import multiprocessing
import time

import airsim
import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import euler_matrix, quaternion_from_euler, quaternion_from_matrix


def required(mapping, key, context):
    if key not in mapping:
        raise ValueError("missing %s.%s" % (context, key))
    return mapping[key]


def acquire_worker(worker_index, worker_count, target_rate_hz, continuous_rpc,
                   ip, port, vehicle_name, camera_name, image_type,
                   expected_width, expected_height, stop_event,
                   shared_image, shared_timestamp, shared_lock):
    """Acquire staggered frames without an unbounded queue or RPC flood."""
    client = airsim.MultirotorClient(ip=ip, port=port)
    request = [
        airsim.ImageRequest(
            camera_name, image_type, pixels_as_float=False, compress=False
        )
    ]
    period = 0.0 if continuous_rpc else worker_count / target_rate_hz
    # Workers collectively launch one request every 1/target_rate_hz seconds.
    if stop_event.wait(worker_index / target_rate_hz):
        return
    while not stop_event.is_set():
        cycle_started = time.monotonic()
        try:
            responses = client.simGetImages(request, vehicle_name=vehicle_name)
            if not responses or not responses[0].image_data_uint8:
                continue
            response = responses[0]
            if response.width != expected_width or response.height != expected_height:
                continue
            source = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            with shared_lock:
                np.frombuffer(shared_image, dtype=np.uint8)[:] = source
                shared_timestamp.value = int(response.time_stamp)
        except Exception:
            if stop_event.wait(0.05):
                return
        remaining = period - (time.monotonic() - cycle_started)
        if remaining > 0.0:
            stop_event.wait(remaining)


class AirSimCameraBridge:
    def __init__(self):
        rospy.init_node("airsim_camera_bridge")
        config_path = rospy.get_param(
            "~config_path",
            "",
        )
        if not config_path:
            raise ValueError("~config_path is required; the stack owns camera configuration")
        with open(config_path, "r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)
        if self.config.get("status") != "calibrated":
            rospy.logwarn(
                "camera geometry is marked %s: %s",
                self.config.get("status", "unknown"),
                config_path,
            )

        airsim_cfg = required(self.config, "airsim", "root")
        camera = required(self.config, "camera", "root")
        ros_cfg = required(self.config, "ros", "root")
        self.vehicle_name = str(required(airsim_cfg, "vehicle_name", "airsim"))
        self.camera_name = str(required(airsim_cfg, "camera_name", "airsim"))
        self.width = int(required(camera, "width", "camera"))
        self.height = int(required(camera, "height", "camera"))
        self.rate_hz = float(required(camera, "publish_rate_hz", "camera"))
        self.rpc_workers = int(camera.get("rpc_workers", 1))
        self.continuous_rpc = bool(camera.get("continuous_rpc", True))
        self.publish_buffer_frames = int(camera.get("publish_buffer_frames", 8))
        self.startup_buffer_frames = int(camera.get("startup_buffer_frames", 3))
        if self.rpc_workers < 1:
            raise ValueError("camera.rpc_workers must be at least one")
        if not 1 <= self.startup_buffer_frames <= self.publish_buffer_frames:
            raise ValueError(
                "camera.startup_buffer_frames must be in [1, publish_buffer_frames]"
            )
        self.image_type = int(camera.get("image_type", 0))
        self.optical_frame = str(required(ros_cfg, "optical_frame", "ros"))

        self.image_pub = rospy.Publisher(
            str(required(ros_cfg, "image_topic", "ros")), Image, queue_size=1
        )
        self.info_pub = rospy.Publisher(
            str(required(ros_cfg, "camera_info_topic", "ros")),
            CameraInfo,
            queue_size=1,
        )
        self.camera_info = self.make_camera_info(camera)
        self.publish_static_transforms(camera, ros_cfg)

        self.client = airsim.MultirotorClient(
            ip=str(airsim_cfg.get("ip", "127.0.0.1")),
            port=int(airsim_cfg.get("port", 41451)),
        )
        self.client.confirmConnection()
        for command in camera.get("render_console_commands", []):
            if not self.client.simRunConsoleCommand(str(command)):
                rospy.logwarn("AirSim rejected render console command: %s", command)
        context = multiprocessing.get_context("fork")
        self.stop_event = context.Event()
        self.worker_processes = []
        image_bytes = self.width * self.height * 3
        self.shared_images = [context.RawArray("B", image_bytes)
                              for _ in range(self.rpc_workers)]
        self.shared_timestamps = [context.RawValue("q", -1)
                                  for _ in range(self.rpc_workers)]
        self.shared_locks = [context.Lock() for _ in range(self.rpc_workers)]
        self.worker_last_timestamps = [-1] * self.rpc_workers
        self.last_timestamp = -1
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "AirSim camera bridge ready: vehicle=%s camera=%s %dx%d@%.1f, %d RPC workers",
            self.vehicle_name,
            self.camera_name,
            self.width,
            self.height,
            self.rate_hz,
            self.rpc_workers,
        )

    def make_camera_info(self, camera):
        intrinsics = camera.get("intrinsics") or {}
        fov = math.radians(float(required(camera, "horizontal_fov_deg", "camera")))
        derived_focal = self.width / (2.0 * math.tan(fov / 2.0))
        fx = float(intrinsics.get("fx") or derived_focal)
        fy = float(intrinsics.get("fy") or derived_focal)
        cx = float(intrinsics.get("cx") if intrinsics.get("cx") is not None else self.width / 2.0)
        cy = float(intrinsics.get("cy") if intrinsics.get("cy") is not None else self.height / 2.0)
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = "plumb_bob"
        msg.D = [float(value) for value in intrinsics.get("distortion", [0.0] * 5)]
        msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def publish_static_transforms(self, camera, ros_cfg):
        mount = required(camera, "mount_frd", "camera")
        roll, pitch, yaw = [
            math.radians(float(mount[name]))
            for name in ("roll_deg", "pitch_deg", "yaw_deg")
        ]
        frd_rotation = euler_matrix(roll, pitch, yaw, axes="sxyz")
        frd_to_flu = np.diag([1.0, -1.0, -1.0, 1.0])
        flu_rotation = np.matmul(np.matmul(frd_to_flu, frd_rotation), frd_to_flu)
        link_quaternion = quaternion_from_matrix(flu_rotation)

        link = TransformStamped()
        link.header.stamp = rospy.Time.now()
        link.header.frame_id = str(required(ros_cfg, "base_frame", "ros"))
        link.child_frame_id = str(required(ros_cfg, "camera_link_frame", "ros"))
        link.transform.translation.x = float(mount["x"])
        link.transform.translation.y = -float(mount["y"])
        link.transform.translation.z = -float(mount["z"])
        (
            link.transform.rotation.x,
            link.transform.rotation.y,
            link.transform.rotation.z,
            link.transform.rotation.w,
        ) = map(float, link_quaternion)

        optical = TransformStamped()
        optical.header.stamp = link.header.stamp
        optical.header.frame_id = link.child_frame_id
        optical.child_frame_id = self.optical_frame
        optical_quaternion = quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        (
            optical.transform.rotation.x,
            optical.transform.rotation.y,
            optical.transform.rotation.z,
            optical.transform.rotation.w,
        ) = map(float, optical_quaternion)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_broadcaster.sendTransform([link, optical])

    def publish_response(self, timestamp, image_data):
        if timestamp <= self.last_timestamp:
            return
        self.last_timestamp = timestamp
        stamp = (
            rospy.Time(secs=timestamp // 1_000_000_000, nsecs=timestamp % 1_000_000_000)
            if timestamp > 0
            else rospy.Time.now()
        )
        image_msg = Image()
        image_msg.height = self.height
        image_msg.width = self.width
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = self.width * 3
        image_msg.data = image_data
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.optical_frame
        self.camera_info.header = image_msg.header
        self.image_pub.publish(image_msg)
        self.info_pub.publish(self.camera_info)

    def spin(self):
        for index in range(self.rpc_workers):
            worker = multiprocessing.Process(
                target=acquire_worker,
                name="airsim-image-rpc-%d" % index,
                args=(
                    index,
                    self.rpc_workers,
                    self.rate_hz,
                    self.continuous_rpc,
                    str(self.config["airsim"].get("ip", "127.0.0.1")),
                    int(self.config["airsim"].get("port", 41451)),
                    self.vehicle_name,
                    self.camera_name,
                    self.image_type,
                    self.width,
                    self.height,
                    self.stop_event,
                    self.shared_images[index],
                    self.shared_timestamps[index],
                    self.shared_locks[index],
                ),
                daemon=True,
            )
            worker.start()
            self.worker_processes.append(worker)
        # RPC completions arrive as small batches. Publishing every completion
        # overloads rospy subscribers with >60 Hz of 1.5 MB images. A short,
        # bounded FIFO converts those batches to an explicit 60 Hz cadence.
        poll_rate = rospy.Rate(max(480.0, self.rate_hz * 8.0))
        period = 1.0 / self.rate_hz
        next_publish = None
        frame_buffer = []
        acquired = 0
        published = 0
        empty_slots = 0
        dropped = 0
        max_batch = 0
        max_completion_gap = 0.0
        last_completion = None
        diagnostic_started = time.monotonic()
        while not rospy.is_shutdown():
            try:
                completed = []
                for index in range(self.rpc_workers):
                    timestamp = self.shared_timestamps[index].value
                    if timestamp <= self.worker_last_timestamps[index]:
                        continue
                    with self.shared_locks[index]:
                        timestamp = self.shared_timestamps[index].value
                        if timestamp <= self.worker_last_timestamps[index]:
                            continue
                        image_data = bytes(self.shared_images[index])
                    self.worker_last_timestamps[index] = timestamp
                    completed.append((timestamp, image_data))
                acquired += len(completed)
                if completed:
                    arrived = time.monotonic()
                    max_batch = max(max_batch, len(completed))
                    if last_completion is not None:
                        max_completion_gap = max(
                            max_completion_gap, arrived - last_completion
                        )
                    last_completion = arrived
                    # A delayed worker can finish after a newer worker. Sort the
                    # very small buffer by AirSim capture timestamp and dedupe it.
                    frames_by_stamp = dict(frame_buffer)
                    for timestamp, image_data in completed:
                        if timestamp > self.last_timestamp:
                            frames_by_stamp[timestamp] = image_data
                    frame_buffer = sorted(frames_by_stamp.items())
                    if len(frame_buffer) > self.publish_buffer_frames:
                        overflow = len(frame_buffer) - self.publish_buffer_frames
                        dropped += overflow
                        frame_buffer = frame_buffer[overflow:]

                now = time.monotonic()
                if next_publish is None and len(frame_buffer) >= self.startup_buffer_frames:
                    next_publish = now
                if next_publish is not None and now >= next_publish:
                    if frame_buffer:
                        self.publish_response(*frame_buffer.pop(0))
                        published += 1
                    else:
                        empty_slots += 1
                        next_publish = None
                    # Do not emit a catch-up burst after process scheduling stalls.
                    if next_publish is not None:
                        next_publish += period
                    if next_publish is not None and next_publish <= now:
                        missed = int((now - next_publish) / period) + 1
                        empty_slots += missed
                        next_publish += missed * period

                if now - diagnostic_started >= 5.0:
                    elapsed = now - diagnostic_started
                    rospy.loginfo(
                        "camera cadence: acquired=%.1f Hz published=%.1f Hz "
                        "buffer=%d/%d dropped=%d empty_slots=%d max_batch=%d "
                        "max_completion_gap=%.1f ms",
                        acquired / elapsed,
                        published / elapsed,
                        len(frame_buffer),
                        self.publish_buffer_frames,
                        dropped,
                        empty_slots,
                        max_batch,
                        max_completion_gap * 1000.0,
                    )
                    acquired = 0
                    published = 0
                    empty_slots = 0
                    dropped = 0
                    max_batch = 0
                    max_completion_gap = 0.0
                    diagnostic_started = now
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "AirSim camera publish failed: %s", exc)
            poll_rate.sleep()

    def shutdown(self):
        self.stop_event.set()
        for worker in self.worker_processes:
            worker.join(timeout=1.0)
            if worker.is_alive():
                worker.terminate()


if __name__ == "__main__":
    AirSimCameraBridge().spin()

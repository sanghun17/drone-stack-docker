#!/usr/bin/env python3
"""Create a landing RGB video and publication-ready 3-D trajectory figure."""

import argparse
import bisect
import json
import math
import os

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
import yaml
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401; registers projection on mpl 3.1


ACTIVE_STATES = {"ALIGNING_AT_H_MAX", "DESCENDING"}
TERMINAL_STATES = {
    "TOUCHDOWN",
    "ABORTED_MARKER_LOSS",
    "ABORTED_SAFETY_INTERVENTION",
    "ABORTED_TIMEOUT",
}


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def message_time(message, bag_time):
    header = getattr(message, "header", None)
    if header is not None and header.stamp.to_sec() > 0.0:
        return header.stamp.to_sec()
    return bag_time.to_sec()


def latest_value(times, values, timestamp, default):
    index = bisect.bisect_right(times, timestamp) - 1
    return values[index] if index >= 0 else default


def image_to_bgr(message):
    encoding = message.encoding.lower()
    channels_by_encoding = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
    }
    if encoding not in channels_by_encoding:
        raise ValueError("unsupported ROS image encoding: " + message.encoding)
    channels = channels_by_encoding[encoding]
    row_bytes = message.width * channels
    raw = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
    image = raw[:, :row_bytes].reshape(message.height, message.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "mono8":
        return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    return image.copy()


def find_landing_interval(state_samples, bag_start, bag_end):
    active_start = None
    terminal_end = None
    for timestamp, state in state_samples:
        if active_start is None and state in ACTIVE_STATES:
            active_start = timestamp
        elif active_start is not None and (
            state in TERMINAL_STATES or state == "DISABLED"
        ):
            terminal_end = timestamp
            break
    return active_start or bag_start, terminal_end or bag_end


def read_trial_data(args, environment):
    topics = [args.image_topic, args.estimate_topic, args.ground_truth_topic,
              args.state_topic, args.visible_topic]
    state_samples = []
    visible_samples = []
    estimates = []
    ground_truth = []
    bag_start = math.inf
    bag_end = -math.inf

    spawn = environment["spawn_ned"]
    pad_local_ned = np.array(
        [-float(spawn["x_m"]), -float(spawn["y_m"]), -float(spawn["z_m"])],
        dtype=float,
    )
    pad_from_ned = np.asarray(
        environment["ground_truth"]["pad_from_local_ned_rotation"], dtype=float
    )
    if pad_from_ned.shape != (3, 3):
        raise ValueError("ground-truth rotation must be a 3x3 matrix")

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=topics):
            timestamp = message_time(message, bag_stamp)
            bag_start = min(bag_start, timestamp)
            bag_end = max(bag_end, timestamp)
            if topic == args.state_topic:
                state_samples.append((timestamp, message.data))
            elif topic == args.visible_topic:
                visible_samples.append((timestamp, bool(message.data)))
            elif topic == args.estimate_topic:
                position = message.pose.pose.position
                estimates.append((timestamp, [position.x, position.y, position.z]))
            elif topic == args.ground_truth_topic:
                position = message.pose.pose.position
                local_ned = np.array([position.x, position.y, position.z], dtype=float)
                pad_position = np.matmul(pad_from_ned, local_ned - pad_local_ned)
                ground_truth.append((timestamp, pad_position.tolist()))
    if not math.isfinite(bag_start) or not math.isfinite(bag_end):
        raise RuntimeError("bag contains none of the requested topics")
    state_samples.sort()
    visible_samples.sort()
    estimates.sort()
    ground_truth.sort()
    interval = find_landing_interval(state_samples, bag_start, bag_end)
    return state_samples, visible_samples, estimates, ground_truth, interval


def select_interval(samples, start, end):
    return [(timestamp, value) for timestamp, value in samples
            if start <= timestamp <= end]


def make_video(args, output_path, state_samples, visible_samples, start, end):
    state_times = [item[0] for item in state_samples]
    state_values = [item[1] for item in state_samples]
    visible_times = [item[0] for item in visible_samples]
    visible_values = [item[1] for item in visible_samples]
    frame_times = []
    frame_shape = None
    with rosbag.Bag(args.bag, "r") as bag:
        for _, message, bag_stamp in bag.read_messages(topics=[args.image_topic]):
            timestamp = message_time(message, bag_stamp)
            if start <= timestamp <= end:
                frame_times.append(timestamp)
                if frame_shape is None:
                    frame_shape = image_to_bgr(message).shape
    if not frame_times:
        raise RuntimeError("no RGB frames in landing interval")
    if args.video_fps is not None:
        fps = args.video_fps
    elif len(frame_times) > 1 and frame_times[-1] > frame_times[0]:
        fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
    else:
        fps = 60.0
    fps = max(1.0, min(120.0, fps))
    height, width = frame_shape[:2]
    writer = cv2.VideoWriter(
        output_path, cv2.VideoWriter_fourcc(*args.codec), fps, (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open video writer for " + output_path)
    font = cv2.FONT_HERSHEY_SIMPLEX
    written = 0
    try:
        with rosbag.Bag(args.bag, "r") as bag:
            for _, message, bag_stamp in bag.read_messages(topics=[args.image_topic]):
                timestamp = message_time(message, bag_stamp)
                if not start <= timestamp <= end:
                    continue
                image = image_to_bgr(message)
                if image.shape[:2] != (height, width):
                    raise RuntimeError("RGB image dimensions change within the bag")
                state = latest_value(state_times, state_values, timestamp, "WAITING")
                visible = latest_value(visible_times, visible_values, timestamp, False)
                elapsed = timestamp - start
                label = "t=%5.2f s   %s   pose:%s" % (
                    elapsed, state, "VALID" if visible else "LOST"
                )
                overlay = image.copy()
                cv2.rectangle(
                    overlay, (10, 10), (min(width - 10, 690), 54), (0, 0, 0), -1
                )
                cv2.addWeighted(overlay, 0.58, image, 0.42, 0.0, image)
                color = (80, 230, 80) if visible else (70, 70, 255)
                cv2.putText(image, label, (22, 41), font, 0.68, color, 2, cv2.LINE_AA)
                writer.write(image)
                written += 1
    finally:
        writer.release()
    return {"frame_count": written, "fps": fps, "width": width, "height": height}


def set_axes_equal(ax, points, pad_size):
    all_points = np.vstack(points)
    minimum = all_points.min(axis=0)
    maximum = all_points.max(axis=0)
    minimum[0:2] = np.minimum(minimum[0:2], -pad_size / 2.0)
    maximum[0:2] = np.maximum(maximum[0:2], pad_size / 2.0)
    center = (minimum + maximum) / 2.0
    radius = max((maximum - minimum).max() / 2.0, 0.1)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(0.0, maximum[2] + 0.05)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((2.0 * radius, 2.0 * radius, maximum[2] + 0.05))


def make_trajectory_figure(args, output_path, estimates, ground_truth, start, end,
                           pad_size):
    selected_estimates = select_interval(estimates, start, end)
    selected_ground_truth = select_interval(ground_truth, start, end)
    if not selected_estimates or not selected_ground_truth:
        raise RuntimeError("estimate or ground-truth trajectory is absent in landing interval")
    estimate_points = np.asarray([item[1] for item in selected_estimates], dtype=float)
    ground_truth_points = np.asarray(
        [item[1] for item in selected_ground_truth], dtype=float
    )

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 9,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
        }
    )
    figure = plt.figure(figsize=(6.6, 5.2))
    axis = figure.add_subplot(111, projection="3d")
    half = pad_size / 2.0
    xx, yy = np.meshgrid([-half, half], [-half, half])
    axis.plot_surface(xx, yy, np.zeros_like(xx), color="#d9d9d9", alpha=0.42,
                      shade=False, edgecolor="#777777", linewidth=0.7)
    axis.plot(
        ground_truth_points[:, 0], ground_truth_points[:, 1], ground_truth_points[:, 2],
        color="#1f4e79", linewidth=2.2, label="Ground truth", zorder=4
    )
    axis.plot(
        estimate_points[:, 0], estimate_points[:, 1], estimate_points[:, 2],
        color="#d55e00", linewidth=1.55, label="Marker estimate", zorder=5
    )
    axis.scatter(*ground_truth_points[0], color="#009e73", marker="o", s=34,
                 label="Start", depthshade=False)
    axis.scatter(*ground_truth_points[-1], color="#cc0000", marker="X", s=45,
                 label="Touchdown", depthshade=False)
    axis.set_xlabel(r"$x_P$ (m)", labelpad=7)
    axis.set_ylabel(r"$y_P$ (m)", labelpad=7)
    axis.set_zlabel(r"$z_P$ (m)", labelpad=7)
    if args.title:
        axis.set_title(args.title, pad=12)
    axis.view_init(elev=args.elevation, azim=args.azimuth)
    axis.grid(True, alpha=0.35)
    axis.legend(loc="upper right", framealpha=0.95)
    set_axes_equal(axis, [estimate_points, ground_truth_points], pad_size)
    figure.tight_layout()
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    return {
        "estimate_samples": len(estimate_points),
        "ground_truth_samples": len(ground_truth_points),
    }


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    module_dir = os.path.dirname(script_dir)
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="trial ROS bag")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument(
        "--environment-config",
        default=os.path.join(module_dir, "config", "baseline_environment.yaml"),
    )
    parser.add_argument("--image-topic", default="/landing/camera/image_raw")
    parser.add_argument("--estimate-topic", default="/landing/vehicle_pose_pad")
    parser.add_argument(
        "--ground-truth-topic", default="/landing/ground_truth/airsim_local_ned"
    )
    parser.add_argument("--state-topic", default="/landing/controller/state")
    parser.add_argument("--visible-topic", default="/landing/target_visible")
    parser.add_argument("--video-fps", type=float, default=None)
    parser.add_argument("--codec", default="mp4v", help="four-character OpenCV codec")
    parser.add_argument("--dpi", type=int, default=300)
    parser.add_argument("--elevation", type=float, default=24.0)
    parser.add_argument("--azimuth", type=float, default=-55.0)
    parser.add_argument("--title", default="")
    args = parser.parse_args()
    args.bag = os.path.abspath(args.bag)
    if not os.path.isfile(args.bag):
        raise FileNotFoundError(args.bag)
    output_dir = args.output_dir or os.path.join(
        os.path.dirname(args.bag), os.path.splitext(os.path.basename(args.bag))[0] + "_figures"
    )
    os.makedirs(output_dir, exist_ok=True)
    environment = load_yaml(args.environment_config)
    states, visibility, estimates, ground_truth, interval = read_trial_data(
        args, environment
    )
    start, end = interval
    base = os.path.splitext(os.path.basename(args.bag))[0]
    video_path = os.path.join(output_dir, base + "_rgb.mp4")
    trajectory_path = os.path.join(output_dir, base + "_trajectory_3d.png")
    video = make_video(args, video_path, states, visibility, start, end)
    trajectory = make_trajectory_figure(
        args,
        trajectory_path,
        estimates,
        ground_truth,
        start,
        end,
        float(environment["environment"]["pad_size_m"]),
    )
    metadata = {
        "bag": args.bag,
        "landing_start_s": start,
        "landing_end_s": end,
        "landing_duration_s": end - start,
        "video": dict(video, path=video_path),
        "trajectory_figure": dict(trajectory, path=trajectory_path, dpi=args.dpi),
    }
    metadata_path = os.path.join(output_dir, base + "_figure_metadata.json")
    with open(metadata_path, "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(video_path)
    print(trajectory_path)
    print(metadata_path)


if __name__ == "__main__":
    main()

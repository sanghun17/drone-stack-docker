#!/usr/bin/env python3
"""Overlay multiple landing trajectories on the AirSim paper overview image."""

import argparse
import glob
import math
import os
import re

import cv2
import numpy as np
import rosbag
import yaml


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


def landing_interval(states, bag_start, bag_end):
    start = None
    end = None
    for timestamp, state in sorted(states):
        if start is None and state in ACTIVE_STATES:
            start = timestamp
        elif start is not None and (state in TERMINAL_STATES or state == "DISABLED"):
            end = timestamp
            break
    return start or bag_start, end or bag_end


def read_trajectory(bag_path, environment, topics):
    states = []
    ground_truth = []
    estimates = []
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
    requested = [topics["state"], topics["ground_truth"], topics["estimate"]]
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, message, bag_stamp in bag.read_messages(topics=requested):
            timestamp = message_time(message, bag_stamp)
            bag_start = min(bag_start, timestamp)
            bag_end = max(bag_end, timestamp)
            if topic == topics["state"]:
                states.append((timestamp, message.data))
            elif topic == topics["ground_truth"]:
                position = message.pose.pose.position
                local_ned = np.array([position.x, position.y, position.z], dtype=float)
                ground_truth.append(
                    (timestamp, pad_from_ned @ (local_ned - pad_local_ned))
                )
            elif topic == topics["estimate"]:
                position = message.pose.pose.position
                estimates.append(
                    (timestamp, np.array([position.x, position.y, position.z], dtype=float))
                )
    if not math.isfinite(bag_start) or not math.isfinite(bag_end):
        raise RuntimeError("no requested trajectory topics in " + bag_path)
    start, end = landing_interval(states, bag_start, bag_end)
    gt = np.asarray([value for timestamp, value in ground_truth if start <= timestamp <= end])
    estimate = np.asarray(
        [value for timestamp, value in estimates if start <= timestamp <= end]
    )
    if gt.size == 0 or estimate.size == 0:
        raise RuntimeError("landing interval has no GT or estimate in " + bag_path)
    return gt, estimate


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    rx = np.array(
        [[1, 0, 0], [0, math.cos(roll), -math.sin(roll)],
         [0, math.sin(roll), math.cos(roll)]], dtype=float
    )
    ry = np.array(
        [[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
         [-math.sin(pitch), 0, math.cos(pitch)]], dtype=float
    )
    rz = np.array(
        [[math.cos(yaw), -math.sin(yaw), 0],
         [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]], dtype=float
    )
    return rz @ ry @ rx


def project_pad_points(points, camera):
    # The overview camera is placed in map/world NED coordinates whose origin
    # is the pad centre. Pad coordinates are x=north, y=west, z=up.
    world_ned = np.column_stack((points[:, 0], -points[:, 1], -points[:, 2]))
    camera_position = np.array(
        [camera["x_m"], camera["y_m"], camera["z_m"]], dtype=float
    )
    camera_from_world = rotation_matrix(
        camera["roll_deg"], camera["pitch_deg"], camera["yaw_deg"]
    ).T
    camera_frd = (camera_from_world @ (world_ned - camera_position).T).T
    width = float(camera["width"])
    height = float(camera["height"])
    focal = width / (2.0 * math.tan(math.radians(camera["horizontal_fov_deg"]) / 2.0))
    valid = camera_frd[:, 0] > 1.0e-6
    pixels = np.empty((len(points), 2), dtype=float)
    pixels[:, 0] = width / 2.0 + focal * camera_frd[:, 1] / camera_frd[:, 0]
    pixels[:, 1] = height / 2.0 + focal * camera_frd[:, 2] / camera_frd[:, 0]
    return np.rint(pixels[valid]).astype(np.int32)


def draw_dashed_polyline(image, points, color, thickness=2, dash=9.0, gap=6.0):
    if len(points) < 2:
        return
    draw = True
    remaining = dash
    for first, second in zip(points[:-1], points[1:]):
        start = first.astype(float)
        delta = second.astype(float) - start
        length = float(np.linalg.norm(delta))
        if length < 1.0e-6:
            continue
        direction = delta / length
        offset = 0.0
        while offset < length:
            step = min(remaining, length - offset)
            if draw:
                a = tuple(np.rint(start + direction * offset).astype(int))
                b = tuple(np.rint(start + direction * (offset + step)).astype(int))
                cv2.line(image, a, b, color, thickness, cv2.LINE_AA)
            offset += step
            remaining -= step
            if remaining <= 1.0e-6:
                draw = not draw
                remaining = dash if draw else gap


def trial_number(path):
    match = re.search(r"trial_(\d+)", os.path.basename(path))
    return int(match.group(1)) if match else 0


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    module_dir = os.path.dirname(script_dir)
    root = os.path.abspath(os.path.join(module_dir, "../../.."))
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", help="bag paths or glob expressions")
    parser.add_argument(
        "--background",
        default=os.path.join(root, "experiments/aruco-landing/paper/simulation_environment.png"),
    )
    parser.add_argument(
        "--output",
        default=os.path.join(
            root, "experiments/aruco-landing/paper/simulation_environment_10_trials.png"
        ),
    )
    parser.add_argument(
        "--environment-config",
        default=os.path.join(module_dir, "config/baseline_environment.yaml"),
    )
    parser.add_argument("--state-topic", default="/landing/controller/state")
    parser.add_argument(
        "--ground-truth-topic", default="/landing/ground_truth/airsim_local_ned"
    )
    parser.add_argument("--estimate-topic", default="/landing/vehicle_pose_pad")
    args = parser.parse_args()

    bag_paths = []
    for expression in args.bags:
        matches = glob.glob(expression)
        bag_paths.extend(matches if matches else [expression])
    bag_paths = sorted({os.path.abspath(path) for path in bag_paths}, key=trial_number)
    if not bag_paths:
        raise RuntimeError("no bags selected")
    missing = [path for path in bag_paths if not os.path.isfile(path)]
    if missing:
        raise FileNotFoundError(missing[0])

    environment = load_yaml(args.environment_config)
    camera = environment["external_cameras"]["paper_overview"]
    background = cv2.imread(args.background, cv2.IMREAD_COLOR)
    if background is None:
        raise FileNotFoundError(args.background)
    expected_shape = (int(camera["height"]), int(camera["width"]))
    if background.shape[:2] != expected_shape:
        raise ValueError(
            "background is %sx%s, expected %sx%s"
            % (background.shape[1], background.shape[0], expected_shape[1], expected_shape[0])
        )

    topics = {
        "state": args.state_topic,
        "ground_truth": args.ground_truth_topic,
        "estimate": args.estimate_topic,
    }
    overlay = background.copy()
    palette = [
        (230, 159, 0), (86, 180, 233), (0, 158, 115), (240, 228, 66),
        (0, 114, 178), (213, 94, 0), (204, 121, 167), (51, 204, 255),
        (120, 200, 80), (255, 125, 80),
    ]
    metadata = []
    for index, bag_path in enumerate(bag_paths):
        gt, estimate = read_trajectory(bag_path, environment, topics)
        gt_pixels = project_pad_points(gt, camera)
        estimate_pixels = project_pad_points(estimate, camera)
        color = palette[index % len(palette)]
        if len(gt_pixels) > 1:
            cv2.polylines(overlay, [gt_pixels], False, color, 4, cv2.LINE_AA)
        draw_dashed_polyline(overlay, estimate_pixels, (255, 255, 255), thickness=2)
        start = tuple(gt_pixels[0])
        finish = tuple(gt_pixels[-1])
        cv2.circle(overlay, start, 8, color, -1, cv2.LINE_AA)
        cv2.circle(overlay, start, 10, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.line(overlay, (finish[0] - 7, finish[1] - 7),
                 (finish[0] + 7, finish[1] + 7), color, 3, cv2.LINE_AA)
        cv2.line(overlay, (finish[0] - 7, finish[1] + 7),
                 (finish[0] + 7, finish[1] - 7), color, 3, cv2.LINE_AA)
        label = str(trial_number(bag_path) or index + 1)
        cv2.putText(overlay, label, (start[0] + 11, start[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, color, 2, cv2.LINE_AA)
        metadata.append((label, color, len(gt), len(estimate)))

    # Use the otherwise empty upper-left void for a compact, non-occluding legend.
    x0, y0, width, row = 42, 42, 430, 31
    height = 78 + row * int(math.ceil(len(metadata) / 2.0))
    legend = overlay.copy()
    cv2.rectangle(legend, (x0, y0), (x0 + width, y0 + height), (0, 0, 0), -1)
    cv2.addWeighted(legend, 0.78, overlay, 0.22, 0.0, overlay)
    cv2.putText(overlay, "Baseline: 10 randomized landing trials", (x0 + 18, y0 + 29),
                cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.line(overlay, (x0 + 20, y0 + 52), (x0 + 65, y0 + 52),
             (180, 220, 255), 4, cv2.LINE_AA)
    cv2.putText(overlay, "Ground truth", (x0 + 76, y0 + 59),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1, cv2.LINE_AA)
    draw_dashed_polyline(
        overlay, np.array([[x0 + 230, y0 + 52], [x0 + 275, y0 + 52]]),
        (255, 255, 255), thickness=2
    )
    cv2.putText(overlay, "Marker estimate", (x0 + 286, y0 + 59),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1, cv2.LINE_AA)
    for index, (label, color, _, _) in enumerate(metadata):
        column = index % 2
        line = index // 2
        x = x0 + 22 + column * 205
        y = y0 + 88 + line * row
        cv2.line(overlay, (x, y - 5), (x + 38, y - 5), color, 4, cv2.LINE_AA)
        cv2.putText(overlay, "Trial " + label, (x + 49, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.51, (245, 245, 245), 1, cv2.LINE_AA)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    if not cv2.imwrite(args.output, overlay):
        raise RuntimeError("failed to write " + args.output)
    print(args.output)
    for label, _, gt_count, estimate_count in metadata:
        print("trial %s: GT=%d estimate=%d" % (label, gt_count, estimate_count))


if __name__ == "__main__":
    main()

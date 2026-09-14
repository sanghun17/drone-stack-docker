#!/usr/bin/env python3
"""Capture a reproducible paper-ready overview from an AirSim external camera."""

import argparse
import math
import os
import time

import airsim
import cv2
import numpy as np
import yaml


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    module_dir = os.path.dirname(script_dir)
    workspace_root = "/work" if os.path.isdir("/work/modules") else os.path.abspath(
        os.path.join(module_dir, "../../..")
    )
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--environment-config",
        default=os.path.join(module_dir, "config", "baseline_environment.yaml"),
    )
    parser.add_argument(
        "--camera-config",
        default=os.path.join(module_dir, "config", "landing_camera.yaml"),
    )
    parser.add_argument(
        "--output",
        default=os.path.join(
            workspace_root,
            "experiments",
            "aruco-landing",
            "paper",
            "simulation_environment.png",
        ),
    )
    parser.add_argument("--airsim-host", default="127.0.0.1")
    parser.add_argument("--airsim-port", type=int, default=41451)
    parser.add_argument("--warmup-frames", type=int, default=3)
    args = parser.parse_args()

    environment = load_yaml(args.environment_config)
    config = environment["external_cameras"]["paper_overview"]
    camera_config = load_yaml(args.camera_config)
    vehicle_name = str(camera_config["airsim"]["vehicle_name"])
    client = airsim.MultirotorClient(ip=args.airsim_host, port=args.airsim_port)
    client.confirmConnection()
    spawn = environment["spawn_ned"]
    # AirSim's pose RPC is expressed relative to the vehicle's configured
    # initial pose. Zero therefore places it at spawn_ned (2 m above this pad),
    # rather than at the UE PlayerStart origin.
    vehicle_pose = airsim.Pose(
        airsim.Vector3r(0.0, 0.0, 0.0),
        airsim.to_quaternion(0.0, 0.0, math.radians(float(spawn.get("yaw_deg", 0.0)))),
    )
    camera_name = "paper_overview"
    pose = airsim.Pose(
        airsim.Vector3r(
            float(config["x_m"]), float(config["y_m"]), float(config["z_m"])
        ),
        airsim.to_quaternion(
            math.radians(float(config["pitch_deg"])),
            math.radians(float(config["roll_deg"])),
            math.radians(float(config["yaw_deg"])),
        ),
    )
    client.simPause(True)
    try:
        client.simSetVehiclePose(vehicle_pose, True, vehicle_name=vehicle_name)
        client.simSetCameraPose(camera_name, pose, external=True)
        # Force render targets to refresh after teleporting while keeping the
        # vehicle essentially at its staging pose.
        client.simContinueForFrames(max(1, args.warmup_frames))
        time.sleep(0.1)
        responses = client.simGetImages(
            [airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, True)],
            external=True,
        )
    finally:
        client.simPause(False)
    if not responses or not responses[0].image_data_uint8:
        raise RuntimeError(
            "external camera returned no image; regenerate settings and restart AirSim"
        )
    encoded = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("failed to decode AirSim overview image")
    parent = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(parent, exist_ok=True)
    if not cv2.imwrite(args.output, image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
        raise RuntimeError("failed to write " + args.output)
    print(args.output)
    print("%dx%d" % (image.shape[1], image.shape[0]))


if __name__ == "__main__":
    main()

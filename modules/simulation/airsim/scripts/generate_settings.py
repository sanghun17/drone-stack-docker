#!/usr/bin/env python3
"""Generate AirSim settings.json from a stack-owned camera/environment profile."""

import argparse
import json
import os
import sys

import yaml


def load_config(path):
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    for section in ("airsim", "camera", "ros"):
        if section not in config:
            raise ValueError("missing required config section: %s" % section)
    return config


def load_environment(path):
    if not path:
        return None
    with open(path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if "spawn_ned" not in config or "environment" not in config:
        raise ValueError("environment config requires environment and spawn_ned sections")
    return config


def generate(config, environment_config=None, mmap_path=None):
    airsim_cfg = config["airsim"]
    camera = config["camera"]
    mount = camera["mount_frd"]
    capture = {
        "PublishToRos": 0,
        "ImageType": int(camera.get("image_type", 0)),
        "Width": int(camera["width"]),
        "Height": int(camera["height"]),
        "FOV_Degrees": float(camera["horizontal_fov_deg"]),
        "TargetGamma": float(camera.get("target_gamma", 1.5)),
        "MotionBlurAmount": float(camera.get("motion_blur_amount", 0.0)),
    }
    # AirSim stores the viewport camera as ImageType -1 separately from the
    # Scene capture component. Set both so simGetCameraInfo() reports the same
    # FOV used to render ImageType.Scene.
    camera_component = {
        "ImageType": -1,
        "FOV_Degrees": float(camera["horizontal_fov_deg"]),
        "MotionBlurAmount": float(camera.get("motion_blur_amount", 0.0)),
    }
    camera_entry = {
        "CaptureSettings": [camera_component, capture],
        "X": float(mount["x"]),
        "Y": float(mount["y"]),
        "Z": float(mount["z"]),
        "Roll": float(mount["roll_deg"]),
        "Pitch": float(mount["pitch_deg"]),
        "Yaw": float(mount["yaw_deg"]),
    }
    vehicle = {
        "VehicleType": "SimpleFlight",
        "DefaultVehicleState": "Armed",
        "AllowAPIAlways": True,
        "Cameras": {str(airsim_cfg["camera_name"]): camera_entry},
    }
    if environment_config is not None:
        spawn = environment_config["spawn_ned"]
        vehicle.update(
            {
                "X": float(spawn["x_m"]),
                "Y": float(spawn["y_m"]),
                "Z": float(spawn["z_m"]),
                "Yaw": float(spawn.get("yaw_deg", 0.0)),
            }
        )
    settings = {
        "SettingsVersion": 1.2,
        "SimMode": "Multirotor",
        "ViewMode": str(airsim_cfg.get("view_mode", "NoDisplay")),
        "ClockSpeed": 1.0,
        "ClockType": "ScalableClock",
        "Vehicles": {
            str(airsim_cfg["vehicle_name"]): vehicle
        },
        # Keep the sensor render target allocated and refreshed every game
        # frame. This also avoids Vulkan target reallocation between the tiny
        # operator viewport and the 720x720 sensor capture.
        "SubWindows": [
            {
                "WindowID": 0,
                "CameraName": str(airsim_cfg["camera_name"]),
                "ImageType": int(camera.get("image_type", 0)),
                "VehicleName": str(airsim_cfg["vehicle_name"]),
                "Visible": True,
            }
        ],
    }
    if str(camera.get("transport", "rpc")) == "mmap":
        mmap_path = mmap_path or camera.get("mmap_path")
        if not mmap_path:
            raise ValueError("mmap camera transport requires --mmap-path or camera.mmap_path")
        mmap_path = os.path.abspath(os.path.expanduser(str(mmap_path)))
        settings["Recording"] = {
            "Enabled": True,
            "RecordOnMove": False,
            "RecordInterval": 1.0 / float(
                camera.get("capture_poll_rate_hz", camera["publish_rate_hz"])
            ),
            "Folder": "mmap:" + mmap_path,
            "Cameras": [
                {
                    "CameraName": str(airsim_cfg["camera_name"]),
                    "ImageType": int(camera.get("image_type", 0)),
                    "PixelsAsFloat": False,
                    "Compress": False,
                    "VehicleName": str(airsim_cfg["vehicle_name"]),
                }
            ],
        }
    if environment_config is not None and environment_config.get("external_cameras"):
        settings["ExternalCameras"] = {}
        for name, external in environment_config["external_cameras"].items():
            external_capture = {
                "ImageType": int(external.get("image_type", 0)),
                "Width": int(external["width"]),
                "Height": int(external["height"]),
                "FOV_Degrees": float(external["horizontal_fov_deg"]),
                "MotionBlurAmount": float(external.get("motion_blur_amount", 0.0)),
                "TargetGamma": float(external.get("target_gamma", 1.5)),
            }
            settings["ExternalCameras"][str(name)] = {
                "X": float(external["x_m"]),
                "Y": float(external["y_m"]),
                "Z": float(external["z_m"]),
                "Pitch": float(external["pitch_deg"]),
                "Roll": float(external["roll_deg"]),
                "Yaw": float(external["yaw_deg"]),
                "CaptureSettings": [external_capture],
            }
    return settings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument(
        "--environment-config",
        default=None,
        help="optional baseline environment/spawn YAML",
    )
    parser.add_argument("--output", default="-", help="output path, or - for stdout")
    parser.add_argument(
        "--mmap-path",
        default=None,
        help="mmap recording path (required when transport=mmap unless set in YAML)",
    )
    args = parser.parse_args()
    document = json.dumps(
        generate(
            load_config(args.config),
            load_environment(args.environment_config),
            mmap_path=args.mmap_path,
        ), indent=2
    ) + "\n"
    if args.output == "-":
        sys.stdout.write(document)
        return
    parent = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(parent, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write(document)
    print(args.output)


if __name__ == "__main__":
    main()

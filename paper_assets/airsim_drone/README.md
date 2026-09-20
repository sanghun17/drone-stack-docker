# AirSim drone PNG

`airsim_drone_top_transparent_4k.png`: 4096 × 4096, RGBA, transparent background.
Captured from above using a perspective camera (pitch -90°, FOV 40°).
The red propellers/front of the drone point toward the top of the image.

The original `/AirSim/Blueprints/BP_FlyingPawn` was spawned in an isolated UE4
editor scene. Body and all four propellers use their original meshes/materials.
No flight simulation or AI image generation was used. UE4's native high-resolution
screenshot and custom-depth alpha mask created the PNG directly.
Pixels outside the drone have alpha 0 (their invisible RGB is UE4 mask green).

Project: `.build/airsim-drone-capture/AirSimDroneCapture.uproject`
Map: `/Game/DroneFigure/DroneOnly`
Script: `tools/airsim_drone_capture/capture.py`
Capture details and original blueprint checksum: `capture_metadata.json`.

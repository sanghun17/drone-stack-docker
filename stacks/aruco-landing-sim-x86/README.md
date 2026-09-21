# ArUco landing AirSim stack

`stack.yml` composes the ROS container. This directory owns its camera/recorder
profiles, optimized camera bridge, AirSim patches and Unreal landing environment.
Shared AirSim client functionality lives in `modules/simulation/airsim`.

Run these commands from the repository root on the x86 simulator host:

```bash
./setup.sh clone aruco-landing-sim-x86
./setup.sh up aruco-landing-sim-x86
./setup.sh build-ws aruco-landing-sim-x86
./stacks/aruco-landing-sim-x86/scripts/build_baseline_map.sh
./stacks/aruco-landing-sim-x86/scripts/build_camera_bridge.sh
./stacks/aruco-landing-sim-x86/scripts/run_baseline_sim.sh
```

`config/sim.env` supplies the installed Unreal/AirSim locations. The map builder
uses this stack's `config/baseline_environment.yaml` and the pad generator in the
ArUco component workspace. Its generated settings and textures live under
`.build/aruco-landing-sim-x86/baseline`. `config/landing_camera.yaml` defines camera
resolution, intrinsics, body transform, rate and ROS interfaces.

The simulator runs on the host. In separate terminals, launch the camera bridge,
perception and landing planner with the selected stack's module entrypoints. The
simulation control adapter is `scripts/run_control.sh`; the repeatable validation
runner is `scripts/run_controller_validation.sh`. These commands control the
simulator and should only be used with the intended simulation ROS master.

```bash
./setup.sh run aruco-landing-sim-x86 simulation/airsim/run_camera.sh
./stacks/aruco-landing-sim-x86/scripts/run_control.sh
```

For a standalone simulator, use `scripts/package_baseline_sim.sh` followed by
`scripts/run_packaged_sim.sh`. Packages remain under
`.build/aruco-landing-sim-x86/package/LinuxNoEditor`. Changing the map or vehicle
requires repackaging; camera settings remain an external JSON file.

New recordings and validation results go under `flight_logs/aruco-landing/`.
Paper campaigns, metrics/figure generators and old results are preserved under
`~/drone-stack-archive/20260921-cleanup/stack-assets/aruco-landing-sim-x86/`;
previous recordings are in the archive's `experiments/aruco-landing/`. They are
not runtime dependencies. See the archive README for the original documentation.

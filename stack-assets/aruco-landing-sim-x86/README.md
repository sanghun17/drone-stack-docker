# ArUco landing AirSim stack assets

This directory is owned by the `aruco-landing-sim-x86` stack. Unreal/AirSim runs
on the ml host; the ROS/RPC bridge runs in `drone-stack-aruco-landing-sim-x86`.
Reusable AirSim client/camera functionality stays in `modules/simulation/airsim`;
this directory owns the landing map, camera/spawn profile, project-local bridge,
trial campaign, and evaluation tooling. It does not import or launch risk-aware.

`config/landing_camera.yaml` is the single source of truth for camera resolution,
rate, FOV/intrinsics, mount extrinsic, names, frames, and ROS topics. Its initial
intrinsics are calibrated; only the on-body mount transform remains explicitly
marked provisional until it is measured.

Generate the host-side AirSim settings file:

```bash
python3 modules/simulation/airsim/scripts/generate_settings.py \
  --config stack-assets/aruco-landing-sim-x86/config/landing_camera.yaml \
  --environment-config stack-assets/aruco-landing-sim-x86/config/baseline_environment.yaml \
  --mmap-path "$PWD/.build/aruco-landing-sim-x86/landing_camera.mmap" \
  --output .build/aruco-landing-sim-x86/airsim-settings.json
```

## Minimal baseline map

The landing stack owns a separate, reproducible UE4 project; no risk-aware map
or config is edited. Its baseline is defined in
`config/baseline_environment.yaml`:

- paper pad `L = 0.7 m`
- centered gray `7 x 7 m` floor (10 L)
- no walls, props, sky, or environment meshes
- AirSim NED origin at the pad center and vehicle start at `z = -2 m`

The simulator renders the experimental 720x720 square image directly at 53.9
degrees. The real 1280x720 camera uses a centered 720x720 processing crop.
Temporal AA, motion blur, and depth of field are disabled by the bridge so the
binary cells remain sharp near touchdown. AirSim runs in `NoDisplay` mode on a
dedicated GPU (`graphicsadapter=2` on this host) with offscreen rendering. The
landing-local UE plugin asynchronously reads the continuously rendered target
into a memory-mapped file; the C++ ROS bridge publishes only unique frames.
Internal capture polling runs at 120 Hz to avoid missing render callbacks, while
ROS delivery and the estimator/controller contract remain 60 Hz. A 12-frame
fresh-frame ring absorbs short UE hitches (about 106 ms median timestamp age in
the validation run); frames are never duplicated. The external paper camera
remains available through its image API.
Generate the texture, map, and matching `settings.json` with:

```bash
./stack-assets/aruco-landing-sim-x86/tools/build_baseline_map.sh
```

Then run the editor project in game mode and start the ROS camera adapter in a
second terminal:

```bash
./stack-assets/aruco-landing-sim-x86/tools/run_baseline_sim.sh
./modules/simulation/airsim/run_camera.sh
```

The UE map uses centimeters, while the generator and settings manifest use
meters. The pad is an unlit nearest-neighbour material so its binary edges and
physical scale are preserved. `build_baseline_map.sh` generates the `.umap`
deterministically and only symlinks the installed vanilla AirSim plugin.

Launch the existing Unreal environment with that file using its `-settings`
argument, then start the bridge:

```bash
./modules/simulation/airsim/run_camera.sh
```

The bridge publishes:

- `/landing/camera/image_raw`
- `/landing/camera/camera_info`
- `base_link -> landing_camera_link -> landing_camera_optical_frame` static TF

The control adapter consumes `/landing/cmd_vel_pad`, converts pad-frame commands
to AirSim body FRD using the marker-derived vehicle attitude, and calls AirSim
directly without MAVROS. Ground truth is published only for post-hoc analysis;
the estimator and controller do not subscribe to it.

The current simulation defines touchdown at an estimated camera-to-pad height
of 0.20 m. The visual floor and pad have collision disabled so the stock AirSim
vehicle collision body cannot terminate a trial early; the controller and
adapter issue a zero-velocity stop at the threshold. These choices are owned by
`common_experiment.yaml` and `baseline_environment.yaml`, respectively.

Generate all five proposed two-marker configurations (PNG, SVG, exact-scale
PDF, metric estimator manifest, and Unreal OBJ/MTL) plus a labelled preview:

```bash
./stack-assets/aruco-landing-sim-x86/tools/generate_proposed_pad_assets.sh
```

To put one candidate into the map, select the matching layout while rebuilding:

```bash
AIRSIM_LANDING_PAD_LAYOUT="$PWD/ws/aruco-landing/src/aruco_landing/config/proposed_pad_1_layout.yaml" \
AIRSIM_LANDING_PAD_PREFIX=proposed_pad_1 \
./stack-assets/aruco-landing-sim-x86/tools/build_baseline_map.sh
```

### Automated matched trials and RGB bags

With the stack and packaged simulator running, execute one pad condition:

```bash
./stack-assets/aruco-landing-sim-x86/tools/run_trials.sh --pad-name baseline
```

The first invocation creates a deterministic matched staging manifest at
`experiments/aruco-landing/staging.json`. Reuse it after loading the proposed
pad:

```bash
./stack-assets/aruco-landing-sim-x86/tools/run_trials.sh \
  --pad-name proposed \
  --manifest /work/experiments/aruco-landing/staging.json
```

Every trial writes a compressed ROS bag containing the 720x720 RGB image,
CameraInfo, detections, estimates, commands, controller state, TF, and AirSim
ground truth. `trials.csv` and `summary.json` contain pose availability,
localization RMSE, horizontal touchdown error, and success rate. AirSim contact
can optionally be used as a terminal event, but is disabled for the current
estimated-camera-height touchdown protocol.

The camera bridge and marker estimator are C++. A preflight check requires at
least 59.5 Hz by default; `--allow-rate-shortfall` is only for non-paper smoke
tests. The 2026-09-07 ten-trial baseline run measured 59.94 Hz at preflight and
59.46 Hz mean delivery while simultaneously controlling and recording RGB. Most
trials were 59.99--60.00 Hz, with host-wide render hitches lowering the minimum
trial to 58.10 Hz. Trial-level JSON/CSV records whether the cadence threshold
was met, so such runs are not silently accepted as 60 Hz data.

Generate the landing RGB video and the publication 3-D trajectory plot from any
trial bag:

```bash
./stack-assets/aruco-landing-sim-x86/tools/make_trial_figure.sh \
  experiments/aruco-landing/<pad>/<trial>/bags/<trial>.bag
```

Generate standalone Matplotlib 3-D figures for all ten baseline GT and
marker-estimated trajectories. The script writes both a combined plot and a
2x5 trial panel figure:

```bash
./stack-assets/aruco-landing-sim-x86/tools/make_multi_trial_trajectory_figure.sh
```

The default outputs are
`experiments/aruco-landing/paper/baseline_10_trials_trajectory_3d.png` and
`experiments/aruco-landing/paper/baseline_10_trials_trajectory_panels.png`.

The command writes an MP4, a 300 dpi PNG, and a JSON manifest beside the bag.
It automatically restricts both visualizations to the landing-mode interval.

The AirSim settings also define a deterministic 1920x1080 external camera for
the simulation-setup figure. After restarting the simulator with regenerated
settings, capture its oblique pad/floor/vehicle view with:

```bash
./stack-assets/aruco-landing-sim-x86/tools/capture_environment_figure.sh
```

The default output is
`experiments/aruco-landing/paper/simulation_environment.png`.

### Standalone Linux deployment

Package the generated map and AirSim plugin so the simulator can run without
UE4Editor:

```bash
./stack-assets/aruco-landing-sim-x86/tools/package_baseline_sim.sh
./stack-assets/aruco-landing-sim-x86/tools/run_packaged_sim.sh
```

The cooked build is written under
`.build/aruco-landing-sim-x86/package/LinuxNoEditor`. Camera and vehicle settings
remain external in `.build/aruco-landing-sim-x86/baseline/settings.json`, so
changing them does not require repackaging the Unreal assets.

Changing the pad texture/map or vehicle pawn does require repackaging. Supply a
future layout through `AIRSIM_LANDING_PAD_LAYOUT` and choose its artifact name
with `AIRSIM_LANDING_PAD_PREFIX`.

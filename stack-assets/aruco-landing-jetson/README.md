# ArUco landing hardware adapter

This stack reuses the same estimator, controller, and session recorder as the
AirSim stack. Only the See3CAM publisher, nominal camera-to-body TF, and MAVROS
arming/webcam trigger behavior are hardware adapters.

The standalone intrinsic/extrinsic calibration interface, required artifacts,
Motive rigid-body frame checks, and final acceptance checklist are documented
in [`docs/camera_calibration_handoff.md`](docs/camera_calibration_handoff.md).

To inspect the recovered September 15 intrinsic with the existing RViz/noVNC
utility, run `stack-assets/aruco-landing-jetson/tools/start_intrinsic_preview.sh`
on ML. See [intrinsic preview](docs/intrinsic_preview.md) for the browser URL,
original/undistorted ROS topics, calibration provenance, and camera diagnostics.

With the FC and OptiTrack powered off, run the non-actuating bench profile on
the Jetson after pulling the ml-host commit:

```bash
./setup.sh build aruco-landing-jetson
./setup.sh up aruco-landing-jetson
stack-assets/aruco-landing-jetson/tools/run_bench_profile.sh
```

The landing controller is enabled to measure command generation, but this
stack intentionally contains no MAVROS actuator adapter. The physical pad's
marker geometry is not surveyed, so detection, fusion availability, timing,
and load metrics are usable; fused pose accuracy and derived velocity are
explicitly provisional. Set `ARUCO_HARDWARE_LAYOUT` to a surveyed layout before
using pose values quantitatively.

The 2026-09-14 bench uses stack-local `SEE3CAM_GAIN=1`; gain 10 saturated
31.6% of the recorded frame under the current lighting. The camera exposes no
UVC focus control. Physically focus the lens or increase the camera-to-pad
distance before qualifying marker detection and fusion. The timing/recording
result and this remaining hardware blocker are captured in
`results/camera_bench_20260914.json`.

## One-time camera-to-body extrinsic capture

FC and MAVROS are not needed. Keep the marker pad rigidly fixed, and make sure
the Motive `pure` rigid-body origin and axes represent `base_link`. Start only
OptiTrack, See3CAM, and the calibration recorder in separate
terminals:

```bash
./setup.sh run aruco-landing-jetson odometry/optitrack
./setup.sh run aruco-landing-jetson sensor/see3cam-24cug
stack-assets/aruco-landing-jetson/tools/run_camera_body_extrinsic_recorder.sh
```

Do not run the provisional static `base_link -> see3cam_optical_frame` TF from
`run_bench_profile.sh`; that transform is the unknown being calibrated. Start
and stop the bag from another terminal:

```bash
DSD_CONTAINER=drone-stack-aruco-landing-jetson \
  modules/utility/session-recorder/sessionctl.sh start extrinsic-01
# Move slowly in x/y/z and excite roll, pitch, and yaw while markers stay visible.
DSD_CONTAINER=drone-stack-aruco-landing-jetson \
  modules/utility/session-recorder/sessionctl.sh stop
```

Capture roughly 30--60 seconds with several distinct attitudes. Translation at
one fixed attitude is not enough to identify the full six-degree-of-freedom
extrinsic. Bags are written under
`experiments/aruco-landing/camera-body-extrinsic/`. The dedicated profile keeps
lossless original images, CameraInfo, camera stream status,
and `/vrpn_client_node/pure/pose`; it does not call the
external webcam recorder.

The joint body-camera / global-pad estimation equations, target requirements,
capture commands and subsequent PX4 SITL transition checks are in
[`docs/extrinsic_session.md`](docs/extrinsic_session.md).

For the final metric result, use one surveyed board or one continuously visible
marker whose printed side length is known. The current bench multi-marker
layout is image-inferred, so its fused pose is useful as a cross-check but is
not an accurate calibration target.

During a later flight test, MAVROS arming starts both the rosbag recorder and
the existing `/recorder/start` webcam service. Disarming stops both. The same
recorder uses a service trigger in simulation and does not call a webcam.
Start exactly one MAVROS instance (this stack includes `control/mavros`); do
not run a second MAVROS node from the risk-aware container on the shared ROS
master at the same time.

## OptiTrack-first localization transition

`physical_pad_estimator` owns the session-only OptiTrack-pad alignment and
publishes `/landing/vision_pose_marker`. `odometry/landing-vision-pose` only
selects between that already aligned pose and `/vrpn_client_node/pure/pose`.
It does not estimate or publish a second global-pad transform.

The hardware profile enables trial-gated switching and 0.5 s marker-loss fallback.
`planner_aruco-landing.sh` launches the router; do not launch a second adapter.
The router publishes `/landing/vision_pose_selected`; the existing common
flight-safety MUX remains the sole MAVROS vision publisher. Before pilot OFFBOARD,
the router forwards OptiTrack only. See [the landing workflow](docs/landing_trial.md)
for qualification, rejection, failed-hold behavior and restart requirements.

The September 19 **static pose-bag** replay passed source transition and external
vision fusion checks in isolated PX4 v1.11.3 SITL. The subsequent three-approach hand-carried replay passed with an explicit
IMU axis adjustment in the simulator; see the guide for this qualification.
Closed-loop landing validation remains a separate stage. Capture scripts, test
commands and results are in [the transition guide](docs/pose_transition_sitl.md).

## Measured physical pad and online body pose

Run `bash scripts/perception_aruco-landing.sh` alongside the camera and OptiTrack.
The [physical estimator guide](docs/physical_pad_estimator.md) lists pad-body,
marker-derived odom-body, session-only alignment and annotated-image topics.
Pad geometry is in `config/physical_pad.yaml`; global pad placement is learned
online for each execution and is never loaded from a calibration YAML.

## September 20 hardware handoff

The operator reported landing completed. The latest camera checks passed after
USB cable replacement on battery power, including an approximately 90-second
post-reboot observation before the operator requested shutdown. These checks do
not establish the electrical root cause or guarantee sustained 60 Hz in every
condition. See the [camera/cable validation report](results/camera-cable-validation-20260920/README.md)
for measured rates, failed software experiments, repository revisions and final
stopped-node state. The [landing trial guide](docs/landing_trial.md) describes the
current OptiTrack-based OFFBOARD workflow and landing policies.

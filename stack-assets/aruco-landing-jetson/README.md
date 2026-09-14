# ArUco landing hardware adapter

This stack reuses the same estimator, controller, and session recorder as the
AirSim stack. Only the See3CAM publisher, nominal camera-to-body TF, and MAVROS
arming/webcam trigger behavior are hardware adapters.

The standalone intrinsic/extrinsic calibration interface, required artifacts,
Motive rigid-body frame checks, and final acceptance checklist are documented
in [`docs/camera_calibration_handoff.md`](docs/camera_calibration_handoff.md).

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
OptiTrack, See3CAM, the estimator, and the calibration recorder in separate
terminals:

```bash
./setup.sh run aruco-landing-jetson odometry/optitrack
./setup.sh run aruco-landing-jetson sensor/see3cam-24cug
./setup.sh run aruco-landing-jetson perception/aruco-landing
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
compressed images, CameraInfo, per-marker and fused camera-frame poses,
estimator quality, and `/vrpn_client_node/pure/pose`; it does not call the
external webcam recorder.

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

MAVROS and PX4 are not modified. `odometry/landing-vision-pose` is an adapter
whose sole output is `/mavros/vision_pose/pose`:

```text
OptiTrack T_global_body -------------------------> adapter output -> MAVROS
             + marker T_pad_body -> T_global_pad -> marker shadow --^
```

The checked-in first-flight policy locks the output to OptiTrack
(`LANDING_ALLOW_MARKER_SWITCH=false`). Marker observations are still registered
against OptiTrack and published on `/landing/vision_pose_marker`; they cannot
reach EKF2. Run these modules in separate terminals after the Jetson has pulled
and built the ml-host commit:

```bash
./setup.sh run aruco-landing-jetson control/mavros
./setup.sh run aruco-landing-jetson odometry/optitrack
./setup.sh run aruco-landing-jetson sensor/see3cam-24cug
./setup.sh run aruco-landing-jetson perception/aruco-landing
./setup.sh run aruco-landing-jetson odometry/landing-vision-pose
./setup.sh run aruco-landing-jetson utility/session-recorder
```

Do not run `control/flight-safety`'s legacy estimation mux in parallel: there
must be exactly one publisher on `/mavros/vision_pose/pose`. The flight-safety
package is present only because the instrumented VRPN client builds against its
generic diagnostic header.

Before arming, verify the locked source and sole publisher:

```bash
rostopic echo -n1 /landing/vision_pose_source
rostopic info /mavros/vision_pose/pose
rostopic echo -n1 /landing/pose_transition/status
```

For a non-armed hand-carried recording, start and stop the shared recorder
explicitly (arming continues to trigger the same recorder during flight):

```bash
rosservice call /session_recorder/set_recording "data: true"
# carry the aircraft through the approach and marker-acquisition trajectory
rosservice call /session_recorder/set_recording "data: false"
```

After a successful OptiTrack-only run, extract `T_optitrack_pad`:

```bash
stack-assets/aruco-landing-jetson/tools/estimate_pad_alignment.sh \
  experiments/aruco-landing/hardware-sessions/<flight>.bag
```

This writes the default alignment under the ignored experiment directory, not
the source tree. Validate that transform and source-transition continuity in
PX4 SITL before setting `LANDING_ALLOW_MARKER_SWITCH=true`. The adapter also
requires fresh sources, qualified registration, and bounded position/attitude
jumps; no automatic fallback is hidden after a marker-source dropout.

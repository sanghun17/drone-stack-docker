# Joint extrinsic capture and source-transition validation

## Current capture interface

The camera and OptiTrack alone provide the required measurements. The recorder
uses `config/camera_body_extrinsic_recorder.yaml` and waits for a service trigger.
It records original RGB `image_raw` with lossless bag-level LZ4 compression,
CameraInfo, `/vrpn_client_node/pure/pose`, stream status, diagnostics and TF.
It does not require live pad pose estimation, an assumed body-camera TF, PX4,
MAVROS or the landing controller.

On Jetson, start these in separate terminals if not already running:

```bash
bash scripts/sensor_seecam.sh
./setup.sh run aruco-landing-jetson odometry/optitrack
bash stack-assets/aruco-landing-jetson/tools/run_camera_body_extrinsic_recorder.sh
```

Start and stop an actual hand-carried capture:

```bash
DSD_CONTAINER=drone-stack-aruco-landing-jetson \
  modules/utility/session-recorder/sessionctl.sh start extrinsic-handcarried-01
# Keep the pad fixed; slowly translate and vary roll, pitch and yaw for 60–90 s.
DSD_CONTAINER=drone-stack-aruco-landing-jetson \
  modules/utility/session-recorder/sessionctl.sh stop
```

Keep at least one complete, sharp marker visible; avoid covering its corners.
Record a short stationary segment at each end and several distinct attitudes.
Do not change the lens focus, camera mounting or Motive body definition.
Motive `pure` represents body FLU (+X forward, +Y left, +Z up) at body center.

The September 19 hand-carried capture is complete and jointly calibrated.
The user's 30 mm central reference is verified as DICT_7X7_50 ID 21.
See [measured transforms, map and validation](../results/extrinsic-20260919/README.md).
The held-out body-origin position RMS is 2.60 cm; AXZY closure position RMS is
1.31 cm and orientation RMS is 1.07 degrees. Live transition validation remains
separate from this offline calibration.

## Transform definitions

`T_A_B` maps coordinates from B into A. Let G=OptiTrack `odom`, B=`base_link`,
C=`see3cam_optical_frame`, P=the fixed, metrically defined pad/target frame.

Observed: `A(t)=T_G_B(t)` and `Z(t)=T_C_P(t)` from calibrated image PnP.
Unknowns for the initial calibration:

```text
X = T_B_C                       body-camera mount, fixed across sessions
Y = T_G_P                       pad placement in OptiTrack, fixed during this bag
A(t) X Z(t) = Y                 fit both X and Y, with an estimated time offset
```

OpenCV PnP returns `T_C_P`, not `T_P_C`. If an observation is expressed as
`T_P_C`, invert it before using the equation above. A pad survey in global
OptiTrack coordinates is unnecessary; pad geometry still needs metric scale.
Varying attitude is essential to identify the mount translation and rotation.

The local pad-specific solver is `tools/calibrate_pad_extrinsics.py`. It
reconstructs the planar metric map, evaluates both planar camera-pose
hypotheses, and jointly fits X, Y and relative timing with temporal held-out
validation. The colleague's ChArUco solver at
`/home/hmcl/camera-calibration/scripts/solve_extrinsic.py` was a reference;
its detector is not used on this ArUco pad. Original captures remain preserved.

Required reviewed outputs: `base_link_to_see3cam_optical_frame.yaml` (X),
session Y in the diagnostic report (never a runtime placement file), inverses/frame conventions, detections and time
offset, excitation checks and held-out closure errors. A fit is not accepted
merely because it produced YAML files.

## Later landing and PX4 SITL validation

With reviewed X fixed, collect qualified simultaneous OptiTrack and pad
observations before switching:

```text
Y_i = A_i X Z_i                 robust online estimate of the new pad placement
T_P_B = inverse(Z) inverse(X)   body pose derived from the pad image
T_G_B_marker = Y T_P_B          marker-derived body pose in OptiTrack coordinates
```

The physical estimator now owns Y and publishes `/landing/vision_pose_marker`.
The updated `landing_vision_pose_adapter.py` consumes that already aligned
`PoseStamped`; it only selects the input source. It checks qualified alignment,
fresh measurements and quality flags, timestamp overlap, a stable qualification
interval, and bounded pose disagreement. Hardware switching defaults remain
false, and its MAVROS output must have no competing publisher. See the
[current capture and SITL workflow](pose_transition_sitl.md).

After this bag is solved and a consistent metric pad map is available:

1. Recompute `T_P_B` from the saved images with reviewed X and the pad model.
2. Replay poses, visibility and real detection inlier IDs on an isolated ML ROS
   master. Rebase all replay timestamps consistently; do not compare historic
   stamps to wall-clock freshness or publish replay data on Jetson's master.
3. Run a separate PX4 SITL + MAVROS instance with local UDP endpoints. Feed the
   adapter's global body pose to SITL, first with switching disabled, then with
   `allow_marker_switch=true`, `auto_switch=true` after readiness checks pass.
4. Assert exactly one vision-pose publisher, OptiTrack-to-marker source change,
   bounded position/orientation discontinuity, valid EKF external-vision fusion,
   and explicit behavior for stale/lost markers. Inspect ROS and PX4 logs.

A prerecorded hand-carried bag can verify calibration, pose-source transition
and PX4 measurement acceptance. It cannot alone validate closed-loop landing
control: recorded movement does not respond to SITL's commands. That needs the
interactive simulator/hardware stage after this replay validation.

## September 19 readiness evidence

The 8-second static preflight bag contains 425 original frames and 746 body
poses. Every image has exactly matching CameraInfo time. Nearest body-pose time
differences: median 2.597 ms, p95 4.656 ms, max 9.877 ms; this measures timestamp
proximity, not ground-truth sensor clock synchronization. Raw images run at
60 Hz, rectification at 20 Hz and OptiTrack at 100 Hz. Available disk space was
207 GiB; the static preflight used 658 MB. Recorder returned to IDLE afterward;
the hand-carried capture subsequently completed and is documented in the calibration report linked above.

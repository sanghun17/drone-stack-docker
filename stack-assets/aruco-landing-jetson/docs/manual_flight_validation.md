# Manual-flight recording and landing previews

Run on **Jetson**, from `~/drone-stack-docker`. Camera, physical-pad estimator,
OptiTrack, MAVROS and flight-safety should already be running. These remain shared
modules; this tool does not start/stop them, change PX4 parameters, arm, or set modes.

```bash
bash scripts/record_aruco-manual.sh prepare  # start observation nodes; no recording
bash scripts/record_aruco-manual.sh check    # verify actual input and telemetry
bash scripts/record_aruco-manual.sh start    # wait for RECORDING before flight
bash scripts/record_aruco-manual.sh status
bash scripts/record_aruco-manual.sh stop     # wait for finalized .bag
bash scripts/record_aruco-manual.sh down     # optional: stop observation nodes only
```

Start refuses if the actual MAVROS vision output is not an exact, fresh relay of
OptiTrack with `/vision_pose_mux` as its only publisher, if the selected input is
not OptiTrack, if FCU/required estimator flags are unavailable, if a normal
flight-control command producer is present, or if preview nodes are missing.
The marker need not be visible at recording start. An in-flight audit publishes
`/landing/manual_audit/status` at 1 Hz. It reports problems but never changes the
vehicle's input or mode. A later unauthorized graph/source change can invalidate
this audit; it is not a hardware interlock.

## Signal paths

```text
OptiTrack ── existing flight-safety vision_pose_mux (mocap) ── MAVROS ── real EKF2
    │
    └─ ArUco pose router ◀─ globally aligned marker body pose
             │  qualify >= 1 s + quality/jump gates; loss >= 0.5 s -> fresh OptiTrack
             └─ /landing/vision_pose_selected   (candidate only in this test)

OptiTrack and candidate poses + this session's online global pad placement
    └─ body and optical-camera poses in pad frame
        └─ existing landing controller, preview_only=true
            └─ /landing/shadow/{optitrack,transition}/cmd_vel_pad
```

The shared MUX is retained. Common-module changes provide generic input-topic
arguments, forward script arguments, and separate the two MUX service namespaces.
They contain no ArUco dependency. Existing default mocap and VIO paths are retained.
On the next normal flight-safety launch:

```bash
bash scripts/control_flight-safety.sh estimation_source:=mocap
# FUTURE external-estimation stage only, after its separate validation:
# bash scripts/control_flight-safety.sh estimation_source:=external \
#   external_pose_topic:=/landing/vision_pose_selected
```

`estimation_source` selects measurements for EKF2, not an OFFBOARD command source.
The ArUco router owns the 1 s / 0.5 s policy. The shared MUX only chooses one input.
The old already-running common modules need not be restarted for this manual
recording; new MUX names/configuration apply on their next launch. The audit
understands both old `/mux/selected` (checks publisher identity) and new
`/vision_pose_mux/selected`. This tool never calls a MUX selection service.

## Preview interpretation

Both preview controllers run at 60 Hz. They use the existing gains (Kp .8,
Kd .15), speed cap 2 m/s, downward command .5 m/s, camera-origin horizontal
reference, and camera-height touchdown threshold .20 m with existing latency
prediction. Commands are velocities in `landing_pad`, not MAVROS body/NED
setpoints. No command bridge subscribes to these preview topics.

Unlike a real landing run, **preview mode** automatically resumes calculating
when a fresh, above-threshold pose returns after touchdown/loss. This permits
repeated manual passes. Default real-controller terminal/disabled behavior is
unchanged. At rest close to the pad, TOUCHDOWN and zero commands are expected.
During the candidate's 0.5 s loss window, stale preview poses are withheld and
preview commands go to zero; only the actual EKF2 predicts its own state.

Pad placement is estimated and frozen in memory for the current estimator
session, never loaded as reusable global-pad calibration. Keep the pad fixed
through that session. The reusable body-camera extrinsic and pad geometry stay
in their own calibration files.

## Recording and scope

Bags and metadata: `experiments/aruco-landing/manual-flight/` on Jetson.
Includes OptiTrack, marker/candidate poses, source/qualification status, both
preview poses/commands/states, actual MAVROS vision/local pose/odom/velocity,
IMU, estimator flags, flight mode/arming/RC, PX4 statustext, flight-safety status,
diagnostics, TF and the 10 Hz detected JPEG stream. Full-rate raw images are not
recorded. ROS parameters, a read-only PX4 parameter subset, and calibration
contents are saved beside the bag. Capture uses the existing transition recorder
with a separate manual-flight profile and state file.

This flight can establish **real EKF2 continuity with OptiTrack** and inspect
landing command direction. The candidate source transitions are recorded but
**do not exercise source switching in the real EKF2**. Replay those inputs in an
isolated SITL to evaluate switching; retain the onboard PX4 ULog for internal
EKF innovations/reset counters not fully exposed by MAVROS. A passed stationary
routing/flag audit is not proof of flight stability or resolution of the previously
observed IMU/OptiTrack body-axis discrepancy.

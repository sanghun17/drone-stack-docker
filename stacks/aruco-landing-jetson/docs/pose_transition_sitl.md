# Pose-source capture and isolated PX4 SITL replay

The physical estimator publishes `/landing/vision_pose_marker` in `odom`.
The source adapter consumes that pose directly and selects either it or
`/vrpn_client_node/pure/pose`. It no longer estimates pad alignment or duplicates
perception topics. Hardware switch defaults remain disabled. No hardware mux,
FCU parameter, arming state or controller was changed during the SITL test.

## Hand-carried capture (Jetson)

Temporary entry scripts are installed on Jetson:

```bash
bash /tmp/aruco-transition/start.sh
# Wait for RECORDING. Hold still 5 s, then slowly translate/rotate for 60–90 s.
# Keep the stationary pad visible, and finish with another 5 s stationary.
bash /tmp/aruco-transition/stop.sh
bash /tmp/aruco-transition/status.sh
```

Start checks that both global poses are fresh and in `odom`. Stop sends SIGINT
only to this recorder and waits for the bag to finalize. Neither script starts
flight control. Files are under
`flight_logs/aruco-landing/pose-transition/handcarried-<timestamp>.bag`.
They include both global poses, pad/body poses, quality flags and inlier IDs,
CameraInfo, annotated JPEG images/clicks, IMU, MAVROS state/input and TF.
Original full-rate raw images are not recorded by this pose-transition profile.
The implementation is `scripts/pose_transition/capture.py`.

## Replay on ML

```bash
# Copy the completed bag from Jetson, then run from the repository root:
bash stacks/aruco-landing-jetson/scripts/pose_transition/run_sitl.sh \
  flight_logs/aruco-landing/pose-transition/handcarried-<timestamp>.bag \
  --output flight_logs/aruco-landing/pose-transition/sitl-handcarried-01
```

Use a new output directory for every run. The tool uses the existing local
`/home/ml/PX4/PX4-Autopilot` v1.11.3 checkout/binary and its matching jMAVSim.
`PX4_ROOT` can override the checkout path; this harness expects the v1.11 startup
layout and parameter names. ROS Noetic/MAVROS, Java (jdk.compiler module), NumPy
and pyulog are already available on ML. The simulator classes build under
`.build/aruco-transition-jmavsim` without installing OS packages.

Isolation: ROS master `127.0.0.1:11341`, PX4 instance 7/system ID 8, simulator TCP
4567, MAVROS UDP 14547 <-> PX4 14587. GCS output is localhost:14657 and MAVROS GCS
forwarding is disabled. Occupied test endpoints cause an abort. The harness
starts and cleans up only its own processes; it never connects to Jetson's ROS
master or serial FCU and never sends arm/mode/setpoint commands.

Bag receipt times and measurement header times are shifted by the same offset
to current wall time. Pose age/latency and inter-source timing remain intact.
The test begins on OptiTrack, qualifies overlap for >=1 second and >=30 samples,
then automatically switches to markers. Default maximum disagreement is 15 cm /
12 degrees, nearest timestamp separation <=35 ms, source/quality age <=200 ms.
Output measurement timestamps must strictly increase, including at the switch.
After selecting markers, missing OptiTrack does not stop marker output. Missing
markers or invalid quality withhold output; no pose is manufactured. Optional
`auto_fallback_to_optitrack:=true` selects fresh OptiTrack after
`marker_loss_timeout_s:=0.5` without a qualified marker pose. This option defaults
to false for hardware. A manual return with both sources visible still requires
fresh consistent overlap.

The harness suppresses OptiTrack after 65% of the replay and marker poses from
80–90% to exercise dropout/recovery. It checks one output publisher, source
transition, monotonic timestamps, dropout behavior, PX4 external-vision fusion
flags and filter faults. ROS observations, logs, PX4 ULog and `summary.json`
remain in the output directory. The summary distinguishes matched-source
pose disagreement from the actual consecutive output step at handover.

**Scope:** prerecorded motion is open-loop. jMAVSim supplies its own IMU and
cannot respond to recorded hand motion. Static replay validates external-vision
acceptance and source routing; moving replay can reveal source discontinuities
but may produce EKF innovations from mismatched simulated inertial motion. It
is not a validation of closed-loop landing or real-flight failover. A coherent
simulated motion/IMU trajectory is needed for that next stage.

## September 19 static baseline

`static-pose-pair-20260919-01.bag` contains 30 s of live paired poses. The first
isolated run passed all nine checks. At handover, matched input disagreement
was 0.884 mm / 0.179 degrees; the consecutive output position step was 1.524 mm
and measurement-time gap 11.17 ms. PX4 fused external position, yaw and height,
with zero filter-fault flags and no innovation-rejection flags during fusion.
There were 242 output messages after OptiTrack cutoff before marker dropout,
zero inside the marker-dropout window, and 160 after recovery. PX4 stayed disarmed.
See `../results/pose-transition-sitl-20260919/static-summary.json`.

The new hand-carried bag was captured by the user and all three approaches were
replayed in SITL. See the [conditional hand-carried results](../../../../drone-stack-archive/20260921-cleanup/stack-assets/aruco-landing-jetson/results/pose-transition-sitl-20260919/handcarried/README.md),
including the IMU/OptiTrack body-axis mismatch and the explicit replay-only
yaw correction. No capture is started automatically.
The updated source adapter is not automatically launched on the hardware graph;
flight-safety's existing vision mux retains that output until a separate deployment.

## Three approaches with recorded IMU and timed fallback

The user requested a 0.5 s marker-loss timeout followed by return to fresh
OptiTrack, then repeated marker reacquisition. The test option implements this
without changing the hardware default. Qualification restarts after a loss.

The following command reproduces the **axis-adjusted sensor replay**, not an
unmodified hardware configuration. The IMU yaw adjustment is explicit because
the recorded FCU and OptiTrack body axes differ by approximately 90 degrees:

```bash
bash stacks/aruco-landing-jetson/scripts/pose_transition/run_sitl.sh \
  flight_logs/aruco-landing/pose-transition/handcarried-20260919-120243-437248.bag \
  --output flight_logs/aruco-landing/pose-transition/sitl-new-run \
  --recorded-imu --imu-yaw-deg -90 --natural-visibility \
  --marker-hold 1 --fallback-timeout .5
```

`--marker-hold 0` compares immediate qualified switching. `--optitrack-only`
provides the reference run. `--natural-visibility` preserves the bag's actual
marker gaps, rather than injecting the static test's artificial dropouts.
`--instance 8 --ros-port 11342` selects a second isolated simulation instance;
its simulator/MAVROS/PX4 endpoints derive from that instance number.

Recorded 50 Hz IMU samples are interpolated to 250 Hz; no higher-frequency
measurement information is invented. Auxiliary magnetic field/barometer are
simulated, and the stationary prefix warms up the EKF before motion playback.
The real pad/camera transform, TF tree, Motive settings and FCU parameters remain
unchanged. Results establish source-routing and estimator continuity under the
stated replay assumptions, not closed-loop flight validity.

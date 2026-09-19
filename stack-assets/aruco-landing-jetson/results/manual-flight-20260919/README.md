# Manual-flight evaluation — 2026-09-19

**Verdict: actual OptiTrack-driven PX4 localization remained continuous in this
recording. OptiTrack landing-command direction passed. The candidate marker
transition/landing path failed this flight's offline evaluation and must not be
used as the real EKF2 input in its current form.** No vehicle settings or production
code were changed during this analysis; everything ran offline on ML.

Sources:
- `experiments/aruco-landing/manual-flight/manual-flight-20260919-131105-329410.bag`
  (121.43 s, 22:11:06.31–22:13:07.74 KST)
- `/home/ml/webcam_recorder/recordings/flight_2026-09-19_22-11-37/flight_2026-09-19_22-11-37.bag`
  (60.86 s, includes raw MAVLink)
- Calibration and ROS-parameter snapshots beside the manual bag.

All elapsed times below are relative to the manual bag start. Main ARM interval:
31.616–91.633 s (60.017 s). An earlier brief ARM interval was 8.616–12.611 s.
Every recorded FCU mode was POSCTL; no OFFBOARD landing was executed.

## Actual PX4 / EKF2

| Check | Result |
|---|---|
| Actual MAVROS vision vs OptiTrack | 12,092 exact timestamp/position/orientation matches, zero differing poses; one unmatched boundary sample |
| FCU connected / mode | Connected throughout; POSCTL throughout |
| MAVROS estimator flags | All 121 samples: attitude, horizontal/vertical velocity, relative horizontal position and absolute vertical position valid; no accel-error or GPS-glitch flag |
| Main-flight local pose | 1,800 samples, 29.998 Hz |
| Maximum local-pose receipt gap | 45.73 ms |
| Largest local-position sample step | 3.084 cm (includes actual motion) |
| Largest step after subtracting OptiTrack-measured motion | 0.954 cm |
| Position error vs time-interpolated OptiTrack | Median 0.910 cm, 95th percentile 2.235 cm, max 2.642 cm; no spatial fit applied |
| Raw MAVLink odometry reset counter | 1,830 samples, constant 17; zero observed increments |
| Raw estimator ratios, 61 samples | Horizontal position max .05048; vertical position .02973; heading .03187 |

The counter's value 17 predates this analyzed interval; it does not mean 17 resets
happened during this flight. PX4 v1.16.2 sends `vehicle_odometry.reset_counter`
through MAVLink ODOMETRY and identifies it as autopilot output:
[PX4 ODOMETRY source](https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.16.2/src/modules/mavlink/streams/ODOMETRY.hpp).
The `mag_ratio` wire field carries the heading test ratio in this firmware, not
proof that magnetometer fusion was enabled:
[PX4 ESTIMATOR_STATUS source](https://raw.githubusercontent.com/PX4/PX4-Autopilot/v1.16.2/src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp).
Velocity/HAGL/TAS ratios were non-finite and are represented by null in the JSON;
they are not counted as passing innovation tests.

No large output discontinuity or estimator loss was observed. This is not a
complete internal ECL fault/rejection audit: the physical FCU ULog was not
available. In particular, the **actual EKF2 did not receive the marker candidate**.

## Landing-command evaluation

The reference is OptiTrack body pose, transformed using this session's frozen
pad placement and calibrated body-to-optical-camera lever arm. The pad's global
placement was not independently surveyed. Commands remain pad-frame velocities;
body/NED conversion and an OFFBOARD bridge were not exercised.

| Metric during main flight | OptiTrack preview | Transition preview |
|---|---:|---:|
| Publish rate, entire bag | 60.000 Hz | 60.000 Hz |
| Max command receipt gap | 23.08 ms | 25.20 ms |
| Active samples checked for direction, >5 cm radial error | 3,079 | 3,055 |
| Toward pad according to independent OptiTrack reference | 100% | 86.35% |
| Outward-pointing command samples | 0 | 417 (~6.95 s at 60 Hz) |
| Toward fraction while candidate marker selected | 100% | 68.72% (1,333 samples) |
| Maximum horizontal speed | 1.377 m/s | 1.320 m/s |
| Vertical command values | -0.5 or 0 m/s | -0.5 or 0 m/s |

Both controllers point toward their *own* estimated target. That self-consistency
is insufficient: the transition controller's input is sometimes wrong. Its
independent camera-position error reaches 1.960 m while active. Reconstructing
the configured PD/latency formula from recorded poses matches the published
commands to a maximum discrepancy of about .0023 m/s; incorrect input, rather
than an obvious command-axis/sign reversal, explains the main disagreement.

These are command-direction checks, not a closed-loop landing success test.

## Candidate transitions

| Event | Time (s) | Qualification / timeout | Candidate output change |
|---|---:|---|---:|
| OptiTrack -> marker | 39.241 | 1.302 s / 30 good samples | 4.43 cm |
| Marker -> OptiTrack | 45.580 | 0.515 s since last valid marker | **1.986 m** after a .522 s publication gap |
| OptiTrack -> marker | 71.921 | 1.199 s / 30 good samples | 7.53 cm |
| Marker -> OptiTrack | 92.480 | 0.515 s since last valid marker | 1.85 cm after a .520 s publication gap |

The second fallback happened just after DISARM. There were two qualified entries
in this bag; the previous hand-carried session's three-entry expectation does
not apply automatically.

While marker was selected, the candidate position disagreed with time-matched
OptiTrack by median **30.9 cm**, 95th percentile **1.032 m**, max **1.927 m**.
The 15 cm / 12 degree consistency gate is checked for entry, but
`PoseTransition.ingest()` currently forwards selected marker poses based on
freshness and visibility/inlier quality even after consistency fails.
`last_valid_marker_receipt` also refreshes without that consistency condition.
Consequently, disagreement can grow until marker loss, and timeout fallback
then returns abruptly to OptiTrack. This is an observed failing path, not evidence
that the real PX4 jumped.

The underlying reason for the marker pose error itself (PnP ambiguity, calibration,
image timing, or another geometry issue) is not yet established by this report.

## Additional faults found

1. **Preview loses its frozen pad transform on marker loss.** The estimator only
   publishes pad-placement/ready messages on valid marker detections, but
   `manual_flight_preview.py` expires both after one second. OptiTrack remained
   fresh (<13 ms in inspected samples), yet its preview became invalid at
   36.222–36.491, 45.939–46.191 and 47.189–47.641 s: **0.972 s total** in the main
   flight, producing zero commands. A session-frozen transform must remain usable
   until an explicit alignment reset, with session/estimator liveness checked
   separately. This also needs fixing before OptiTrack-based OFFBOARD landing.
2. **Capture/estimation did not sustain 60 Hz while ARM recording was active.**
   The camera-info stream delivered 1,434 frames over the main interval, **23.89 Hz**.
   Before ARM it was near 60 Hz; after recording stopped it recovered near 60 Hz.
   Flight-safety's configured `record_all: true` subscribes to raw, color, rectified
   and their compressed images (all present in the 5.9 GB bag). Camera status
   changed from raw/color/rect subscriber counts 1/0/0 to 3/2/2. The capture loop
   performs JPEG/rectification synchronously when requested. These observations
   and code identify a likely recording-load bottleneck; a controlled recording
   profile A/B test is still needed to quantify the cause. Keep common recorder
   code generic; choose an ArUco recording profile instead of adding stack-specific
   exclusions to common behavior.
3. **Camera timeout after flight:** `select timeout` at 114.349 s (after DISARM at
   91.633 s), with no processing at the end of the manual recording. Do not describe
   this as an in-flight EKF failure.
4. **Audit false alarm:** the shared ARM recorder `/record_<timestamp>` subscribes
   to preview commands for logging. The new manual audit treated it as an unexpected
   command consumer. All 66 failing audit samples are explained by recorder-consumer
   warnings, not by a detected change of real vision source. A recorder is not an
   actuation bridge; its identity should be verified/recognized by the audit.

## Required before the next OFFBOARD stage

- Correct frozen-pad lifetime handling and the recorder audit false positive.
- Select a recording profile that preserves camera/estimator throughput; retain
  webcam triggering and shared safety functionality.
- Diagnose the marker pose error; enforce selected-source validity continuously,
  and validate bounded fallback behavior on this failing bag in isolated replay.
- Re-evaluate OptiTrack-only command continuity first; keep real EKF input on
  OptiTrack until the marker path passes replay/SITL and its separate flight review.

Artifacts: `summary.json`, `mavlink_ekf.json`, `overview.png/.pdf`, and
`candidate_diagnostics.png/.pdf`. Analysis programs:
`tools/pose_transition/analyze_manual_flight.py` and `extract_mavlink_ekf.py`.

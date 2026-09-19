# New fixed-body calibration: control and EKF2 recheck

Analysis ran on ML without a ROS master, PX4 connection, or setpoint output. The bag is manual-flight-20260919-131105-329410.bag. Main armed interval is 31.616–91.633 s (60.017 s), POSCTL throughout.

## Recomputed commands

The table gives maximum absolute command components in the **pad frame**; these are hypothetical commands from current code, not executed aircraft speeds. The recorded controllers had zero angular.z; the new yaw feedback is included only in the offline replay.

| Component | OptiTrack pose | Auto marker/OptiTrack selection |
|---|---:|---:|
| vx (m/s) | 1.071 | 1.063 |
| vy (m/s) | 0.692 | 0.692 |
| vz (m/s) | 0.500 | 0.500 |
| angular.x (deg/s) | 0.000 | 0.000 |
| angular.y (deg/s) | 0.000 | 0.000 |
| angular.z / yaw rate (deg/s) | 20.054 | 20.054 |

- vz is either 0 or -0.5 m/s (descent). Yaw command ranges from -0.35 to +0.35 rad/s. Angular x/y are zero in this velocity/yaw-rate interface; these are not PX4 attitude actuator commands.
- Configured bounds: horizontal vector norm ≤2 m/s; descent speed 0.5 m/s; yaw rate ≤0.35 rad/s (20.054°/s). The actual horizontal maxima are 1.202/1.201 m/s, and 3D speed maxima are 1.302/1.301 m/s.
- X/Y direction is toward the independently reconstructed OptiTrack camera-in-pad position in 100% of evaluated active samples for both paths (radius >5 cm, horizontal command >1 cm/s). This tests command direction, not closed-loop landing stability.
- Target yaw is body yaw 0 relative to pad +X. Kp=1/s, deadband=1°, cap=0.35 rad/s. OptiTrack path has 100% correct yaw direction on evaluated nonzero samples. Marker-selection path has 99.37%. Actual reference yaw error reaches about 99°, so the rate cap is meaningful.
- Descent starts simultaneously with horizontal/yaw feedback; there is no yaw-alignment gate. The manually flown trajectory does not demonstrate that yaw reaches zero before touchdown, especially in the roughly 99-degree-error portions.
- Marker yaw opposes the instantaneous OptiTrack reference during 59.200–59.333 s and 77.100–77.283 s: 19 ticks / 0.317 s total. Maximum opposing rate is 9.36°/s; true error reaches 10.94°. Input poses are about 67–111 ms old. Delayed/erroneous marker attitude during a yaw crossing is consistent with this result; the controller does not predict yaw as it does horizontal position.

## Remaining command gaps

- OptiTrack-only preview becomes invalid for 0.967 s total: 36.217–36.483, 45.933–46.183, 47.183–47.633 s. It zeros commands despite continuing OptiTrack. Current manual_flight_preview.py expires BOTH the frozen session pad transform and alignment-ready flag after 1 s without publication. The estimator only republishes these on valid detections; marker loss incorrectly invalidates an otherwise usable fixed pad reference.
- Marker-selection preview has 1.883 s total invalid time. In addition to that shared pad-expiry issue, selected marker poses can exceed the 0.2 s preview freshness limit before the 0.5 s fallback fires. These zeros are separate from the intended TOUCHDOWN zeros.
- Nine existing physical-pad/yaw unit tests passed. These existing behaviors were measured, not silently changed in this analysis. They prevent treating this as an unconditional OFFBOARD pass.

## Recorded real EKF2

- Actual MAVROS vision: 12,092 exact OptiTrack stamp/pose matches, zero differing matches, one unmatched recording-boundary sample.
- Required attitude, horizontal/vertical velocity, relative horizontal position and absolute height status flags remain true in all 121 recorded status samples. GPS glitch and accelerometer error flags remain false. Absolute horizontal/global flags being false are not treated as faults in this GPS-disabled local-position setup.
- Main-flight MAVROS local position: 1,800 samples, 29.998 Hz, maximum receipt gap 45.73 ms. OptiTrack position disagreement: median 0.91 cm, p95 2.24 cm, maximum 2.64 cm. Largest step after subtracting OptiTrack motion: 0.954 cm.
- Raw MAVLink ODOMETRY: 1,830 samples, reset_counter remains 17 throughout (zero increments, not 17 new resets). Maximum ESTIMATOR_STATUS ratios: horizontal position 0.0505, vertical position 0.0297, heading 0.0319. No STATUSTEXT entries in this full recording.
- No recorded evidence of EKF shutdown or an odometry reset during the flight. This does not establish every internal EKF event without a ULog, nor validate feeding the corrected marker pose into EKF2. Real flight used OptiTrack only.

## Reproduction and scope

- LandingController, Preview and LandingVisionPoseAdapter are the real current classes. Only ROS transport/parameters/timers are replaced with in-memory bindings. Controller/preview timers are scheduled at 60 Hz and router status at 50 Hz. This is not a measured new hardware throughput test.
- The primary full-flight table assumes a previously initialized pad alignment, as the real session had. Because pre-bag observations are missing, it retrospectively uses the pad transform learned by a cold-start replay at 39.160 s. This coordinate reference uses later bag data; it is not an independent all-unseen-data accuracy test. A separate cold-start replay is saved in the parent directory and gives the same per-axis maxima.
- Subscriptions, callback transport jitter and exact original timer phase are not reproduced. Shadow outputs only are computed. Detection results are reused from recorded Camera←Pad poses; new extrinsic and original timestamp correction are applied.
- Mount fitting used portions of these recordings. The earlier calibration report contains temporally held-out geometry tests; this full-bag controller replay is a diagnostic check.

```bash
OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/replay_fixed_body_control.py
OPENBLAS_NUM_THREADS=1 python3 stack-assets/aruco-landing-jetson/tools/pose_transition/replay_fixed_body_control.py --prealigned
python3 stack-assets/aruco-landing-jetson/tools/pose_transition/report_fixed_body_control.py
```

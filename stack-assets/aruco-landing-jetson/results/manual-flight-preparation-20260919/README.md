# Manual flight preparation, 2026-09-19

Deployed to Jetson. Real flight recording has not been started by this preparation.
Existing MAVROS, OptiTrack, flight-safety and physical-pad estimator stayed running.
Only the ArUco observation launch was started/restarted. No arming, mode, PX4
parameter or real MUX selection writes were made.

- Live audit: sole MAVROS vision publisher `/vision_pose_mux`, selected
  `/vrpn_client_node/pure/pose`; roughly 200 exactly matching poses over its
  latest 2 seconds, zero mismatches; disarmed POSCTL.
- Required MAVROS attitude/velocity/local-position flags true; reported
  accelerometer error and GPS-glitch flags false.
- Eight-second live observation: OptiTrack 99.88 Hz, MAVROS vision 99.88 Hz,
  candidate 99.87 Hz, MAVROS local pose 29.97 Hz. Maximum receipt intervals:
  actual vision 30.22 ms; local pose 40.26 ms. Latest yaw approximately 6.9 degrees
  for both OptiTrack and MAVROS local pose; these latest samples are not synchronized.
- Preview commands 60 Hz, no flight-command consumer. At low stationary height
  TOUCHDOWN/zero command is expected.
- Read-only FCU values: SDLOG_MODE=0, SDLOG_PROFILE=1. No logging setting changed.

`test_pose_transition.py`: 8 passed. The isolated ROS test
`tests/check_manual_flight_ros.py` passed at 60.00 Hz preview rate. It exercised
mocap bypass while the candidate switches to marker, qualification/fallback,
independent MUX services, generic external input selection, command direction,
touchdown re-entry, stale-input zero commands, rejection of recording with an
external EKF input, and synthetic bag start/stop/message contents. It also
resolved common flight-safety launches for default, VIO, and arbitrary external
sources. The private localhost ROS master and its children were cleaned up.
Synthetic bags existed only in a temporary test directory and were removed.

The live common modules still use their original startup configuration. Generic
input arguments and separate MUX service names take effect on their next normal
launch; this recording does not require restarting them. Stationary telemetry and
an isolated ROS integration test do not establish flight stability or actual PX4
marker-switch performance. See `docs/manual_flight_validation.md` for scope and
commands. Preserve onboard ULog for full EKF diagnostics.

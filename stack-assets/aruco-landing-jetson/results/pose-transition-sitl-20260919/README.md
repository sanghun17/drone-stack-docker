# Static pose-bag source transition — PASS

On September 19, paired live OptiTrack and marker poses were recorded for 30 s
and replayed through the updated pose-source adapter, MAVROS and PX4 v1.11.3
SITL with headless jMAVSim. All nine automated checks passed; six source-router
unit tests also passed. See `static-summary.json` for measurements and checks.

- Handover source disagreement: 0.884 mm, 0.179 degrees.
- Consecutive output step: 1.524 mm; timestamp gap: 11.17 ms.
- One MAVROS vision input publisher; strictly increasing measurement timestamps.
- Marker output survives OptiTrack cutoff, stops during injected marker loss,
  and resumes with fresh marker observations.
- ULog confirms external position/yaw/height fusion, zero EKF filter faults and
  zero innovation-rejection flags during fusion. Vehicle stayed disarmed.

Detailed ROS logs, observations and PX4 ULog are in the ignored
`experiments/aruco-landing/pose-transition/sitl-static-01` directory.
Reproduction and capture commands: `../../docs/pose_transition_sitl.md`.

This static open-loop test does not validate moving-trajectory EKF consistency
or closed-loop landing. The subsequent three-approach capture and conditional SITL results are documented
in [the hand-carried report](handcarried/README.md).
The hardware MAVROS input remains owned by the existing flight-safety mux.

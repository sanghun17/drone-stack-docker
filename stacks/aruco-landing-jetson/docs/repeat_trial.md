# Pilot-triggered repeated landing experiment

Start the usual camera, perception, OptiTrack, MAVROS and flight-safety modules.
Launch one planner, with the explicit experiment option:

```bash
bash scripts/planner_aruco-landing.sh repeat_test:=true
```

This option permits automatic arming. The default planner still requires manual
takeoff. A new pilot POSCTL-to-OFFBOARD edge on the ground starts each experiment;
launching/restarting the node while already in OFFBOARD never starts an experiment.

Before each arm, the planner requires a fresh successful sync gate for all
finalized recordings. The generic host sync worker verifies bag and manifest
SHA-256 hashes on ML and finalization of both configured webcam videos. A pending
recording, unavailable worker/receiver, missing manifest or incomplete video
blocks arm. If OFFBOARD was selected before readiness, return to POSCTL and select
OFFBOARD again after `data_sync_ready` and `offboard_entry_ready` become true.

The phases are:

1. ARMING: request arm once and wait for the FCU armed state; reject/timeout fails.
2. TAKEOFF: hold the initial XY and rise to global OptiTrack **body Z=2.0 m**.
3. CENTER: settle at global XY=(0,0), retaining Z=2.0 m.
4. RANDOM_POSITION: move to a uniformly sampled XY point inside the inset fence.
5. APPROACH: begin the existing return-to-(0,0) and marker qualification logic.
6. DESCEND: existing marker landing and dual-height termination policy.
7. COMPLETE: wait for verified transfer and the next pilot mode transition.

Takeoff/center/random motion has combined XY and vertical command limits of
0.5 m/s. Arrival requires 3D position error <=0.10 m and measured speed <=0.10 m/s
continuously for 1 s. The current XY fence is ±2.5 m, with its 0.4 m safety margin
plus 0.3 m experiment inset: random coordinates are within ±1.8 m, at least
0.75 m from the origin. Stage timeout is 30 s; arming timeout is 5 s.
These settings belong to `config/repeat_trial.yaml`; the fence is read from
`config/geofence.yaml`. Random goals and phase changes are included in trial status
and the bag; the repeat configuration and module locks are included in metadata.

Only APPROACH/DESCEND enable the marker router. Seeing the pad while rising or
moving to the random target cannot start descent. The existing approach may stop
before reaching (0,0) when a marker is detected and qualified, matching the prior
landing experiment. Its approach limit is 0.5 m/s; landing XY cap is 2.0 m/s and
descent command is 0.5 m/s.

Pilot takeover cancels the cycle. Health loss, unexpected disarm, an arming refusal
or a stage timeout does not automatically retry. The generic safety API can clear
only a mission-requested termination on fresh, healthy, disarmed ground state in
pilot mode. Fault-triggered termination and pilot KILL remain authoritative.

The shared sync worker and installer accept log directory, SSH destination,
destination directory, SSH key and optional camera names as arguments. They contain
no ArUco/Jetson/ML paths and can serve other stacks using separate service names.

Verification uses unit tests and a localhost mock FCU with integrated commanded
velocities. It does not model PX4 dynamics or certify real takeoff behavior.

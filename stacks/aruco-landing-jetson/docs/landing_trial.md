# Manual takeoff → OFFBOARD landing trial

The ArUco hardware profile now connects the **trial-gated OptiTrack → marker router**
to the existing common vision MUX. MAVROS and flight-safety remain stack-neutral:
the safety launcher accepts generic `FLIGHT_SAFETY_ESTIMATION_SOURCE=external`
and `FLIGHT_SAFETY_EXTERNAL_POSE_TOPIC=/landing/vision_pose_selected` supplied by
the ArUco runtime profile. Other stacks keep their existing source defaults.

Start camera, OptiTrack, MAVROS, perception and flight-safety. The planner launcher
starts its own pose router; **do not launch a second pose router separately**.
The MUX must select `/landing/vision_pose_selected` and vision output must match
that candidate stream. Before pilot OFFBOARD, the router forwards OptiTrack only.
The planner validates current router status, output identity, local pose and safety.

The runtime policy enables marker switching, automatic switching, and 0.5-second
fallback. The router additionally requires a fresh planner permission heartbeat
and a fresh connected, armed OFFBOARD state. Loss of permission returns to fresh
OptiTrack. Marker samples must remain within 15 cm / 12 degrees of time-paired
OptiTrack after the switch as well; rejected samples do not refresh the loss timer.
The trial continues to require OptiTrack for cross-checks and dual-height termination.

On Jetson, from `~/drone-stack-docker`, use one entrypoint (may start before takeoff):

```bash
bash scripts/planner_aruco-landing.sh
# Optional observation-only run instead:
bash scripts/planner_aruco-landing.sh dry_run:=true
```

There is no separate start service/script in the normal operator flow. The planner prepares in the background and streams current-position holds while the pilot retains manual control. It never arms or selects OFFBOARD. For the default `force_disarm` finish policy, preparation requires the generic safety termination interface to be enabled. The ArUco stack opts in through runtime.env; other stacks remain disabled. The optional `landing_finish_mode:=auto_land` policy instead checks COM_DISARM_LAND > 0. `/landing/trial/status` reports `offboard_entry_ready`; at least one second of healthy preparation is required.

1. Start on the ground, take off manually in POSCTL, then select OFFBOARD once prepared. Ground/disarmed entry or unprepared entry does not start approach. Starting the program when already OFFBOARD does not start a mission either; a new pilot mode edge is required.
2. Capture the current altitude and heading on OFFBOARD entry. Approach the known OptiTrack global XY=(0,0), holding entry altitude. XY velocity is proportional to horizontal error (gain 0.8/s), capped at a combined 0.5 m/s. Final transformed approach commands are independently capped to 0.5 m/s as well.
3. Pre-OFFBOARD marker observations never count toward source selection or descent qualification. Once a valid new marker is seen during approach, stop horizontal motion and hold altitude while validating observations for at least 1 s and 30 router samples. Before qualification, invalid observations reset the dwell and approach may resume. The router switches only after its time-paired position/rotation checks pass; descent waits until marker output has actually been published and the route is healthy. Session alignment may be prepared beforehand. There is no pose blending; a timestamp-monotonic handoff can briefly withhold output.
4. Descend with the existing PD controller and pad-relative yaw target 0 degrees. Horizontal command cap 2.0 m/s, descent 0.5 m/s, yaw-rate cap 0.35 rad/s (20.1 degrees/s). Final transformed descent commands also use the 2.0 m/s combined XY cap. These phase-specific limits are selected by `LANDING_APPROACH_SPEED_MPS`, `LANDING_HORIZONTAL_SPEED_MPS`, and `LANDING_DESCENT_SPEED_MPS` in the stack runtime.env. Marker validity requires at least 1 inlier, fresh poses and agreement with OptiTrack within 0.15 m / 12 degrees.
5. Invalid marker observations brake descent immediately. Missing or rejected marker poses for 0.5 s return estimation to fresh OptiTrack and latch failed position hold. Rediscovery never resumes descent or reselects marker in the same OFFBOARD episode; leave autonomous mode and initiate a new trial. If OptiTrack is stale, the router does not fabricate a fallback pose; common safety remains authoritative. During airborne AUTO.LAND, loss requests POSCTL. Leaving OFFBOARD manually and deliberately re-entering after preparation starts a new trial; no separate reset command is needed.
6. The stack selects `force_disarm` with `LANDING_TERMINATION_HEIGHT_SOURCE=optitrack`. Only after entering DESCEND, a fresh OptiTrack-derived **camera height above the aligned pad in [0, 0.20] m** triggers `/flight_safety_response/request_termination`. Height is `(inverse(Global←Pad) × Global←Body × Body←Camera)[2,3]`, not global Z or body height. Current marker visibility is not required for this terminal request. Healthy safety/routing, fresh OptiTrack and local/FCU state, armed OFFBOARD and valid unchanged session alignment remain required. This terminal gate is evaluated before marker-loss/fallback failure in the same tick; a previously failed/cancelled trial never resumes or cuts. Above the threshold, marker loss retains its existing brake and failed-hold behavior. The request worker recomputes the height and rechecks the same conditions immediately before calling safety. No averaging/dwell is added. The generic launch default `termination_height_source:=dual_height` retains the previous marker <=0.20 m AND OptiTrack <=0.25 m policy; `auto_land` behavior is unchanged.
7. The common safety node independently requires opt-in, fresh connected armed OFFBOARD state, no pilot manual override, and an admitted fresh Normal stream. Safety alone sends MAV_CMD_COMPONENT_ARM_DISARM (400), param1=0, param2=21196. This stops motors before contact; it does not assert physical touchdown. The planner waits for armed=false before COMPLETE. Rejected or unconfirmed requests fail into hold; they are not repeatedly fired.
8. Safety's existing KILL latch remains active after success. In repeat mode, return to POSCTL while disarmed; preparation may clear a mission-requested termination through the existing guarded safety reset. Other fault/kill latches remain authoritative. The LED therefore retains the existing solid-red automatic KILL meaning. Optional `landing_finish_mode:=auto_land` retains the former PX4 landing handoff.

For OptiTrack-only operation, run safety with `estimation_source:=mocap` and planner with `estimation_transition:=false`; change both together.

Configuration/code changes require restarting safety and planner; editing files does not change running Python processes. Restart on the ground while disarmed. No service in this document should be manually invoked as a preflight check: the termination service is an actuator, not a query.

A frozen pad transform does not expire because marker messages stop. Explicit alignment reset or reference change invalidates an active trial. Moving vehicle samples still expire. These are command limits, not guarantees on measured vehicle speed.

## LEDs

Existing priorities are preserved: manual KILL with RC OFFBOARD = red/green blink; automatic KILL = solid red; emergency LAND = amber/green. A healthy OFFBOARD failed-trial position hold with fresh setpoints = **green/white at 2 Hz**. Intentional normal AUTO.LAND = cyan blink. Missing commands and emergency conditions retain their existing higher-priority indications. The common LED consumes generic `/control/mission_status` JSON; dry-run status cannot change the LED.

## Recording and verification scope

The stack-owned flight-safety ARM recording profile includes trial status, command inputs, yaw, generic mission status, mux selection and MAVROS estimator/state topics. Start flight-safety before arming. It retains compressed detection images and webcam recording without adding raw-image subscriptions. The older manual observation audit script is not the launcher for this OFFBOARD experiment.

Validation: Focused lifecycle/velocity/LED/reference tests passed; isolated localhost integration used the real safety response, vision mux, trial and existing landing controller with a **mock FCU**. It also checked pre-arm startup with visible markers, preparation without start/reset service calls, a fresh post-OFFBOARD marker dwell, the final XY cap, approach altitude, marker dwell, loss-to-hold (~0.513 s), no automatic retry, AUTO.LAND handoff, ground+disarm completion, AUTO.LAND loss→POSCTL and sole safety/MUX publishers. This is not a PX4 flight-dynamics simulation or proof of real-flight performance. No real vehicle was armed or commanded during these tests.

## Stack-owned geofence

ArUco reads `stacks/aruco-landing-jetson/config/geofence.yaml`: enabled_axes=[x,y], XY bounds ±2.5 m, WARN margin 0.4 m. Both Z upper/lower boundary checks are disabled. RViz shows an XY rectangle at the current vehicle height, not a ceiling/floor. Pose liveness and consistency checks remain active.

Risk-aware reads `stacks/d435i-voxblox/config/geofence.yaml`: enabled_axes=[x,y,z], XY ±2.5 m, Z [-0.5,2.0] m and margin 0.4 m, identical to its prior settings. The common safety module accepts `geofence_config` and contains no stack-specific axis policy. Each stack selects its file through runtime.env; no image rebuild is needed. Restart flight-safety while disarmed to apply.

## Integrated transition validation (2026-09-20)

45 Python tests passed. Isolated localhost integration with real planner, router,
MUX and safety response plus a mock FCU verified pre-OFFBOARD lock, stop-to-qualify,
marker output acknowledgment before descent, inconsistent-marker braking,
0.5-second loss → OptiTrack + failed hold, no retry on rediscovery, a new pilot
OFFBOARD trial, and dual-height force-disarm acknowledgment. OptiTrack-only
regression also passed. These are not EKF2 flight-dynamics tests.

The 18:32 flight bag replay using recorded trial authorization rejects all six
post-switch samples exceeding 15 cm that the previous router forwarded. The
representative switch still has 9.67 cm / 2.35 degree input difference and a
66.6 ms output gap; this change does not claim to eliminate discontinuity.

## ArUco consistency limits

The stack supplies `config/consistency.yaml` through the generic
`consistency_config` launch argument: WARN above 0.20 m, ERROR above 0.50 m.
Other stacks retain 0.10 m / 0.25 m. Response policy, source freshness and
marker routing gates are unchanged. Restart flight-safety while disarmed;
these thresholds are read once at diagnosis startup.

Single-marker update: physical PnP, pose router and trial now accept one valid marker. Zero markers still withhold pose; reprojection, freshness, alignment and pose-agreement checks remain active.

## OptiTrack terminal policy validation (2026-10-15)

The second inside-path flight lost qualified optical poses near the camera-height
cutoff and reached `marker_lost_trial_failed` without a termination request.
Read-only replay of both preserved flight bags compares the old dual-height gate
with the OptiTrack gate in `data/manifests/optitrack-terminal-replay-20261015.json`.
For the second bag, the new lifecycle requests termination at OptiTrack camera
height 0.1926 m with no qualified marker. This is counterfactual lifecycle replay,
not a new closed-loop flight or physical-contact certification.

Focused tests cover threshold boundaries, stale/missing/nonfinite height, pad
frame/mount conversion, pilot takeover, invalid alignment, phase restrictions
and the service-worker recheck. Localhost integration uses the real planner,
router, MUX and safety with a mock FCU: high-altitude marker loss still latches
failed hold; low-height loss issues one generic safety termination and waits for
disarm. No test sends commands to the hardware master.

The hardware stack selects `LANDING_TRAJECTORY_RESTART_ON_START=true`: every
planner invocation archives the exact old progress as
`flight_logs/inside20_progress.previous.<time_ns>.json` and creates a fresh queue
at the first selected reference, even after an interrupted or completed run.
Recorded bags remain unchanged. Within one invocation each successful landing
advances to the next reference; failures still halt that invocation. Startup
never starts motion: a fresh prepared pilot OFFBOARD edge is still required.
The generic launch default is false, retaining durable resume/refusal behavior
for callers that do not select the hardware startup policy.

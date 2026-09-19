# Manual takeoff → OFFBOARD landing trial

Current hardware profile uses **OptiTrack for all flight estimation**. Marker measurements qualify descent and landing height; the controller uses OptiTrack transformed into the session pad frame. The shared vision mux, safety response and MAVROS stay stack-neutral. No persisted global pad location is loaded.

Start the existing camera, OptiTrack, MAVROS, perception and flight-safety modules with the vision mux selected to `/vrpn_client_node/pure/pose`. Do not run another local controller publishing `/local_controller/setpoint_raw/local`. The trial refuses a competing publisher at startup and rejects preparation unless live vision matches OptiTrack and safety is OK.

On Jetson, from `~/drone-stack-docker`, use one entrypoint (may start before takeoff):

```bash
bash scripts/planner_aruco-landing.sh
# Optional observation-only run instead:
bash scripts/planner_aruco-landing.sh dry_run:=true
```

There is no separate start service/script in the normal operator flow. The planner prepares in the background and streams current-position holds while the pilot retains manual control. It never arms or selects OFFBOARD. For the default `force_disarm` finish policy, preparation requires the generic safety termination interface to be enabled. The ArUco stack opts in through runtime.env; other stacks remain disabled. The optional `landing_finish_mode:=auto_land` policy instead checks COM_DISARM_LAND > 0. `/landing/trial/status` reports `offboard_entry_ready`; at least one second of healthy preparation is required.

1. Start on the ground, take off manually in POSCTL, then select OFFBOARD once prepared. Ground/disarmed entry or unprepared entry does not start approach. Starting the program when already OFFBOARD does not start a mission either; a new pilot mode edge is required.
2. Capture the current altitude and heading on OFFBOARD entry. Approach the known OptiTrack global XY=(0,0), holding entry altitude. XY velocity is proportional to horizontal error (gain 0.8/s), capped at a combined 0.5 m/s. Final transformed commands are independently capped to 0.5 m/s as well.
3. Pre-OFFBOARD marker observations never count toward descent qualification. Once a valid new marker is seen during approach, stop horizontal approach and hold altitude while validating observations for 1 s; then descend. Missing observations reset qualification. Session alignment may be prepared by the estimator beforehand, but cannot trigger or redirect approach.
4. Descend with the existing PD controller and pad-relative yaw target 0 degrees. Horizontal command cap 0.5 m/s, descent 0.3 m/s, yaw-rate cap 0.35 rad/s (20.1 degrees/s). Marker validity requires at least 3 inliers, fresh poses and agreement with OptiTrack within 0.15 m / 12 degrees.
5. Invalid marker observations brake descent immediately. Loss for 0.5 s latches failed position hold. Rediscovery never resumes the trial in OFFBOARD. During airborne AUTO.LAND, loss requests POSCTL. Leaving OFFBOARD manually and deliberately re-entering after preparation starts a new trial; no separate reset command is needed.
6. Default finish policy is now `force_disarm`, explicitly requested by the operator. Only after entering DESCEND, a valid marker camera height in [0, 0.20] m AND fresh OptiTrack-derived camera height in [0, 0.25] m trigger a request to `/flight_safety_response/request_termination`. OptiTrack height is `(inverse(Global←Pad) × Global←Body × Body←Camera)[2,3]`, not global Z. Existing marker freshness, pose agreement and inlier gates remain active. The request worker rechecks both conditions and armed OFFBOARD before calling safety. No averaging/dwell is added to these thresholds.
7. The common safety node independently requires opt-in, fresh connected armed OFFBOARD state, no pilot manual override, and an admitted fresh Normal stream. Safety alone sends MAV_CMD_COMPONENT_ARM_DISARM (400), param1=0, param2=21196. This stops motors before contact; it does not assert physical touchdown. The planner waits for armed=false before COMPLETE. Rejected or unconfirmed requests fail into hold; they are not repeatedly fired.
8. Safety's existing KILL latch remains active after success. Restart safety and planner while disarmed before another flight. The LED therefore retains the existing solid-red automatic KILL meaning. Optional `landing_finish_mode:=auto_land` retains the former PX4 landing handoff.

Configuration/code changes require restarting safety and planner; editing files does not change running Python processes. Restart on the ground while disarmed. No service in this document should be manually invoked as a preflight check: the termination service is an actuator, not a query.

A frozen pad transform does not expire because marker messages stop. Explicit alignment reset or reference change invalidates an active trial. Moving vehicle samples still expire. These are command limits, not guarantees on measured vehicle speed.

## LEDs

Existing priorities are preserved: manual KILL with RC OFFBOARD = red/green blink; automatic KILL = solid red; emergency LAND = amber/green. A healthy OFFBOARD failed-trial position hold with fresh setpoints = **green/white at 2 Hz**. Intentional normal AUTO.LAND = cyan blink. Missing commands and emergency conditions retain their existing higher-priority indications. The common LED consumes generic `/control/mission_status` JSON; dry-run status cannot change the LED.

## Recording and verification scope

The stack-owned flight-safety ARM recording profile includes trial status, command inputs, yaw, generic mission status, mux selection and MAVROS estimator/state topics. Start flight-safety before arming. It retains compressed detection images and webcam recording without adding raw-image subscriptions. The older manual observation audit script is not the launcher for this OFFBOARD experiment.

Validation: Focused lifecycle/velocity/LED/reference tests passed; isolated localhost integration used the real safety response, vision mux, trial and existing landing controller with a **mock FCU**. It also checked pre-arm startup with visible markers, preparation without start/reset service calls, a fresh post-OFFBOARD marker dwell, the final XY cap, approach altitude, marker dwell, loss-to-hold (~0.513 s), no automatic retry, AUTO.LAND handoff, ground+disarm completion, AUTO.LAND loss→POSCTL and sole safety/MUX publishers. This is not a PX4 flight-dynamics simulation or proof of real-flight performance. No real vehicle was armed or commanded during these tests.

## Stack-owned geofence

ArUco reads `stack-assets/aruco-landing-jetson/config/geofence.yaml`: enabled_axes=[x,y], XY bounds ±2.5 m, WARN margin 0.4 m. Both Z upper/lower boundary checks are disabled. RViz shows an XY rectangle at the current vehicle height, not a ceiling/floor. Pose liveness and consistency checks remain active.

Risk-aware reads `stack-assets/d435i-voxblox/config/geofence.yaml`: enabled_axes=[x,y,z], XY ±2.5 m, Z [-0.5,2.0] m and margin 0.4 m, identical to its prior settings. The common safety module accepts `geofence_config` and contains no stack-specific axis policy. Each stack selects its file through runtime.env; no image rebuild is needed. Restart flight-safety while disarmed to apply.

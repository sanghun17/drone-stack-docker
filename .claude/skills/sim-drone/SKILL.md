---
name: sim-drone
description: Drone control (teleport, move, hold, status). Usage examples - /sim-drone status, /sim-drone teleport 0 0 2, /sim-drone move 2 1 2 0.5
user_invocable: true
---

# Drone Control

Control the drone via initialize_simulator's dynamic_reconfigure and teleport services.
Based on `automation_experiment.py` Phase B (`phase_b_initialize_drone`).

## CRITICAL: container era — initialize_simulator lives in `drone-stack-sim-x86`

`initialize_simulator.py` (the node this skill drives — services + dynamic_reconfigure) now runs
INSIDE the `drone-stack-sim-x86` container, brought up with:
```bash
bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_init_sim.sh
```
(equivalent: `cd /home/ml/drone-stack-docker && ./setup.sh run sim-x86 planner/risk-aware-sim/run_init_sim.sh`)
— one blocking tmux pane, same pattern as every other sim-x86 node. `run_init_sim.sh` already
handles container creation/start + Ctrl+C teardown internally; don't reproduce `docker exec`
by hand for THAT part. roscore/UE4/airsim_node stay on the HOST (tmux window 0) — only the
initializer node moved. Normally this is brought up as part of `/sim-start`.

Every `rosservice`/`rosparam`/`rostopic`/`rosnode`/`dynparam` call below, on the other hand,
IS hand-rolled through `docker exec` — these are ad-hoc introspection/control calls, not a
launch, so there's no run_*.sh wrapper for them. Route them into the same container by
convention (the old `~/risk-aware_planning/devel` host workspace this used to rely on is going
away, and some of this stack's other services do carry custom types — e.g. `/jax/*` — so the
container-exec habit needs to be uniform, not case-by-case). Pattern used throughout this file:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && <command>'
```

## Prerequisites Check (run before any command)
```bash
# 1. initialize_simulator alive?
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice list 2>/dev/null | grep -q "teleport_to_position"'

# 2. gt_odom data available?
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /gt_odom -n 1 > /dev/null 2>&1'

# 3. Scenario config must be loaded (platform=sim)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam get /system/platform 2>/dev/null'
```
If (1) or (2) fails → **WARN user**: "Run /sim-start first"
If (3) fails or returns empty → run (same container, same pattern):
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && /work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/config/load_config.sh sim_gt'
```

## Commands

### `status` (or no arguments)
Print current drone position and velocity:
```bash
echo "=== Drone Status ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1
echo "=== Velocity ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/twist/twist/linear -n 1' 2>&1
```

### `teleport <x> <y> <z> [yaw]`
Instant teleport (from Phase B Step 2). Default yaw=0.

```bash
# 1. Disarm for safe teleport (dynparam-triggered — actual disarm is a no-op now,
#    arming was a sim+mavros/PX4-SITL concept removed from initialize_simulator.py;
#    the field is still a valid dynamic_reconfigure key, harmless to set)
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'disarm_drone': true}\"" 2>/dev/null
sleep 0.3

# 2. Disable position hold during teleport
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'move_to_xyz': false}\"" 2>/dev/null
sleep 0.3

# 3. Set teleport target
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/x <x>'
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/y <y>'
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/z <z>'
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/yaw <yaw>'

# 4. Call teleport service (double for stability, from Phase B)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 2
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 1

# 5. Engage position hold at new position
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'target_x': <x>, 'target_y': <y>, 'target_z': <z>, 'target_yaw': <yaw>, 'move_to_xyz': true}\""
```

#### Verify (from Phase B Step 4)
Check GT position matches target within 0.6m tolerance, 5 consecutive stable checks:
```bash
# Poll GT odom (via container) and check distance to target
for i in $(seq 1 10); do
  POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose -n 1' 2>&1)
  X=$(echo "$POS" | grep -A1 "position:" | grep "x:" | awk '{print $2}')
  Y=$(echo "$POS" | grep -A1 "position:" | grep "y:" | awk '{print $2}')
  Z=$(echo "$POS" | grep "z:" | head -1 | awk '{print $2}')

  # Calculate distance (use python for float math — pure host math, no ROS needed)
  DIST=$(python3 -c "import math; print(f'{math.sqrt(($X-<target_x>)**2 + ($Y-<target_y>)**2 + ($Z-<target_z>)**2):.3f}')")
  echo "Check $i: pos=($X, $Y, $Z) dist=${DIST}m"

  python3 -c "exit(0 if $DIST < 0.6 else 1)" && echo "Within tolerance" || echo "Still moving..."
  sleep 1
done
```

### `move <x> <y> <z> [yaw]`
Smooth movement via velocity control (from Phase B Step 3). Default yaw=0.

**IMPORTANT: Must disable move_to_xyz first, then re-enable with new target.**
If you just set new target without toggling, the old target stays active.

```bash
# 1. Disable current tracking
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'move_to_xyz': false}\"" 2>/dev/null
sleep 1

# 2. Set new target and enable tracking
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'target_x': <x>, 'target_y': <y>, 'target_z': <z>, 'target_yaw': <yaw>, 'move_to_xyz': true}\""
```

#### Verify
Same GT position check as teleport — wait for convergence within 0.6m.
**NOTE:** `move` is slower than teleport. Max wait 60s (from Phase B).

If other nodes (like LA-Planner control_bridge) are also sending velocity commands to AirSim,
movement may not work. Kill competing control nodes first (via the same container — wherever
they actually run, the ROS master is shared so `rosnode kill` reaches them regardless):
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /la_control_bridge' 2>/dev/null
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /control_bridge' 2>/dev/null
```

### `hold`
Hold current position (prevent drift during node launches):
```bash
# Get current position
POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
CUR_X=$(echo "$POS" | grep "x:" | awk '{print $2}')
CUR_Y=$(echo "$POS" | grep "y:" | awk '{print $2}')
CUR_Z=$(echo "$POS" | grep "z:" | awk '{print $2}')

docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'target_x': $CUR_X, 'target_y': $CUR_Y, 'target_z': $CUR_Z, 'move_to_xyz': true}\""
```

### `release`
Stop position hold (let drone be controlled by other nodes):
```bash
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'move_to_xyz': false}\""
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Teleport doesn't move drone | position hold overriding | Disable move_to_xyz first |
| Move command ignored | another control node sending commands | Kill /la_control_bridge, /control_bridge |
| GT odom timeout | sensor publisher or odom publisher down (`run_sensor_pub.sh`) | Check /sim-start health |
| "Failed to connect to AirSim" | IP mismatch | Check airsim_ip in scenarios/sim_*.yaml |
| Position oscillates | PD gains too high or competing controllers | Release other controllers first |
| `docker exec` itself fails / no such container | `drone-stack-sim-x86` not up | `bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_init_sim.sh` (or `/sim-start`) first |

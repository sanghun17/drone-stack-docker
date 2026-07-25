---
name: sim-takeoff
description: Initialize drone position, launch planning nodes, and start autonomous exploration
user_invocable: true
---

# Simulation Takeoff Skill

Teleport drone to start position, launch all planning/control nodes, and begin autonomous exploration.

**Container era**: FAST-LIVO, voxblox, exploration, JAX, and SO(3) control all run inside the
`drone-stack-sim-x86` container now (module `odometry/fast-livo-sim` / `planner/risk-aware-sim`) —
Phase C below launches each one via its `run_*.sh` in its own tmux window (auto-enters the
container, sources everything it needs, forwards Ctrl+C as SIGINT — don't prepend
`source devel/setup.bash`). Service calls into that stack (teleport, planner/control toggles,
`/jax/*` topic checks) go through `docker exec` — the old bare-host devel workspace they used to
resolve against is gone, so a bare host shell can't resolve their custom msg/srv anymore.

## Prerequisites
- `/sim-start` must have been run successfully (infrastructure running)
- roscore, Unreal, AirSim, sensor publisher, initialize_simulator all active

## Procedure

### Phase B: Initialize Drone Position

#### Step 1: Check the localization axis (read-only — scenario yaml is the SoT)
```bash
rosparam get /system/localization
```
Use this to determine the gt vs vio flow. Never `rosparam set` the /system axis
enums at runtime — they come from `load_config.sh <sim_gt|sim_vio>`.

#### Step 2: Teleport drone to start position
```bash
# Set teleport target (default: origin, 3m altitude)
rosparam set /system/sim/teleport/x 0.0
rosparam set /system/sim/teleport/y 0.0
rosparam set /system/sim/teleport/z 3.0
rosparam set /system/sim/teleport/yaw 0.0

# Trigger teleport via service (runs inside the container — initialize_simulator.py)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 2
# Double teleport for stability
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 2
```

#### Step 3: Fine-tune position with move_to_xyz via dynamic reconfigure
```bash
# Set target and enable continuous tracking
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'target_x': 0.0, 'target_y': 0.0, 'target_z': 3.0, 'target_yaw': 0.0, 'move_to_xyz': true}\""
```

#### Step 4: Verify position
```bash
# Check drone position from GT odometry (plain nav_msgs/Odometry — reads fine from the host)
timeout 5 rostopic echo /gt_odom/pose/pose/position -n 1
```
Position should be within 0.6m of target.

### Phase C: Launch Planning Nodes

Each node launches in its own tmux window via that module's `run_*.sh`, matching the current live
layout (`infra`, `init_sim` from `/sim-start`, then one window per node below).

#### Step 5: Launch nodes in order
For **vio** localization:
```bash
tmux new-window -t risk_aware_planning -n fast_livo 2>/dev/null || true
tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m
sleep 3
```

For **both gt and vio**:
```bash
# Uncertainty VoxBlox (mapping only — separate from exploration below)
tmux new-window -t risk_aware_planning -n voxblox 2>/dev/null || true
tmux send-keys -t risk_aware_planning:voxblox "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_voxblox.sh" C-m
sleep 3

# Exploration Planner
tmux new-window -t risk_aware_planning -n exploration 2>/dev/null || true
tmux send-keys -t risk_aware_planning:exploration "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_exploration.sh" C-m
sleep 3

# JAX MPPI Local Planner (first run ~80s — JAX JIT compile)
tmux new-window -t risk_aware_planning -n jax 2>/dev/null || true
tmux send-keys -t risk_aware_planning:jax "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_jax.sh" C-m
sleep 5

# Eval Data Recording
tmux new-window -t risk_aware_planning -n eval 2>/dev/null || true
tmux send-keys -t risk_aware_planning:eval "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_eval.sh" C-m
sleep 1

# SO(3) Control Bridge (traj_server + so3_control_bridge — provides /control_bridge/toggle_running)
tmux new-window -t risk_aware_planning -n so3 2>/dev/null || true
tmux send-keys -t risk_aware_planning:so3 "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_so3.sh" C-m
sleep 3
```

#### Step 6: Verify nodes are running
```bash
# Check critical nodes
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode list' 2>/dev/null | grep -E "planner|jax|control_bridge|voxblox"

# Check JAX trajectory publishing — ⚠ the sim runs under use_sim_time: `rostopic hz` reports
# nothing useful here (measured pitfall). Use `rostopic echo -n 1` to confirm data is flowing.
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 15 rostopic echo /jax/optimal_trajectory -n 1'
```

### Phase D: Start Exploration

#### Step 7: Start planner
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /planner/planner_node/toggle_running "data: true"'
```

#### Step 8: Enable control bridge
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /control_bridge/toggle_running "data: true"'
```

#### Step 9: Start evaluation
```bash
rosparam set /evaluation_running true
```

For AirSim modes, also:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /eval_data_node/start_evaluation "{}"'
```

### Health Check
```bash
# Verify drone is moving (velocity should be non-zero) — plain nav_msgs/Odometry, host is fine
timeout 5 rostopic echo /gt_odom/twist/twist/linear -n 3

# Verify trajectory being published — ⚠ use_sim_time: `rostopic hz` is useless here, use echo
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 10 rostopic echo /jax/optimal_trajectory -n 1'

# Verify control commands being sent — same use_sim_time caveat
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 10 rostopic echo /control_bridge/executing_trajectory -n 1'
```

## For LA-Planner Testing
When testing LA-Planner instead of the risk-aware planner, replace Phase C and D with:
```bash
docker exec -it drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && roslaunch la_planner_bridge la_planner_airsim.launch auto_trigger:=true'
```
Unlike the `run_*.sh`-based launches above, this path has no host-side SIGINT trap — Ctrl+C on the
pane is not reliable. Stop it with:
```bash
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch la_planner_bridge la_planner_airsim.launch"
```
See `/sim-la-planner` for the full LA-Planner procedure (kill → teleport → LIVO restart →
convergence check <2m → 5-step launch) — this is just the launch/stop command, not a substitute
for that skill.

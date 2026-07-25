---
name: sim-stop
description: Stop exploration, disable control, and clean up nodes
user_invocable: true
---

# Simulation Stop Skill

Gracefully stop the running experiment and clean up nodes. Infrastructure (roscore, Unreal, AirSim) stays running for next iteration.

**Container era**: the nodes this skill stops/disables (control bridge, planner, voxblox, FAST-LIVO,
eval, `initialize_simulator`) all run inside the `drone-stack-sim-x86` container now — `rosservice`
calls to them go through `docker exec` (the old bare-host devel workspace this used to run against
is gone, so a bare host shell can't resolve their custom msg/srv anymore), and stopping a node pane
means Ctrl+C on its tmux window, not `rosnode kill` from a bare host shell.

## Procedure

### Step 1: Disable control bridge (stop sending velocity commands)
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /control_bridge/toggle_running "data: false"'
```

### Step 2: Stop planner
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /planner/planner_node/toggle_running "data: false"'
```

### Step 3: Stop evaluation
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
rosparam set /evaluation_running false
```

### Step 4: Enable position hold (prevent drift)
```bash
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'move_to_xyz': false}\""
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/toggle_setpoint_publishing "data: true"'
```

### Step 5: Stop experiment nodes (keep JAX + SO(3) control running)
Primary: Ctrl+C the tmux window running each node — its `run_*.sh` wrapper's own trap forwards
that into the container as SIGINT for a clean node teardown. Backup, in case a pane already died
and left an orphaned node (or the trap didn't fire): `docker exec ... pkill -INT` using the exact
`roslaunch ...` match string from that module's own `run_*.sh` (`__M=` variable there).

```bash
# FAST-LIVO (odometry/fast-livo-sim/run.sh)
tmux send-keys -t risk_aware_planning:fast_livo C-c 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null

# Voxblox mapping (planner/risk-aware-sim/run_voxblox.sh)
tmux send-keys -t risk_aware_planning:voxblox C-c 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch active_3d_planning_app_reconstruction uncertainty_voxblox.launch" 2>/dev/null

# Exploration planner (planner/risk-aware-sim/run_exploration.sh)
tmux send-keys -t risk_aware_planning:exploration C-c 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch active_3d_planning_app_reconstruction exploration_planner.launch" 2>/dev/null

# Eval data recording (planner/risk-aware-sim/run_eval.sh)
tmux send-keys -t risk_aware_planning:eval C-c 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch" 2>/dev/null

# Vision pose throttle (VIO path only — no dedicated run_*.sh/tmux window, may not exist in gt mode)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /vision_pose_throttle 2>/dev/null || true'

sleep 1
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && echo y | rosnode cleanup' 2>/dev/null
```

Note: JAX MPPI controller (`jax` tmux window) and SO(3) control bridge (`so3` tmux window —
provides `/control_bridge/toggle_running`) are **persistent** — do NOT kill them unless doing a
full shutdown (`/sim-kill`).

### Step 6: Verify cleanup
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode list' 2>/dev/null | grep -E "planner|voxblox|eval|mapping"
```
Should return empty (all experiment nodes killed).

## Full Shutdown (kill everything including infrastructure)
Only do this when completely done with all experiments — prefer `/sim-kill`, which also cleans up
the container-side ROS nodes explicitly (see that skill) instead of relying on the tmux
kill-session -> trap chain alone:
```bash
tmux kill-session -t risk_aware_planning
```
This kills all panes (host side), which in turn should tear down every containerized node's
`roslaunch`/python process via its `run_*.sh` trap, plus Unreal, AirSim, roscore — but if you
want a *verified* clean slate, run `/sim-kill` instead of this one-liner.

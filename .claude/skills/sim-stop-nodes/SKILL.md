---
name: sim-stop-nodes
description: Stop planner/mapping nodes only (keep infrastructure). Usage - /sim-stop-nodes
user_invocable: true
---

# Stop Planner Nodes (Keep Infrastructure)

Kill all planner/mapping nodes while keeping infrastructure (roscore, Unreal, AirSim, sensors, initialize_simulator) alive. Based on `automation_experiment.py` Phase A (`phase_a_stop_nodes`).

Use this between experiment iterations or when switching planners.

**Container era**: FAST-LIVO, voxblox, exploration, JAX, SO(3) control, and eval all run inside the
`drone-stack-sim-x86` container now (module `odometry/fast-livo-sim` / `planner/risk-aware-sim`).
`network_mode:host` (no `pid: host`) means the container has its own PID namespace — a host-side
`ps aux`/`pkill` sweep can no longer see or kill them at all. Stopping a node pane means Ctrl+C on
its tmux window (the `run_*.sh` wrapper's trap forwards that into the container as SIGINT); the
backup path for an orphaned node is `docker exec ... pkill -INT` using the exact `roslaunch ...`
string from that module's own `run_*.sh` (`__M=` variable there). Sensors/`initialize_simulator`
stay up (they're the Step 4/5 panes from `/sim-start`, untouched here).

## CRITICAL: ROS Environment
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311
export ROS_IP=192.168.50.12
source /opt/ros/noetic/setup.bash
```

## Difference from /sim-kill

| | /sim-stop-nodes | /sim-kill |
|---|---|---|
| Planner nodes | Kill | Kill |
| FAST-LIVO2 | Kill | Kill |
| roscore | **Keep** | Kill |
| Unreal | **Keep** | Kill |
| AirSim | **Keep** | Kill |
| Sensors | **Keep** | Kill |
| init_simulator | **Keep** | Kill |
| tmux session | **Keep** | Kill |

## Procedure

### Step 1: Disable control services (prevent drift)
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /control_bridge/toggle_running "data: false"' 2>/dev/null
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: false"' 2>/dev/null
```

### Step 2: Re-engage position hold (prevent crash during shutdown)
`/gt_odom` is a plain `nav_msgs/Odometry` topic — reads fine straight from the host. The
dynamic_reconfigure write needs the container (custom-generated Config class):
```bash
POS=$(timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1 2>&1)
CUR_X=$(echo "$POS" | grep "x:" | awk '{print $2}')
CUR_Y=$(echo "$POS" | grep "y:" | awk '{print $2}')
CUR_Z=$(echo "$POS" | grep "z:" | awk '{print $2}')
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'target_x': $CUR_X, 'target_y': $CUR_Y, 'target_z': $CUR_Z, 'move_to_xyz': true}\"" 2>/dev/null
```

### Step 3: Kill all planner/mapping nodes

Primary — Ctrl+C each node's tmux window (`run_*.sh`'s own trap relays it into the container):
```bash
for w in fast_livo voxblox exploration jax so3 eval; do
  tmux send-keys -t risk_aware_planning:$w C-c 2>/dev/null
done
sleep 1
```

Backup — in case a pane already died and left an orphaned node, `pkill -INT` inside the container
using the exact `__M=` match string from each module's own `run_*.sh`:
```bash
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch active_3d_planning_app_reconstruction uncertainty_voxblox.launch" 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch active_3d_planning_app_reconstruction exploration_planner.launch" 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch local_controller ours_jax.launch" 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch local_controller ours_so3_stack.launch" 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch" 2>/dev/null
sleep 2
```

### Step 4: Cleanup zombies
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && echo y | rosnode cleanup' 2>/dev/null
```

### Step 5: Verify

Only infrastructure nodes should remain:
```bash
echo "=== Remaining Nodes ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode list' 2>/dev/null

echo "=== Infrastructure Health ==="
timeout 5 rostopic echo /gt_odom -n 1 > /dev/null 2>&1 \
  && echo "[OK] GT odom (infra alive)" || echo "[WARN] GT odom down"
timeout 5 rostopic echo /camera/depth/image_raw/header -n 1 > /dev/null 2>&1 \
  && echo "[OK] Depth (infra alive)" || echo "[WARN] Depth down"
rosservice list 2>/dev/null | grep -q "teleport_to_position" \
  && echo "[OK] init_simulator (infra alive)" || echo "[WARN] init_simulator down"
```

Expected remaining nodes: `/rosout`, `/airsim_node`, `/airsim_rgb_depth_publisher`, `/airsim_gt_odom_publisher`, `/initialize_simulator`, `/clock_relay`, TF publishers, nodelet managers.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Drone falls after stop | Position hold not engaged — run Step 2 manually |
| Infrastructure nodes also died | Something killed the tmux session — run /sim-start |
| Zombie nodes in rosnode list | `echo y \| rosnode cleanup` again (inside the container, see Step 4) |
| Node still alive after Ctrl+C | Pane may already have been dead — use the Step 3 backup `docker exec ... pkill -INT` for that node |

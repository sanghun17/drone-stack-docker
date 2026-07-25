---
name: sim-run
description: Run a full experiment pipeline. Usage - /sim-run la-planner, /sim-run ours (future)
user_invocable: true
---

# Run Experiment Pipeline

Combines unit skills into a full experiment pipeline. Argument selects which planner to run.

## CRITICAL: container era

`fast_livo`, voxblox, the exploration planner, `local_controller` (JAX/SO3) and `initialize_simulator`
now run INSIDE the `drone-stack-sim-x86` container via `modules/{odometry/fast-livo-sim,planner/risk-aware-sim}/run_*.sh`
— each one launched with a plain `bash <script>` in its own tmux pane (the script itself handles
container start + `docker exec` entry + Ctrl+C teardown; never reproduce `docker exec` by hand for
a launch). roscore, UE4, `airsim_node` (conda `airsim` env) stay on the HOST, tmux window 0 —
untouched by this migration.

All ad-hoc `rostopic`/`rosservice`/`rosnode`/`ps aux` introspection below (no run_*.sh wrapper
exists for these — they're not launches) goes through `docker exec` into the same container by
convention, since the old `~/risk-aware_planning/devel` host workspace this used to rely on is
going away:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && <command>'
```
`ps aux`/`kill` for a process now running INSIDE the container must also go through `docker exec`
(container has its own PID namespace — a host-side `ps`/`kill -9 <pid>` targets the wrong PID).

## Prerequisites
Infrastructure must be running (`/sim-start`). Verify:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic list > /dev/null 2>&1' && echo "roscore OK" || echo "FAIL: run /sim-start"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /gt_odom -n 1 > /dev/null 2>&1' && echo "sensors OK" || echo "FAIL: run /sim-start"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice list 2>/dev/null | grep -q "teleport_to_position"' && echo "init_sim OK" || echo "FAIL: run /sim-start"
```

---

## `la-planner`

Full pipeline: stop old nodes → drone position → FAST-LIVO2 → LA-Planner → enable exploration.

> **la_planner_bridge**: unlike FAST-LIVO2/voxblox/exploration/jax/so3/sensor-pub/init-sim/eval
> above, no dedicated `modules/planner/risk-aware-sim/run_*.sh` wraps it — but it runs fine inside
> `drone-stack-sim-x86` (package resolves, built under `ws/risk-aware/build/la_planner_bridge`; it
> lives at `ws/risk-aware/src/risk_aware_planning/la_planner/la_planner_bridge`, same bind-mounted
> tree). It's launched with a one-line `docker exec` instead of a `bash <script>` pane. **`/sim-la-planner`
> is the source of truth for the full kill→teleport→LIVO-restart→convergence→launch procedure** —
> Step 4 below only shows the containerized launch line itself. This `docker exec` path has **no
> host-side SIGINT trap** (unlike the run_*.sh scripts) — Ctrl+C in the tmux pane won't reliably
> stop it; use the `pkill -INT` line below instead. FAST-LIVO2 (Step 3) IS in the confirmed table
> and is converted to `run.sh` below.

### Step 1: Clean previous nodes (`/sim-stop-nodes`)
```bash
# Disable controls
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /control_bridge/toggle_running "data: false"' 2>/dev/null
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: false"' 2>/dev/null

# Kill non-infrastructure nodes (via container — rosnode reaches whatever's registered
# with the shared master regardless of where the client itself runs)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && INFRA_NODES="rosout airsim_node airsim_rgb_depth_publisher airsim_gt_odom_publisher initialize_simulator clock_relay map_to_odom imu_to_base airsim_sensor_left rgb_optical_to_depth_optical nodelet_manager nodelet1 pcl_manager voxel_grid rviz"; for node in $(rosnode list 2>/dev/null); do SKIP=false; for infra in $INFRA_NODES; do echo "$node" | grep -q "$infra" && SKIP=true && break; done; $SKIP && continue; rosnode kill $node 2>/dev/null & done; wait; echo "y" | rosnode cleanup 2>/dev/null'

# Kill process trees — these patterns predate containerization, but all of them now run
# inside drone-stack-sim-x86: fastlivo_mapping/"roslaunch fast_livo"/exploration_node/
# traj_server via the confirmed run_*.sh table, la_sensor_bridge/la_control_bridge/
# "roslaunch la_planner_bridge"/feature_accumulator via the same la_planner_bridge
# container launch as Step 4 (see note above — live-resolved 2026-07-25). Run this
# inside the container accordingly.
docker exec drone-stack-sim-x86 bash -lc 'for pattern in "fastlivo_mapping" "roslaunch fast_livo" "exploration_node" "traj_server" "la_sensor_bridge" "la_control_bridge" "roslaunch la_planner_bridge" "feature_accumulator"; do pids=$(ps aux | grep "$pattern" | grep -v grep | awk "{print \$2}"); [ -n "$pids" ] && echo "$pids" | xargs kill -9 2>/dev/null; done'
sleep 1
```

### Step 2: Position drone (`/sim-drone teleport`)
```bash
TARGET_X=0.0; TARGET_Y=0.0; TARGET_Z=2.0; TARGET_YAW=0.0

# Check the scenario config is loaded (read-only — never rosparam set the
# /system axis enums; they come from load_config.sh <sim_gt|sim_vio>)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam get /system/platform' 2>/dev/null | grep -q "sim" || { echo "config not loaded — run load_config.sh sim_gt (or sim_vio) first"; exit 1; }

# Disarm + disable hold
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'disarm_drone': true}\"" 2>/dev/null
sleep 0.3
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'move_to_xyz': false}\"" 2>/dev/null
sleep 0.3

# Teleport
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/x $TARGET_X"
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/y $TARGET_Y"
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/z $TARGET_Z"
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/yaw $TARGET_YAW"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 2
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 1

# Position hold
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'target_x': $TARGET_X, 'target_y': $TARGET_Y, 'target_z': $TARGET_Z, 'target_yaw': $TARGET_YAW, 'move_to_xyz': true}\"" 2>/dev/null
```

#### Verify position
```bash
sleep 3
POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
X=$(echo "$POS" | grep "x:" | awk '{print $2}')
Y=$(echo "$POS" | grep "y:" | awk '{print $2}')
Z=$(echo "$POS" | grep "z:" | awk '{print $2}')
DIST=$(python3 -c "import math; print(f'{math.sqrt(($X-$TARGET_X)**2 + ($Y-$TARGET_Y)**2 + ($Z-$TARGET_Z)**2):.3f}')")
echo "Drone at ($X, $Y, $Z) dist=${DIST}m from target"
```
Should be < 0.6m.

### Step 3: Start FAST-LIVO2 (`/sim-fast-livo start`)
```bash
# Kill duplicate windows first, then create fresh
for win in $(tmux list-windows -t risk_aware_planning -F "#{window_index}:#{window_name}" 2>/dev/null | grep "fast_livo" | awk -F: '{print $1}' | sort -rn); do
  tmux kill-window -t risk_aware_planning:$win 2>/dev/null
done
tmux new-window -t risk_aware_planning -n fast_livo
tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m
```
(equivalent: `bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh` blocking directly
in the pane, or `cd /home/ml/drone-stack-docker && ./setup.sh run sim-x86 odometry/fast-livo-sim/run.sh`
— no `source devel/setup.bash` prefix needed, the script sources inside the container itself.)

#### Verify
```bash
for i in $(seq 1 5); do
  sleep 4
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1 > /dev/null 2>&1' && echo "FAST-LIVO2 OK" && break
  echo "waiting... ($((i*4))s)"
done
```

### Step 4: Start LA-Planner (`/sim-la-planner start`)
```bash
# Kill duplicate windows first, then create fresh
for win in $(tmux list-windows -t risk_aware_planning -F "#{window_index}:#{window_name}" 2>/dev/null | grep "la_planner" | awk -F: '{print $1}' | sort -rn); do
  tmux kill-window -t risk_aware_planning:$win 2>/dev/null
done
tmux new-window -t risk_aware_planning -n la_planner
# No dedicated run_*.sh (see note above) — one-line docker exec instead. Full procedure: /sim-la-planner.
tmux send-keys -t risk_aware_planning:la_planner "docker exec -it drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && roslaunch la_planner_bridge la_planner_airsim.launch auto_trigger:=true'" C-m
```
Stop (no host-side SIGINT trap here, unlike the run_*.sh scripts — Ctrl+C in the pane is not
reliable, use this instead):
```bash
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch la_planner_bridge la_planner_airsim.launch"
```

#### Verify
```bash
for i in $(seq 1 5); do
  sleep 4
  ALL_OK=true
  docker exec drone-stack-sim-x86 bash -lc 'for node in exploration_node traj_server la_sensor_bridge; do ps aux | grep "$node" | grep -v grep | grep -v pgrep > /dev/null 2>&1 || exit 1; done' || ALL_OK=false
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /la/odom_world -n 1 > /dev/null 2>&1' || ALL_OK=false
  $ALL_OK && echo "LA-Planner OK" && break
  echo "waiting... ($((i*4))s)"
done
```

### Step 5: Enable exploration (`/sim-la-planner enable`)
```bash
# Release position hold
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosrun dynamic_reconfigure dynparam set /initialize_simulator \"{'move_to_xyz': false}\"" 2>/dev/null
sleep 0.5

# Enable control bridge
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: true"'
```

### Step 6: Final health check
```bash
echo "=== LA-Planner Experiment Health Check ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1 > /dev/null 2>&1' \
  && echo "[OK] FAST-LIVO2 odom" || echo "[FAIL] FAST-LIVO2"
docker exec drone-stack-sim-x86 bash -lc 'ps aux | grep "exploration_node" | grep -v grep > /dev/null 2>&1' \
  && echo "[OK] exploration_node" || echo "[FAIL] exploration_node"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /la/depth -n 1 > /dev/null 2>&1' \
  && echo "[OK] Depth bridge" || echo "[FAIL] Depth bridge"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /sdf_map/occupancy_local -n 1 > /dev/null 2>&1' \
  && echo "[OK] SDF map" || echo "[FAIL] SDF map"

docker exec drone-stack-sim-x86 bash -lc 'PID=$(ps aux | grep "exploration_node" | grep -v grep | awk "{print \$2}" | head -1); [ -n "$PID" ] && echo "[INFO] Memory: $(echo "scale=0; $(ps -p $PID -o rss= 2>/dev/null)/1024" | bc)MB"'

echo "================================================"
echo "LA-Planner experiment running!"
echo "Monitor: tmux attach -t risk_aware_planning"
echo "Stop: /sim-stop-nodes"
```

---

## `ours`

**→ `/sim-eval-ours` 스킬 사용** (sim-eval-ours.md — 2026-07-05 라이브 검증 완료).
checkpoint 배포 preflight → 인프라(vio 모드) → window 1 pane 준비 → `automation_experiment.py`
(Phase B~E 자동) → 오프라인 평가 → 판정 체크리스트까지 end-to-end 절차와 함정 모음 포함.

---
name: sim-la-planner
description: LA-Planner control (start, enable, verify, status, stop). Requires /sim-fast-livo running. Usage - /sim-la-planner start, /sim-la-planner enable, /sim-la-planner verify, /sim-la-planner go
user_invocable: true
---

# LA-Planner Control

Start, control, and monitor LA-Planner (comparison baseline planner).
LA-Planner uses FAST-LIVO2's visual features for localization-aware planning.

**Container era**: LA-Planner (`la_planner_bridge` + the rest of the `la_planner/` tree under
`ws/risk-aware/src/risk_aware_planning`) IS part of the same source tree bind-mounted into
`drone-stack-sim-x86` (module `planner/risk-aware-sim`), and IS built by that module's
`build_ws.sh` (`catkin build` over the whole workspace — no `CATKIN_IGNORE` excludes it; confirmed
by `devel/lib/exploration_manager/exploration_node` and `devel/lib/local_plan_manager/traj_server`
existing after a `build-ws` run). It just has **no dedicated `run_*.sh`** in
`planner/risk-aware-sim/module.yml`'s `run:` list (only voxblox/exploration/jax/eval/so3/sensor_pub/
init_sim do) — so it's launched via a one-line `docker exec` below, not `bash .../run_X.sh`. FAST-LIVO2
stays in `odometry/fast-livo-sim` per the `/sim-fast-livo` skill.

⚠ **Because there's no dedicated `run_*.sh`, there's no host-side trap forwarding Ctrl+C into the
container** (unlike fast-livo/voxblox/jax/so3, whose `run.sh`/`run_*.sh` each `trap INT` and do an
explicit `docker exec ... pkill -INT -f "$__M"` — see those scripts' headers). `docker exec` alone
does not reliably forward SIGINT (documented pitfall baked into every other run script in this
repo). So below, tmux Ctrl+C is sent for good measure but the **reliable** stop mechanism is an
explicit `docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch la_planner_bridge la_planner_airsim.launch"`.

## ⚠ MANDATORY START ORDER — do not skip, reorder, or shortcut

**① kill old LA-Planner → ② teleport drone → ③ restart FAST-LIVO2 → ④ verify GT vs LIVO
convergence (< 2m) → ⑤ launch LA-Planner.** Teleporting makes FAST-LIVO2's VIO estimate diverge
completely — "it already looks converged, skip the wait" is never valid. This order is preserved
byte-for-byte from the bare-metal procedure below; only *how* each step executes (container vs.
bare-metal) changed.

## CRITICAL: ROS Environment (container era)

Host tmux panes no longer need manual `ROS_MASTER_URI`/`ROS_IP` exports. For any ad-hoc
rostopic/rosservice/rosnode query from the host (all `status`/`verify` checks below), go through
the container — the old bare-metal tree's devel is gone, so custom msg/srv can't resolve from a
bare host shell anymore:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && <cmd>'
```
This is the default (risk-aware/LA-Planner/AirSim side: `/gt_odom`, `/la/*`, `/sdf_map/*`,
`/planning/*`, `initialize_simulator`). FAST-LIVO2's own topics (`/aft_mapped_to_init_odom`,
`/fast_livo/visual_features`) use `/work/ws/fast-livo-sim/devel/setup.bash` instead — used where
noted below (see also `/sim-fast-livo`).

⚠ The sim runs under `use_sim_time` — `rostopic hz` reports nothing useful here (measured
pitfall). Always use `rostopic echo -n 1` (as in every check below) to confirm data is actually
flowing, never rate counters.

## Prerequisites
```bash
# 1. Infrastructure running (/sim-start)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /gt_odom -n 1' > /dev/null 2>&1

# 2. FAST-LIVO2 running (/sim-fast-livo start)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1

# 3. Drone must be within map bounds (z ~-1.0 for Modern Livingroom ENU)
POS_Z=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position/z -n 1' 2>&1 | head -1)
echo "Drone z=$POS_Z"
```
If (1) fails → "Run /sim-start first"
If (2) fails → "Run /sim-fast-livo start first"
If (3) z outside [-2.5, 0.5] → Run `/sim-fast-livo preflight` then `/sim-fast-livo recover` to teleport + restart.
(Teleport requires `initialize_simulator` node — preflight ensures it's alive.)

## Commands

### `status`
Check LA-Planner components:
```bash
echo "=== LA-Planner Status ==="

# Core nodes (run INSIDE the container now)
for node in exploration_node traj_server la_sensor_bridge la_control_bridge; do
  docker exec drone-stack-sim-x86 bash -c "ps aux | grep '$node' | grep -v grep" > /dev/null 2>&1 \
    && echo "[OK] $node" || echo "[FAIL] $node"
done

# Bridge topics (actual data)
for topic in /la/odom_world /la/depth /la/sensor_pose; do
  docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo $topic -n 1" > /dev/null 2>&1 \
    && echo "[OK] $topic" || echo "[FAIL] $topic"
done

# SDF map building
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /sdf_map/occupancy_local -n 1' > /dev/null 2>&1 \
  && echo "[OK] SDF map" || echo "[FAIL] SDF map"

# Feature reception (dynamic mode)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /la/visual_features -n 1' > /dev/null 2>&1 \
  && echo "[OK] Visual features" || echo "[INFO] No visual features"

# Feature map health: check if FeatureMap is updating beyond seed
# Seed-only = cloud count stays constant (typically 24). If hits grow, dynamic features are arriving.
FEAT_LOG=$(tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "\[FeatureMap\] query" | tail -3)
if [ -n "$FEAT_LOG" ]; then
  echo "=== Feature Map ==="
  echo "$FEAT_LOG"
  CLOUD_COUNTS=$(echo "$FEAT_LOG" | grep -oP 'cloud=\K[0-9]+' | sort -u)
  if [ "$(echo "$CLOUD_COUNTS" | wc -l)" -eq 1 ] && [ "$(echo "$CLOUD_COUNTS")" -le 30 ]; then
    echo "[WARN] Feature map stuck on seed features only (cloud=$CLOUD_COUNTS) — FAST-LIVO may not be publishing /fast_livo/visual_features"
  else
    echo "[OK] Feature map updating (cloud counts: $(echo $CLOUD_COUNTS | tr '\n' ' '))"
  fi
else
  echo "[WARN] No FeatureMap query logs found"
fi

# FSM state
tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "FSM.*state:" | tail -3

# Memory check
PID=$(docker exec drone-stack-sim-x86 bash -c "ps aux | grep 'exploration_node' | grep -v grep | awk '{print \$1}'" | head -1)
if [ -n "$PID" ]; then
  RSS=$(docker exec drone-stack-sim-x86 ps -p $PID -o rss= 2>/dev/null)
  echo "[INFO] exploration_node memory: $(echo "scale=0; $RSS/1024" | bc)MB"
fi
```

### `verify`
Full health check: prerequisites → nodes → bridge topics → SDF map → features → FSM → memory → control state.
Run this to confirm everything is working before enabling.

```bash
echo "=== Prerequisites ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /gt_odom -n 1' > /dev/null 2>&1 \
  && echo "[OK] AirSim/sim running" || echo "[FAIL] No /gt_odom - run /sim-start first"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 \
  && echo "[OK] FAST-LIVO2 running" || echo "[FAIL] No FAST-LIVO2 - run /sim-fast-livo start first"

echo "=== Core Nodes ==="
for node in exploration_node traj_server la_sensor_bridge la_control_bridge; do
  docker exec drone-stack-sim-x86 bash -c "ps aux | grep '$node' | grep -v grep" > /dev/null 2>&1 \
    && echo "[OK] $node" || echo "[FAIL] $node"
done

echo "=== Bridge Topics ==="
for topic in /la/odom_world /la/depth /la/sensor_pose; do
  docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo $topic -n 1" > /dev/null 2>&1 \
    && echo "[OK] $topic" || echo "[FAIL] $topic"
done

echo "=== SDF Map ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /sdf_map/occupancy_local -n 1' > /dev/null 2>&1 \
  && echo "[OK] SDF map building" || echo "[FAIL] SDF map not building"

echo "=== Visual Features Pipeline ==="
# Check source (FAST-LIVO2) and relay (sensor_bridge) separately
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /fast_livo/visual_features -n 1' > /dev/null 2>&1 \
  && echo "[OK] FAST-LIVO2 publishing /fast_livo/visual_features" \
  || echo "[FAIL] FAST-LIVO2 NOT publishing features — likely IMU-LiDAR sync hang"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /la/visual_features -n 1' > /dev/null 2>&1 \
  && echo "[OK] sensor_bridge relaying /la/visual_features" \
  || echo "[FAIL] sensor_bridge NOT relaying features"

# Check FAST-LIVO2 for IMU-LiDAR sync errors (common hang cause)
SYNC_ERRORS=$(tmux capture-pane -t risk_aware_planning:fast_livo -p -J 2>&1 | grep -c "IMU and LiDAR not synced")
if [ "$SYNC_ERRORS" -gt 5 ]; then
  LAST_LOG_TIME=$(tmux capture-pane -t risk_aware_planning:fast_livo -p -J 2>&1 | grep -oP '\[\K[0-9]+\.' | tail -1 | tr -d '.')
  CLOCK=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rostopic echo /clock/clock/secs -n 1' 2>/dev/null)
  if [ -n "$LAST_LOG_TIME" ] && [ -n "$CLOCK" ]; then
    DELTA=$((CLOCK - LAST_LOG_TIME))
    if [ "$DELTA" -gt 30 ]; then
      echo "[FAIL] FAST-LIVO2 HUNG — last log ${DELTA}s ago, $SYNC_ERRORS sync errors. RESTART NEEDED!"
    fi
  fi
  echo "[WARN] FAST-LIVO2 has $SYNC_ERRORS IMU-LiDAR sync errors"
fi

# Check FeatureMap dynamic update (seed-only = problem)
FEAT_LOG=$(tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "\[FeatureMap\] query" | tail -1)
if [ -n "$FEAT_LOG" ]; then
  CLOUD=$(echo "$FEAT_LOG" | grep -oP 'cloud=\K[0-9]+')
  if [ "$CLOUD" -le 30 ]; then
    echo "[FAIL] FeatureMap stuck on seed only (cloud=$CLOUD) — no dynamic features!"
  else
    echo "[OK] FeatureMap has $CLOUD features (dynamic updates working)"
  fi
fi

echo "=== Drone Position ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1

echo "=== FSM State ==="
tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "FSM.*state:" | tail -3

echo "=== Memory ==="
PID=$(docker exec drone-stack-sim-x86 bash -c "pgrep -f 'exploration_node'" | head -1)
if [ -n "$PID" ]; then
  RSS=$(docker exec drone-stack-sim-x86 ps -p $PID -o rss= 2>/dev/null)
  echo "exploration_node RSS: $(echo "scale=0; $RSS/1024" | bc)MB (PID=$PID)"
fi

echo "=== Control Bridge ==="
# Just query status without changing it
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/twist/twist/linear -n 1' 2>&1
```

If all checks pass and FSM shows `MOVE_TO_NEXT_GOAL` / `REPLAN` / `PUB_TRAJ`, the system is ready.
If any check fails, fix that component before proceeding.

### `start`
Launch LA-Planner with AirSim bridge. Control bridge starts DISABLED by default.
**Always kills old instances first** to avoid stale tmux pane issues.

**IMPORTANT: Order matters! (see the MANDATORY START ORDER box above)**
1. Kill old LA-Planner
2. Teleport drone to start position (changes drone position)
3. Restart FAST-LIVO2 (must happen AFTER teleport so it converges at new position)
4. Verify FAST-LIVO2 odom matches GT
5. Launch LA-Planner

```bash
# Step 1: Kill any existing LA-Planner (same as `stop`)
tmux send-keys -t risk_aware_planning:la_planner C-c C-c 2>/dev/null
sleep 0.5
# Reliable stop: explicit SIGINT to the roslaunch parent INSIDE the container — no
# dedicated run_*.sh wraps la_planner (see the header note), so don't rely on the
# tmux Ctrl+C above alone.
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch la_planner_bridge la_planner_airsim.launch" 2>/dev/null
sleep 0.5
for pattern in "exploration_node" "traj_server" "la_sensor_bridge" "la_control_bridge" "roslaunch la_planner_bridge" "odom_visualization_la"; do
  docker exec drone-stack-sim-x86 pkill -9 -f "$pattern" 2>/dev/null
done
sleep 0.5
for win in $(tmux list-windows -t risk_aware_planning -F "#{window_index}:#{window_name}" 2>/dev/null | grep "la_planner" | awk -F: '{print $1}' | sort -rn); do
  tmux kill-window -t risk_aware_planning:$win 2>/dev/null
done
sleep 0.5
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && echo y | rosnode cleanup' 2>/dev/null
sleep 0.5

# Step 2: Teleport drone FIRST (before FAST-LIVO restart)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/x 0.0 && rosparam set /system/sim/teleport/y 0.0 && rosparam set /system/sim/teleport/z 3.0 && rosparam set /system/sim/teleport/yaw 0.0 && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 1

# Step 3: Restart FAST-LIVO2 (so it converges at teleported position) — same
# kill/relaunch mechanism as /sim-fast-livo restart, inlined here since the order
# across this whole block is load-bearing.
echo "Restarting FAST-LIVO2 at new position..."
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
sleep 0.3
docker exec drone-stack-sim-x86 pkill -9 -f "fastlivo_mapping" 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /laserMapping && echo y | rosnode cleanup' 2>/dev/null
sleep 0.5
tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m

# Step 4: Wait for FAST-LIVO2 odom convergence (GT vs LIVO < 2m) — DO NOT SKIP even
# if it "looks" converged already: teleport just invalidated the VIO estimate.
echo "Waiting for FAST-LIVO2 convergence with GT..."
for i in $(seq 1 10); do
  sleep 4
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 || { echo "odom waiting... $((i*4))s"; continue; }
  GT_X=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 2 rostopic echo /gt_odom/pose/pose/position/x -n 1' 2>&1 | head -1)
  LV_X=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 2 rostopic echo /aft_mapped_to_init_odom/pose/pose/position/x -n 1' 2>&1 | head -1)
  GT_Y=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 2 rostopic echo /gt_odom/pose/pose/position/y -n 1' 2>&1 | head -1)
  LV_Y=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 2 rostopic echo /aft_mapped_to_init_odom/pose/pose/position/y -n 1' 2>&1 | head -1)
  GT_Z=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 2 rostopic echo /gt_odom/pose/pose/position/z -n 1' 2>&1 | head -1)
  LV_Z=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 2 rostopic echo /aft_mapped_to_init_odom/pose/pose/position/z -n 1' 2>&1 | head -1)
  DIST=$(python3 -c "import math; print(f'{math.sqrt(($GT_X-$LV_X)**2+($GT_Y-$LV_Y)**2+($GT_Z-$LV_Z)**2):.2f}')" 2>/dev/null)
  echo "t=$((i*4))s GT-LIVO dist=${DIST}m"
  python3 -c "exit(0 if float('$DIST') < 2.0 else 1)" 2>/dev/null && echo "[OK] FAST-LIVO2 converged!" && break
done

# Step 5: Create fresh tmux window and launch LA-Planner. No dedicated run_*.sh
# exists for this node (see header note) — this docker exec one-liner IS the launch
# mechanism, sourcing risk-aware/devel + sim.env + ros_env.sh INSIDE the container.
tmux new-window -t risk_aware_planning -n la_planner
sleep 0.3
tmux send-keys -t risk_aware_planning:la_planner "docker exec -it drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && roslaunch la_planner_bridge la_planner_airsim.launch auto_trigger:=true'" C-m
```
Doc-equivalent (same launch, interactive shell instead of a one-liner — useful for manual poking):
```bash
cd /home/ml/drone-stack-docker && ./setup.sh sh sim-x86
# then inside the container shell:
source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && roslaunch la_planner_bridge la_planner_airsim.launch auto_trigger:=true
```

#### Verify (max 20s)
Check that all 4 core nodes are running AND bridge topics have data:
```bash
for i in $(seq 1 5); do
  sleep 4
  ALL_OK=true

  # Check nodes exist (INSIDE the container)
  for node in exploration_node traj_server la_sensor_bridge; do
    docker exec drone-stack-sim-x86 bash -c "ps aux | grep '$node' | grep -v grep" > /dev/null 2>&1 || ALL_OK=false
  done

  # Check bridge data
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /la/odom_world -n 1' > /dev/null 2>&1 || ALL_OK=false
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /la/depth -n 1' > /dev/null 2>&1 || ALL_OK=false

  $ALL_OK && echo "LA-Planner OK at $((i*4))s" && break
  echo "waiting... ($((i*4))s)"
done
```

#### Feature Check & FAST-LIVO Recovery
After LA-Planner is verified, ensure FAST-LIVO2 is healthy.

**IMPORTANT**: FAST-LIVO2 only publishes visual features when the drone is moving.
At this stage (before `enable`), we only check that FAST-LIVO2's odom is alive
and not stuck on IMU-LiDAR sync errors. Feature count verification happens
AFTER `enable` in the post-enable check.

```bash
echo "=== FAST-LIVO2 health check ==="

# 1. Check odom is alive
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 \
  && echo "[OK] FAST-LIVO2 odom alive" \
  || { echo "[FAIL] FAST-LIVO2 odom dead"; NEED_RESTART=true; }

# 2. Check GT vs FAST-LIVO odom convergence (must be close)
if [ "${NEED_RESTART:-false}" = "false" ]; then
  GT_POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
  GT_X=$(echo "$GT_POS" | grep "x:" | awk '{print $2}')
  GT_Y=$(echo "$GT_POS" | grep "y:" | awk '{print $2}')
  GT_Z=$(echo "$GT_POS" | grep "z:" | awk '{print $2}')
  LV_POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /aft_mapped_to_init_odom/pose/pose/position -n 1' 2>&1)
  LV_X=$(echo "$LV_POS" | grep "x:" | awk '{print $2}')
  LV_Y=$(echo "$LV_POS" | grep "y:" | awk '{print $2}')
  LV_Z=$(echo "$LV_POS" | grep "z:" | awk '{print $2}')
  if [ -n "$GT_X" ] && [ -n "$LV_X" ]; then
    DIST=$(python3 -c "import math; print(f'{math.sqrt(($GT_X-$LV_X)**2+($GT_Y-$LV_Y)**2+($GT_Z-$LV_Z)**2):.2f}')" 2>/dev/null)
    echo "GT=($GT_X,$GT_Y,$GT_Z) LIVO=($LV_X,$LV_Y,$LV_Z) dist=$DIST"
    if python3 -c "exit(0 if float('$DIST') > 2.0 else 1)" 2>/dev/null; then
      echo "[FAIL] FAST-LIVO2 odom diverged from GT by ${DIST}m — need restart"
      NEED_RESTART=true
    else
      echo "[OK] FAST-LIVO2 odom converged (error=${DIST}m)"
    fi
  fi
fi

# 3. Check for IMU-LiDAR sync hang (FAST-LIVO prints errors then stops processing)
SYNC_ERRORS=$(tmux capture-pane -t risk_aware_planning:fast_livo -p -J 2>&1 | grep -c "IMU and LiDAR not synced")
if [ "$SYNC_ERRORS" -gt 3 ]; then
  LAST_LOG=$(tmux capture-pane -t risk_aware_planning:fast_livo -p -J 2>&1 | tail -1)
  echo "[WARN] FAST-LIVO2 has $SYNC_ERRORS sync errors. Last log: $LAST_LOG"
  NEED_RESTART=true
fi

# 4. Restart if needed — same mechanism as /sim-fast-livo restart: SIGINT the
#    roslaunch parent INSIDE the container by run.sh's own __M= string, then
#    relaunch via run.sh in the fast_livo tmux window.
if [ "${NEED_RESTART:-false}" = "true" ]; then
  echo "[ACTION] Restarting FAST-LIVO2..."
  tmux send-keys -t risk_aware_planning:fast_livo C-c C-c C-c 2>/dev/null
  sleep 0.3
  docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
  sleep 0.3
  docker exec drone-stack-sim-x86 pkill -9 -f "fastlivo_mapping" 2>/dev/null
  sleep 0.5
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /laserMapping && echo y | rosnode cleanup' 2>/dev/null
  sleep 0.5

  tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m

  # Wait for odom convergence (not just existence — must be stable)
  echo "Waiting for FAST-LIVO2 odom convergence..."
  for i in $(seq 1 8); do
    sleep 4
    docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 \
      && echo "FAST-LIVO2 odom OK at $((i*4))s" && break
    echo "waiting... $((i*4))s"
  done
else
  echo "[OK] FAST-LIVO2 healthy, no restart needed"
fi
```

#### Wait for Planning
After nodes and features are up, wait for the FSM to reach a planning state:
```bash
echo "=== Waiting for planning ==="
PLAN_OK=false
for i in $(seq 1 15); do
  sleep 2
  FSM_STATE=$(tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "FSM.*state:" | tail -1)
  echo "$FSM_STATE"
  if echo "$FSM_STATE" | grep -qE "MOVE_TO_NEXT_GOAL|REPLAN|PUB_TRAJ"; then
    PLAN_OK=true
    echo "[OK] Planning active at $((i*2))s"
    break
  fi
done
if ! $PLAN_OK; then
  echo "[WARN] FSM did not reach planning state within 30s — check logs"
  tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep -E "No Frontier|No Best|TASK_FAIL" | tail -5
fi
```

If `exploration_node` keeps crashing:
- Check memory: `docker exec drone-stack-sim-x86 ps -p $(docker exec drone-stack-sim-x86 bash -c "pgrep -f exploration_node") -o rss=` — if growing fast, memory leak
- Check tmux pane for crash logs: `tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | tail -10`

### `restart`
Convenience: `stop` + `start` + verify + feature check + wait for planning.
Execute the `stop` section, then `start` section, then run the verify, feature check, and planning wait steps.

### `enable`
Enable LA-Planner's control bridge to start sending velocity commands to AirSim.

**WARNING:** This makes the drone move! Make sure:
1. LA-Planner is in MOVE_TO_NEXT_GOAL state (check with `status`)
2. No other controller is sending commands (kill /control_bridge first if needed)
3. initialize_simulator position hold is released

```bash
# Verify planning is active first
FSM_STATE=$(tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "FSM.*state:" | tail -1)
if ! echo "$FSM_STATE" | grep -qE "MOVE_TO_NEXT_GOAL|REPLAN|PUB_TRAJ"; then
  echo "[FAIL] LA-Planner not in planning state: $FSM_STATE"
  echo "Run 'start' first and wait for MOVE_TO_NEXT_GOAL"
  exit 1
fi

# Enable LA-Planner control
# Note: control_bridge_la.py now auto-disables init_sim velocity hold on first command
# No need to manually release init_sim hold — handoff is automatic
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: true"' 2>&1
```

#### Post-Enable Verify
After enable, the drone starts moving. Verify control + features + node health:
```bash
sleep 5

echo "=== Control ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/twist/twist/linear -n 1' 2>&1

echo "=== Trajectory ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 10 rostopic echo /planning/pos_cmd -n 1' > /dev/null 2>&1 \
  && echo "[OK] Trajectory commands" || echo "[FAIL] No trajectory commands"

echo "=== Node health ==="
docker exec drone-stack-sim-x86 bash -c "ps aux | grep 'exploration_node' | grep -v grep" > /dev/null 2>&1 \
  && echo "[OK] exploration_node alive" || echo "[FAIL] exploration_node CRASHED"

echo "=== Feature pipeline (drone must be moving for features) ==="
# Wait 10s for drone to move and FAST-LIVO to extract features
sleep 10

# Check FAST-LIVO source: must have width > 1 (not just 1 point per frame)
FEAT_WIDTH=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /fast_livo/visual_features/width -n 3' 2>&1 | grep -v "^---" | sort -rn | head -1)
if [ -z "$FEAT_WIDTH" ] || [ "$FEAT_WIDTH" = "" ]; then
  echo "[FAIL] FAST-LIVO2 not publishing features — may need restart"
elif [ "$FEAT_WIDTH" -le 2 ] 2>/dev/null; then
  echo "[WARN] FAST-LIVO2 publishing only $FEAT_WIDTH features/frame — too few, may need restart"
else
  echo "[OK] FAST-LIVO2 features: $FEAT_WIDTH pts/frame"
fi

# Check LA-Planner FeatureMap dynamic update
FEAT_LOG=$(tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "FeatureMap" | tail -1)
if [ -n "$FEAT_LOG" ]; then
  CLOUD=$(echo "$FEAT_LOG" | grep -oP 'cloud=\K[0-9]+')
  if [ "$CLOUD" -le 30 ] 2>/dev/null; then
    echo "[WARN] FeatureMap stuck on seed (cloud=$CLOUD) — dynamic features not arriving"
    echo "  Check: is /la/visual_features being published? Is feature_map subscribing to it?"
  else
    echo "[OK] FeatureMap has $CLOUD features (dynamic updates working)"
  fi
fi
```

### `go`
Convenience command: start + wait for planning + enable + post-enable verify.
Equivalent to running `start`, waiting for MOVE_TO_NEXT_GOAL, then `enable`,
then running the Post-Enable Verify section.

Execute the `start` section above, including FAST-LIVO health check and planning wait.
If planning is active (MOVE_TO_NEXT_GOAL / REPLAN / PUB_TRAJ), automatically
execute the `enable` section, then run Post-Enable Verify.

**IMPORTANT**: After post-enable verify, if FeatureMap is stuck on seed (cloud<=30),
this means FAST-LIVO2 features are not reaching LA-Planner. In this case:
1. Check if `/fast_livo/visual_features` has width > 2 (FAST-LIVO side)
2. Check if `/la/visual_features` is being published (sensor_bridge relay)
3. If FAST-LIVO width=1, it hasn't converged yet — wait longer or restart FAST-LIVO2

### `disable`
Disable control bridge and re-engage position hold:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: false"' 2>&1
sleep 0.5

# Re-engage position hold at current position via teleport service
POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
CUR_X=$(echo "$POS" | grep "x:" | awk '{print $2}')
CUR_Y=$(echo "$POS" | grep "y:" | awk '{print $2}')
CUR_Z=$(echo "$POS" | grep "z:" | awk '{print $2}')
docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/x $CUR_X && rosparam set /system/sim/teleport/y $CUR_Y && rosparam set /system/sim/teleport/z $CUR_Z && rosparam set /system/sim/teleport/yaw 0.0 && rosservice call /initialize_simulator/teleport_to_position \"{}\"" 2>/dev/null
```

### `stop`
Kill all LA-Planner nodes AND clean up the tmux window so `start` can reuse it:
```bash
# Disable control first
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: false"' 2>/dev/null

# Send Ctrl-C to tmux pane (best-effort — no dedicated run.sh trap for la_planner, see header note)
tmux send-keys -t risk_aware_planning:la_planner C-c C-c 2>/dev/null
sleep 0.5

# Reliable stop: explicit SIGINT to the roslaunch parent INSIDE the container
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch la_planner_bridge la_planner_airsim.launch" 2>/dev/null
sleep 0.5

# Force kill remaining processes
for pattern in "exploration_node" "traj_server" "la_sensor_bridge" "la_control_bridge" "roslaunch la_planner_bridge" "odom_visualization_la"; do
  docker exec drone-stack-sim-x86 pkill -9 -f "$pattern" 2>/dev/null
done
sleep 1

# Kill the tmux window (ensures fresh pane on next start)
for win in $(tmux list-windows -t risk_aware_planning -F "#{window_index}:#{window_name}" 2>/dev/null | grep "la_planner" | awk -F: '{print $1}' | sort -rn); do
  tmux kill-window -t risk_aware_planning:$win 2>/dev/null
done
sleep 0.5

# Cleanup stale ROS node registrations
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && echo y | rosnode cleanup' 2>/dev/null
```

## LA-Planner Architecture

```
FAST-LIVO2 ──→ /fast_livo/visual_features ──→ sensor_bridge ──→ /la/visual_features
                                                    │
AirSim ──→ /gt_odom ──→ sensor_bridge ──→ /la/odom_world ──→ exploration_node
       ──→ /camera/depth ──→ sensor_bridge ──→ /la/depth ──→     (SDF map)
       ──→ TF tree ──→ sensor_bridge ──→ /la/sensor_pose ──→     
                                                    │
                                              exploration_node ──→ /planning/trajectory
                                                    │
                                              traj_server ──→ /planning/pos_cmd
                                                    │
                                              control_bridge_la ──→ AirSim velocity API
```

All of the above runs INSIDE `drone-stack-sim-x86` now except AirSim itself (host, Window 0).

## Key Parameters (la_planner_airsim.launch)

| Parameter | Current | Description |
|-----------|---------|-------------|
| auto_trigger | true | Auto-start exploration |
| min_feature_num_plan | 0 | Min features for viewpoint (0 = disabled) |
| use_dynamic_features | true | Subscribe to /la/visual_features |
| map_size_x/y | 20m | SDF map bounds |
| map_size_z | 4m | Vertical map extent |
| sdf_map/resolution | 0.2m | Voxel resolution |
| cluster_min | 5 | Min frontier cluster size |

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Memory explodes (>1GB in seconds) | `visual_scores` or `visual_all_frontier` is true | Set to false in algorithm.xml |
| "No Frontier" forever | SDF map not building or raycaster origin mismatch | Check recenterMap log, verify map origin |
| FSM stuck in INIT | Frontiers found but no viewpoints | Check DBG-SVP/DBG-CFV logs |
| Drone doesn't move after enable | Another controller competing | Kill /control_bridge, release init_sim hold |
| exploration_node segfault | Various code issues | Check tmux log, may need code fixes |
| TASK_FAIL | All viewpoints failed | Restart LA-Planner |
| tmux Ctrl+C doesn't stop the roslaunch | No dedicated run.sh trap for la_planner (see header) | Use the explicit `docker exec ... pkill -INT -f "roslaunch la_planner_bridge la_planner_airsim.launch"` from `stop`/`start` Step 1 |

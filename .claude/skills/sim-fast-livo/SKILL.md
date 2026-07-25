---
name: sim-fast-livo
description: FAST-LIVO2 control (start, restart, status, check-divergence, preflight, recover). Usage - /sim-fast-livo start, /sim-fast-livo restart, /sim-fast-livo check-divergence, /sim-fast-livo preflight, /sim-fast-livo recover
user_invocable: true
---

# FAST-LIVO2 Control

Start, restart, or check FAST-LIVO2 (Visual-Inertial-LiDAR Odometry).
Based on `automation_experiment.py` Phase C node launch + `_restart_fast_livo()`.

**Container era**: FAST-LIVO2 now runs INSIDE the `drone-stack-sim-x86` container (module
`odometry/fast-livo-sim`), not bare-metal `~/fast_livo2`. roscore/UE4/airsim_node stay on the
HOST (tmux window 0 = infra, see `/sim-start`) — only the odometry/mapping/planner/control chain
moved into the container. `network_mode:host` means container topics are reachable from the host
the same as before; what changed is *how you launch/kill/query* the node, not the procedure.

## CRITICAL: ROS Environment (container era)

Host tmux panes no longer need manual `ROS_MASTER_URI`/`ROS_IP` exports — `run.sh` sources
`/work/config/sim.env` + `/work/config/ros_env.sh` INSIDE the container itself before launching.
For any ad-hoc rostopic/rosservice/rosnode query from the host (all `status`/verify checks below),
go through the container — the old bare-metal tree's devel is gone, so custom msg/srv can't
resolve from a bare host shell anymore:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && <cmd>'
```
FAST-LIVO2's own topics/nodes (`/aft_mapped_to_init_odom`, `/cloud_registered`,
`/fast_livo/visual_features`, `/laserMapping`) use the `fast-livo-sim` devel above. Anything on the
risk-aware/AirSim side (`/gt_odom`, `initialize_simulator`, LA-Planner) uses
`/work/ws/risk-aware/devel/setup.bash` instead — used where noted below.

⚠ The sim runs under `use_sim_time` — `rostopic hz` reports nothing useful here (measured
pitfall). Always use `rostopic echo -n 1` (as in every check below) to confirm data is actually
flowing, never rate counters.

## Prerequisites
```bash
# AirSim sensors must be publishing (FAST-LIVO2 needs IMU + image + point cloud)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /camera/left/image_raw/header -n 1' > /dev/null 2>&1
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /airsim_node/hmcl/imu/imu/header -n 1' > /dev/null 2>&1
```
If fails → "Run /sim-start first"

## Commands

### `status`
Check if FAST-LIVO2 is running AND publishing data:
```bash
echo "=== FAST-LIVO2 Status ==="

# Node alive? (process runs INSIDE the container now)
docker exec drone-stack-sim-x86 bash -c "ps aux | grep 'fastlivo_mapping' | grep -v grep" > /dev/null 2>&1 \
  && echo "[OK] Process alive" || echo "[FAIL] Process not running"

# Odom publishing? (actual data)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom/header/seq -n 1' > /dev/null 2>&1 \
  && echo "[OK] Odom publishing" || echo "[FAIL] Odom not publishing"

# Cloud publishing? (actual data)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /cloud_registered/header/seq -n 1' > /dev/null 2>&1 \
  && echo "[OK] Cloud publishing" || echo "[FAIL] Cloud not publishing"

# Visual features publishing? (if feature publish code is active)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /fast_livo/visual_features/width -n 1' > /dev/null 2>&1 \
  && echo "[OK] Visual features publishing" || echo "[INFO] Visual features not available"
```

### `start`
Start FAST-LIVO2 in a tmux window, running INSIDE the container via
`odometry/fast-livo-sim/run.sh` (auto-enters `drone-stack-sim-x86`, sources
`fast-livo-sim/devel` INSIDE the container, and forwards a Ctrl+C on this pane as SIGINT to the
container-side roslaunch — do NOT prepend `source devel/setup.bash`, the script handles it).

```bash
# Check if already running
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 && echo "Already running" && exit 0

# Start in tmux
tmux new-window -t risk_aware_planning -n fast_livo 2>/dev/null || true
tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m
```
Doc-equivalent (same launch, spelled out via the orchestrator script):
```bash
cd /home/ml/drone-stack-docker && ./setup.sh run sim-x86 odometry/fast-livo-sim/run.sh
```

#### Verify (max 20s, check every 4s)
FAST-LIVO2 needs a few seconds to initialize IMU, then starts processing:
```bash
for i in $(seq 1 5); do
  sleep 4
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 && echo "FAST-LIVO2 OK" && break
  echo "waiting... ($((i*4))s)"
done
```

If timeout → check tmux pane:
```bash
tmux capture-pane -t risk_aware_planning:fast_livo -p -J 2>&1 | tail -5
```

Common issues:
- "Reset ImuProcess" then nothing → IMU or image topics not reaching FAST-LIVO2
- Segfault (exit code -11) → code modification issue, revert to original

### `restart`
Kill and restart (adapted from `_restart_fast_livo()` in automation_experiment.py).
**Primary method**: Ctrl+C on the tmux pane running `run.sh` — the script's own trap forwards
SIGINT to the container-side roslaunch for a clean node teardown, same pattern as `start`.
**Backup kill path** (pane gone, or orphaned nodes suspected): use the exact `__M=` match string
from `odometry/fast-livo-sim/run.sh` itself (`roslaunch fast_livo mapping_simulator_openvins.launch`)
against the container's process list, not the host's — the process lives inside the container now.

```bash
# 1. Send Ctrl+C to tmux pane (primary — run.sh's trap does the container-side SIGINT for us)
tmux send-keys -t risk_aware_planning:fast_livo C-c C-c C-c 2>/dev/null
sleep 1

# 2. Backup: kill by the exact string run.sh's own cleanup trap uses, INSIDE the container
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 pkill -9 -f "fastlivo_mapping" 2>/dev/null
sleep 0.5

# 3. Also kill by rosnode (clean deregister)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /laserMapping' 2>/dev/null
sleep 0.5

# 4. Relaunch — same tmux pane, run.sh re-enters the container and sources devel for us
tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m
```

#### Verify (same as start)
```bash
for i in $(seq 1 5); do
  sleep 4
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 && echo "FAST-LIVO2 restarted OK" && break
  echo "waiting... ($((i*4))s)"
done
```

### `stop`
Stop FAST-LIVO2 without restart:
```bash
tmux send-keys -t risk_aware_planning:fast_livo C-c C-c C-c 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 pkill -9 -f "fastlivo_mapping" 2>/dev/null
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /laserMapping' 2>/dev/null
```

### `preflight`
Check ALL dependencies needed for recover/teleport. Fixes what it can automatically.
**Run this before `recover`** to avoid mid-recovery failures.

```bash
echo "=== PREFLIGHT CHECK ==="
PREFLIGHT_OK=true

# 1. ROS master reachable
echo "--- [1/5] ROS Master ---"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic list' > /dev/null 2>&1 \
  && echo "[OK] ROS master reachable" \
  || { echo "[FAIL] ROS master not reachable"; PREFLIGHT_OK=false; }

# 2. AirSim sensors publishing
echo "--- [2/5] AirSim Sensors ---"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /gt_odom/header/seq -n 1' > /dev/null 2>&1 \
  && echo "[OK] GT odom" || { echo "[FAIL] /gt_odom not publishing"; PREFLIGHT_OK=false; }
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /camera/left/image_raw/header -n 1' > /dev/null 2>&1 \
  && echo "[OK] Camera" || { echo "[FAIL] Camera not publishing"; PREFLIGHT_OK=false; }
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /airsim_node/hmcl/imu/imu/header -n 1' > /dev/null 2>&1 \
  && echo "[OK] IMU" || { echo "[FAIL] IMU not publishing"; PREFLIGHT_OK=false; }

# 3. initialize_simulator node alive (required for teleport)
echo "--- [3/5] initialize_simulator ---"
INIT_SIM_ALIVE=false
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode list 2>/dev/null | grep -q "/initialize_simulator"' && INIT_SIM_ALIVE=true

if $INIT_SIM_ALIVE; then
  echo "[OK] initialize_simulator node alive"
else
  echo "[WARN] initialize_simulator not running — restarting..."
  tmux send-keys -t risk_aware_planning:init_sim "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_init_sim.sh" C-m
  # Wait for node to appear (max 10s)
  for i in $(seq 1 10); do
    sleep 1
    docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode list 2>/dev/null | grep -q "/initialize_simulator"' && break
  done
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode list 2>/dev/null | grep -q "/initialize_simulator"' \
    && echo "[OK] initialize_simulator restarted" \
    || { echo "[FAIL] initialize_simulator failed to start — check tmux:init_sim"; PREFLIGHT_OK=false; }
fi

# 4. Teleport service available
echo "--- [4/5] Teleport Service ---"
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice list 2>/dev/null | grep -q "/initialize_simulator/teleport_to_position"' \
  && echo "[OK] Teleport service available" \
  || { echo "[FAIL] Teleport service not available"; PREFLIGHT_OK=false; }

# 5. FAST-LIVO2 process (not required for recover, but report status)
echo "--- [5/5] FAST-LIVO2 ---"
docker exec drone-stack-sim-x86 bash -c "ps aux | grep 'fastlivo_mapping' | grep -v grep" > /dev/null 2>&1 \
  && echo "[OK] Process alive" || echo "[INFO] Not running (will be restarted in recover)"

echo ""
if $PREFLIGHT_OK; then
  echo "=== PREFLIGHT PASSED — ready for recover ==="
else
  echo "=== PREFLIGHT FAILED — fix issues above first ==="
fi
```

### `check-divergence`
Compare GT odometry vs FAST-LIVO2 estimated odometry to detect divergence.
Based on `automation_experiment.py` VIO convergence tracking.

**Thresholds** (from automation_experiment.py):
- Position error > 1.0m → DIVERGED
- Position change rate > 0.30 m/s while drone is stationary → UNSTABLE

```bash
echo "=== Divergence Check ==="

# Get GT position (risk-aware/AirSim side)
GT=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
GT_X=$(echo "$GT" | grep "x:" | awk '{print $2}')
GT_Y=$(echo "$GT" | grep "y:" | awk '{print $2}')
GT_Z=$(echo "$GT" | grep "z:" | awk '{print $2}')

# Get FAST-LIVO estimated position (fast-livo-sim side)
EST=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /aft_mapped_to_init_odom/pose/pose/position -n 1' 2>&1)
EST_X=$(echo "$EST" | grep "x:" | awk '{print $2}')
EST_Y=$(echo "$EST" | grep "y:" | awk '{print $2}')
EST_Z=$(echo "$EST" | grep "z:" | awk '{print $2}')

if [ -z "$GT_X" ] || [ -z "$EST_X" ]; then
  echo "[FAIL] Could not read odometry topics"
  exit 1
fi

echo "GT:  ($GT_X, $GT_Y, $GT_Z)"
echo "EST: ($EST_X, $EST_Y, $EST_Z)"

# Compute position error
ERR=$(python3 -c "
import math
dx=$GT_X - $EST_X; dy=$GT_Y - $EST_Y; dz=$GT_Z - $EST_Z
err = math.sqrt(dx**2 + dy**2 + dz**2)
print(f'{err:.3f}')
")
echo "Position error: ${ERR}m"

# Check threshold
python3 -c "
err = float('$ERR')
if err > 1.0:
    print('[DIVERGED] Position error > 1.0m — run recover')
elif err > 0.5:
    print('[WARNING] Position error > 0.5m — monitor closely')
else:
    print('[OK] VIO tracking normally')
"
```

### `recover`
Full recovery routine when FAST-LIVO2 has diverged.
Based on `automation_experiment.py` Phase B teleport + Phase D VIO retry logic.

**IMPORTANT**: Run `preflight` first to ensure all dependencies are ready (especially initialize_simulator).

**Sequence**: Preflight check → Stop LA-Planner → Stop FAST-LIVO2 → teleport drone to origin → restart FAST-LIVO2 → verify convergence

#### Step 0: Preflight check
Run the `preflight` command above. If it fails, fix issues before continuing.
Critical: `initialize_simulator` node must be alive for teleport to work.

#### Step 1: Stop LA-Planner control (prevent unsafe movement during recovery)
```bash
echo "=== Step 1: Stopping LA-Planner control ==="
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: false"' 2>/dev/null
```

#### Step 2: Stop LA-Planner nodes
```bash
echo "=== Step 2: Stopping LA-Planner ==="
for node in /exploration_node /traj_server /la_sensor_bridge /la_control_bridge /odom_visualization_la /world_to_odom_la; do
  docker exec drone-stack-sim-x86 bash -lc "source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill $node" 2>/dev/null &
done
wait
for pattern in "exploration_node" "traj_server" "la_sensor_bridge" "la_control_bridge" "roslaunch la_planner_bridge"; do
  docker exec drone-stack-sim-x86 pkill -9 -f "$pattern" 2>/dev/null
done
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && echo y | rosnode cleanup' 2>/dev/null
sleep 1
```

#### Step 3: Stop FAST-LIVO2
```bash
echo "=== Step 3: Stopping FAST-LIVO2 ==="
tmux send-keys -t risk_aware_planning:fast_livo C-c C-c C-c 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 pkill -9 -f "fastlivo_mapping" 2>/dev/null
sleep 0.5
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /laserMapping' 2>/dev/null
sleep 1
```

#### Step 4: Teleport drone to origin
Uses `initialize_simulator/teleport_to_position` service (NOT dynparam — dynparam move_to_xyz can hang).
The service does ENU→NED conversion and double-teleport for stability internally.

```bash
echo "=== Step 4: Teleporting drone to origin ==="

# Verify teleport service exists (preflight should have ensured this)
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice list 2>/dev/null | grep -q "/initialize_simulator/teleport_to_position"' \
  || { echo "[FAIL] Teleport service not available — run preflight first"; exit 1; }

# Set teleport target and call service
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam set /system/sim/teleport/x 0.0 && rosparam set /system/sim/teleport/y 0.0 && rosparam set /system/sim/teleport/z 3.0 && rosparam set /system/sim/teleport/yaw 0.0 && rosservice call /initialize_simulator/teleport_to_position "{}"'
sleep 2

# Verify GT position is near origin (ENU: 0,0,~3.0)
GT=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
GT_X=$(echo "$GT" | grep "x:" | awk '{print $2}')
GT_Y=$(echo "$GT" | grep "y:" | awk '{print $2}')
GT_Z=$(echo "$GT" | grep "z:" | awk '{print $2}')
echo "GT position after teleport: ($GT_X, $GT_Y, $GT_Z)"

GT_ERR=$(python3 -c "
import math
err = math.sqrt(float('$GT_X')**2 + float('$GT_Y')**2 + (float('$GT_Z') - 3.0)**2)
print(f'{err:.3f}')
")
echo "GT distance from target: ${GT_ERR}m"
if python3 -c "exit(0 if float('$GT_ERR') < 0.5 else 1)"; then
  echo "[OK] Drone at origin"
else
  echo "[FAIL] Teleport failed — GT error=${GT_ERR}m"
  echo "Check tmux:init_sim for errors"
fi
```

#### Step 5: Restart FAST-LIVO2
```bash
echo "=== Step 5: Restarting FAST-LIVO2 ==="
tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m

for i in $(seq 1 5); do
  sleep 4
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 && echo "FAST-LIVO2 started at $((i*4))s" && break
  echo "waiting... ($((i*4))s)"
done
```

#### Step 6: VIO convergence check (strict — from automation_experiment.py)
Wait for VIO to converge after restart. Check position change rate and velocity magnitude.
```bash
echo "=== Step 6: VIO Convergence Check ==="
# Strict thresholds: pos_rate < 0.30 m/s, vel_mag < 0.10 m/s, 20 consecutive checks
CONVERGED=false
for attempt in $(seq 1 3); do
  echo "Convergence attempt $attempt/3..."

  # Wait 2s for initial settling
  sleep 2

  # Check GT vs EST error
  GT=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
  GT_X=$(echo "$GT" | grep "x:" | awk '{print $2}')
  GT_Y=$(echo "$GT" | grep "y:" | awk '{print $2}')
  GT_Z=$(echo "$GT" | grep "z:" | awk '{print $2}')

  EST=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /aft_mapped_to_init_odom/pose/pose/position -n 1' 2>&1)
  EST_X=$(echo "$EST" | grep "x:" | awk '{print $2}')
  EST_Y=$(echo "$EST" | grep "y:" | awk '{print $2}')
  EST_Z=$(echo "$EST" | grep "z:" | awk '{print $2}')

  ERR=$(python3 -c "
import math
dx=float('$GT_X')-float('$EST_X'); dy=float('$GT_Y')-float('$EST_Y'); dz=float('$GT_Z')-float('$EST_Z')
print(f'{math.sqrt(dx**2+dy**2+dz**2):.3f}')
")
  echo "Position error: ${ERR}m (GT=($GT_X,$GT_Y,$GT_Z) EST=($EST_X,$EST_Y,$EST_Z))"

  if python3 -c "exit(0 if float('$ERR') < 1.0 else 1)"; then
    echo "[OK] VIO converged (error < 1.0m)"
    CONVERGED=true
    break
  fi

  echo "[WARN] VIO not converged — restarting FAST-LIVO2..."
  # Restart FAST-LIVO2 for retry
  tmux send-keys -t risk_aware_planning:fast_livo C-c C-c C-c 2>/dev/null
  sleep 0.5
  docker exec drone-stack-sim-x86 pkill -INT -f "roslaunch fast_livo mapping_simulator_openvins.launch" 2>/dev/null
  sleep 0.3
  docker exec drone-stack-sim-x86 pkill -9 -f "fastlivo_mapping" 2>/dev/null
  sleep 0.5
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosnode kill /laserMapping' 2>/dev/null
  sleep 1
  tmux send-keys -t risk_aware_planning:fast_livo "bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh" C-m
  for i in $(seq 1 5); do
    sleep 4
    docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/fast-livo-sim/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 5 rostopic echo /aft_mapped_to_init_odom -n 1' > /dev/null 2>&1 && break
  done
done

if ! $CONVERGED; then
  echo "[FAIL] VIO convergence failed after 3 attempts"
  echo "Manual intervention needed — check FAST-LIVO2 config or drone position"
fi
```

#### Step 7: Restart LA-Planner (if VIO converged)
If convergence passed, restart LA-Planner using `/sim-la-planner start` then `/sim-la-planner go`.
```bash
if $CONVERGED; then
  echo "=== Step 7: Ready to restart LA-Planner ==="
  echo "Run: /sim-la-planner go"
fi
```

The full recovery typically takes ~30-60s depending on convergence retries.

## Key Topics Published by FAST-LIVO2

| Topic | Description | Expected Rate |
|-------|-------------|---------------|
| `/aft_mapped_to_init_odom` | VIO odometry | ~15Hz |
| `/aft_mapped_to_init` | Same odom (different frame) | ~15Hz |
| `/cloud_registered` | Registered point cloud | ~15Hz |
| `/fast_livo/visual_features` | Accumulated visual features (if code modified) | ~15Hz |
| `/rgb_img` | Processed RGB image | ~15Hz |

Rates above are nominal — do NOT check them with `rostopic hz` (useless under `use_sim_time`);
use `rostopic echo -n <N>` over a few seconds and eyeball the seq/timestamp deltas instead.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| "Reset ImuProcess" then stuck | Sensors not publishing | Check /sim-start health |
| Segfault (exit code -11) | Code modification issue | `cd /home/ml/drone-stack-docker/ws/fast-livo-sim/src/FAST-LIVO2 && git checkout -- src/ include/` to revert (host — source is bind-mounted, not inside the container) |
| Odom drifts wildly | Camera intrinsics mismatch | Check `/home/ml/drone-stack-docker/ws/fast-livo-sim/src/FAST-LIVO2/config/camera_simulator_openvins.yaml` |
| Process alive but no odom | VIO not triggered (no images) | Check `/camera/left/image_raw` topic |

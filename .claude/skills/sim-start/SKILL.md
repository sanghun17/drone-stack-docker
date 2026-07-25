---
name: sim-start
description: Start simulation infrastructure (roscore, Unreal, AirSim, sensors). Checks each component and skips if already running.
user_invocable: true
---

# Simulation Infrastructure Start

Start the common simulation infrastructure needed by ALL planners.
Each step: **verify** (actual data received?) → already OK → skip / not OK → start → re-verify → fail if still not OK.

**Container era**: roscore, Unreal Engine, and the AirSim ROS node (Steps 1-3) stay on the HOST
(tmux window `infra`) — unchanged. The sensor publisher (Step 4) and `initialize_simulator.py`
(Step 5) now run INSIDE the `drone-stack-sim-x86` container (module `planner/risk-aware-sim`),
started via that module's `run_sensor_pub.sh` / `run_init_sim.sh` instead of
`conda activate airsim && roslaunch ...` by hand — same tmux panes/windows as before, just a
different launch command. `network_mode:host` means their topics/services are reachable from the
host exactly as before.

## CRITICAL: ROS Environment

**Every bash command that uses ROS must source the environment first.** Claude's bash shell does NOT inherit tmux/bashrc settings.

```bash
# MUST prepend to EVERY ros command outside tmux:
export ROS_MASTER_URI=http://192.168.50.12:11311
export ROS_IP=192.168.50.12
export ROS_HOSTNAME=192.168.50.12
source /opt/ros/noetic/setup.bash
```

For tmux panes, `source ~/.bashrc` handles this (bashrc has these exports).

## CRITICAL: Verification Method

Use **actual data reception**, not node/topic existence:

```bash
# CORRECT: verify actual data arrives
timeout 5 rostopic echo /gt_odom -n 1 > /dev/null 2>&1

# WRONG: node can exist but not publish data
rosnode list | grep airsim_node
```

## CRITICAL: pgrep Self-Match Bug

`pgrep -f "MyFirstUE4"` matches its own command string. Use `ps aux | grep` instead:

```bash
# CORRECT:
ps aux | grep "MyFirstUE4" | grep -v grep | grep -v pgrep > /dev/null 2>&1

# WRONG (matches itself):
pgrep -f "MyFirstUE4" > /dev/null 2>&1
```

## CRITICAL: tmux Pane Limit

tmux panes have a screen size limit (~4 panes per window). After that `tmux split-window` silently fails.
**Use `tmux new-window` for additional components:**

```bash
# First 3-4 components: split-window OK
tmux split-window -t risk_aware_planning:infra

# After that: use new-window
tmux new-window -t risk_aware_planning -n init_sim
```

---

## Step 0: Pre-flight Diagnostics

### 0a. ROS Network Environment
```bash
python3 -c "import socket; ip = socket.gethostbyname(socket.gethostname()); print(f'{socket.gethostname()} -> {ip}')"
grep -E "^export ROS_" ~/.bashrc | grep -v "^#"
```

Check for:
- `ROS_HOSTNAME` typo (was `ROS_HOSTNAEM` before — fixed 2026-04-06)
- `ROS_MASTER_URI` pointing to correct IP (`192.168.50.12`)
- hostname resolves to `127.0.1.1` is OK **if** `ROS_HOSTNAME` is set in bashrc

### 0b. AirSim IP
```bash
CONFIGURED_IP=$(grep "airsim_ip" /home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/config/scenarios/sim_gt.yaml | awk '{print $2}' | tr -d '"')
ACTUAL_IPS=$(ip addr show | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | cut -d/ -f1)
echo "Configured: $CONFIGURED_IP"
echo "Available: $ACTUAL_IPS"
```
- Configured IP not in available IPs → **WARN user** (gt_odom_publisher and initialize_simulator will crash with "Retry connection over the limit")
- Last known working IP: `192.168.50.12` (changed from .207 on 2026-04-06 due to router change)
- Files containing this IP: `airsim_gt_odom_publisher.py`, `initialize_simulator.py`, `scenarios/sim_*.yaml`, `px4.launch`

### 0c. Display
```bash
echo "DISPLAY=$DISPLAY"
```
- Empty → **WARN**: Unreal Engine cannot start without display.

---

## Step 1: roscore

### Verify
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
timeout 3 rostopic list > /dev/null 2>&1 && echo "RUNNING" || echo "NOT_RUNNING"
```

### If NOT_RUNNING → Start
```bash
tmux new-session -d -s risk_aware_planning -n infra 2>/dev/null || true
tmux send-keys -t risk_aware_planning:infra "source ~/.bashrc && roscore" C-m
```

### Re-verify (max 10s, check every 2s)
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
for i in $(seq 1 5); do
  timeout 3 rostopic list > /dev/null 2>&1 && echo "roscore OK" && break
  sleep 2
  echo "waiting... ($((i*2))s)"
done
```

**If still failing:** roscore binds to the IP in `ROS_MASTER_URI`. Check Step 0a — if bashrc has wrong `ROS_HOSTNAME` or typo, roscore starts but nobody can connect.

### Post: Load config
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
/home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/config/load_config.sh sim_gt
```
No `devel/setup.bash` to source here anymore — `load_config.sh` only does `rosparam load` +
`rospack find` (with a `SCRIPT_DIR`-relative fallback when `rospack find` can't resolve
without a sourced workspace), so it runs fine straight off the host's base ROS install.

---

## Step 2: Unreal Engine

### Verify
```bash
ps aux | grep "MyFirstUE4" | grep -v grep | grep -v pgrep > /dev/null 2>&1 && echo "RUNNING" || echo "NOT_RUNNING"
```
(Unreal has no ROS topic — process check is the only pre-AirSim option. 단 ps grep이 간헐적으로
빗나가는 사례 있음(2026-07-05): AirSim node가 이미 떠 있다면 `/airsim_node/clock` 수신이
UE4+AirSim 생존의 확실한 판정이다 — clock이 오면 UE4는 무조건 살아 있음.)

### If NOT_RUNNING → Start

**Binary location:** Search for it (v4 uses `MyFirstUE4-Linux-Shipping`, v6 uses `MyFirstUE4`):
```bash
UE_BIN=$(find ~/Downloads -name "MyFirstUE4-Linux-Shipping" -type f -executable 2>/dev/null | head -1)
[ -z "$UE_BIN" ] && UE_BIN=$(find ~/Downloads -name "MyFirstUE4" -type f -executable -path "*/Binaries/Linux/*" 2>/dev/null | head -1)
echo "Using: $UE_BIN"
```

```bash
tmux split-window -t risk_aware_planning:infra 2>/dev/null || true
tmux send-keys -t risk_aware_planning:infra.1 "export DISPLAY=:0 && cd $(dirname $UE_BIN) && ./$(basename $UE_BIN)" C-m
```

**GPU 주의 (2026-07-07, 불량 GPU 19:00 UUID 제외 적용 후):** 플래그 없이 기본 실행이 정답
(Vulkan Device 0 = X 서버와 같은 68:00). `-graphicsadapter`는 쓰지 말 것 — `=1`은
llvmpipe(CPU 렌더러: 안 죽지만 depth가 0.38Hz로 떨어져 캠페인이 조용히 무효가 됨),
`=2`는 불량 카드(present 불가 에러). 기동 후 반드시 두 가지 확인:
① `Saved/Logs/MyFirstUE4.log`에서 `Using Device 0` ② depth ≈ 15Hz (아래 명령, ~144 나와야 정상):
```bash
N=$(timeout 10 rostopic echo /camera/depth/image_raw/header/seq 2>/dev/null | grep -c "^[0-9]"); echo "depth: $((N/10)) Hz"
```

### Re-verify (max 30s, check every 5s)
Unreal takes ~20-25 seconds to load rendering engine.
```bash
for i in $(seq 1 6); do
  sleep 5
  ps aux | grep "MyFirstUE4" | grep -v grep | grep -v pgrep > /dev/null 2>&1 && echo "Unreal OK" && break
  echo "waiting... ($((i*5))s)"
done
```

---

## Step 3: AirSim ROS Node

### Verify
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
timeout 5 rostopic echo /airsim_node/clock -n 1 > /dev/null 2>&1 && echo "RUNNING" || echo "NOT_RUNNING"
```

### If NOT_RUNNING → Start
```bash
tmux split-window -t risk_aware_planning:infra 2>/dev/null || true
tmux send-keys -t risk_aware_planning:infra.2 \
  "source ~/.bashrc && source /home/ml/miniforge3/etc/profile.d/conda.sh && conda activate airsim && cd ~/AirSim_vanila/ros && source devel/setup.bash && roslaunch airsim_ros_pkgs airsim_node.launch" C-m
```

### Re-verify (max 15s, check every 3s)
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
for i in $(seq 1 5); do
  sleep 3
  timeout 5 rostopic echo /airsim_node/clock -n 1 > /dev/null 2>&1 && echo "AirSim OK" && break
  echo "waiting... ($((i*3))s)"
done
```

**If "Waiting for connection" persists:** AirSim can't connect to Unreal. Either:
- Unreal is not fully loaded yet (wait longer)
- Previous AirSim session left stale connection → restart Unreal first, then AirSim

---

## Step 4: Sensor Publisher (depth, RGB, odom, TF)

Runs inside the `drone-stack-sim-x86` container now (was a host `conda activate airsim && roslaunch`
pane) — the launch itself (`airsim_sensor_punlisher.launch`) is unchanged, it's started via the
module's `run_sensor_pub.sh` instead. Same pane (`infra.3`), same tmux flow, just a different
command string — the script auto-enters/creates the container and sources everything it needs
itself (do NOT prepend `source devel/setup.bash` here).

### Verify
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
timeout 5 rostopic echo /camera/depth/image_raw/header -n 1 > /dev/null 2>&1 && echo "RUNNING" || echo "NOT_RUNNING"
```

### If NOT_RUNNING → Start
```bash
tmux split-window -t risk_aware_planning:infra 2>/dev/null || true
tmux send-keys -t risk_aware_planning:infra.3 \
  "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_sensor_pub.sh" C-m
```
Doc-equivalent: `cd /home/ml/drone-stack-docker && ./setup.sh run sim-x86 planner/risk-aware-sim/run_sensor_pub.sh`.
Defaults to `localization:=gt` (matches the old pane's `localization:=gt`) — override with
`LOC=vio` exported before the `bash` call for the vio scenario.

### Re-verify (max 15s, check every 3s)
Check BOTH depth and gt_odom — they come from different nodes in the same launch:
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
DEPTH_OK=false; ODOM_OK=false
for i in $(seq 1 5); do
  sleep 3
  timeout 5 rostopic echo /camera/depth/image_raw/header -n 1 > /dev/null 2>&1 && DEPTH_OK=true
  timeout 5 rostopic echo /gt_odom/pose/pose/position -n 1 > /dev/null 2>&1 && ODOM_OK=true
  $DEPTH_OK && $ODOM_OK && echo "Sensors OK (depth+odom)" && break
  echo "waiting... depth=$DEPTH_OK odom=$ODOM_OK ($((i*3))s)"
done
$DEPTH_OK || echo "[FAIL] Depth camera"
$ODOM_OK || echo "[WARN] GT odom failed"
```

**If depth OK but odom FAIL:** `airsim_gt_odom_publisher.py` crashed due to AirSim IP mismatch. The publisher connects to AirSim Python API using the IP in `scenarios/sim_*.yaml` (and hardcoded fallback in the script). Check:
```bash
grep "airsim_ip" /home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/config/scenarios/sim_gt.yaml
grep "MultirotorClient" /home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts/airsim_gt_odom_publisher.py
```

---

## Step 5: Initialize Simulator

Runs inside the `drone-stack-sim-x86` container now (was a host `conda activate airsim && python3`
pane) — started via `run_init_sim.sh` instead. Same window (`init_sim`), just a different
command string.

### Verify
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
rosservice list 2>/dev/null | grep -q "/initialize_simulator/teleport_to_position" && echo "RUNNING" || echo "NOT_RUNNING"
```

### If NOT_RUNNING → Start

**Use `tmux new-window` (not split-window) because pane limit is usually reached by this step:**
```bash
tmux new-window -t risk_aware_planning -n init_sim 2>/dev/null || true
tmux send-keys -t risk_aware_planning:init_sim \
  "bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_init_sim.sh" C-m
```
Doc-equivalent: `cd /home/ml/drone-stack-docker && ./setup.sh run sim-x86 planner/risk-aware-sim/run_init_sim.sh`.

### Re-verify (max 20s, check every 4s)
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
for i in $(seq 1 5); do
  sleep 4
  rosservice list 2>/dev/null | grep -q "/initialize_simulator/teleport_to_position" && echo "InitSim OK" && break
  echo "waiting... ($((i*4))s)"
done
```

**If failed:** `initialize_simulator.py` also uses AirSim Python API. Same IP mismatch issue as Step 4 odom. Check tmux pane for "Failed to connect to AirSim: Retry connection over the limit".

---

## Step 6: Final Health Check

```bash
export ROS_MASTER_URI=http://192.168.50.12:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash

echo "=== Simulation Infrastructure Health Check ==="

timeout 3 rostopic list > /dev/null 2>&1 \
  && echo "[OK] roscore" || echo "[FAIL] roscore"

ps aux | grep "MyFirstUE4" | grep -v grep | grep -v pgrep > /dev/null 2>&1 \
  && echo "[OK] Unreal Engine" || echo "[FAIL] Unreal Engine"

timeout 5 rostopic echo /airsim_node/clock -n 1 > /dev/null 2>&1 \
  && echo "[OK] AirSim clock" || echo "[FAIL] AirSim clock"

timeout 5 rostopic echo /camera/depth/image_raw/header -n 1 > /dev/null 2>&1 \
  && echo "[OK] Depth camera" || echo "[FAIL] Depth camera"

timeout 5 rostopic echo /camera/left/image_raw/header -n 1 > /dev/null 2>&1 \
  && echo "[OK] RGB camera" || echo "[FAIL] RGB camera"

timeout 5 rostopic echo /gt_odom/pose/pose/position -n 1 > /dev/null 2>&1 \
  && echo "[OK] GT odometry" || echo "[FAIL] GT odometry"

rosservice list 2>/dev/null | grep -q "/initialize_simulator/teleport_to_position" \
  && echo "[OK] Initialize Simulator" || echo "[FAIL] Initialize Simulator"

echo "================================================"
```

---

## Troubleshooting Quick Reference

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `rostopic list` timeout | ROS_MASTER_URI mismatch between roscore and client | Ensure both use same IP (check bashrc) |
| roscore starts but can't connect | bashrc `ROS_HOSTNAME` typo (was `HOSTNAEM`) | Fix typo in `~/.bashrc` |
| AirSim "Waiting for connection" forever | Unreal not loaded, or stale connection from previous session | Kill Unreal, wait 30s, restart both |
| gt_odom "Retry connection over the limit" | AirSim IP in config doesn't match machine IP | Update `airsim_ip` in `scenarios/sim_*.yaml` + hardcoded IP in `airsim_gt_odom_publisher.py` |
| init_simulator same retry error | Same IP mismatch | Same fix — also check `initialize_simulator.py` |
| `tmux split-window` silently fails | Too many panes in one window (~4 max) | Use `tmux new-window -n name` instead |
| `pgrep -f "MyFirstUE4"` always true | pgrep matches its own command string | Use `ps aux \| grep ... \| grep -v grep` |
| Depth OK but RGB missing | Sensor publisher partial crash | Restart sensor publisher launch (`bash .../run_sensor_pub.sh` in pane `infra.3`) |
| Everything times out after restart | Previous session's nodes holding ports/connections | `tmux kill-session`, kill all ros/unreal processes (and container ROS nodes — see `/sim-kill`), start fresh |

---
name: sim-kill
description: Kill ALL simulation processes (tmux, roscore, Unreal, AirSim, all nodes). Complete shutdown for clean slate.
user_invocable: true
---

# Simulation Full Shutdown

Kill everything for a completely clean slate. Tested and verified to work in one shot.

**Container era**: roscore/Unreal/AirSim node still run bare-metal on the HOST (unchanged below).
Everything else — FAST-LIVO, voxblox, exploration, JAX, SO(3) control, sensor publisher, eval,
`initialize_simulator.py` — now runs INSIDE the `drone-stack-sim-x86` container. `network_mode:host`
(no `pid: host`) means the container has its OWN PID namespace: a host-side `ps aux` can no longer
see those processes at all, so a container-side cleanup step (Step 4 below) is required — the old
host-only `ps aux` sweep alone is no longer a complete shutdown. **Do NOT `docker stop`/`down` the
container as part of this** — see the note at Step 4.

## CRITICAL: ROS Environment
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311
export ROS_IP=192.168.50.12
source /opt/ros/noetic/setup.bash
```

## Procedure

Execute all steps in sequence:

### Step 1: Disable control services (prevent drift during shutdown)
These services live inside the container now (`local_controller`/`initialize_simulator`
packages) — go through `docker exec` (the old bare-host devel workspace they used to resolve
against is gone, so a bare host `rosservice call` can't resolve their custom msg/srv anymore):
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /control_bridge/toggle_running "data: false"' 2>/dev/null
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: false"' 2>/dev/null
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /initialize_simulator/toggle_setpoint_publishing "data: false"' 2>/dev/null
```
Ignore errors — services may already be down.

### Step 2: Kill all ROS nodes (parallel for speed) + cleanup zombies
Same graph-level operation as before, just run from inside the container so it has the
workspace it needs for anything with a custom msg/srv along the way:
```bash
docker exec drone-stack-sim-x86 bash -lc '
source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh
for node in $(rosnode list 2>/dev/null | grep -v rosout); do
  rosnode kill $node 2>/dev/null &
done
wait
echo "y" | rosnode cleanup 2>/dev/null
'
```

### Step 3: Kill tmux session (takes pane processes with it)
```bash
tmux kill-session -t risk_aware_planning 2>/dev/null
sleep 1
```
This kills the HOST-side `bash run_*.sh` wrapper processes for every containerized node pane
(fast_livo/voxblox/exploration/jax/so3/sensor-pub/eval/init_sim) — each wrapper's own trap
*should* forward that as SIGINT into the container, but don't rely on it alone: Step 4 below is
the explicit backup.

### Step 4: Kill ROS nodes inside the container
```bash
docker exec drone-stack-sim-x86 pkill -INT -f roslaunch 2>/dev/null
docker exec drone-stack-sim-x86 pkill -INT -f initialize_simulator.py 2>/dev/null
sleep 1
docker exec drone-stack-sim-x86 pkill -9 -f roslaunch 2>/dev/null
docker exec drone-stack-sim-x86 pkill -9 -f initialize_simulator.py 2>/dev/null
```
`roslaunch` catches FAST-LIVO/voxblox/exploration/JAX/SO(3)/sensor-pub/eval (all launched via
`roslaunch`, see each module's `run_*.sh`); `initialize_simulator.py` is a plain `rosrun`/python
process, no `roslaunch` wrapper. Ignore errors — nodes may already be down.

⚠ **Do NOT `docker stop`/`docker compose down` the container here** — that's not part of a normal
shutdown, recreating it is expensive (image build + catkin build). Only do that if you specifically
need to reset the container itself (stale mount after a repo move, etc.), and say so explicitly.

### Step 5: Kill any survivors via ps aux + kill -9 (HOST only)
`network_mode:host` does NOT share the PID namespace, so this only ever catches HOST-side
stragglers — Unreal, roscore, and the AirSim ROS node/conda env. FAST-LIVO/planner/JAX/etc. no
longer show up here at all (they're in the container's own PID namespace) — that's what Step 4
is for.
```bash
for pattern in "MyFirstUE4" "roscore" "rosmaster" "airsim"; do
  pids=$(ps aux | grep "$pattern" | grep -v grep | grep -v pgrep | awk '{print $2}')
  if [ -n "$pids" ]; then
    echo "$pids" | xargs kill -9 2>/dev/null
  fi
done
sleep 3
```

### Step 6: Verify clean
```bash
echo "=== Shutdown Verification ==="
tmux has-session -t risk_aware_planning 2>/dev/null && echo "[WARN] tmux" || echo "[OK] tmux"
ps aux | grep "MyFirstUE4" | grep -v grep | grep -v pgrep > /dev/null 2>&1 && echo "[WARN] Unreal" || echo "[OK] Unreal"
ps aux | grep "roscore\|rosmaster" | grep -v grep > /dev/null 2>&1 && echo "[WARN] roscore" || echo "[OK] roscore"
ps aux | grep "airsim" | grep -v grep > /dev/null 2>&1 && echo "[WARN] AirSim" || echo "[OK] AirSim"
docker exec drone-stack-sim-x86 bash -c "ps aux | grep -E 'roslaunch|initialize_simulator.py' | grep -v grep" > /dev/null 2>&1 \
  && echo "[WARN] container ROS nodes (fast-livo/voxblox/exploration/jax/so3/sensor-pub/eval/init_sim)" \
  || echo "[OK] container ROS nodes"
echo "==============================="
```

If any [WARN] on the host lines, get PID with `ps aux | grep <name>` and `kill -9 <PID>` directly.
If the container line [WARN]s, re-run Step 4 (or `docker exec drone-stack-sim-x86 bash -c "ps aux | grep roslaunch"` to see what's still up).

## Why This Order Works
1. **Services first** — stops velocity commands so drone doesn't fly away
2. **rosnode kill parallel** — graceful shutdown, faster than sequential
3. **tmux kill-session** — kills the shell processes hosting each component (host side; each
   containerized node's `run_*.sh` trap should relay this into the container too)
4. **container pkill** — explicit backup in case a trap didn't fire (pane already dead, `docker
   exec` didn't forward the signal, etc.) — catches the `roslaunch` wrapper processes themselves,
   which individual `rosnode kill` calls in Step 2 don't reliably tear down
5. **ps aux kill survivors (host)** — catches anything that escaped tmux on the HOST side (e.g.
   Unreal runs outside tmux's process group)
6. **sleep 3 / verify** — wait for processes to fully exit before verification, and check BOTH
   host and container

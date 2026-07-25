---
name: sim-record-rviz
description: Record RViz visualization video and share to Discord. Usage - /sim-record-rviz [duration_seconds]
user_invocable: true
---

# Record RViz Video & Share

Record LA-Planner RViz visualization and send the video to the active Discord channel.

## CRITICAL: Environment

RViz itself, and the `rosnode kill /rviz` that precedes it, stay HOST-side on purpose (see the
note below) — keep the host ROS env:
```bash
export ROS_MASTER_URI=http://192.168.50.12:11311
export ROS_IP=192.168.50.12
export DISPLAY=:0
source /opt/ros/noetic/setup.bash
```
Anything that talks to a `la_planner_bridge`/`la_control_bridge` service (Step 4) goes through
`docker exec` into `drone-stack-sim-x86` instead — same convention as `/sim-run`'s `la-planner`
path (that whole node lives in that container now, resolved live 2026-07-25):
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && <command>'
```

## Note: container era — two separate `/rviz` nodes can exist, this skill deliberately stays host-side

`modules/planner/risk-aware-sim/run_sensor_pub.sh` (containerized `airsim_sensor_punlisher.launch`)
now brings up its OWN `rviz` node too, but with a different, more basic config
(`active_3d_planning_app_reconstruction/cfg/gazebo.rviz`, sensor/TF debugging) — and it registers
under the exact same ROS node name `/rviz`. This skill deliberately keeps using the richer
`/home/ml/laplanner.rviz` config (occupancy map, trajectory, frontiers, point cloud) launched
HOST-side, for two concrete reasons:
- `/home/ml/laplanner.rviz` is a host-only file — it is **not** in any of `risk-aware-sim`'s
  container mounts (only `RISK_AWARE_PLANNING_SRC`, `SIM_HOST_PY`, `SIM_RISK_AWARE_ASSETS` are
  mounted), so `docker exec ... rviz -d /home/ml/laplanner.rviz` can't see it.
- the host already has `ros-noetic-rviz` installed natively, and the container shares the host's
  X11 socket/`DISPLAY` (`compose.yml` mounts `/tmp/.X11-unix` and passes `DISPLAY` through) —
  so a host-launched RViz window is visually indistinguishable from a container one to
  `xdotool`; there's no capture-side reason to containerize this specific launch.

The pre-existing `rosnode kill /rviz` in Step 2 already does the right thing here: whichever
`/rviz` is currently registered (this skill's previous run, or `run_sensor_pub.sh`'s own) gets
killed before the laplanner one starts fresh, so the two never fight over the ROS graph name.

## Dependencies
- Python packages: `mss`, `opencv-python-headless` (host pip install if missing — this tool runs
  on the host, unrelated to the sim-x86 container)
- `xdotool` (system package, already installed)
- RViz config: `/home/ml/laplanner.rviz`
- Screen recorder script: `/tmp/screen_record.py`

## Commands

### Default (record and share)
Arguments: optional duration in seconds (default: 15)

#### Step 1: Ensure screen recorder script exists
If `/tmp/screen_record.py` doesn't exist, create it:
```python
#!/usr/bin/env python3
"""Screen record a specific X11 window using mss + opencv."""
import sys, time, subprocess, mss, cv2, numpy as np

def get_window_geometry(window_id):
    result = subprocess.run(['xdotool', 'getwindowgeometry', str(window_id)],
                            capture_output=True, text=True)
    lines = result.stdout.strip().split('\n')
    pos_part = [l for l in lines if 'Position' in l][0].split(':')[1].strip().split('(')[0].strip()
    x, y = map(int, pos_part.split(','))
    geo_part = [l for l in lines if 'Geometry' in l][0].split(':')[1].strip()
    w, h = map(int, geo_part.split('x'))
    return x, y, w, h

def record(window_id, duration, output_path, fps=10):
    x, y, w, h = get_window_geometry(window_id)
    print(f"Recording window {window_id}: pos=({x},{y}) size={w}x{h} for {duration}s at {fps}fps")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_path, fourcc, fps, (w, h))
    interval = 1.0 / fps
    start = time.time()
    frame_count = 0
    with mss.mss() as sct:
        monitor = {"top": y, "left": x, "width": w, "height": h}
        while (time.time() - start) < duration:
            t0 = time.time()
            img = np.array(sct.grab(monitor))
            frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            out.write(frame)
            frame_count += 1
            elapsed = time.time() - t0
            if elapsed < interval:
                time.sleep(interval - elapsed)
    out.release()
    print(f"Saved {frame_count} frames ({time.time()-start:.1f}s) to {output_path}")

if __name__ == '__main__':
    window_id = int(sys.argv[1])
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 10
    output = sys.argv[3] if len(sys.argv) > 3 else '/tmp/recording.mp4'
    record(window_id, duration, output)
```

#### Step 2: Launch RViz (if not running) with laplanner config
```bash
# Check if RViz is already running with laplanner config
RVIZ_WID=$(xdotool search --name "laplanner.rviz" 2>/dev/null | head -1)

if [ -z "$RVIZ_WID" ]; then
  echo "Launching RViz with laplanner.rviz..."
  # Kill any existing rviz — this may be run_sensor_pub.sh's own /rviz (gazebo.rviz, in the
  # container) or a stale laplanner one; either way the node name collides, see note above.
  rosnode kill /rviz 2>/dev/null; sleep 1

  rosrun rviz rviz -d /home/ml/laplanner.rviz &

  # Wait for window to appear (max 10s)
  for i in $(seq 1 10); do
    sleep 1
    RVIZ_WID=$(xdotool search --name "laplanner.rviz" 2>/dev/null | head -1)
    [ -n "$RVIZ_WID" ] && break
  done
fi

if [ -z "$RVIZ_WID" ]; then
  echo "[FAIL] RViz window not found"
  exit 1
fi
echo "RViz window ID: $RVIZ_WID"
```

#### Step 3: Bring RViz to foreground
```bash
xdotool windowactivate --sync $RVIZ_WID
xdotool windowraise $RVIZ_WID
sleep 1
```

#### Step 4: Record (with optional enable trigger)
Default duration: 30 seconds. User can specify a different duration.

**IMPORTANT**: Start recording BEFORE enabling control, so the flight start is captured.
The sequence should be:
1. Start recording in background
2. Wait 2s (capture pre-flight state)
3. Enable LA-Planner control (`docker exec` into `drone-stack-sim-x86` — see CRITICAL above)
4. Wait for recording to finish

```bash
DURATION=${1:-30}
OUTPUT="/tmp/rviz_recording_$(date +%Y%m%d_%H%M%S).mp4"

# Start recording in background
python3 /tmp/screen_record.py $RVIZ_WID $DURATION $OUTPUT &
REC_PID=$!

# Wait 2s then enable control
sleep 2
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /la_control_bridge/toggle_running "data: true"'

# Wait for recording to finish
wait $REC_PID
```

If LA-Planner control is already enabled (drone already flying), skip the enable step
and just record directly:
```bash
python3 /tmp/screen_record.py $RVIZ_WID $DURATION $OUTPUT
```

#### Step 5: Share to Discord
Send the video to the active Discord channel with a brief status summary.
Include current state info in the message (e.g., FSM state, VIO error, GT position).
```bash
# Gather status for message
FSM_STATE=$(tmux capture-pane -t risk_aware_planning:la_planner -p -J 2>&1 | grep "FSM.*state:" | tail -1 | sed 's/.*state: //')
GT_POS=$(docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/pose/pose/position -n 1' 2>&1)
```
Then use the Discord reply tool to send the video file with a message like:
"RViz recording ({duration}s). FSM: {state}. GT: ({x}, {y}, {z})"

## Notes
- Video is captured at 10fps, mp4v codec
- ~0.2MB/s file size (15s ≈ 3MB, well under Discord 25MB limit)
- RViz must be on a visible display (DISPLAY=:0)
- If RViz window is on a secondary monitor, it still captures correctly via pixel coordinates
- The laplanner.rviz config shows: occupancy map, trajectory, frontiers, point cloud
- `la_planner_bridge`/`la_control_bridge` run inside `drone-stack-sim-x86` (no dedicated
  `run_*.sh`, launched via a one-line `docker exec` — see `/sim-run`'s `la-planner` path /
  `/sim-la-planner` for how that pane gets started and stopped)

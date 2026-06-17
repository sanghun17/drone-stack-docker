#!/bin/bash
# One command for the flight-safety diagnostic view: the monitor node (observe-only,
# DETACHED) + an rqt_runtime_monitor GUI viewed in a BROWSER (headless via VNC/noVNC, the
# same trick as utility_rviz/rqt). `stop` tears both down.
#   bash scripts/utility_diagnostic.sh          # start monitor + GUI
#   bash scripts/utility_diagnostic.sh stop     # stop both
#
# GUI shares the headless desktop :99 / VNC 5900 / web 6080 with rviz/rqt/realsense -> ONE browser tab
# (scroll the empty desktop, or right-click the title bar -> Send To Desktop, to give it its own
# workspace). `stop` kills ONLY the monitor node + its GUI window — NOT the shared Xvfb/x11vnc/
# websockify, which the other GUIs live on.
set -e
C=drone-stack-d435i-voxblox
HERE="$(dirname "$(readlink -f "$0")")"
MON="roslaunch flight_safety monitoring.launch"
DN=99; VNC=5900; WEB=6080   # SHARED GUI desktop (same as rviz/rqt/realsense)

if [ "${1:-start}" = "stop" ]; then
  # kill ONLY our two pieces: the monitor node + its rqt_runtime_monitor window. The Xvfb/x11vnc/
  # websockify are the SHARED desktop (rviz/rqt/realsense live there too) -> leave them up.
  docker exec "$C" pkill -INT -f "$MON"           2>/dev/null || true
  docker exec "$C" pkill -f "rqt_runtime_monitor" 2>/dev/null || true
  echo ">> diagnostic stopped (monitor node + its GUI). Shared desktop :$DN stays up for rviz/rqt/realsense."
  exit 0
fi

docker start "$C" >/dev/null 2>&1
# (1) monitor node — observe-only, detached (light). RESTART on every run so config edits
# take effect: stop the old node + CLEAR its rosparams (a plain reload MERGES, so removed/
# renamed keys would linger and the node would see a stale+new mix), then start fresh.
# Liveness by rosnode, NOT pgrep (pid1 `sleep infinity` never reaps; a dead roslaunch fools pgrep).
NODE=/flight_safety_monitor
alive(){ docker exec "$C" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; rosnode list 2>/dev/null | grep -qx $NODE"; }
if alive; then
  echo ">> restarting monitor (reloads config)"
  docker exec "$C" pkill -INT -f "$MON" 2>/dev/null || true   # no death-wait: relaunch bumps the old name
fi
docker exec "$C" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; rosparam delete $NODE" >/dev/null 2>&1 || true
docker exec -d "$C" bash /work/modules/control/flight-safety/run_monitor.sh
for _ in $(seq 1 14); do alive && break; sleep 0.5; done
if alive; then
  echo ">> monitor node started (DETACHED — stop with: $0 stop)"
else
  echo "ERROR: $NODE did not come up — flight_safety is probably not built (or no roscore)."
  echo "  build it:  ./setup.sh build-ws d435i-voxblox"
  echo "       (or:  docker exec $C bash -lc 'source /opt/ros/noetic/setup.bash; cd /work/ws/risk-aware; catkin build flight_safety')"
  exit 1
fi
# (2) rqt_runtime_monitor GUI (reads /diagnostics raw) via the shared headless-VNC launcher.
# NOT restarted — rqt_runtime_monitor reads /diagnostics LIVE, so it reflects the monitor's new
# config automatically; re-running just re-attaches (fast). (For a truly fresh rqt that also
# drops stale entries, use `stop` then start.) Invoke as an rqt plugin, NOT `rqt_runtime_monitor`
# (that exe is in lib/, rosrun-style, not on PATH).
exec "$HERE/_vnc_gui.sh" "$DN" "$VNC" "$WEB" monitor \
  rqt --standalone rqt_runtime_monitor.runtime_monitor.RuntimeMonitor "$@"

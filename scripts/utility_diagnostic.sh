#!/bin/bash
# One command for the flight-safety diagnostic view: the monitor node (observe-only,
# DETACHED) + an rqt_runtime_monitor GUI viewed in a BROWSER (headless via VNC/noVNC, the
# same trick as utility_rviz/rqt). `stop` tears both down.
#   bash scripts/utility_diagnostic.sh          # start monitor + GUI
#   bash scripts/utility_diagnostic.sh stop     # stop both
#
# GUI runs on its OWN display :97 / VNC 5902 / web 6082 (rviz=99/5900/6080, rqt=98/5901/6081).
set -e
C=drone-stack-d435i-voxblox
HERE="$(dirname "$(readlink -f "$0")")"
MON="roslaunch flight_safety monitoring.launch"
DN=97; VNC=5902; WEB=6082

if [ "${1:-start}" = "stop" ]; then
  docker exec "$C" pkill -INT -f "$MON"             2>/dev/null || true
  docker exec "$C" pkill -f "rqt_runtime_monitor"   2>/dev/null || true
  docker exec "$C" pkill -f "x11vnc.*-rfbport $VNC" 2>/dev/null || true
  docker exec "$C" pkill -f "websockify.*:$WEB"     2>/dev/null || true
  docker exec "$C" pkill -f "Xvfb :$DN"             2>/dev/null || true
  echo ">> diagnostic stopped (monitor + GUI on display :$DN)"
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
  docker exec "$C" pkill -INT -f "$MON" 2>/dev/null || true
  for _ in $(seq 1 10); do alive || break; sleep 0.5; done
fi
docker exec "$C" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; rosparam delete $NODE" >/dev/null 2>&1 || true
docker exec -d "$C" bash /work/modules/control/flight-safety/run_monitor.sh
for _ in $(seq 1 12); do alive && break; sleep 1; done
if alive; then
  echo ">> monitor node started (DETACHED — stop with: $0 stop)"
else
  echo "ERROR: $NODE did not come up — flight_safety is probably not built (or no roscore)."
  echo "  build it:  ./setup.sh build-ws d435i-voxblox"
  echo "       (or:  docker exec $C bash -lc 'source /opt/ros/noetic/setup.bash; cd /work/ws/risk-aware; catkin build flight_safety')"
  exit 1
fi
# (2) rqt_runtime_monitor GUI (reads /diagnostics raw) via the shared headless-VNC launcher.
# Restart it too: kill the old rqt (a separate docker exec, so pkill excludes its own pid —
# safe) so _vnc_gui starts a fresh window; this also clears stale diagnostic entries. The VNC
# infra (Xvfb/x11vnc/websockify) stays up, so the browser just reconnects. Invoke as an rqt
# plugin, NOT `rqt_runtime_monitor` (that exe is in lib/, rosrun-style, not on PATH).
docker exec "$C" pkill -f "rqt_runtime_monitor.runtime_monitor" 2>/dev/null || true; sleep 2
exec "$HERE/_vnc_gui.sh" "$DN" "$VNC" "$WEB" monitor \
  rqt --standalone rqt_runtime_monitor.runtime_monitor.RuntimeMonitor "$@"

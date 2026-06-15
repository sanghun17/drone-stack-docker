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
# (1) monitor node — observe-only, detached (light). Idempotent: start only if absent.
if docker exec "$C" pgrep -f "$MON" >/dev/null 2>&1; then
  echo ">> monitor already running"
else
  docker exec -d "$C" bash /work/modules/control/flight-safety/run_monitor.sh
  echo ">> monitor node started (DETACHED — stop with: $0 stop)"
fi
# (2) rqt_runtime_monitor GUI (reads /diagnostics) via the shared headless-VNC launcher.
exec "$HERE/_vnc_gui.sh" "$DN" "$VNC" "$WEB" monitor rqt_runtime_monitor "$@"

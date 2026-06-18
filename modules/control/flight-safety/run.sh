#!/bin/bash
# control/flight-safety: the whole module at once -- observe (L1+L2) + response (L3, KILL AUTHORITY)
# + estimator mux in one roslaunch, plus the rqt_runtime_monitor /diagnostics view in a browser.
# Actuation gated by require_armed. Ctrl-C kills all of it.
__C=drone-stack-d435i-voxblox
__M="roslaunch flight_safety safety.launch"
__NODES="flight_safety_(diagnosis|monitor|response)|vision_pose_mux"
__killall(){ docker exec "$__C" pkill -INT -f "$__M"    >/dev/null 2>&1
             docker exec "$__C" pkill      -f "$__NODES" >/dev/null 2>&1
             docker exec "$__C" pkill      -f "rqt_runtime_monitor" >/dev/null 2>&1; }   # GUI window; shared VNC infra spared

if [ ! -f /.dockerenv ]; then
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  docker exec "$__C" bash -lc 'source /work/config/ros_env.sh; source /work/modules/ensure_roscore.sh'  # master up before the GUI
  "$__R/scripts/_vnc_gui.sh" 99 5900 6080 monitor \
    rqt --standalone rqt_runtime_monitor.runtime_monitor.RuntimeMonitor || true   # diagnostic GUI (best-effort)
  __TT=$([ -t 1 ] && echo -it || echo -i)
  trap '__killall; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}"; __rc=$?
  __killall
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch flight_safety safety.launch

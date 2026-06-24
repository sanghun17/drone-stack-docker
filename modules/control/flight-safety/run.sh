#!/bin/bash
# control/flight-safety: the whole module at once -- observe (L1+L2) + response (L3, KILL AUTHORITY)
# + estimator mux in one roslaunch, plus the rqt_runtime_monitor /diagnostics view in a browser.
# Actuation gated by require_armed. Ctrl-C kills all of it.
__C=drone-stack-d435i-voxblox
__M="roslaunch flight_safety safety.launch"
__NODES="flight_safety_(diagnosis|monitor|response)|vision_pose_mux|robot_odom_relay"
__killall(){ docker exec "$__C" pkill -INT -f "$__M"    >/dev/null 2>&1
             docker exec "$__C" pkill      -f "$__NODES" >/dev/null 2>&1
             docker exec "$__C" pkill      -f "fs_led_node.py" >/dev/null 2>&1   # SIGTERM -> node blanks the LED on exit
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
source /work/ws/flight-safety/devel/setup.bash --extend   # flight_safety pkg + Fault/FlightState msgs
source /work/ws/risk-aware/devel/setup.bash --extend      # mavros_msgs (vendored in risk-aware), runtime only
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh
# status LED (control_lane -> APA102), separate process so a GPIO stall can't touch the safety loop
taskset -c "${CPUS_POOL}" python3 /work/modules/control/flight-safety/fs_led_node.py >/tmp/fs_led.log 2>&1 &
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch flight_safety safety.launch

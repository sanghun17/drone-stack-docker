#!/bin/bash
# control/local-controller: velocity control bridge (control_bridge.py, `local_controller`
# pkg). /jax/optimal_trajectory -> PD velocity -> /mavros/setpoint_raw/local. control=mavros +
# odom=/robot/odom are hardcoded in control_bridge (no scenario). Commands are DISABLED
# by default -- call /control_bridge/toggle_running "data: true" to actually send. Needs
# control/mavros + control/flight-safety (for /robot/odom) up first. Ctrl-C stops the node;
# the shared roscore is left alone.

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> SIGINT the node INSIDE the container (docker exec doesn't forward it
  # reliably). roscore left alone — shared master other modules use.
  __M="control_bridge.py"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned the node
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP / CPUS_* — single source, edit-and-go
source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4

# No scenario load: control_bridge hardcodes control=mavros + odom=/robot/odom.

# CPUS_POOL (config/ros_env.sh): stay OFF camera cores 0-1 / fast-livo cores 2-3.
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" rosrun local_controller control_bridge.py "$@"

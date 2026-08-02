#!/bin/bash
# control/local-controller: verified real-flight stack.
# /jax/optimal_trajectory -> B-spline traj_server -> /planning/pos_cmd ->
# control_bridge -> flight_safety MUX -> MAVROS/PX4. Commands are DISABLED by
# default -- call /control_bridge/toggle_running "data: true" to send. Needs
# control/mavros + control/flight-safety first. Ctrl-C leaves the shared roscore up.

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
  # Normal execution has one path. pos_step/tau is the only diagnostic mode.
  case "${1:-}" in
    pos_step|tau)   __M="ours_pos_step" ;;
    *)              __M="ours_mavros_stack" ;;
  esac
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

# Numeric/runtime settings come from planning.yaml through the control launch.

# CPUS_POOL (config/ros_env.sh): stay OFF camera cores 0-1 / fast-livo cores 2-3.
__T="taskset -c ${CPUS_POOL:?config/ros_env.sh not sourced}"
case "${1:-}" in
  pos_step|tau)   shift; exec $__T roslaunch local_controller ours_pos_step.launch "$@" ;;
  *)              exec $__T roslaunch local_controller ours_mavros_stack.launch "$@" ;;
esac

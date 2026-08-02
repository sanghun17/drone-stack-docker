#!/bin/bash
# control/mavros: MAVROS bridge to the PX4 flight controller (px4.launch).
# Connection values live in config/stack.env and are passed to px4.launch via
# exported FCU_URL/GCS_URL. mavros is built in the risk-aware workspace.

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"   # recreate $__C if missing / stale-mounted (repo moved)
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the launch INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT roslaunch (clean node
  # teardown). roscore is left alone — it's the shared master other modules use.
  __M="roslaunch mavros px4.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
[ -f /work/config/stack.env ] && source /work/config/stack.env
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
: "${FCU_URL:?FCU_URL missing from /work/config/stack.env}"
: "${GCS_URL:?GCS_URL missing from /work/config/stack.env}"
export FCU_URL GCS_URL           # px4.launch reads both via $(optenv ...)

source /work/modules/ensure_roscore.sh   # master up on $ROS_MASTER_PORT — TCP probe, not a blind sleep 4

# px4.launch = PX4-flavoured MAVROS. Never pass connection args inline here;
# roslaunch silently ignores empty `arg:=`, while the required env checks above
# fail loudly before the FCU bridge starts.
# CPUS_POOL (config/ros_env.sh): stay OFF camera cores 0-1 (uvc watchdog protection).
exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" roslaunch mavros px4.launch "$@"

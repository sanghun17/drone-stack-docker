#!/bin/bash
# control/mavros: MAVROS bridge to the PX4 flight controller (px4.launch).
# Params (fcu_url, gcs_url) live in px4.launch; fcu_url falls back to $FCU_URL from
# config/stack.env via optenv. mavros is built in the risk-aware workspace (ws/risk-aware).

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
  trap 'cleanup; exit 130' INT TERM
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
[ -f /work/config/stack.env ] && source /work/config/stack.env   # FCU_URL
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
[ -n "${FCU_URL:-}" ] && export FCU_URL   # let px4.launch read it via $(optenv FCU_URL ...); value lives in the launch

if ! pgrep -f "roscore -p ${ROS_MASTER_PORT}" >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_PORT}" >/tmp/roscore_${ROS_MASTER_PORT}.log 2>&1 & sleep 4
fi

# px4.launch = PX4-flavoured MAVROS. All params (fcu_url, gcs_url) are managed IN
# px4.launch — never pass them inline here (roslaunch silently ignores empty `arg:=`).
exec roslaunch mavros px4.launch "$@"

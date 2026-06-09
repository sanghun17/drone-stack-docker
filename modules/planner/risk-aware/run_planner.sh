#!/bin/bash
# planner/risk-aware: exploration planner bring-up (real_planning.launch) =
# mav_active_3d_planning exploration + odom relay + bbox params.
# Voxblox is SEPARATE: run run_voxblox.sh alongside this (mapping and planner
# are independent launches, mirroring the ml/sim structure). Needs camera + fast-livo.
# Trigger after up: rosservice call /planner/planner_node/toggle_running "data: true"

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox; docker start "$__C" >/dev/null 2>&1
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  __TT=$([ -t 1 ] && echo -it || echo -i)
  # Ctrl+C here -> stop the launch INSIDE the container too. docker exec does not
  # reliably forward SIGINT, so do it explicitly: SIGINT roslaunch (clean node
  # teardown). roscore is left alone — it's the shared master other modules use.
  __M="roslaunch active_3d_planning_app_reconstruction real_planning.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup            # also catch crash/normal exit that orphaned nodes
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go
if ! pgrep -f "roscore -p ${ROS_MASTER_PORT}" >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_PORT}" >/tmp/roscore_${ROS_MASTER_PORT}.log 2>&1 & sleep 4
fi
exec roslaunch active_3d_planning_app_reconstruction real_planning.launch "$@"

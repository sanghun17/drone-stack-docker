#!/bin/bash
# planner/risk-aware: exploration planner bring-up (real_planning.launch) =
# voxblox + mav_active_3d_planning exploration + odom relay + bbox params.
# INCLUDES voxblox (don't also run run_voxblox.sh). Needs camera + fast-livo.
# Trigger after up: rosservice call /planner/planner_node/toggle_running "data: true"

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox; docker start "$__C" >/dev/null 2>&1
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  __TT=$([ -t 1 ] && echo -it || echo -i)
  exec docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
PORT="${ROS_MASTER_PORT:-11399}"
export ROS_MASTER_URI="http://localhost:${PORT}"
if ! pgrep -f "roscore -p ${PORT}" >/dev/null 2>&1; then
  roscore -p "${PORT}" >/tmp/roscore_${PORT}.log 2>&1 & sleep 4
fi
exec roslaunch active_3d_planning_app_reconstruction real_planning.launch "$@"

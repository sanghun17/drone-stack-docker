#!/bin/bash
# odometry/fast-livo: FAST-LIVO2 on the D435i -> /aft_mapped_to_odom + odom TF.
# Needs the camera up. fast_livo is built in /livo_ws (catkin) from the mounted source.

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
source /work/ws/fast-livo/devel/setup.bash
PORT="${ROS_MASTER_PORT:-11399}"
export ROS_MASTER_URI="http://localhost:${PORT}"

if ! pgrep -f "roscore -p ${PORT}" >/dev/null 2>&1; then
  roscore -p "${PORT}" >/tmp/roscore_${PORT}.log 2>&1 &
  sleep 4
fi

exec roslaunch fast_livo mapping_d435i.launch "$@"

#!/bin/bash
# sensor/realsense-d435i: D435i driver. All camera params live in d435i.launch.
# VERIFIED on Jetson Orin onboard USB: pointcloud ~23Hz, color/depth 15Hz, IMU 202Hz.
# If you hit "Bond broken, exiting" on start, just re-run — the initial_reset HW reset
# occasionally trips the nodelet bond; a clean retry catches it.

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
source /work/config/ros_env.sh   # ROS_MASTER_URI / ROS_IP — single source, edit-and-go

# ensure a master (idempotent)
if ! pgrep -f "roscore -p ${ROS_MASTER_PORT}" >/dev/null 2>&1; then
  roscore -p "${ROS_MASTER_PORT}" >/tmp/roscore_${ROS_MASTER_PORT}.log 2>&1 & sleep 4
fi

exec roslaunch "$(dirname "${BASH_SOURCE[0]}")/d435i.launch" "$@"

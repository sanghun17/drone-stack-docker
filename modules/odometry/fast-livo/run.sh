#!/bin/bash
# odometry/fast-livo: FAST-LIVO2 on the D435i -> /aft_mapped_to_odom + odom TF.
# Needs the camera up. fast_livo is built in /livo_ws (catkin) from the mounted source.
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

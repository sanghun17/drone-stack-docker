#!/bin/bash
# planner/risk-aware: voxblox uncertainty mapping (voxblox_d435i.launch). MAPPING ONLY.
# Needs camera + fast-livo (transforms the cloud into the FAST-LIVO odom frame).
# NOTE: run_planner.sh already includes voxblox — don't run both.
set -e
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
PORT="${ROS_MASTER_PORT:-11399}"
export ROS_MASTER_URI="http://localhost:${PORT}"
if ! pgrep -f "roscore -p ${PORT}" >/dev/null 2>&1; then
  roscore -p "${PORT}" >/tmp/roscore_${PORT}.log 2>&1 & sleep 4
fi
exec roslaunch active_3d_planning_app_reconstruction voxblox_d435i.launch "$@"

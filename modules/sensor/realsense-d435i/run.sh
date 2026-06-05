#!/bin/bash
# sensor/realsense-d435i: D435i driver. VERIFIED config (Jetson Orin + onboard USB hub):
#   initial_reset:=true  -> wakes the IMU motion module
#   enable_sync:=false   -> HW-clock timestamps otherwise break the depth/color syncer
#   allow_no_texture_points -> publish points even without a color match
# All 4 streams: pointcloud ~23Hz, color/depth 15Hz, IMU 202Hz.

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
PORT="${ROS_MASTER_PORT:-11399}"
export ROS_MASTER_URI="http://localhost:${PORT}"

if ! pgrep -f "roscore -p ${PORT}" >/dev/null 2>&1; then
  roscore -p "${PORT}" >/tmp/roscore_${PORT}.log 2>&1 &
  sleep 4
fi

exec roslaunch realsense2_camera rs_camera.launch \
  ${RS_SERIAL:+serial_no:=${RS_SERIAL}} \
  initial_reset:=true \
  depth_width:=640 depth_height:=480 depth_fps:=15 \
  color_width:=640 color_height:=480 color_fps:=15 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=linear_interpolation \
  filters:=pointcloud allow_no_texture_points:=true \
  enable_sync:=false "$@"

#!/bin/bash
# See3CAM_24CUG UVC publisher. Defaults are the modes validated on the Jetson:
# 1280x720 UYVY at a sustained 60 Hz. Calibration is loaded only when a real
# serial-specific yaml exists; fabricated intrinsics are unsafe for ArUco pose.
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/scripts/lib/select_stack.sh"
  dsd_select_stack "sensor/see3cam-24cug" || exit $?
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/scripts/lib/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -x see3cam_node >/dev/null 2>&1 || true; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi

source /work/config/ros_env.sh
source /opt/ros/noetic/setup.bash
set -u
source /work/scripts/lib/ensure_roscore.sh

: "${SEE3CAM_SERIAL:=1A3958060A020900}"
: "${SEE3CAM_DEVICE:=/dev/v4l/by-id/usb-e-con_systems_See3CAM_24CUG_${SEE3CAM_SERIAL}-video-index0}"
: "${SEE3CAM_WIDTH:=1280}"
: "${SEE3CAM_HEIGHT:=720}"
: "${SEE3CAM_FPS:=60}"
: "${SEE3CAM_PIXEL_FORMAT:=uyvy}"
: "${SEE3CAM_FRAME_ID:=see3cam_optical_frame}"
: "${SEE3CAM_CAMERA_NAME:=see3cam_24cug_${SEE3CAM_SERIAL}}"
: "${SEE3CAM_EXPOSURE_AUTO:=1}"
: "${SEE3CAM_EXPOSURE_ABSOLUTE:=150}"
: "${SEE3CAM_GAIN:=10}"
: "${SEE3CAM_RECTIFY_FPS:=20}"

# Build the publisher with the same usb_cam capture source and one OpenCV ABI.
module_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
binary="/work/.build/see3cam-demand/$(uname -m)/see3cam_node"
if [ ! -x "$binary" ] || [ "$module_dir/see3cam_node.cpp" -nt "$binary" ] || \
   [ "$module_dir/CMakeLists.txt" -nt "$binary" ] || [ "$module_dir/build.sh" -nt "$binary" ] || \
   [ "$module_dir/vendor/usb_cam/usb_cam.cpp" -nt "$binary" ] || \
   [ "$module_dir/vendor/usb_cam/include/usb_cam/usb_cam.h" -nt "$binary" ]; then
  bash "$module_dir/build.sh"
fi

if [ ! -e "$SEE3CAM_DEVICE" ]; then
  echo "ERROR: See3CAM device not found: $SEE3CAM_DEVICE" >&2
  echo "       available stable camera paths:" >&2
  find /dev/v4l/by-id -maxdepth 1 -type l -print 2>/dev/null >&2 || true
  exit 1
fi

# This camera's auto exposure selected ~31 ms during the 60 Hz test, reducing
# the delivered stream to ~15 Hz. Keep exposure below one frame period by
# default; all controls remain overridable for different lighting.
echo ">> Configuring See3CAM exposure/gain (USB control timeout: 15 s)"
if timeout --kill-after=2s 15s v4l2-ctl --device="$SEE3CAM_DEVICE" --set-ctrl="exposure_auto=${SEE3CAM_EXPOSURE_AUTO},exposure_absolute=${SEE3CAM_EXPOSURE_ABSOLUTE},gain=${SEE3CAM_GAIN}"; then
  :
else
  control_rc=$?
  echo "ERROR: See3CAM control setup failed (exit $control_rc): $SEE3CAM_DEVICE" >&2
  echo "       Capture has not started. An 'unknown control' can also follow failed USB control enumeration." >&2
  echo "       Check v4l2-ctl --list-ctrls and host kernel USB/UVC logs; requested exposure/gain were not confirmed." >&2
  exit "$control_rc"
fi

calib="/work/modules/sensor/see3cam-24cug/calibration/${SEE3CAM_SERIAL}.yaml"
camera_info_arg=()
if [ -f "$calib" ]; then
  camera_info_arg+=("_camera_info_url:=file://$calib")
  echo ">> See3CAM calibration: $calib"
else
  echo "WARN: no See3CAM calibration at $calib; CameraInfo K/D will be uncalibrated" >&2
fi

echo ">> See3CAM $SEE3CAM_DEVICE ${SEE3CAM_WIDTH}x${SEE3CAM_HEIGHT}@${SEE3CAM_FPS} $SEE3CAM_PIXEL_FORMAT exposure=${SEE3CAM_EXPOSURE_ABSOLUTE} gain=${SEE3CAM_GAIN}"
# The private node handle preserves the existing topic prefix. Both raw and
# compressed subscribers count as demand; no images are captured while idle.
exec taskset -c "${CPUS_CAMERA:?config/ros_env.sh not sourced}" \
  "$binary" \
    "__ns:=/landing" \
    "__name:=camera" \
    "_video_device:=$SEE3CAM_DEVICE" \
    "_image_width:=$SEE3CAM_WIDTH" \
    "_image_height:=$SEE3CAM_HEIGHT" \
    "_framerate:=$SEE3CAM_FPS" \
    "_pixel_format:=$SEE3CAM_PIXEL_FORMAT" \
    "_color_format:=yuv422p" \
    "_io_method:=mmap" \
    "_camera_name:=$SEE3CAM_CAMERA_NAME" \
    "_camera_frame_id:=$SEE3CAM_FRAME_ID" \
    "_rectify_fps:=$SEE3CAM_RECTIFY_FPS" \
    "${camera_info_arg[@]}" \
    "$@"

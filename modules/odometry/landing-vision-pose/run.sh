#!/bin/bash
# OptiTrack -> marker source transition adapter. MAVROS remains untouched and
# receives this output only when flight-safety estimation_source:=external with external_pose_topic:=/landing/vision_pose_selected.
# shellcheck disable=SC1090,SC1091
if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/select_stack.sh"
  dsd_select_stack "odometry/landing-vision-pose" || exit $?
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> odometry/landing-vision-pose'}"
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  __M="landing_vision_pose_adapter.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec "$__TT" "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/aruco-landing/devel/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh

: "${LANDING_ALLOW_MARKER_SWITCH:=false}"
: "${LANDING_AUTO_SWITCH:=false}"
: "${LANDING_AUTO_FALLBACK_TO_OPTITRACK:=false}"
: "${LANDING_MARKER_LOSS_TIMEOUT_S:=0.5}"

echo "[landing-vision-pose] OptiTrack is the initial source; marker switch allowed=$LANDING_ALLOW_MARKER_SWITCH auto=$LANDING_AUTO_SWITCH"
echo "[landing-vision-pose] candidate output: /landing/vision_pose_selected (flight-safety MUX chooses EKF2 input)"
echo "[landing-vision-pose] OptiTrack fallback=$LANDING_AUTO_FALLBACK_TO_OPTITRACK after ${LANDING_MARKER_LOSS_TIMEOUT_S}s marker loss"
exec taskset -c "${CPUS_ESTIMATION:?config/ros_env.sh not sourced}" \
  roslaunch aruco_landing landing_vision_pose_adapter.launch \
    allow_marker_switch:="$LANDING_ALLOW_MARKER_SWITCH" \
    auto_switch:="$LANDING_AUTO_SWITCH" \
    auto_fallback_to_optitrack:="$LANDING_AUTO_FALLBACK_TO_OPTITRACK" \
    marker_loss_timeout_s:="$LANDING_MARKER_LOSS_TIMEOUT_S" \
    require_trial_enable:="${LANDING_REQUIRE_TRIAL_ENABLE:-false}" \
    reject_inconsistent_marker:="${LANDING_REJECT_INCONSISTENT_MARKER:-false}" \
    "$@"

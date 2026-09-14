#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> simulation/airsim'}"
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){
    docker exec "$__C" pkill -INT -f "[a]irsim_camera_bridge" >/dev/null 2>&1 || true
    docker exec "$__C" pkill -INT -f "[a]irsim_camera_bridge.py" >/dev/null 2>&1 || true
  }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi

source /work/config/ros_env.sh
source /opt/ros/noetic/setup.bash
set -u
source /work/modules/ensure_roscore.sh

: "${AIRSIM_CAMERA_CONFIG:?stack must set AIRSIM_CAMERA_CONFIG}"
if [ -n "${AIRSIM_CAMERA_BRIDGE_BIN:-}" ]; then
  if [ ! -x "$AIRSIM_CAMERA_BRIDGE_BIN" ] && [ -n "${AIRSIM_CAMERA_BRIDGE_BUILD_SCRIPT:-}" ]; then
    bash "$AIRSIM_CAMERA_BRIDGE_BUILD_SCRIPT"
  fi
  [ -x "$AIRSIM_CAMERA_BRIDGE_BIN" ] || {
    echo "ERROR: AIRSIM_CAMERA_BRIDGE_BIN is not executable: $AIRSIM_CAMERA_BRIDGE_BIN" >&2
    exit 2
  }
  exec "$AIRSIM_CAMERA_BRIDGE_BIN" "_config_path:=$AIRSIM_CAMERA_CONFIG" "$@"
fi
exec python3 /work/modules/simulation/airsim/scripts/airsim_camera_bridge.py \
  "_config_path:=$AIRSIM_CAMERA_CONFIG" "$@"

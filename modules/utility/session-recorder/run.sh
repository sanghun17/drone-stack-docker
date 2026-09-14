#!/bin/bash
# Profile-driven rosbag session recorder. Hardware and simulation use the same
# node and service API; only the trigger adapter, webcam, topics and output path
# come from the stack-owned YAML.
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> utility/session-recorder'}"
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "[s]ession_recorder_node.py" >/dev/null 2>&1 || true; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec "$__TT" "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi

source /opt/ros/noetic/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh

: "${SESSION_RECORDER_CONFIG:?stack must set SESSION_RECORDER_CONFIG}"
if [ ! -f "$SESSION_RECORDER_CONFIG" ]; then
  echo "ERROR: session recorder config not found: $SESSION_RECORDER_CONFIG" >&2
  exit 2
fi

rosparam delete /session_recorder >/dev/null 2>&1 || true
rosparam load "$SESSION_RECORDER_CONFIG" /session_recorder
exec python3 /work/modules/utility/session-recorder/session_recorder_node.py "$@"

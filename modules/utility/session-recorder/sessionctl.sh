#!/bin/bash
# Start/stop the common recording session manually. This is the simulation
# trigger and the FC-off hardware bench-test trigger.
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> utility/session-recorder/sessionctl.sh'}"
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  exec docker exec "$__TT" "$__C" bash "/work/${__S#$__R/}" "$@"
fi

source /opt/ros/noetic/setup.bash
source /work/config/ros_env.sh

action="${1:-status}"
case "$action" in
  start)
    label="${2:-manual}"
    rosparam set /session_recorder/session_label "$label"
    rosservice call /session_recorder/set_recording "data: true"
    ;;
  stop)
    rosservice call /session_recorder/set_recording "data: false"
    ;;
  status)
    rostopic echo -n 1 /session_recorder/status
    ;;
  *)
    echo "usage: sessionctl.sh {start [label]|stop|status}" >&2
    exit 2
    ;;
esac

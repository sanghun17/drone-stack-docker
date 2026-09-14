#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  __C="${DSD_CONTAINER:-drone-stack-aruco-landing-jetson}"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1 || true
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "landing_controller" >/dev/null 2>&1 || true; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi

source /work/config/ros_env.sh
source /opt/ros/noetic/setup.bash
source /work/ws/aruco-landing/devel/setup.bash
set -u
source /work/modules/ensure_roscore.sh

exec roslaunch aruco_landing landing_controller.launch "$@"

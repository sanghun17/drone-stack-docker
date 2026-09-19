#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/select_stack.sh"
  dsd_select_stack "planner/aruco-landing" || exit $?
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1 || true
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "[r]oslaunch aruco_landing landing_trial.launch" >/dev/null 2>&1 || true; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi

source /work/config/ros_env.sh
source /opt/ros/noetic/setup.bash
source /work/ws/aruco-landing/devel/setup.bash
source /work/ws/flight-safety/devel/setup.bash --extend
set -u
source /work/modules/ensure_roscore.sh

exec taskset -c "${CPUS_CONTROL:?}" roslaunch aruco_landing landing_trial.launch "$@"

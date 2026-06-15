#!/bin/bash
# control/flight-safety: M2 reaction tier (safety.launch). *** KILL AUTHORITY ***
# Not in module.yml `run:` — start manually, intentionally, only when intervention is wanted.

if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __TT=$([ -t 1 ] && echo -it || echo -i)
  __M="roslaunch flight_safety safety.launch"
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi
set -e
source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh
exec roslaunch flight_safety safety.launch "$@"

#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  __C="${DSD_CONTAINER:-drone-stack-aruco-landing-sim-x86}"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1 || true
  __TT=$([ -t 1 ] && echo -it || echo -i)
  exec docker exec "$__TT" "$__C" bash "/work/${__S#$__R/}" "$@"
fi

source /work/config/ros_env.sh
source /opt/ros/noetic/setup.bash
source /work/ws/aruco-landing/devel/setup.bash
source /work/modules/ensure_roscore.sh

exec python3 /work/stack-assets/aruco-landing-sim-x86/scripts/airsim_control_adapter.py "$@"

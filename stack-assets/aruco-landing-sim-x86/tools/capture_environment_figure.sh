#!/bin/bash
# Capture the configured AirSim overview camera as a publication PNG.
# shellcheck disable=SC1090,SC1091
set -eo pipefail

if [ ! -f /.dockerenv ]; then
  __C="${DSD_CONTAINER:-drone-stack-aruco-landing-sim-x86}"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1 || true
  __TT=$([ -t 1 ] && echo -it || echo -i)
  __ARGS=("$@")
  for __I in "${!__ARGS[@]}"; do
    if [[ "${__ARGS[$__I]}" == "$__R/"* ]]; then
      __ARGS[$__I]="/work/${__ARGS[$__I]#$__R/}"
    fi
  done
  exec docker exec "$__TT" "$__C" bash "/work/${__S#$__R/}" "${__ARGS[@]}"
fi

source /opt/ros/noetic/setup.bash --extend

exec python3 /work/stack-assets/aruco-landing-sim-x86/scripts/capture_environment_figure.py "$@"

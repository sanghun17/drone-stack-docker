#!/bin/bash
# Risk-aware exploration runtime selected by RISK_AWARE_PROFILE=hardware|airsim.
set -e

if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/select_stack.sh"
  dsd_select_stack "planner/risk-aware" || exit $?
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> planner/risk-aware/run_planner.sh'}"
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __PROFILE="$(docker exec "$__C" printenv RISK_AWARE_PROFILE 2>/dev/null || true)"
  case "${__PROFILE:-hardware}" in
    hardware) __M="roslaunch active_3d_planning_app_reconstruction exploration_planner_d435i.launch" ;;
    airsim) __M="roslaunch active_3d_planning_app_reconstruction exploration_planner.launch" ;;
    *) echo "ERROR: invalid RISK_AWARE_PROFILE=$__PROFILE" >&2; exit 2 ;;
  esac
  __TT=$([ -t 1 ] && echo -it || echo -i)
  cleanup(){ docker exec "$__C" pkill -INT -f "$__M" >/dev/null 2>&1 || true; }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"; __rc=$?
  cleanup
  exit $__rc
fi

source /opt/ros/noetic/setup.bash
source /work/ws/risk-aware/devel/setup.bash
if [ "${RISK_AWARE_PROFILE:-hardware}" = airsim ]; then
  source /work/config/sim.env
fi
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh

case "${RISK_AWARE_PROFILE:-hardware}" in
  hardware)
    mkdir -p /work/flight_logs
    __PLOG="/work/flight_logs/planner_stdout_$(date +%F_%H-%M-%S).log"
    echo "[run_planner] tee planner stdout -> $__PLOG"
    set -o pipefail
    taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" \
      roslaunch active_3d_planning_app_reconstruction exploration_planner_d435i.launch "$@" \
      2>&1 | tee "$__PLOG"
    ;;
  airsim)
    exec roslaunch active_3d_planning_app_reconstruction exploration_planner.launch "$@"
    ;;
  *) echo "ERROR: RISK_AWARE_PROFILE must be hardware or airsim" >&2; exit 2 ;;
esac

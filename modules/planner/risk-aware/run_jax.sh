#!/bin/bash
# Risk-aware JAX/local-control runtime selected by RISK_AWARE_PROFILE.
set -e

if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/scripts/lib/select_stack.sh"
  dsd_select_stack "planner/risk-aware" || exit $?
  : "${DSD_CONTAINER:?set DSD_CONTAINER or invoke through './setup.sh run <stack> planner/risk-aware/run_jax.sh'}"
  __C="$DSD_CONTAINER"
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  source "$__R/scripts/lib/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  __PROFILE="$(docker exec "$__C" printenv RISK_AWARE_PROFILE 2>/dev/null || true)"
  case "${__PROFILE:-hardware}" in
    hardware) __M="jax_main_node_ros_new.py" ;;
    airsim) __M="roslaunch local_controller ours_jax.launch" ;;
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
source /work/scripts/lib/ensure_roscore.sh

export XLA_PYTHON_CLIENT_PREALLOCATE="${XLA_PYTHON_CLIENT_PREALLOCATE:-false}"
export XLA_PYTHON_CLIENT_MEM_FRACTION="${XLA_PYTHON_CLIENT_MEM_FRACTION:-0.3}"
export PYTHONUNBUFFERED=1

case "${RISK_AWARE_PROFILE:-hardware}" in
  hardware)
    JAX="/work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py"
    exec taskset -c "${CPUS_POOL:?config/ros_env.sh not sourced}" \
      python3 "$JAX" --gpu 0 --planner motion_primitives --mode exploration \
      /imu:=/mavros/imu/data "$@"
    ;;
  airsim)
    exec roslaunch local_controller ours_jax.launch gpu:=1 planner:=motion_primitives \
      localization:="${LOC:-gt}" "$@"
    ;;
  *) echo "ERROR: RISK_AWARE_PROFILE must be hardware or airsim" >&2; exit 2 ;;
esac

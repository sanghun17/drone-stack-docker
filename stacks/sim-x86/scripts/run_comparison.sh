#!/usr/bin/env bash
# July 2026 comparison runtime, isolated from the default hardware/JAX profile.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ ! -f /.dockerenv ]; then
  case "${1:-}" in
    sensor) MATCH='roslaunch active_3d_planning_app_reconstruction airsim_sensor_punlisher.launch' ;;
    voxblox) MATCH='roslaunch active_3d_planning_app_reconstruction uncertainty_voxblox.launch' ;;
    pure-global) MATCH='roslaunch active_3d_planning_app_reconstruction exploration_planner.launch' ;;
    pure-local) MATCH='python3 /work/ws/risk-aware-comparison/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py' ;;
    la) MATCH='roslaunch la_planner_bridge la_planner_airsim.launch' ;;
    control) MATCH='roslaunch local_controller ours_so3_stack.launch' ;;
    initialize) MATCH='python3 /work/ws/risk-aware-comparison/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts/initialize_simulator.py' ;;
    eval) MATCH='roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch' ;;
    rhem) MATCH='roslaunch dsd_rhem_bridge rhem.launch' ;;
    validate) MATCH='python3 /work/stacks/sim-x86/scripts/validate_planner_chain.py' ;;
    reset) MATCH='python3 /work/stacks/sim-x86/scripts/reset_comparison.py' ;;
    *) MATCH='' ;;
  esac
  cleanup() {
    if [ -n "$MATCH" ]; then
      docker exec drone-stack-sim-x86 pkill -INT -f "$MATCH" >/dev/null 2>&1 || true
    fi
  }
  trap 'cleanup; exit 130' INT TERM HUP
  docker exec -i drone-stack-sim-x86 bash /work/stacks/sim-x86/scripts/run_comparison.sh "$@"
  exit $?
fi
source /opt/ros/noetic/setup.bash
source "$ROOT/ws/risk-aware-comparison/devel/setup.bash"
source "$ROOT/config/sim.env"
source "$ROOT/config/ros_env.sh"
export RISK_AWARE_CHECKPOINTS="$RISK_AWARE_CHECKPOINTS/comparison-20260706"
export PYTHONUNBUFFERED=1
REPO="$ROOT/ws/risk-aware-comparison/src/risk_aware_planning"
COMPONENT="${1:?usage: run_comparison.sh config|reset|sensor|initialize|voxblox|pure-global|pure-local|la|rhem|control|eval|validate [arguments]}"
shift
case "$COMPONENT" in
  config)
    rosparam load "$ROOT/stacks/sim-x86/config/comparison-20260706-sensors.yaml" /
    exec rosparam load "$ROOT/stacks/sim-x86/config/comparison-20260706-params.yaml" /
    ;;
  reset)
    exec python3 "$ROOT/stacks/sim-x86/scripts/reset_comparison.py" "$@"
    ;;
  validate)
    exec python3 "$ROOT/stacks/sim-x86/scripts/validate_planner_chain.py" "$@"
    ;;
  rhem)
    exec bash "$ROOT/stacks/sim-x86/scripts/run_rhem.sh" "$@"
    ;;
  sensor)
    exec roslaunch active_3d_planning_app_reconstruction airsim_sensor_punlisher.launch localization:=vio "$@"
    ;;
  initialize)
    exec python3 "$REPO/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts/initialize_simulator.py" "$@"
    ;;
  voxblox)
    exec roslaunch active_3d_planning_app_reconstruction uncertainty_voxblox.launch "$@"
    ;;
  pure-global)
    exec roslaunch active_3d_planning_app_reconstruction exploration_planner.launch "$@"
    ;;
  pure-local)
    export XLA_PYTHON_CLIENT_PREALLOCATE=false
    export XLA_PYTHON_CLIENT_MEM_FRACTION=0.3
    export PYTHONUNBUFFERED=1
    # Execute the pinned Python source directly; the old ROS wrapper assumes
    # ~/risk-aware_planning. Both nodes are scoped to this process group.
    roslaunch local_controller ours_jax_adapter.launch &
    ADAPTER_PID=$!
    trap 'kill -INT "$ADAPTER_PID" 2>/dev/null || true' EXIT
    python3 "$REPO/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py" \
      --gpu 1 --planner motion_primitives --mode exploration "$@"
    ;;
  la)
    export PLANNER_PROFILE=comparison-20260706
    exec bash "$ROOT/modules/planner/la-planner/run.sh" "$@"
    ;;
  control)
    exec roslaunch local_controller ours_so3_stack.launch "$@"
    ;;
  eval)
    mkdir -p "$ROOT/flight_logs/comparison"
    exec roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch \
      data_directory:="$ROOT/flight_logs/comparison" "$@"
    ;;
  *) echo "Unknown comparison component: $COMPONENT" >&2; exit 2 ;;
esac

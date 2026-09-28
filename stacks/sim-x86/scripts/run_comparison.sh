#!/usr/bin/env bash
# July 2026 comparison runtime, isolated from the default hardware/JAX profile.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-sim-x86
  __R="$ROOT"
  source "$ROOT/scripts/lib/ensure_container.sh"
  docker start "$__C" >/dev/null
  case "${1:-}" in
    fast) MATCH='roslaunch.*(mapping_simulator_openvins.launch|fast_livo_gt_diagnostic.launch)' ;;
    sensor) MATCH='(roslaunch.*airsim_sensor_(punlisher|pipeline).launch|bash .*/run_airsim_sensors.sh)' ;;
    voxblox) MATCH='roslaunch active_3d_planning_app_reconstruction uncertainty_voxblox.launch' ;;
    pure-global) MATCH='roslaunch active_3d_planning_app_reconstruction exploration_planner.launch' ;;
    pure-local) MATCH='python3 /work/ws/risk-aware-comparison/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/jax_main_node_ros_new.py' ;;
    la) MATCH='roslaunch la_planner_bridge la_planner_airsim.launch' ;;
    control) MATCH='roslaunch.*comparison_control.launch' ;;
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
COMPONENT="${1:?usage: run_comparison.sh config|config-gt|sources|reset|sensor|fast|initialize|voxblox|pure-global|pure-local|la|rhem|control|eval|validate [arguments]}"
shift
case "$COMPONENT" in
  sensor|initialize|reset) python3 "$ROOT/stacks/sim-x86/scripts/wait_airsim.py" ;;
esac
case "$COMPONENT" in
  config|config-gt)
    python3 "$ROOT/stacks/sim-x86/scripts/configure_sources.py" --check-only "$@"
    rosparam load "$ROOT/stacks/sim-x86/config/comparison-20260706-sensors.yaml" /
    rosparam load "$ROOT/stacks/sim-x86/config/comparison-20260706-params.yaml" /
    if [ "$COMPONENT" = config-gt ]; then
      set -- --planning-source gt --control-source gt "$@"
    fi
    exec python3 "$ROOT/stacks/sim-x86/scripts/configure_sources.py" "$@"
    ;;
  sources)
    exec rosparam get /comparison/sources
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
  fast)
    source "$ROOT/ws/fast-livo-sim/devel/setup.bash" --extend
    if [ "$(rosparam get /system/localization)" = gt ]; then
      exec roslaunch "$ROOT/stacks/sim-x86/config/launch/fast_livo_gt_diagnostic.launch" "$@"
    fi
    exec roslaunch fast_livo mapping_simulator_openvins.launch "$@"
    ;;
  sensor)
    python3 "$ROOT/stacks/sim-x86/scripts/comparison_odom_sources.py" &
    SOURCES_PID=$!
    trap 'kill -INT "$SOURCES_PID" 2>/dev/null || true' EXIT
    if [ "$(rosparam get /comparison/sensor_calibration 2>/dev/null || echo historical)" = airsim ]; then
      RANGE_RAYS=false
      if [ "$(rosparam get /comparison/rhem_map_rays 2>/dev/null || echo clipped)" = full ]; then
        RANGE_RAYS=true
      fi
      bash "$ROOT/stacks/sim-x86/scripts/run_airsim_sensors.sh" \
        localization:="$(rosparam get /system/localization)" publish_range_rays:="$RANGE_RAYS" "$@"
    else
      # Explicit historical replay keeps its original publisher and geometry.
      roslaunch active_3d_planning_app_reconstruction airsim_sensor_punlisher.launch \
        localization:="$(rosparam get /system/localization)" "$@"
    fi
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
    exec roslaunch "$ROOT/stacks/sim-x86/config/launch/comparison_control.launch" \
      planning_odom_topic:="$(rosparam get /system/odom_topic)" \
      control_odom_topic:="$(rosparam get /comparison/sources/control_odom_topic)" "$@"
    ;;
  eval)
    mkdir -p "$ROOT/flight_logs/comparison"
    exec roslaunch active_3d_planning_app_reconstruction runtime_evaluator.launch \
      data_directory:="$ROOT/flight_logs/comparison" "$@"
    ;;
  *) echo "Unknown comparison component: $COMPONENT" >&2; exit 2 ;;
esac

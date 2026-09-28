#!/usr/bin/env bash
# Stack-owned sensor wiring and MixTraj conversion; generic RHEM lives in modules/.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-sim-x86
  __R="$ROOT"
  source "$ROOT/scripts/lib/ensure_container.sh"
  docker start "$__C" >/dev/null
  trap 'docker exec drone-stack-sim-x86 pkill -INT -f "^/usr/bin/python3 /opt/ros/noetic/bin/roslaunch dsd_rhem_bridge rhem.launch" || true' INT TERM HUP
  docker exec -i drone-stack-sim-x86 bash /work/stacks/sim-x86/scripts/run_rhem.sh "$@"
  exit $?
fi
source /opt/ros/noetic/setup.bash
source "$ROOT/ws/risk-aware-comparison/devel/setup.bash"
source "$ROOT/ws/rhem/devel/setup.bash" --extend
source "$ROOT/config/sim.env"
source "$ROOT/config/ros_env.sh"
export PYTHONUNBUFFERED=1
python3 "$ROOT/stacks/sim-x86/scripts/prepare_rhem_runtime.py"
CLOUD_TOPIC=/voxel_grid/output
if [ "$(rosparam get /comparison/rhem_map_rays 2>/dev/null || echo clipped)" = full ]; then
  CLOUD_TOPIC=/voxel_grid/rays
fi
roslaunch dsd_rhem_bridge rhem.launch \
  odom_topic:="$(rosparam get /system/odom_topic)" \
  imu_topic:="$(rosparam get /system/imu_topic)" \
  image_topic:=/camera/left/image_raw cloud_topic:="$CLOUD_TOPIC" world_frame:=odom \
  filter_config:="$ROOT/.build/sim-x86/rhem/rovio.info" \
  camera_config:="$ROOT/.build/sim-x86/rhem/camera.yaml" \
  planner_config:="$ROOT/.build/sim-x86/rhem/planner.yaml" "$@" &
PLANNER_PID=$!
python3 "$ROOT/stacks/sim-x86/scripts/rhem_control_adapter.py" __ns:=/rhem &
ADAPTER_PID=$!
PROBE_PID=
if [ "$(rosparam get /comparison/rhem_diagnostics 2>/dev/null || echo false)" = true ]; then
  python3 "$ROOT/stacks/sim-x86/scripts/rhem_belief_probe.py" &
  PROBE_PID=$!
fi
cleanup() {
  kill -INT "$ADAPTER_PID" "$PLANNER_PID" ${PROBE_PID:+"$PROBE_PID"} 2>/dev/null || true
  wait "$ADAPTER_PID" "$PLANNER_PID" ${PROBE_PID:+"$PROBE_PID"} 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
wait -n "$PLANNER_PID" "$ADAPTER_PID"

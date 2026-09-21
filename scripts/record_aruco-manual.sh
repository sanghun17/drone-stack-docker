#!/usr/bin/env bash
# Jetson host entrypoint. Prepare observes only; recording starts only on start.
set -eo pipefail
case "${1:-}" in
  prepare|check|down) tool=manual_runtime.py ;;
  start|stop|status) tool=capture.py ;;
  *) echo 'Usage: bash scripts/record_aruco-manual.sh {prepare|check|start|stop|status|down}'; exit 2 ;;
esac
command="$1"
if [ ! -f /.dockerenv ]; then
  source "$(cd "$(dirname "${BASH_SOURCE[0]}")/lib" && pwd)/select_stack.sh"
  dsd_select_stack "perception/aruco-landing"
  exec docker exec "$DSD_CONTAINER" bash /work/scripts/record_aruco-manual.sh "$command"
fi
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
source /opt/ros/noetic/setup.bash
source /work/ws/aruco-landing/devel/setup.bash --extend
source /work/ws/flight-safety/devel/setup.bash --extend
source /work/config/ros_env.sh
args=("$command")
[ "$tool" != capture.py ] || args+=(--profile manual-flight)
exec python3 "/work/stacks/aruco-landing-jetson/scripts/pose_transition/$tool" "${args[@]}"

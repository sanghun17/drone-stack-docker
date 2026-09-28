#!/usr/bin/env bash
# Called inside a sourced ROS environment by either Risk runtime entrypoint.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
export AIRSIM_CAMERA_CONFIG="$ROOT/stacks/sim-x86/config/airsim_cameras.yaml"
bash "$ROOT/modules/simulation/airsim/run_camera.sh" &
CAMERA_PID=$!
roslaunch "$ROOT/stacks/sim-x86/config/launch/airsim_sensor_pipeline.launch" "$@" &
PIPELINE_PID=$!
cleanup() {
  kill -INT "$CAMERA_PID" "$PIPELINE_PID" 2>/dev/null || true
  wait "$CAMERA_PID" "$PIPELINE_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
set +e
wait -n "$CAMERA_PID" "$PIPELINE_PID"
code=$?
exit "$code"

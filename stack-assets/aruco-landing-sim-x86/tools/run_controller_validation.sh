#!/bin/bash
# Run a marker-only baseline-pad campaign for horizontal-controller validation.
# Simulator ground truth is subscribed by the trial runner for metrics only.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stack-assets/aruco-landing-sim-x86"
CONTAINER="${DSD_CONTAINER:-drone-stack-aruco-landing-sim-x86}"
TRIALS="${ARUCO_VALIDATION_TRIALS:-10}"
SEED="${ARUCO_VALIDATION_SEED:-2909}"
LABEL="${ARUCO_VALIDATION_LABEL:-controller-validation}"
OUTPUT_DIR="${ARUCO_VALIDATION_OUTPUT_DIR:-$ROOT/experiments/aruco-landing/controller-validation}"
MINIMUM_RATE="${ARUCO_VALIDATION_MINIMUM_RATE_HZ:-59.5}"
KP="${ARUCO_VALIDATION_KP:-}"
KD="${ARUCO_VALIDATION_KD:-}"
BUILD_MAP="${ARUCO_VALIDATION_BUILD_MAP:-0}"
RECORD_BAGS="${ARUCO_VALIDATION_RECORD_BAGS:-0}"
LAYOUT_HOST="${ARUCO_VALIDATION_LAYOUT:-$ROOT/ws/aruco-landing/src/aruco_landing/config/paper_pad_layout.yaml}"
PAD_PREFIX="${ARUCO_VALIDATION_PAD_PREFIX:-yang_baseline}"
LAYOUT_CONTAINER="/work/${LAYOUT_HOST#$ROOT/}"
RUN_DIR="$OUTPUT_DIR/$LABEL"
LOG_DIR="$RUN_DIR/logs"
LOG_DIR_CONTAINER="/work/${LOG_DIR#$ROOT/}"
MANIFEST_HOST="$OUTPUT_DIR/matched_staging_seed_${SEED}.json"
MANIFEST_CONTAINER="/work/${MANIFEST_HOST#$ROOT/}"
SIM_PID=""

stop_ros_nodes() {
  docker exec "$CONTAINER" bash -lc '
    pkill -TERM -f "[a]irsim_landing_camera_bridge" || true
    pkill -TERM -f "[p]aper_pad_estimator" || true
    pkill -TERM -f "[l]anding_controller" || true
    pkill -TERM -f "[a]irsim_control_adapter.py" || true
    pkill -INT -f "[r]osbag record" || true
  ' >/dev/null 2>&1 || true
  sleep 1
  # The adapter can block in an AirSim RPC while the simulator is exiting and
  # therefore not service SIGTERM. Escalate only these landing-node patterns.
  docker exec "$CONTAINER" bash -lc '
    pkill -KILL -f "[a]irsim_landing_camera_bridge" || true
    pkill -KILL -f "[p]aper_pad_estimator" || true
    pkill -KILL -f "[l]anding_controller" || true
    pkill -KILL -f "[a]irsim_control_adapter.py" || true
    pkill -KILL -f "[r]osbag record" || true
  ' >/dev/null 2>&1 || true
}

stop_simulator() {
  if [ -n "$SIM_PID" ] && kill -0 "$SIM_PID" 2>/dev/null; then
    kill -TERM "$SIM_PID" 2>/dev/null || true
    for _ in $(seq 1 5); do
      kill -0 "$SIM_PID" 2>/dev/null || break
      sleep 1
    done
    kill -KILL "$SIM_PID" 2>/dev/null || true
    wait "$SIM_PID" 2>/dev/null || true
  fi
  SIM_PID=""
}

cleanup() {
  stop_ros_nodes
  stop_simulator
}
trap cleanup EXIT INT TERM HUP

wait_for_rpc() {
  for _ in $(seq 1 90); do
    if ss -ltn | rg -q ':41451\b'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

mkdir -p "$LOG_DIR"
docker start "$CONTAINER" >/dev/null

if [ "$BUILD_MAP" = 1 ]; then
  AIRSIM_LANDING_PAD_LAYOUT="$LAYOUT_HOST" AIRSIM_LANDING_PAD_PREFIX="$PAD_PREFIX" \
    "$MODULE/tools/build_baseline_map.sh" >"$LOG_DIR/map_build.log" 2>&1
fi

"$MODULE/tools/run_baseline_sim.sh" -RenderOffscreen -graphicsadapter=2 \
  -Unattended -NoSplash >"$LOG_DIR/simulator.log" 2>&1 &
SIM_PID=$!
if ! wait_for_rpc; then
  echo "AirSim RPC did not become ready" >&2
  exit 1
fi

stop_ros_nodes
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/simulation/airsim/run_camera.sh >'$LOG_DIR_CONTAINER/camera.log' 2>&1"
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/perception/aruco-landing/run_estimator.sh pad_size_m:=0.7 layout_file:='$LAYOUT_CONTAINER' >'$LOG_DIR_CONTAINER/estimator.log' 2>&1"

controller_args=""
if [ -n "$KP" ] || [ -n "$KD" ]; then
  if [ -z "$KP" ] || [ -z "$KD" ]; then
    echo "Set both ARUCO_VALIDATION_KP and ARUCO_VALIDATION_KD" >&2
    exit 2
  fi
  controller_args="override_gains:=true kp_xy:=$KP kd_xy:=$KD"
fi
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/control/aruco-landing/run.sh $controller_args >'$LOG_DIR_CONTAINER/controller.log' 2>&1"
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/stack-assets/aruco-landing-sim-x86/tools/run_control.sh >'$LOG_DIR_CONTAINER/control.log' 2>&1"

# UE reports RPC-ready before the render/mmap stream reaches steady cadence.
sleep 10
runner_args=(
  --pad-name "$LABEL"
  --trials "$TRIALS"
  --seed "$SEED"
  --manifest "$MANIFEST_CONTAINER"
  --output-dir "/work/${OUTPUT_DIR#$ROOT/}"
  --minimum-camera-rate "$MINIMUM_RATE"
  --lateral-bound 0.05
  --lateral-eval-height-max 0.50
  --maximum-technical-retries 8
)
if [ -f "$RUN_DIR/summary.json" ]; then
  runner_args+=(--resume)
fi
if [ "$RECORD_BAGS" != 1 ]; then
  runner_args+=(--no-bag)
fi
"$MODULE/tools/run_trials.sh" "${runner_args[@]}" | tee "$LOG_DIR/trials.log"

python3 - "$RUN_DIR/summary.json" <<'PY'
import json, sys
summary = json.load(open(sys.argv[1], encoding="utf-8"))
worst_linf = summary["maximum_low_altitude_lateral_linf_m"]
print("validation:", json.dumps({
    "trials": summary["trial_count"],
    "successes": summary["success_count"],
    "within_5cm": summary["lateral_bound_met_count"],
    "all_within_5cm": summary["all_trials_meet_lateral_bound"],
    "worst_linf_cm": None if worst_linf is None else 100.0 * worst_linf,
    "min_camera_hz": summary["minimum_trial_camera_source_rate_hz"],
    "max_pose_age_p95_ms": summary["maximum_trial_marker_pose_age_p95_ms"],
}, indent=2))
PY

#!/bin/bash
# Run the matched 10-trial AirSim campaign for the Yang baseline and proposed pads.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
MODULE="$ROOT/stack-assets/aruco-landing-sim-x86"
CONTAINER="${DSD_CONTAINER:-drone-stack-aruco-landing-sim-x86}"
OUTPUT_DIR="${ARUCO_CAMPAIGN_OUTPUT_DIR:-$ROOT/experiments/aruco-landing/final-L070-h020}"
TRIALS="${ARUCO_CAMPAIGN_TRIALS:-10}"
SEED="${ARUCO_CAMPAIGN_SEED:-1701}"
MINIMUM_RATE="${ARUCO_CAMPAIGN_MINIMUM_RATE_HZ:-59.5}"
INCLUDE_BASELINE="${ARUCO_CAMPAIGN_INCLUDE_BASELINE:-1}"
MANIFEST_HOST="$OUTPUT_DIR/matched_staging_seed_${SEED}.json"
MANIFEST_CONTAINER="/work/${MANIFEST_HOST#$ROOT/}"
SIM_PID=""

declare -a CONDITION_NAMES=(
  "yang-baseline"
  "proposed-pad-1"
  "proposed-pad-3"
  "proposed-pad-4"
  "proposed-pad-5"
)
declare -a PAD_PREFIXES=(
  "yang_baseline"
  "proposed_pad_1"
  "proposed_pad_3"
  "proposed_pad_4"
  "proposed_pad_5"
)
declare -a LAYOUTS=(
  "$ROOT/ws/aruco-landing/src/aruco_landing/config/paper_pad_layout.yaml"
  "$ROOT/ws/aruco-landing/src/aruco_landing/config/proposed_pad_1_layout.yaml"
  "$ROOT/ws/aruco-landing/src/aruco_landing/config/proposed_pad_3_layout.yaml"
  "$ROOT/ws/aruco-landing/src/aruco_landing/config/proposed_pad_4_layout.yaml"
  "$ROOT/ws/aruco-landing/src/aruco_landing/config/proposed_pad_5_layout.yaml"
)

if [ "$INCLUDE_BASELINE" != 1 ]; then
  CONDITION_NAMES=("${CONDITION_NAMES[@]:1}")
  PAD_PREFIXES=("${PAD_PREFIXES[@]:1}")
  LAYOUTS=("${LAYOUTS[@]:1}")
fi

stop_ros_nodes() {
  docker exec "$CONTAINER" bash -lc '
    pkill -TERM -f "[a]irsim_landing_camera_bridge" || true
    pkill -TERM -f "[p]aper_pad_estimator" || true
    pkill -TERM -f "[l]anding_controller" || true
    pkill -TERM -f "[a]irsim_control_adapter.py" || true
    pkill -INT -f "[r]osbag record" || true
  ' >/dev/null 2>&1 || true
  sleep 1
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
      if ! kill -0 "$SIM_PID" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    if kill -0 "$SIM_PID" 2>/dev/null; then
      kill -KILL "$SIM_PID" 2>/dev/null || true
    fi
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
  local attempt
  for attempt in $(seq 1 90); do
    if ss -ltn | rg -q ':41451\b'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_ros_nodes() {
  local layout_container="$1"
  local log_dir_container="$2"
  docker exec -d "$CONTAINER" bash -lc \
    "exec /work/modules/simulation/airsim/run_camera.sh >'$log_dir_container/camera.log' 2>&1"
  docker exec -d "$CONTAINER" bash -lc \
    "exec /work/modules/perception/aruco-landing/run_estimator.sh pad_size_m:=0.7 layout_file:='$layout_container' >'$log_dir_container/estimator.log' 2>&1"
  docker exec -d "$CONTAINER" bash -lc \
    "exec /work/modules/control/aruco-landing/run.sh >'$log_dir_container/controller.log' 2>&1"
  docker exec -d "$CONTAINER" bash -lc \
    "exec /work/stack-assets/aruco-landing-sim-x86/tools/run_control.sh >'$log_dir_container/control.log' 2>&1"
}

make_success_artifacts() {
  local condition="$1"
  local run_dir="$OUTPUT_DIR/$condition"
  local bag_relative
  bag_relative="$(python3 - "$run_dir/trials.csv" <<'PY'
import csv, sys
with open(sys.argv[1], newline="", encoding="utf-8") as stream:
    for row in csv.DictReader(stream):
        if row["success"].lower() == "true":
            print(row["bag"])
            break
PY
)"
  if [ -z "$bag_relative" ]; then
    echo "[$condition] no successful trial; MP4 not generated"
    return 0
  fi
  local paper_dir="$run_dir/paper"
  local bag_host="$run_dir/$bag_relative"
  mkdir -p "$paper_dir"
  "$MODULE/tools/make_trial_figure.sh" "$bag_host" \
    --output-dir "$paper_dir" --video-fps 60
  local base
  base="$(basename "${bag_host%.bag}")"
  ffmpeg -y -v warning -i "$paper_dir/${base}_rgb.mp4" -an \
    -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p -g 1 -bf 0 \
    "$paper_dir/${condition}_successful_rgb_60fps_h264.mp4"
  ffmpeg -v error -i "$paper_dir/${condition}_successful_rgb_60fps_h264.mp4" \
    -f null -
}

mkdir -p "$OUTPUT_DIR"
docker start "$CONTAINER" >/dev/null

for index in "${!CONDITION_NAMES[@]}"; do
  condition="${CONDITION_NAMES[$index]}"
  prefix="${PAD_PREFIXES[$index]}"
  layout_host="${LAYOUTS[$index]}"
  layout_container="/work/${layout_host#$ROOT/}"
  run_dir="$OUTPUT_DIR/$condition"
  log_dir="$run_dir/logs"
  log_dir_container="/work/${log_dir#$ROOT/}"
  resume_args=()

  if [ -f "$run_dir/summary.json" ]; then
    completed="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("trial_count", 0))' "$run_dir/summary.json")"
    if [ "$completed" -eq "$TRIALS" ]; then
      echo "[$condition] already contains $TRIALS trials; preserving it"
      make_success_artifacts "$condition"
      continue
    fi
    if [ "$completed" -gt "$TRIALS" ]; then
      echo "[$condition] existing result has too many rows ($completed/$TRIALS)" >&2
      exit 1
    fi
    echo "[$condition] resuming after $completed/$TRIALS accepted trials"
    resume_args=(--resume)
  fi

  mkdir -p "$log_dir"
  echo "[$condition] building metric 0.70 m pad map"
  AIRSIM_LANDING_PAD_LAYOUT="$layout_host" AIRSIM_LANDING_PAD_PREFIX="$prefix" \
    "$MODULE/tools/build_baseline_map.sh" >"$log_dir/map_build.log" 2>&1

  condition_attempt=0
  while true; do
    condition_attempt=$((condition_attempt + 1))
    if [ "$condition_attempt" -gt 5 ]; then
      echo "[$condition] exceeded five simulator recovery attempts" >&2
      exit 1
    fi
    echo "[$condition] starting AirSim (condition attempt $condition_attempt)"
    "$MODULE/tools/run_baseline_sim.sh" -RenderOffscreen -graphicsadapter=2 \
      -Unattended -NoSplash >>"$log_dir/simulator.log" 2>&1 &
    SIM_PID=$!
    if ! wait_for_rpc; then
      echo "[$condition] AirSim RPC did not become ready; restarting" >&2
      stop_simulator
      continue
    fi

    stop_ros_nodes
    start_ros_nodes "$layout_container" "$log_dir_container"
    # RPC becomes available before UE texture/shader streaming reaches steady
    # cadence. Give the camera path time to warm before the runner's own checks.
    sleep 10
    echo "[$condition] collecting remaining matched trials"
    attempt_log="$log_dir/trials_attempt_${condition_attempt}.log"
    set +e
    "$MODULE/tools/run_trials.sh" \
      --pad-name "$condition" \
      --trials "$TRIALS" \
      --seed "$SEED" \
      --manifest "$MANIFEST_CONTAINER" \
      --output-dir "/work/${OUTPUT_DIR#$ROOT/}" \
      --minimum-camera-rate "$MINIMUM_RATE" \
      --maximum-technical-retries 8 \
      "${resume_args[@]}" >"$attempt_log" 2>&1 &
    runner_pid=$!
    simulator_failed=0
    while kill -0 "$runner_pid" 2>/dev/null; do
      if ! kill -0 "$SIM_PID" 2>/dev/null; then
        simulator_failed=1
        echo "[$condition] AirSim exited during collection; stopping runner"
        kill -INT "$runner_pid" 2>/dev/null || true
        sleep 2
        kill -KILL "$runner_pid" 2>/dev/null || true
        break
      fi
      sleep 1
    done
    wait "$runner_pid"
    runner_status=$?
    set -e
    tee -a "$log_dir/trials.log" <"$attempt_log"
    stop_ros_nodes
    stop_simulator

    completed=0
    if [ -f "$run_dir/summary.json" ]; then
      completed="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("trial_count", 0))' "$run_dir/summary.json")"
    fi
    if [ "$completed" -eq "$TRIALS" ]; then
      break
    fi
    echo "[$condition] recovery required after $completed/$TRIALS accepted trials (runner=$runner_status simulator_failed=$simulator_failed)"
    resume_args=(--resume)
  done

  make_success_artifacts "$condition"
  echo "[$condition] complete"
done

python3 "$MODULE/scripts/summarize_pad_campaign.py" "$OUTPUT_DIR" \
  --conditions "${CONDITION_NAMES[@]}"
echo "Campaign complete: $OUTPUT_DIR"

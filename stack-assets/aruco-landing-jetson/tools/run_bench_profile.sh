#!/bin/bash
# FC/OptiTrack-independent camera -> estimator -> controller throughput profile.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTAINER="${DSD_CONTAINER:-drone-stack-aruco-landing-jetson}"
DURATION="${ARUCO_BENCH_DURATION_S:-20}"
LABEL="${ARUCO_BENCH_LABEL:-aruco-bench}"
REQUIRE_WEBCAM="${ARUCO_BENCH_REQUIRE_WEBCAM:-1}"
WEBCAM_WAIT_S="${ARUCO_BENCH_WEBCAM_WAIT_S:-30}"
LAYOUT_HOST="${ARUCO_HARDWARE_LAYOUT:-$ROOT/stack-assets/aruco-landing-jetson/config/bench_pad_layout.yaml}"
DICTIONARY="${ARUCO_HARDWARE_DICTIONARY:-DICT_7X7_50}"
PAD_SIZE_M="${ARUCO_HARDWARE_PAD_SIZE_M:-0.64}"
LAYOUT_CONTAINER="/work/${LAYOUT_HOST#$ROOT/}"
OUTPUT_HOST="${ARUCO_BENCH_OUTPUT_DIR:-$ROOT/experiments/aruco-landing/hardware-profiles}"
OUTPUT_CONTAINER="/work/${OUTPUT_HOST#$ROOT/}"
LOG_HOST="$OUTPUT_HOST/logs"
LOG_CONTAINER="/work/${LOG_HOST#$ROOT/}"
STARTED_ROSCORE=0

if [ ! -f "$LAYOUT_HOST" ]; then
  echo "ERROR: layout file not found: $LAYOUT_HOST" >&2
  exit 2
fi

cleanup() {
  docker exec "$CONTAINER" bash -lc '
    rosservice call /landing_controller/enable "data: false" >/dev/null 2>&1 || true
    rosservice call /session_recorder/set_recording "data: false" >/dev/null 2>&1 || true
    pkill -TERM -f "[u]sb_cam_node" || true
    pkill -TERM -f "[p]aper_pad_estimator" || true
    pkill -TERM -f "[p]ad_relative_vehicle_state.py" || true
    pkill -TERM -f "[l]anding_controller_node.py" || true
    pkill -TERM -f "[s]ession_recorder_node.py" || true
    pkill -TERM -f "[s]tatic_transform_publisher.*see3cam_optical_frame" || true
  ' >/dev/null 2>&1 || true
  if [ "$STARTED_ROSCORE" -eq 1 ]; then
    docker exec "$CONTAINER" bash -lc '
      pkill -TERM -f "[r]osmaster.*--core" || true
      pkill -TERM -f "[r]oscore" || true
    ' >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM HUP

mkdir -p "$LOG_HOST"
docker start "$CONTAINER" >/dev/null
cleanup

if ! docker exec "$CONTAINER" bash -lc \
  'source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; rosparam get /run_id' \
  >/dev/null 2>&1; then
  docker exec -d "$CONTAINER" bash -lc \
    "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; exec roscore >'$LOG_CONTAINER/roscore.log' 2>&1"
  STARTED_ROSCORE=1
  for _ in $(seq 1 50); do
    if docker exec "$CONTAINER" bash -lc \
      'source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; rosparam get /run_id' \
      >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done
  if ! docker exec "$CONTAINER" bash -lc \
    'source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; rosparam get /run_id' \
    >/dev/null 2>&1; then
    echo "ERROR: ROS master did not become ready" >&2
    exit 1
  fi
fi

webcam_ready=0
for _ in $(seq 1 "$WEBCAM_WAIT_S"); do
  if docker exec "$CONTAINER" bash -lc '
    source /opt/ros/noetic/setup.bash
    source /work/config/ros_env.sh
    rosservice type /recorder/start >/dev/null &&
      rosservice type /recorder/stop >/dev/null
  ' >/dev/null 2>&1; then
    webcam_ready=1
    break
  fi
  sleep 1
done
if [ "$webcam_ready" -eq 1 ]; then
  echo ">> external webcam recorder ready (/recorder/start, /recorder/stop)"
elif [ "$REQUIRE_WEBCAM" = "1" ]; then
  echo "ERROR: external webcam recorder did not register within ${WEBCAM_WAIT_S}s" >&2
  exit 1
else
  echo "WARN: external webcam recorder unavailable; continuing without MP4 capture" >&2
fi

# Nominal down-facing optical transform used only to exercise the controller;
# it must be replaced with surveyed extrinsics before flight evaluation.
docker exec -d "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; exec rosrun tf static_transform_publisher 0 0 0 0.70710678 -0.70710678 0 0 base_link see3cam_optical_frame 100 >'$LOG_CONTAINER/tf.log' 2>&1"
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/sensor/see3cam-24cug/run.sh >'$LOG_CONTAINER/camera.log' 2>&1"
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/perception/aruco-landing/run_estimator.sh pad_size_m:='$PAD_SIZE_M' layout_file:='$LAYOUT_CONTAINER' dictionary:='$DICTIONARY' >'$LOG_CONTAINER/estimator.log' 2>&1"
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/control/aruco-landing/run.sh >'$LOG_CONTAINER/controller.log' 2>&1"
docker exec -d "$CONTAINER" bash -lc \
  "exec /work/modules/utility/session-recorder/run.sh >'$LOG_CONTAINER/session_recorder.log' 2>&1"

sleep 5
docker exec "$CONTAINER" bash -lc \
  "source /opt/ros/noetic/setup.bash; source /work/config/ros_env.sh; exec python3 /work/stack-assets/aruco-landing-jetson/scripts/profile_pipeline.py --duration '$DURATION' --label '$LABEL' --layout '$LAYOUT_CONTAINER' --dictionary '$DICTIONARY' --pad-size-m '$PAD_SIZE_M' --output-dir '$OUTPUT_CONTAINER' --enable-controller"

#!/bin/bash
# Replay an AirSim exploration bag through the latest fixed FAST-LIVO binary.
# Input/output must be under drone-stack-docker for the /work bind mount.

set -euo pipefail

usage() {
  cat <<'EOF'
usage: replay_fastlivo_sim.sh BAG --out OUTPUT.bag [options]

Options:
  --rate R          rosbag playback rate (default: 0.5)
  --port PORT       isolated ROS master port (default: 11435)
  --container NAME  CPU replay container name
  --force           overwrite this command's exact output and sidecars

Required input topics:
  /airsim_node/hmcl/imu/imu
  /camera/left/image_raw
  /voxel_grid/output
  /gt_odom
EOF
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BAG=""
OUT=""
RATE=0.5
PORT=11435
CONTAINER="${FASTLIVO_SIM_REPLAY_CONTAINER:-drone-stack-fastlivo-replay-cpu}"
FORCE=0
INSIDE=0

while [ "$#" -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --container) CONTAINER="$2"; shift 2 ;;
    --force) FORCE=1; shift ;;
    --inside) INSIDE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    *)
      [ -z "$BAG" ] || { echo "only one input bag is allowed" >&2; exit 2; }
      BAG="$1"
      shift
      ;;
  esac
done

if [ -z "$BAG" ] || [ -z "$OUT" ]; then
  usage >&2
  exit 2
fi

if [ "$INSIDE" -eq 0 ]; then
  BAG="$(readlink -m "$BAG")"
  OUT="$(readlink -m "$OUT")"
  [ -f "$BAG" ] || { echo "missing input bag: $BAG" >&2; exit 2; }
  case "$BAG" in
    "$ROOT"/*) BAG_IN="/work/${BAG#"$ROOT"/}" ;;
    *) echo "input must be under $ROOT" >&2; exit 2 ;;
  esac
  case "$OUT" in
    "$ROOT"/*) OUT_IN="/work/${OUT#"$ROOT"/}" ;;
    *) echo "output must be under $ROOT" >&2; exit 2 ;;
  esac

  if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    docker create \
      --name "$CONTAINER" \
      --network host \
      --volume "$ROOT:/work" \
      --workdir /work \
      drone-stack:sim-x86 \
      sleep infinity >/dev/null
  fi
  docker start "$CONTAINER" >/dev/null
  INNER=(
    bash /work/tools/fastlivo/replay_fastlivo_sim.sh
    --inside "$BAG_IN" --out "$OUT_IN" --rate "$RATE" --port "$PORT"
  )
  [ "$FORCE" -eq 0 ] || INNER+=(--force)
  exec docker exec \
    -e ROS_MASTER_HOST=127.0.0.1 \
    -e ROS_MASTER_PORT="$PORT" \
    -e ROS_IP=127.0.0.1 \
    -e ROS_HOSTNAME=localhost \
    "$CONTAINER" "${INNER[@]}"
fi

export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=localhost
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source /work/ws/fast-livo/devel/setup.bash

FASTLIVO_REPO=/work/ws/fast-livo/src
FASTLIVO_BIN=/work/ws/fast-livo/devel/.private/fast_livo/lib/fast_livo/fastlivo_mapping
SIM_CONFIG_REPO=/work/ws/fast-livo-sim/src/FAST-LIVO2
# Ubuntu 20.04's Git backport requires bind-mounted repositories to be trusted
# in the container's protected config (command-line safe.directory is ignored).
for repo in "$FASTLIVO_REPO" "$SIM_CONFIG_REPO"; do
  git config --global --get-all safe.directory 2>/dev/null | grep -Fqx "$repo" ||
    git config --global --add safe.directory "$repo"
done
[ -x "$FASTLIVO_BIN" ] || { echo "missing FAST-LIVO binary: $FASTLIVO_BIN" >&2; exit 2; }
HEAD_EPOCH="$(git -C "$FASTLIVO_REPO" show -s --format=%ct HEAD)"
BIN_EPOCH="$(stat -c %Y "$FASTLIVO_BIN")"
[ "$BIN_EPOCH" -ge "$HEAD_EPOCH" ] || {
  echo "FAST-LIVO binary predates HEAD; run modules/odometry/fast-livo/build_ws.sh" >&2
  exit 2
}
git -C "$FASTLIVO_REPO" diff --quiet -- FAST-LIVO2 || {
  echo "warning: replay uses a build from a dirty FAST-LIVO source tree" >&2
}

[ -f "$BAG" ] || { echo "missing in-container input: $BAG" >&2; exit 2; }
mkdir -p "$(dirname "$OUT")"

BASE="${OUT%.bag}"
NODE_LOG="${BASE}_node.log"
RECORD_LOG="${BASE}_record.log"
CORE_LOG="${BASE}_core.log"
PARAMS="${BASE}_params.yaml"
RECEIPT="${BASE}_receipt.txt"
if [ "$FORCE" -eq 0 ]; then
  for path in "$OUT" "$OUT.active" "$NODE_LOG" "$RECORD_LOG" \
              "$CORE_LOG" "$PARAMS" "$RECEIPT"; do
    [ ! -e "$path" ] || {
      echo "refusing to overwrite existing output (use --force): $path" >&2
      exit 2
    }
  done
else
  rm -f "$OUT" "$OUT.active" "$NODE_LOG" "$RECORD_LOG" \
        "$CORE_LOG" "$PARAMS" "$RECEIPT"
fi

CORE_PID=""
NODE_PID=""
RECORD_PID=""
stop_pid() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 200); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -TERM "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}
cleanup() {
  stop_pid "$RECORD_PID"
  stop_pid "$NODE_PID"
  stop_pid "$CORE_PID"
}
trap cleanup EXIT INT TERM

roscore -p "$PORT" >"$CORE_LOG" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 40); do
  rosparam list >/dev/null 2>&1 && break
  sleep 0.25
done
rosparam list >/dev/null 2>&1 || {
  echo "isolated roscore did not start" >&2
  exit 1
}

rosparam set /use_sim_time true
rosparam load \
  /work/ws/fast-livo-sim/src/FAST-LIVO2/config/simulator_openvins.yaml /
rosparam load \
  /work/ws/fast-livo-sim/src/FAST-LIVO2/config/camera_simulator_openvins.yaml \
  /laserMapping
rosparam dump "$PARAMS"

rosrun fast_livo fastlivo_mapping __name:=laserMapping >"$NODE_LOG" 2>&1 &
NODE_PID=$!
for _ in $(seq 1 80); do
  kill -0 "$NODE_PID" 2>/dev/null || {
    tail -100 "$NODE_LOG" >&2
    exit 1
  }
  rosnode list 2>/dev/null | grep -q '^/laserMapping$' && break
  sleep 0.25
done
rosnode list | grep -q '^/laserMapping$' || {
  echo "FAST-LIVO node did not advertise" >&2
  exit 1
}

rosbag record --lz4 -O "$OUT" \
  /aft_mapped_to_init \
  /aft_mapped_to_body \
  /aft_mapped_to_body_imu_propagated \
  /path /gt_odom /tf /tf_static /rosout >"$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 1

rosbag play --quiet --clock --rate "$RATE" "$BAG" --topics \
  /airsim_node/hmcl/imu/imu \
  /camera/left/image_raw \
  /voxel_grid/output \
  /gt_odom
sleep 3

stop_pid "$RECORD_PID"
RECORD_PID=""
stop_pid "$NODE_PID"
NODE_PID=""

[ -s "$OUT" ] || { echo "missing finalized output: $OUT" >&2; exit 1; }
INFO="$(rosbag info "$OUT")"
echo "$INFO" | grep -Eq '[[:space:]]/aft_mapped_to_init[[:space:]]+[0-9]+ msgs' || {
  echo "output has no FAST-LIVO odometry" >&2
  exit 1
}
echo "$INFO" | grep -Eq '[[:space:]]/gt_odom[[:space:]]+[0-9]+ msgs' || {
  echo "output has no GT passthrough" >&2
  exit 1
}

{
  echo "input=$BAG"
  echo "output=$OUT"
  echo "rate=$RATE"
  echo "fast_livo_commit=$(git -C "$FASTLIVO_REPO" rev-parse HEAD)"
  echo "sim_config_commit=$(git -C "$SIM_CONFIG_REPO" rev-parse HEAD)"
  sha256sum "$BAG" "$OUT"
} >"$RECEIPT"

echo "$INFO"
echo "params:  $PARAMS"
echo "receipt: $RECEIPT"

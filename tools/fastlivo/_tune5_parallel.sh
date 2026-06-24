#!/bin/bash
# Tuning v5 — NATIVE (no Docker) PARALLEL grid. Same 162-config 5-handle factorial + same NO-ALIGN
# position-RMSE eval as _tune5_flight1.sh, but K worker shards each on an ISOLATED roscore port
# (PORT_BASE+slot) so replays run concurrently. Port isolation means it does NOT touch a live roscore
# on 11311 (safe to co-run, though it competes for CPU). Built to run on the ml sim box (native ROS).
#
#   usage: _tune5_parallel.sh [K]          K = parallel workers (default nproc/2)
#   env:   FASTLIVO_WS   fast-livo catkin ws (has devel/setup.bash). default: first of
#                        $HOME/fast_livo2_d435i, /work/ws/fast-livo, <repo>/ws/fast-livo
#          ROS_MASTER_HOST  default localhost ; PORT_BASE default 11400
#          BAG          input bag basename (no .bag). default 2026-06-24-08-53-06-trim
#          EVAL_PY      python with numpy/scipy/evo/rosbags for the ranking. default: first of
#                       $HOME/evo_venv/bin/python3, python3 (skip+print cmd if deps missing)
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
K="${1:-$(( $(nproc) / 2 ))}"; [ "$K" -lt 1 ] && K=1
PORT_BASE="${PORT_BASE:-11400}"
ROS_MASTER_HOST="${ROS_MASTER_HOST:-localhost}"
BAG="${BAG:-2026-06-24-08-53-06-trim}"
BAGPATH="$SCRIPT_DIR/${BAG}.bag"
LAUNCH="$SCRIPT_DIR/mapping_d435i_replay.launch"
BASE="$SCRIPT_DIR/_tune_cmb_rel02_v1k.yaml"
REC_TOPICS="/aft_mapped_to_optitrack /vrpn_client_node/pure/pose /aft_mapped_to_body /aft_mapped_to_init /path /tf /tf_static"
# pick the fast-livo workspace
if [ -z "${FASTLIVO_WS:-}" ]; then
  for w in "$HOME/fast_livo2_d435i" /work/ws/fast-livo "$REPO_ROOT/ws/fast-livo"; do
    [ -f "$w/devel/setup.bash" ] && FASTLIVO_WS="$w" && break
  done
fi
: "${FASTLIVO_WS:?set FASTLIVO_WS to a built fast-livo catkin ws (devel/setup.bash not found)}"
[ -f "$BAGPATH" ] || { echo "bag not found: $BAGPATH"; exit 1; }
echo "[cfg] K=$K  ws=$FASTLIVO_WS  host=$ROS_MASTER_HOST  ports=$PORT_BASE..+$((K-1))  bag=$BAG"

# --- generate the 162 configs (identical handles to _tune5_flight1.sh) ---
gen_one(){ local name=$1 imu=$2 img=$3 bias=$4 lto=$5 dep=$6 f="$SCRIPT_DIR/_t5_$1.yaml"
  cp "$BASE" "$f"
  [ "$imu" = lo ] && sed -i 's/acc_cov: 0.5/acc_cov: 0.1/; s/gyr_cov: 0.3/gyr_cov: 0.1/' "$f"
  [ "$imu" = hi ] && sed -i 's/acc_cov: 0.5/acc_cov: 1.5/; s/gyr_cov: 0.3/gyr_cov: 1.0/' "$f"
  sed -i "s/img_point_cov: 1000/img_point_cov: ${img}/" "$f"
  [ "$bias" = lo ] && sed -i 's/b_acc_cov: 0.001/b_acc_cov: 0.0005/; s/b_gyr_cov: 0.001/b_gyr_cov: 0.0005/' "$f"
  [ "$bias" = hi ] && sed -i 's/b_acc_cov: 0.001/b_acc_cov: 0.005/; s/b_gyr_cov: 0.001/b_gyr_cov: 0.005/' "$f"
  local off; case "$lto" in 000) off="0.0";; 005) off="-0.005";; 010) off="-0.010";; esac
  sed -i "/imu_time_offset: 0.0/a\\  lidar_time_offset: ${off}" "$f"
  sed -i "s/dept_err_rel: 0.02/dept_err_rel: 0.${dep}/" "$f"; }
NAMES=()
for imu in lo mid hi; do for img in 100 1000; do for bias in lo mid hi; do for lto in 000 005 010; do for dep in 01 02 03; do
  n=${imu}_v${img}_b${bias}_l${lto}_d${dep}; gen_one "$n" "$imu" "$img" "$bias" "$lto" "$dep"; NAMES+=("$n")
done; done; done; done; done
echo "[cfg] generated ${#NAMES[@]} configs"

# --- one native replay on a dedicated roscore port ---
run_one(){ local name="$1" port="$2"
  local cfg="$SCRIPT_DIR/_t5_${name}.yaml" out="$SCRIPT_DIR/_t5o_${name}.bag"
  [ -s "$out" ] && { echo "  skip $name (exists)"; return; }
  (
    export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${port}" ROS_IP="127.0.0.1" ROS_HOSTNAME="localhost"
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1
    source "$FASTLIVO_WS/devel/setup.bash" >/dev/null 2>&1
    roscore -p "$port" >/tmp/t5_roscore_${port}.log 2>&1 & local rc=$!
    for _ in $(seq 1 50); do timeout 1 bash -c "exec 3<>/dev/tcp/127.0.0.1/${port}" 2>/dev/null && break; sleep 0.2; done
    rosparam set /use_sim_time true
    roslaunch "$LAUNCH" config:="$cfg" paired_drop:=false odom_guard:=false >/tmp/t5_livo_${port}.log 2>&1 & local lp=$!
    local up=0; for _ in $(seq 1 40); do rostopic info /aft_mapped_to_init 2>/dev/null | grep -q '/laserMapping' && { up=1; break; }; sleep 0.5; done
    if [ "$up" = 1 ]; then
      rosbag record -O "$out" $REC_TOPICS >/tmp/t5_rec_${port}.log 2>&1 & local rp=$!
      sleep 1
      rosbag play --clock "$BAGPATH" >/tmp/t5_play_${port}.log 2>&1
      sleep 2; kill -INT "$rp" 2>/dev/null; sleep 1
    else echo "  WARN $name: laserMapping never came up (see /tmp/t5_livo_${port}.log)"; fi
    kill -INT "$lp" 2>/dev/null; sleep 1
    kill -9 "$rc" "$lp" "${rp:-0}" 2>/dev/null
  )
}

# --- static-shard worker pool: worker w owns port PORT_BASE+w, runs configs[w::K] sequentially ---
worker(){ local w="$1" port=$((PORT_BASE + w)) i
  for ((i=w; i<${#NAMES[@]}; i+=K)); do
    echo "[w$w :$port $(date +%H:%M:%S)] $((i+1))/${#NAMES[@]} ${NAMES[i]}"
    run_one "${NAMES[i]}" "$port"
  done; }
echo "[run] START $(date) — $K workers"
for ((w=0; w<K; w++)); do worker "$w" & done
wait
echo "[run] DONE replays $(date) — $(ls "$SCRIPT_DIR"/_t5o_*.bag 2>/dev/null | wc -l) bags"

# --- eval (parallel, no ROS) ---
mkdir -p "$SCRIPT_DIR/tune5_results"
EVAL_PY="${EVAL_PY:-}"; [ -z "$EVAL_PY" ] && { [ -x "$HOME/evo_venv/bin/python3" ] && EVAL_PY="$HOME/evo_venv/bin/python3" || EVAL_PY="python3"; }
echo "[eval] $EVAL_PY $SCRIPT_DIR/_tune5_eval.py"
if "$EVAL_PY" "$SCRIPT_DIR/_tune5_eval.py" "$SCRIPT_DIR" "$SCRIPT_DIR/tune5_results/RANK.txt"; then
  echo "### TUNING v5 PARALLEL DONE $(date) — $SCRIPT_DIR/tune5_results/RANK.txt ###"
else
  echo "!!! eval deps missing for '$EVAL_PY'. bags are saved; run manually:"
  echo "    <python-with-evo-rosbags> $SCRIPT_DIR/_tune5_eval.py $SCRIPT_DIR $SCRIPT_DIR/tune5_results/RANK.txt"
fi

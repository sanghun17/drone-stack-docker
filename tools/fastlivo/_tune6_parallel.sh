#!/bin/bash
# Tuning v6 — NATIVE PARALLEL coarse-screening grid. Same isolated-port K-worker replay machinery
# as _tune5_parallel.sh, but a re-chosen 7-handle factorial (288 configs) informed by the v5
# marginals: bias_RW was flat -> FIXED at blo(0.0005); lidar_off & dept_err_rel mattered most ->
# refined to their good region; and 3 NEW geometry handles (voxel_size, filter_size_surf, max_layer)
# are screened coarsely. Output prefix _t6o_, results in tune6_results/RANK.txt.
#
#   usage: _tune6_parallel.sh [K]          K = parallel workers (default nproc/2)
#   env:   FASTLIVO_WS, ROS_MASTER_HOST, PORT_BASE, BAG, EVAL_PY  (same as v5)
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
if [ -z "${FASTLIVO_WS:-}" ]; then
  for w in "$HOME/fast_livo2_d435i" /work/ws/fast-livo "$REPO_ROOT/ws/fast-livo"; do
    [ -f "$w/devel/setup.bash" ] && FASTLIVO_WS="$w" && break
  done
fi
: "${FASTLIVO_WS:?set FASTLIVO_WS to a built fast-livo catkin ws (devel/setup.bash not found)}"
[ -f "$BAGPATH" ] || { echo "bag not found: $BAGPATH"; exit 1; }
echo "[cfg] K=$K  ws=$FASTLIVO_WS  host=$ROS_MASTER_HOST  ports=$PORT_BASE..+$((K-1))  bag=$BAG"

# --- generate the 288 configs (7 handles; bias FIXED at blo per v5 marginals) ---
gen_one(){ local name=$1 imu=$2 img=$3 lto=$4 dep=$5 vox=$6 fs=$7 ml=$8 f="$SCRIPT_DIR/_t6_$1.yaml"
  cp "$BASE" "$f"
  # bias_RW fixed low (v5: flat handle, blo marginally best)
  sed -i 's/b_acc_cov: 0.001/b_acc_cov: 0.0005/; s/b_gyr_cov: 0.001/b_gyr_cov: 0.0005/' "$f"
  # imu trust
  [ "$imu" = lo ] && sed -i 's/acc_cov: 0.5/acc_cov: 0.1/; s/gyr_cov: 0.3/gyr_cov: 0.1/' "$f"
  [ "$imu" = hi ] && sed -i 's/acc_cov: 0.5/acc_cov: 1.5/; s/gyr_cov: 0.3/gyr_cov: 1.0/' "$f"
  # img point cov
  sed -i "s/img_point_cov: 1000/img_point_cov: ${img}/" "$f"
  # lidar time offset (append after imu_time_offset)
  local off; case "$lto" in 005) off="-0.005";; 010) off="-0.010";; esac
  sed -i "/imu_time_offset: 0.0/a\\  lidar_time_offset: ${off}" "$f"
  # dept_err_rel (base 0.02) -> 0.0 / 0.01
  local depv; case "$dep" in 00) depv="0.0";; 01) depv="0.01";; esac
  sed -i "s/dept_err_rel: 0.02/dept_err_rel: ${depv}/" "$f"
  # voxel_size (base 0.5)
  local voxv; case "$vox" in 03) voxv="0.3";; 05) voxv="0.5";; 08) voxv="0.8";; esac
  sed -i "s/voxel_size: 0.5/voxel_size: ${voxv}/" "$f"
  # filter_size_surf (base 0.1)
  local fsv; case "$fs" in 005) fsv="0.05";; 020) fsv="0.20";; esac
  sed -i "s/filter_size_surf: 0.1/filter_size_surf: ${fsv}/" "$f"
  # max_layer (base 2)
  sed -i "s/max_layer: 2/max_layer: ${ml}/" "$f"; }
NAMES=()
for imu in lo hi; do for img in 100 1000; do for lto in 005 010; do for dep in 00 01; do
for vox in 03 05 08; do for fs in 005 020; do for ml in 1 2 3; do
  n=im${imu}_v${img}_l${lto}_d${dep}_vox${vox}_fs${fs}_ml${ml}
  gen_one "$n" "$imu" "$img" "$lto" "$dep" "$vox" "$fs" "$ml"; NAMES+=("$n")
done; done; done; done; done; done; done
echo "[cfg] generated ${#NAMES[@]} configs"

# --- one native replay on a dedicated roscore port (identical to v5) ---
run_one(){ local name="$1" port="$2"
  local cfg="$SCRIPT_DIR/_t6_${name}.yaml" out="$SCRIPT_DIR/_t6o_${name}.bag"
  [ -s "$out" ] && { echo "  skip $name (exists)"; return; }
  (
    export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${port}" ROS_IP="127.0.0.1" ROS_HOSTNAME="localhost"
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1
    source "$FASTLIVO_WS/devel/setup.bash" >/dev/null 2>&1
    roscore -p "$port" >/tmp/t6_roscore_${port}.log 2>&1 & local rc=$!
    for _ in $(seq 1 50); do timeout 1 bash -c "exec 3<>/dev/tcp/127.0.0.1/${port}" 2>/dev/null && break; sleep 0.2; done
    rosparam set /use_sim_time true
    roslaunch "$LAUNCH" config:="$cfg" paired_drop:=false odom_guard:=false >/tmp/t6_livo_${port}.log 2>&1 & local lp=$!
    local up=0; for _ in $(seq 1 40); do rostopic info /aft_mapped_to_init 2>/dev/null | grep -q '/laserMapping' && { up=1; break; }; sleep 0.5; done
    if [ "$up" = 1 ]; then
      rosbag record -O "$out" $REC_TOPICS >/tmp/t6_rec_${port}.log 2>&1 & local rp=$!
      sleep 1
      rosbag play --clock "$BAGPATH" >/tmp/t6_play_${port}.log 2>&1
      sleep 2; kill -INT "$rp" 2>/dev/null; sleep 1
    else echo "  WARN $name: laserMapping never came up (see /tmp/t6_livo_${port}.log)"; fi
    kill -INT "$lp" 2>/dev/null; sleep 1
    kill -9 "$rc" "$lp" "${rp:-0}" 2>/dev/null
  )
}

worker(){ local w="$1" port=$((PORT_BASE + w)) i
  for ((i=w; i<${#NAMES[@]}; i+=K)); do
    echo "[w$w :$port $(date +%H:%M:%S)] $((i+1))/${#NAMES[@]} ${NAMES[i]}"
    run_one "${NAMES[i]}" "$port"
  done; }
echo "[run] START $(date) — $K workers"
for ((w=0; w<K; w++)); do worker "$w" & done
wait
echo "[run] DONE replays $(date) — $(ls "$SCRIPT_DIR"/_t6o_*.bag 2>/dev/null | wc -l) bags"

# --- eval (parallel, no ROS) ---
mkdir -p "$SCRIPT_DIR/tune6_results"
EVAL_PY="${EVAL_PY:-}"; [ -z "$EVAL_PY" ] && { [ -x "$HOME/evo_venv/bin/python3" ] && EVAL_PY="$HOME/evo_venv/bin/python3" || EVAL_PY="python3"; }
echo "[eval] $EVAL_PY $SCRIPT_DIR/_tune6_eval.py"
if "$EVAL_PY" "$SCRIPT_DIR/_tune6_eval.py" "$SCRIPT_DIR" "$SCRIPT_DIR/tune6_results/RANK.txt"; then
  echo "### TUNING v6 PARALLEL DONE $(date) — $SCRIPT_DIR/tune6_results/RANK.txt ###"
else
  echo "!!! eval deps missing for '$EVAL_PY'. bags saved; run manually:"
  echo "    <python-with-evo-rosbags> $SCRIPT_DIR/_tune6_eval.py $SCRIPT_DIR $SCRIPT_DIR/tune6_results/RANK.txt"
fi

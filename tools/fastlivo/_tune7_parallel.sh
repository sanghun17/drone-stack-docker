#!/bin/bash
# Tuning v7 — NATIVE PARALLEL. Same isolated-port replay machinery as v5/v6. Drops the v6 fragile
# levels (fs005, vox08, imhi all caused divergence) and SCREENS 4 NEW levers that v5/v6 held fixed:
# beam_err, point_filter_num, max_points_num, gravity_align_en. Other handles fixed at the v6
# robust optimum (imu=lo, lidar_off=-5ms, dept_err_rel=0.01, max_layer=1, bias=blo). 288 configs.
# At the end it rebuilds the cumulative tune_master.csv (v5+v6+v7) via build_master.py.
#   usage: _tune7_parallel.sh [K]
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
echo "[cfg] K=$K  ws=$FASTLIVO_WS  ports=$PORT_BASE..+$((K-1))  bag=$BAG"

# --- generate 288 configs: 7 varied (3 refined + 4 NEW), rest fixed at v6 robust optimum ---
gen_one(){ local name=$1 fs=$2 vox=$3 img=$4 be=$5 pf=$6 mp=$7 ga=$8 f="$SCRIPT_DIR/_t7_$1.yaml"
  cp "$BASE" "$f"
  # --- fixed at v6 robust optimum ---
  sed -i 's/acc_cov: 0.5/acc_cov: 0.1/; s/gyr_cov: 0.3/gyr_cov: 0.1/' "$f"          # imu=lo
  sed -i 's/b_acc_cov: 0.001/b_acc_cov: 0.0005/; s/b_gyr_cov: 0.001/b_gyr_cov: 0.0005/' "$f"  # bias=blo
  sed -i "/imu_time_offset: 0.0/a\\  lidar_time_offset: -0.005" "$f"                 # lidar_off=-5ms
  sed -i "s/dept_err_rel: 0.02/dept_err_rel: 0.01/" "$f"                             # dept_err_rel=0.01
  sed -i "s/max_layer: 2/max_layer: 1/" "$f"                                         # max_layer=1
  # --- refined handles ---
  local fsv;  case "$fs"  in 015) fsv="0.15";; 020) fsv="0.20";; 025) fsv="0.25";; esac
  sed -i "s/filter_size_surf: 0.1/filter_size_surf: ${fsv}/" "$f"
  local voxv; case "$vox" in 03) voxv="0.3";; 05) voxv="0.5";; esac
  sed -i "s/voxel_size: 0.5/voxel_size: ${voxv}/" "$f"
  sed -i "s/img_point_cov: 1000/img_point_cov: ${img}/" "$f"
  # --- NEW levers ---
  local bev;  case "$be"  in 002) bev="0.02";; 005) bev="0.05";; 010) bev="0.10";; esac
  sed -i "s/beam_err: 0.05/beam_err: ${bev}/" "$f"
  sed -i "s/point_filter_num: 1 /point_filter_num: ${pf} /" "$f"
  sed -i "s/max_points_num: 50/max_points_num: ${mp}/" "$f"
  [ "$ga" = 0 ] && sed -i "s/gravity_align_en: true/gravity_align_en: false/" "$f"; }
NAMES=()
for fs in 015 020 025; do for vox in 03 05; do for img in 100 1000; do
for be in 002 005 010; do for pf in 1 3; do for mp in 50 100; do for ga in 1 0; do
  n=fs${fs}_vox${vox}_v${img}_be${be}_pf${pf}_mp${mp}_ga${ga}
  gen_one "$n" "$fs" "$vox" "$img" "$be" "$pf" "$mp" "$ga"; NAMES+=("$n")
done; done; done; done; done; done; done
echo "[cfg] generated ${#NAMES[@]} configs"

run_one(){ local name="$1" port="$2"
  local cfg="$SCRIPT_DIR/_t7_${name}.yaml" out="$SCRIPT_DIR/_t7o_${name}.bag"
  [ -s "$out" ] && { echo "  skip $name (exists)"; return; }
  (
    export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${port}" ROS_IP="127.0.0.1" ROS_HOSTNAME="localhost"
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1
    source "$FASTLIVO_WS/devel/setup.bash" >/dev/null 2>&1
    roscore -p "$port" >/tmp/t7_roscore_${port}.log 2>&1 & local rc=$!
    for _ in $(seq 1 50); do timeout 1 bash -c "exec 3<>/dev/tcp/127.0.0.1/${port}" 2>/dev/null && break; sleep 0.2; done
    rosparam set /use_sim_time true
    roslaunch "$LAUNCH" config:="$cfg" paired_drop:=false odom_guard:=false >/tmp/t7_livo_${port}.log 2>&1 & local lp=$!
    local up=0; for _ in $(seq 1 40); do rostopic info /aft_mapped_to_init 2>/dev/null | grep -q '/laserMapping' && { up=1; break; }; sleep 0.5; done
    if [ "$up" = 1 ]; then
      rosbag record -O "$out" $REC_TOPICS >/tmp/t7_rec_${port}.log 2>&1 & local rp=$!
      sleep 1
      rosbag play --clock "$BAGPATH" >/tmp/t7_play_${port}.log 2>&1
      sleep 2; kill -INT "$rp" 2>/dev/null; sleep 1
    else echo "  WARN $name: laserMapping never came up (see /tmp/t7_livo_${port}.log)"; fi
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
echo "[run] DONE replays $(date) — $(ls "$SCRIPT_DIR"/_t7o_*.bag 2>/dev/null | wc -l) bags"

# --- rebuild the cumulative master table (v5+v6+v7) ---
EVAL_PY="${EVAL_PY:-}"; [ -z "$EVAL_PY" ] && { [ -x "$HOME/evo_venv/bin/python3" ] && EVAL_PY="$HOME/evo_venv/bin/python3" || EVAL_PY="python3"; }
echo "[master] $EVAL_PY build_master.py -> tune_master.csv"
"$EVAL_PY" "$SCRIPT_DIR/build_master.py" "$SCRIPT_DIR" "$SCRIPT_DIR/tune_master.csv" \
  && echo "### TUNING v7 DONE $(date) — tune_master.csv rebuilt (v5+v6+v7) ###"

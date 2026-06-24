#!/bin/bash
# Tuning v8 — NATIVE PARALLEL final precision round. Fixes every well-characterized handle at the v7
# robust optimum (imu=lo, bias=blo, lidar_off=-5ms, dept_err_rel=0.01, max_layer=1, img_point_cov=100,
# point_filter_num=3, beam_err=0.05, gravity_align=on), finely refines the top-cluster geometry
# (filter_size_surf, voxel_size, max_points_num) and SCREENS 3 NEW unexplored VIO/IMU levers:
# patch_size, patch_pyrimid_level, imu_int_frame. 216 configs. Rebuilds tune_master.csv (v5..v8).
#   usage: _tune8_parallel.sh [K]
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

gen_one(){ local name=$1 fs=$2 vox=$3 mp=$4 ps=$5 pl=$6 iif=$7 f="$SCRIPT_DIR/_t8_$1.yaml"
  cp "$BASE" "$f"
  # --- fixed at v7 robust optimum ---
  sed -i 's/acc_cov: 0.5/acc_cov: 0.1/; s/gyr_cov: 0.3/gyr_cov: 0.1/' "$f"            # imu=lo
  sed -i 's/b_acc_cov: 0.001/b_acc_cov: 0.0005/; s/b_gyr_cov: 0.001/b_gyr_cov: 0.0005/' "$f"  # bias=blo
  sed -i "/imu_time_offset: 0.0/a\\  lidar_time_offset: -0.005" "$f"                   # lidar_off=-5ms
  sed -i "s/dept_err_rel: 0.02/dept_err_rel: 0.01/" "$f"                               # dept_err_rel=0.01
  sed -i "s/max_layer: 2/max_layer: 1/" "$f"                                           # max_layer=1
  sed -i "s/img_point_cov: 1000/img_point_cov: 100/" "$f"                              # img=100
  sed -i "s/point_filter_num: 1 /point_filter_num: 3 /" "$f"                           # pf=3
  # beam_err 0.05 / gravity_align_en true already base defaults -> leave
  # --- refined top-cluster geometry ---
  local fsv;  case "$fs"  in 015) fsv="0.15";; 018) fsv="0.18";; 020) fsv="0.20";; esac
  sed -i "s/filter_size_surf: 0.1/filter_size_surf: ${fsv}/" "$f"
  local voxv; case "$vox" in 025) voxv="0.25";; 030) voxv="0.3";; 040) voxv="0.4";; esac
  sed -i "s/voxel_size: 0.5/voxel_size: ${voxv}/" "$f"
  sed -i "s/max_points_num: 50/max_points_num: ${mp}/" "$f"
  # --- NEW VIO/IMU levers ---
  sed -i "s/patch_size: 8/patch_size: ${ps}/" "$f"
  sed -i "s/patch_pyrimid_level: 4/patch_pyrimid_level: ${pl}/" "$f"
  sed -i "s/imu_int_frame: 30/imu_int_frame: ${iif}/" "$f"; }
NAMES=()
for fs in 015 018 020; do for vox in 025 030 040; do for mp in 50 100; do
for ps in 4 8; do for pl in 2 4; do for iif in 20 30 50; do
  n=fs${fs}_vox${vox}_mp${mp}_ps${ps}_pl${pl}_if${iif}
  gen_one "$n" "$fs" "$vox" "$mp" "$ps" "$pl" "$iif"; NAMES+=("$n")
done; done; done; done; done; done
echo "[cfg] generated ${#NAMES[@]} configs"

run_one(){ local name="$1" port="$2"
  local cfg="$SCRIPT_DIR/_t8_${name}.yaml" out="$SCRIPT_DIR/_t8o_${name}.bag"
  [ -s "$out" ] && { echo "  skip $name (exists)"; return; }
  (
    export ROS_MASTER_URI="http://${ROS_MASTER_HOST}:${port}" ROS_IP="127.0.0.1" ROS_HOSTNAME="localhost"
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1
    source "$FASTLIVO_WS/devel/setup.bash" >/dev/null 2>&1
    roscore -p "$port" >/tmp/t8_roscore_${port}.log 2>&1 & local rc=$!
    for _ in $(seq 1 50); do timeout 1 bash -c "exec 3<>/dev/tcp/127.0.0.1/${port}" 2>/dev/null && break; sleep 0.2; done
    rosparam set /use_sim_time true
    roslaunch "$LAUNCH" config:="$cfg" paired_drop:=false odom_guard:=false >/tmp/t8_livo_${port}.log 2>&1 & local lp=$!
    local up=0; for _ in $(seq 1 40); do rostopic info /aft_mapped_to_init 2>/dev/null | grep -q '/laserMapping' && { up=1; break; }; sleep 0.5; done
    if [ "$up" = 1 ]; then
      rosbag record -O "$out" $REC_TOPICS >/tmp/t8_rec_${port}.log 2>&1 & local rp=$!
      sleep 1
      rosbag play --clock "$BAGPATH" >/tmp/t8_play_${port}.log 2>&1
      sleep 2; kill -INT "$rp" 2>/dev/null; sleep 1
    else echo "  WARN $name: laserMapping never came up (see /tmp/t8_livo_${port}.log)"; fi
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
echo "[run] DONE replays $(date) — $(ls "$SCRIPT_DIR"/_t8o_*.bag 2>/dev/null | wc -l) bags"

EVAL_PY="${EVAL_PY:-}"; [ -z "$EVAL_PY" ] && { [ -x "$HOME/evo_venv/bin/python3" ] && EVAL_PY="$HOME/evo_venv/bin/python3" || EVAL_PY="python3"; }
echo "[master] $EVAL_PY build_master.py -> tune_master.csv"
"$EVAL_PY" "$SCRIPT_DIR/build_master.py" "$SCRIPT_DIR" "$SCRIPT_DIR/tune_master.csv" \
  && echo "### TUNING v8 DONE $(date) — tune_master.csv rebuilt (v5..v8) ###"

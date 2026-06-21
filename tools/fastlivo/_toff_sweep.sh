#!/bin/bash
# imu_time_offset A/B: replay 0-250s (covers yaw ~200-260s) for 3 offsets, eval yaw window each.
cd /home/hmcl/drone-stack-docker
BAG=/work/tools/fastlivo/2026-06-19-17-17-03.bag
: > /tmp/toff_results.txt
run() {
  local tag=$1; shift
  local out=/work/tools/fastlivo/flight_toff_${tag}_livo.bag
  echo "[$(date +%H:%M:%S)] === REPLAY $tag ($*) ===" | tee -a /tmp/toff_results.txt
  tools/fastlivo/replay_fastlivo.sh "$BAG" --duration 250 --out "$out" "$@" >/tmp/toff_${tag}.log 2>&1
  docker exec drone-stack-d435i-voxblox bash -lc "cd /work && python3 tools/fastlivo/_yaw_metric.py $out 200 240" 2>&1 | tee -a /tmp/toff_results.txt
}
run base0
run m04 --config /work/tools/fastlivo/d435i_toff_m04.yaml
run p04 --config /work/tools/fastlivo/d435i_toff_p04.yaml
echo "[$(date +%H:%M:%S)] ALL DONE" | tee -a /tmp/toff_results.txt

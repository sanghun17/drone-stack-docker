#!/bin/bash
# Wave 2: confirm gravity_align (N=4 total) + cov sweep (b_*_cov 0.0001 / 0.001(cur) / 0.01).
cd /home/hmcl/drone-stack-docker
BAG=/work/tools/fastlivo/2026-06-19-17-17-03.bag
RES=/tmp/wave2.txt; : > "$RES"
ev(){ docker exec drone-stack-d435i-voxblox bash -lc "cd /work && python3 tools/fastlivo/_eval_run.py $1" 2>&1 | tee -a "$RES"; }
rep(){ local tag=$1; shift
  echo "[$(date +%H:%M:%S)] REPLAY $tag" | tee -a "$RES"
  tools/fastlivo/replay_fastlivo.sh "$BAG" --duration 245 --out "/work/tools/fastlivo/wv2_${tag}_livo.bag" "$@" >"/tmp/wv2_${tag}.log" 2>&1
  ev "/work/tools/fastlivo/wv2_${tag}_livo.bag"
}
echo "== gravity_align ON confirm (N=4 with wave1) ==" | tee -a "$RES"
rep gravon3 --config /work/tools/fastlivo/d435i_grav_on.yaml
rep gravon4 --config /work/tools/fastlivo/d435i_grav_on.yaml
echo "== cov b_*_cov 0.0001 (pre-biashi baseline) ==" | tee -a "$RES"
rep b0001_1 --config /work/tools/fastlivo/d435i_cov_b0001.yaml
rep b0001_2 --config /work/tools/fastlivo/d435i_cov_b0001.yaml
echo "== cov b_*_cov 0.01 (over-inflated) ==" | tee -a "$RES"
rep b01_1 --config /work/tools/fastlivo/d435i_cov_b01.yaml
rep b01_2 --config /work/tools/fastlivo/d435i_cov_b01.yaml
echo "[$(date +%H:%M:%S)] WAVE2 DONE" | tee -a "$RES"

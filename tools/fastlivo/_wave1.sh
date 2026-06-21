#!/bin/bash
# Wave 1: determinism (fixed config repeats) + gravity_align on/off A/B. Eval via _eval_run.py.
cd /home/hmcl/drone-stack-docker
BAG=/work/tools/fastlivo/2026-06-19-17-17-03.bag
RES=/tmp/wave1.txt; : > "$RES"
ev(){ docker exec drone-stack-d435i-voxblox bash -lc "cd /work && python3 tools/fastlivo/_eval_run.py $1" 2>&1 | tee -a "$RES"; }
rep(){ # tag, extra-args...
  local tag=$1; shift
  echo "[$(date +%H:%M:%S)] REPLAY $tag" | tee -a "$RES"
  tools/fastlivo/replay_fastlivo.sh "$BAG" --duration 245 --out "/work/tools/fastlivo/wv1_${tag}_livo.bag" "$@" >"/tmp/wv1_${tag}.log" 2>&1
  ev "/work/tools/fastlivo/wv1_${tag}_livo.bag"
}
echo "== reuse existing fixed-config runs ==" | tee -a "$RES"
ev /work/tools/fastlivo/flight_fixed_full_livo.bag
ev /work/tools/fastlivo/flight_toff_base0_livo.bag
echo "== fresh fixed (determinism) ==" | tee -a "$RES"
rep fixed1
rep fixed2
echo "== gravity_align ON ==" | tee -a "$RES"
rep gravon1 --config /work/tools/fastlivo/d435i_grav_on.yaml
rep gravon2 --config /work/tools/fastlivo/d435i_grav_on.yaml
echo "[$(date +%H:%M:%S)] WAVE1 DONE" | tee -a "$RES"

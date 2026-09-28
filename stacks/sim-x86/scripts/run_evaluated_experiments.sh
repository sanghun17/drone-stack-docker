#!/usr/bin/env bash
# Host batch entrypoint: record trials, then produce historical analysis CSVs.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
output=''
args=("$@")
for ((i=0; i<${#args[@]}; i++)); do
  if [[ ${args[i]} == --output ]]; then output="${args[i+1]:-}"; fi
done
if [[ ! "$output" =~ ^/work/flight_logs/([a-zA-Z0-9_-]+)$ ]]; then
  echo 'Required: --output /work/flight_logs/BATCH (letters, digits, underscore, hyphen)' >&2
  exit 2
fi
batch="${BASH_REMATCH[1]}"
if [[ -e "$ROOT/flight_logs/$batch" || -e "$ROOT/data/results/$batch" ]]; then
  echo 'Batch or analysis output already exists; refusing to overwrite' >&2
  exit 2
fi
bash "$ROOT/stacks/sim-x86/scripts/run_experiments.sh" "$@"
runner_rc=$?
export_rc=1
if [[ -f "$ROOT/flight_logs/$batch/manifest.json" ]]; then
  docker exec drone-stack-sim-x86 bash -c '
    source /opt/ros/noetic/setup.bash
    source /work/ws/risk-aware-comparison/devel/setup.bash
    exec python3 /work/data/analysis/risk-aware/experiment-evaluation/export_batch.py "$1" --output "$2"
  ' bash "$output" "/work/data/results/$batch"
  export_rc=$?
  docker exec drone-stack-sim-x86 chown -R "$(id -u):$(id -g)" "$output" "/work/data/results/$batch"
  python3 - "$ROOT/flight_logs/$batch" "$runner_rc" "$export_rc" <<'PY'
import json, pathlib, sys
pathlib.Path(sys.argv[1], 'pipeline_status.json').write_text(json.dumps({
    'runner_exit_code': int(sys.argv[2]), 'export_exit_code': int(sys.argv[3]),
    'complete': sys.argv[2:] == ['0', '0']}, indent=2) + '\n')
PY
fi
if ((runner_rc != 0)); then exit "$runner_rc"; fi
exit "$export_rc"

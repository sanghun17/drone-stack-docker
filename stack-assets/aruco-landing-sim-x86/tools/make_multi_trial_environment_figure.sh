#!/bin/bash
# Overlay all selected landing bags on the fixed 1920x1080 paper overview.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPT="$ROOT/stack-assets/aruco-landing-sim-x86/scripts/make_multi_trial_environment_figure.py"

if [ "$#" -eq 0 ]; then
  set -- "$ROOT/experiments/aruco-landing/60hz-run/baseline/bags/baseline_trial_"'*.bag'
fi

exec python3 "$SCRIPT" "$@"

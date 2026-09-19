#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script.
exec "$(dirname "$(readlink -f "$0")")/../modules/planner/risk-aware/run_voxblox.sh" "$@"

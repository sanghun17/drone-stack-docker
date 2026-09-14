#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script.
export DSD_CONTAINER="${DSD_CONTAINER:-drone-stack-d435i-voxblox}"
exec "$(dirname "$(readlink -f "$0")")/../modules/planner/risk-aware/run_jax.sh" "$@"

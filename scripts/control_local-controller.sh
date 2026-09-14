#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script.
# The control stack has one normal path; arguments, when present, are roslaunch args.
export DSD_CONTAINER="${DSD_CONTAINER:-drone-stack-d435i-voxblox}"
exec "$(dirname "$(readlink -f "$0")")/../modules/control/local-controller/run.sh" "$@"

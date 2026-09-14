#!/bin/bash
# Thin wrapper -> the flight-safety module's all-in-one run.sh (nodes + diagnostic GUI).
export DSD_CONTAINER="${DSD_CONTAINER:-drone-stack-d435i-voxblox}"
exec "$(dirname "$(readlink -f "$0")")/../modules/control/flight-safety/run.sh"

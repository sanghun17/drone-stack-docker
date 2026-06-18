#!/bin/bash
# Thin wrapper -> the flight-safety module's all-in-one run.sh (nodes + diagnostic GUI).
exec "$(dirname "$(readlink -f "$0")")/../modules/control/flight-safety/run.sh"

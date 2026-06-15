#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script.
exec "$(dirname "$(readlink -f "$0")")/../modules/control/flight-safety/run_monitor.sh" "$@"

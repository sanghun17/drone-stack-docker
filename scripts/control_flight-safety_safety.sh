#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script. *** KILL AUTHORITY ***
exec "$(dirname "$(readlink -f "$0")")/../modules/control/flight-safety/run_safety.sh" "$@"

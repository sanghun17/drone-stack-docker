#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script.
exec "$(dirname "$(readlink -f "$0")")/../modules/control/mavros/run.sh" "$@"

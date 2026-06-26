#!/bin/bash
# Thin wrapper — entrypoint only. ALL config/logic lives in the target run script.
# Default = stage-2 (b-spline: jax_to_mixtraj + traj_server + control_bridge[pos_cmd]).
# Stage-1 (raw JAX velocity): `bash control_local-controller.sh jax_traj`.
exec "$(dirname "$(readlink -f "$0")")/../modules/control/local-controller/run.sh" "${@:-pos_cmd}"

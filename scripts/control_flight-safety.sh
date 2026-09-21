#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/lib/select_stack.sh"
dsd_select_stack control/flight-safety
if [ ! -f "$ROOT/modules/control/flight-safety/run.sh" ]; then
  echo "Missing module package. Run ./setup.sh sync $DSD_STACK" >&2
  exit 2
fi
exec bash "$ROOT/modules/control/flight-safety/run.sh" "$@"

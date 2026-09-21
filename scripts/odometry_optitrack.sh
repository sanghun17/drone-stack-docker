#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/lib/select_stack.sh"
dsd_select_stack odometry/optitrack
if [ ! -f "$ROOT/modules/odometry/optitrack/run.sh" ]; then
  echo "Missing module package. Run ./setup.sh sync $DSD_STACK" >&2
  exit 2
fi
exec bash "$ROOT/modules/odometry/optitrack/run.sh" "$@"

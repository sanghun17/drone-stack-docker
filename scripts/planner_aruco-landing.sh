#!/usr/bin/env bash
# Start before takeoff. Pilot OFFBOARD entry starts the prepared mission.
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$ROOT_DIR/modules/planner/aruco-landing/run.sh" auto_start_on_offboard:=true dry_run:=false "$@"

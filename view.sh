#!/bin/bash
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${DISPLAY:-:0}"

if [ "$#" -gt 0 ]; then
  case "$1" in
    "$ROOT/flight_logs/epic_bags/"*)
      BAG_RELATIVE="${1#"$ROOT/flight_logs/epic_bags/"}"
      shift
      set -- "/bags/$BAG_RELATIVE" "$@"
      ;;
  esac
fi

XHOST_ADDED=0
if command -v xhost >/dev/null 2>&1 &&
   ! xhost 2>/dev/null | grep -Fqx "SI:localuser:root"; then
  if xhost +SI:localuser:root >/dev/null 2>&1; then
    XHOST_ADDED=1
  fi
fi

cleanup_xhost() {
  if [ "$XHOST_ADDED" -eq 1 ]; then
    xhost -SI:localuser:root >/dev/null 2>&1 || true
  fi
}
trap cleanup_xhost EXIT INT TERM HUP

"$ROOT/modules/planner/epic/run_view.sh" "$@"

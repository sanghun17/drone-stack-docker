#!/bin/bash
# Fetch flight_safety into its own workspace (ws/flight-safety). It builds standalone
# (deps are all noetic); mavros_msgs is a runtime-only dep, overlaid from risk-aware.
# Run by `setup.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/flight-safety/src/flight_safety"
REPO="${SAFETY_REPO:-git@github.com:sanghun17/flight_safety.git}"
BRANCH="${SAFETY_BRANCH:-main}"
LEGACY="$ROOT/ws/risk-aware/src/flight_safety"   # pre-split home; drop it so risk-aware stops building a duplicate
[ -d "$LEGACY" ] && { echo ">> removing legacy flight_safety under risk-aware"; rm -rf "$LEGACY"; }
if [ -d "$DST/.git" ]; then
  echo ">> flight_safety exists -> fetch + checkout $BRANCH"
  git -C "$DST" fetch origin "$BRANCH" && git -C "$DST" checkout "$BRANCH"
else
  mkdir -p "$ROOT/ws/flight-safety/src"
  git clone -b "$BRANCH" "$REPO" "$DST"
fi

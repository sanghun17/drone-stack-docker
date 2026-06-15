#!/bin/bash
# Fetch flight_safety into the risk-aware workspace (sibling of risk_aware_planning).
# Run by `setup.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/flight_safety"
REPO="${SAFETY_REPO:-git@github.com:sanghun17/flight_safety.git}"
BRANCH="${SAFETY_BRANCH:-main}"
if [ -d "$DST/.git" ]; then
  echo ">> flight_safety exists -> fetch + checkout $BRANCH"
  git -C "$DST" fetch origin "$BRANCH" && git -C "$DST" checkout "$BRANCH"
else
  mkdir -p "$ROOT/ws/risk-aware/src"
  git clone -b "$BRANCH" "$REPO" "$DST"
fi

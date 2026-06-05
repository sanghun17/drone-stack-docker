#!/bin/bash
# Fetch the risk_aware_planning source into the planner workspace (pinned branch).
# Run by `up.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/risk_aware_planning"
REPO="${RISK_AWARE_REPO:-git@github.com:sanghun17/risk-aware_planning.git}"
BRANCH="${RISK_AWARE_BRANCH:-jetson-orin-agx}"
if [ -d "$DST/.git" ]; then
  echo ">> risk_aware_planning exists -> fetch + checkout $BRANCH"
  git -C "$DST" fetch origin "$BRANCH" && git -C "$DST" checkout "$BRANCH"
else
  mkdir -p "$ROOT/ws/risk-aware/src"
  git clone -b "$BRANCH" "$REPO" "$DST"
fi

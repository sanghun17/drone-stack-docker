#!/bin/bash
# Fetch the shared risk_aware_planning source into its workspace (pinned branch).
# Run by `setup.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/risk_aware_planning"
REPO="${RISK_AWARE_REPO:-git@github.com:sanghun17/risk-aware_planning.git}"
BRANCH="${RISK_AWARE_BRANCH:-main}"
bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"

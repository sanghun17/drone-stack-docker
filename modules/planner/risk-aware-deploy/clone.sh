#!/bin/bash
# Fetch the risk_aware_planning source into the planner workspace (pinned branch).
# Run by `setup.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/risk_aware_planning"
REPO="${RISK_AWARE_REPO:-git@github.com:sanghun17/risk-aware_planning.git}"
BRANCH="${RISK_AWARE_BRANCH:-main}"   # 기본값도 main — 단일브랜치 통일(2026-07-25), stack.env가 정본
bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"

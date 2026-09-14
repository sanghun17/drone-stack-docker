#!/bin/bash
# Fetch the shared risk_aware_planning source into its workspace (pinned branch).
# Run by `setup.sh clone <stack>`. Cloned tree is gitignored by drone-stack.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/risk_aware_planning"
REPO="${RISK_AWARE_REPO:-git@github.com:sanghun17/risk-aware_planning.git}"
BRANCH="${RISK_AWARE_BRANCH:-main}"
bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"

# The former risk-aware-deploy/risk-aware-sim modules used sibling source
# checkouts in this same catkin workspace.  Keep those local trees recoverable,
# but prevent catkin from discovering every package two or three times after
# the modules are consolidated into the profile-driven canonical checkout.
for legacy in \
  "$ROOT/ws/risk-aware/src/risk_aware_planning_deploy_query_ablation" \
  "$ROOT/ws/risk-aware/src/risk_aware_planning_deploy_d79_query_ablation"; do
  if [ -d "$legacy" ]; then
    : > "$legacy/CATKIN_IGNORE"
  fi
done

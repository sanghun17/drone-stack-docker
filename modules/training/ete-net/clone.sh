#!/bin/bash
# Fetch the risk_aware_planning source (ETE-Net lives in uncertainty_predictor/ inside
# it) into the planner workspace (pinned branch). Run by `setup.sh clone <stack>`.
#
# Destination is deliberately the SAME tree that modules/planner/risk-aware/clone.sh
# uses -- it's one repo, and a host that runs both the deploy stack and the training
# stack should have exactly one checkout of it. The clone is idempotent (fetch +
# checkout when .git already exists), so whichever stack you clone first wins and the
# other converges to the same pinned branch.
#
# WHY this module needs its own clone.sh even though planner/risk-aware has one:
# ete-train stacks do NOT include planner/risk-aware (they'd drag in the whole ROS
# planner dep tree for nothing -- see this module's `needs:`). Without a clone.sh here,
# `setup.sh clone ete-train-*` fetched nothing and the code had to come from an
# arbitrary host path via a mount (`${RISK_AWARE_PLANNING_SRC}`), which only worked on
# a machine that happened to have the deploy stack cloned too. That broke the moment
# the training stack was set up on a training-only host. (2026-07-25)
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/risk_aware_planning"
REPO="${RISK_AWARE_REPO:-git@github.com:sanghun17/risk-aware_planning.git}"
BRANCH="${RISK_AWARE_BRANCH:-main}"
bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"

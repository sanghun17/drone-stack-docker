#!/bin/bash
# Fetch the risk_aware_planning source into the planner workspace (pinned branch).
# Run by `setup.sh clone <stack>`.
#
# Destination is the SAME tree modules/planner/risk-aware-deploy and modules/training/ete-net
# use. That is deliberate: risk_aware_planning is ONE repo on ONE branch
# (RISK_AWARE_BRANCH, unified to main 2026-07-25), and sim vs deploy differ only in
# which launch files they start -- not in code. So a host that runs several of these
# stacks keeps exactly one checkout. The clone is idempotent (fetch + checkout when
# .git exists), so whichever stack is cloned first wins and the rest converge.
#
# (Contrast: odometry/fast-livo and fast-livo-sim DO clone different branches
# -- jetson-orin-agx vs ml -- because those two genuinely diverged in source, not just
# in launch files. That divergence is a known debt, not a model to copy.)
#
# WHY this file was missing until 2026-07-25: this module bind-mounted
# ${RISK_AWARE_PLANNING_SRC} onto /work/ws/risk-aware/src/risk_aware_planning instead.
# On this host that mount is a no-op -- the dsd root is already mounted at /work, so
# the same host path lands on the same container path twice. Its only real effect was
# to make sim-x86 unbuildable on a machine that hadn't cloned the deploy stack first.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/risk-aware/src/risk_aware_planning"
REPO="${RISK_AWARE_REPO:-git@github.com:sanghun17/risk-aware_planning.git}"
BRANCH="${RISK_AWARE_BRANCH:-main}"
bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"

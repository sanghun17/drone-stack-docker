#!/bin/bash
# Fetch the EPIC-stack tree into ws/epic. Run by `setup.sh clone <stack>`.
#
# LAYOUT NOTE: unlike every other module here, EPIC-stack's repo root IS the
# catkin workspace root — it ships `src/` (EPIC_poongsan, FAST_LIO,
# livox_ros_driver2, reactive_local_avoidance) plus `execution/` runner scripts
# at the top level. So this clones the repo AT ws/epic, not into ws/epic/src:
# after this, ws/epic/src exists because the repo provides it, and
# ws/epic/{build,devel,logs} are catkin artifacts (gitignored by the parent).
#
# The `execution/*.sh` scripts this module's run_*.sh delegate to therefore live
# at ws/epic/execution/ and resolve their own WORKSPACE_ROOT as ws/epic.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/epic"
REPO="${EPIC_REPO:-https://github.com/kimhyoon/EPIC-stack.git}"
BRANCH="${EPIC_BRANCH:-dev}"

bash "$ROOT/modules/_common/clone_repo.sh" "$DST" "$REPO" "$BRANCH"

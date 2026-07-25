#!/bin/bash
# Fetch the fast_livo `ml` branch (mapping_simulator_openvins.launch — sim-only, does
# NOT exist on jetson-orin-agx) into its OWN workspace src (ws/fast-livo-sim, separate
# from ws/fast-livo which stays pinned to jetson-orin-agx). Run by `setup.sh clone <stack>`.
#
# LAYOUT NOTE: the `ml` branch's repo root IS the FAST-LIVO2 package itself (package.xml
# at root), unlike jetson-orin-agx whose root is the whole livo_ws src space. So here we
# clone into src/FAST-LIVO2 and add rpg_vikit as a sibling — mirroring the ml host's
# working livo_ws (~/fast_livo2/src = {FAST-LIVO2, rpg_vikit}). FAST-LIVO2 hard-depends
# on vikit_common/vikit_ros (package.xml), so without rpg_vikit catkin cannot build.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SRC="$ROOT/ws/fast-livo-sim/src"
REPO="${FASTLIVO_REPO:-git@github.com:sanghun17/fast_livo2_custom.git}"
BRANCH="${FASTLIVO_SIM_BRANCH:-ml}"
VIKIT_REPO="${VIKIT_REPO:-https://github.com/xuankuzcr/rpg_vikit.git}"
VIKIT_COMMIT="${VIKIT_COMMIT:-6c886c8}"   # = ml 호스트 ~/fast_livo2/src/rpg_vikit 현행 (behavior-preserving pin)

mkdir -p "$SRC"
bash "$ROOT/modules/_common/clone_repo.sh" "$SRC/FAST-LIVO2" "$REPO" "$BRANCH"

# rpg_vikit is pinned to a COMMIT, not a branch, so it doesn't fit the branch-based
# helper above (fetch <ref> + checkout <ref> where ref is a branch name) -- kept as its
# own fetch-everything-then-checkout-the-commit logic, unchanged from before extraction.
if [ -d "$SRC/rpg_vikit/.git" ]; then
  echo ">> rpg_vikit exists -> fetch + checkout $VIKIT_COMMIT"
  git -C "$SRC/rpg_vikit" fetch origin && git -C "$SRC/rpg_vikit" checkout "$VIKIT_COMMIT"
else
  git clone "$VIKIT_REPO" "$SRC/rpg_vikit"
  git -C "$SRC/rpg_vikit" checkout "$VIKIT_COMMIT"
fi

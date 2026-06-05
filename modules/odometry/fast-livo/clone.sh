#!/bin/bash
# Fetch the fast_livo source (= the whole livo_ws src space: FAST-LIVO2 + rpg_vikit +
# vision_opencv) into livo_ws/src (pinned branch). Run by `setup.sh clone <stack>`.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
DST="$ROOT/ws/fast-livo/src"
REPO="${FASTLIVO_REPO:-git@github.com:sanghun17/fast_livo2_custom.git}"
BRANCH="${FASTLIVO_BRANCH:-jetson-orin-agx}"
if [ -d "$DST/.git" ]; then
  echo ">> fast_livo exists -> fetch + checkout $BRANCH"
  git -C "$DST" fetch origin "$BRANCH" && git -C "$DST" checkout "$BRANCH"
else
  mkdir -p "$ROOT/ws/fast-livo"
  git clone -b "$BRANCH" "$REPO" "$DST"
fi

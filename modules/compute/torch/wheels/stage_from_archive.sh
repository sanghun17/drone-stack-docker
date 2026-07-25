#!/bin/bash
# compute/torch/wheels: stage the source-built ABI=1 torch wheel into THIS directory
# from this host's archive copy, before `setup.sh build/up` on any stack that sets
# `build_env: {TORCH_VARIANT: src-abi1}` (see install.sh's sm89|sm75 branch). This
# directory is BuildKit's `assets: [wheels]` bind-mount source (module.yml) -- the
# wheel itself is git-untracked (274MB > GitHub's 100MB limit, see README.md), so a
# fresh checkout / new host has nothing here but this script + README.md until this
# runs once.
#
# Archive location is host-specific (this repo is checked out on several machines
# with different disks) -- read from TORCH_WHEEL_ARCHIVE_DIR (config/stack.env,
# override per-host via config/stack.env.local, per project policy NEVER by editing
# the tracked default). No fallback: halts if unset or the wheel isn't there (project
# no-config-fallback policy -- see CLAUDE.md).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
# shellcheck disable=SC1091
[ -f "$ROOT/config/stack.env" ] && { set -a; source "$ROOT/config/stack.env"; set +a; }

WHL="torch-2.2.2-cp38-cp38-linux_x86_64.whl"
[ -n "${TORCH_WHEEL_ARCHIVE_DIR:-}" ] || {
  echo "ERROR: TORCH_WHEEL_ARCHIVE_DIR not set (config/stack.env / config/stack.env.local)." >&2
  exit 1
}
SRC="$TORCH_WHEEL_ARCHIVE_DIR/$WHL"
DST="$SCRIPT_DIR/$WHL"
[ -f "$SRC" ] || {
  echo "ERROR: wheel not found at $SRC (TORCH_WHEEL_ARCHIVE_DIR=$TORCH_WHEEL_ARCHIVE_DIR)." >&2
  exit 1
}

src_md5=$(md5sum "$SRC" | awk '{print $1}')
if [ -f "$DST" ] && [ "$(md5sum "$DST" | awk '{print $1}')" = "$src_md5" ]; then
  echo "already staged (md5 $src_md5): $DST"
  exit 0
fi
# hardlink when same filesystem (0 extra bytes -- true for both known archives today,
# ml and im, each colocated with this checkout's disk), else fall back to a copy.
ln -f "$SRC" "$DST" 2>/dev/null || cp "$SRC" "$DST"
echo "staged (md5 $src_md5): $DST"

#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
expected=scripts/git-hooks
current="$(git -C "$ROOT" config --get core.hooksPath || true)"
if [ "$current" = "$expected" ]; then
  exit 0
fi
if [ -n "$current" ]; then
  echo "Cannot replace existing core.hooksPath=$current; integrate the layout hooks first." >&2
  exit 1
fi
hooks="$(git -C "$ROOT" rev-parse --git-path hooks)"
[[ "$hooks" = /* ]] || hooks="$ROOT/$hooks"
for hook in pre-commit pre-push; do
  if [ -f "$hooks/$hook" ]; then
    echo "Cannot replace existing $hooks/$hook; integrate the layout hooks first." >&2
    exit 1
  fi
done
git -C "$ROOT" config --local core.hooksPath "$expected"
echo 'Installed repository layout pre-commit and pre-push hooks.'

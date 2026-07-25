#!/bin/bash
# Idempotent single-repo clone/update, shared by every module's clone.sh (extracted
# 2026-07-26 -- all of them had IDENTICAL "if $DST/.git exists: fetch + checkout
# $BRANCH, else: git clone -b $BRANCH" logic, differing only in which repo/branch/dest).
#
# Each module's own clone.sh still resolves ITS repo/branch from stack.env with a
# hardcoded fallback (e.g. REPO="${RISK_AWARE_REPO:-git@github.com:...}") and passes
# the already-resolved values here as plain args -- this script does not read any env
# var itself. Keep it that way: setup.sh's `clone` target sources config/stack.env with
# `set -a` specifically so those env vars reach clone.sh as real exports (see setup.sh's
# comment near the top) -- stack.env is meant to be the source of truth, and the
# per-module hardcoded fallback only exists for a bare `bash modules/.../clone.sh` run
# without setup.sh. If this helper started reading env vars directly instead of taking
# args, a caller could accidentally skip a module-specific env var and fall silently
# back to some helper-level default -- reintroducing the exact "hardcoded default wins"
# bug stack.env's `set -a` was added to kill.
#
# Usage: clone_repo.sh <DST> <REPO> <BRANCH>
#   DST     absolute path the repo should be checked out at
#   REPO    git remote URL
#   BRANCH  branch to track -- fetch+checkout if $DST/.git exists, else `clone -b`
set -e
DST="${1:?clone_repo.sh: missing DST (arg1)}"
REPO="${2:?clone_repo.sh: missing REPO (arg2)}"
BRANCH="${3:?clone_repo.sh: missing BRANCH (arg3)}"

if [ -d "$DST/.git" ]; then
  echo ">> $DST exists -> fetch + checkout $BRANCH"
  git -C "$DST" fetch origin "$BRANCH" && git -C "$DST" checkout "$BRANCH"
else
  mkdir -p "$(dirname "$DST")"
  git clone -b "$BRANCH" "$REPO" "$DST"
fi

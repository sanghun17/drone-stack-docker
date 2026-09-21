#!/bin/bash
# Sourced (host side) by each module run.sh, right before `docker start "$__C"`.
#
# A container's bind mounts are frozen at creation time. So if this repo is moved
# or renamed, the existing container still mounts the OLD host path at /work and a
# plain `docker start` would silently hand you an empty /work (dockerd recreates the
# vanished bind source as an empty dir). Detect that — or a missing container — and
# recreate from compose, whose `../..` volume source always tracks the current root.
#
# Caller provides: $__C (container name, e.g. drone-stack-d435i-voxblox), $__R (repo root).
__STK="${__C#drone-stack-}"                 # drone-stack-d435i-voxblox -> d435i-voxblox
__cf="$__R/.build/$__STK/compose.yml"
__have="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/work"}}{{.Source}}{{end}}{{end}}' "$__C" 2>/dev/null || true)"
if [ "$__have" != "$__R" ] && [ -f "$__cf" ]; then
  [ -n "$__have" ] && echo ">> $__C mounts /work from '$__have' != '$__R' — recreating" >&2
  docker compose -f "$__cf" up -d --force-recreate >/dev/null
  # A fresh container loses any runtime tweaks, so re-hook the interactive ROS env
  # (mirrors modules/base/install.sh) — keeps `dsd` auto-sourcing after a recreate.
  # Idempotent, and a no-op once the hook is baked into the image by a rebuild.
  docker exec -i "$__C" bash >/dev/null 2>&1 <<'EOSH' || true
grep -q "drone-stack: ROS env for interactive shells" /root/.bashrc 2>/dev/null && exit 0
cat >> /root/.bashrc <<'RC'

# drone-stack: ROS env for interactive shells
[ -f /work/config/interactive_ros.sh ] && source /work/config/interactive_ros.sh
RC
EOSH
fi
unset __STK __cf __have

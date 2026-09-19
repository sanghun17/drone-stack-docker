#!/bin/bash
# Source this in a host-side module before selecting its Docker container.
dsd_select_stack() {
  local dsd_context_root dsd_context_exports
  dsd_context_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  local -a dsd_context_args=()
  [ -z "${1:-}" ] || dsd_context_args=(--module "$1")
  dsd_context_exports="$(python3 "$dsd_context_root/tools/stack_context.py" resolve --shell "${dsd_context_args[@]}")" || return $?
  eval "$dsd_context_exports"  # Only validated keys with shlex-quoted values from our resolver.
  echo "[stack] $DSD_STACK -> $DSD_CONTAINER" >&2
}

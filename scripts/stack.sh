#!/bin/bash
exec python3 "$(dirname "$(readlink -f "$0")")/../tools/stack_context.py" "$@"

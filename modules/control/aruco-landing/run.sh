#!/usr/bin/env bash
exec bash "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../planner/aruco-landing" && pwd)/run.sh" "$@"

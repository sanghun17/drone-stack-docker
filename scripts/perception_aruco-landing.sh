#!/bin/bash
exec "$(dirname "$(readlink -f "$0")")/../modules/perception/aruco-landing/run.sh" "$@"

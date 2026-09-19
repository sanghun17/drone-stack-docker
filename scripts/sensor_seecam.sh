#!/bin/bash
# Thin wrapper — camera settings and lifecycle stay in the sensor module.
exec "$(dirname "$(readlink -f "$0")")/../modules/sensor/see3cam-24cug/run.sh" "$@"

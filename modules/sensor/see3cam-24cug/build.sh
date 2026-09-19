#!/bin/bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
src="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$src/../../.." && pwd)"
build="$root/.build/see3cam-demand/$(uname -m)"
cmake -S "$src" -B "$build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build" --parallel 2

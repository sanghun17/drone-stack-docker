#!/bin/bash
# planner/epic: environment deps that are NOT apt packages.
#
#   1. Patched Livox-SDK2 — src/livox_ros_driver2's CMakeLists resolves
#      /usr/local/lib/liblivox_lidar_sdk_static.a, so the SDK must exist in the
#      image before the workspace builds.
#   2. GeographicLib geoid dataset — mavros refuses to start without it.
#
# Both are baked into the image on purpose (never installed into a running
# container): a container recreate must not be able to silently break a working
# lidar or a working mavros.
set -e

# ---------------------------------------------------------------------------
# 1. Livox-SDK2 with the Mid-360S dev_type normalization
#
# The Mid-360S reports dev_type 35, which upstream's device_manager drops before
# it ever reaches the command handler (livox_ros_driver2 issue #240) — the lidar
# then simply never appears. The patch normalizes 35 -> 9 (plain Mid-360).
#
# Applied with --forward so a future upstream fix is not a hard error, but the
# anchor is asserted FIRST and the result asserted AFTER: a silent no-op here
# would be worse than a failed build, because the symptom (no lidar) shows up
# only on the flight line.
# ---------------------------------------------------------------------------
git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2
cd /opt/Livox-SDK2
grep -q "kLivoxLidarTypeMid360s" include/livox_lidar_def.h
patch -p1 --forward < /modules/planner/epic/patches/livox-sdk2-mid360s-devtype.patch
grep -q "normalize to Mid-360" sdk_core/device_manager.cpp
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release >/dev/null
make -j"$(nproc)"
make install
ldconfig

# ---------------------------------------------------------------------------
# 2. GeographicLib geoids (mavros runtime). Same dataset control/mavros pulls via
# the upstream installer script; egm96-5 is the one mavros actually reads.
# ---------------------------------------------------------------------------
geographiclib-get-geoids egm96-5

echo ">> planner/epic install.sh done (Livox-SDK2 + geoids)"

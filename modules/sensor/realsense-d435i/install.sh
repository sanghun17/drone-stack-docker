#!/bin/bash
# Source-build librealsense 2.50.0 + realsense-viewer, CUDA-enabled, installed SYSTEM-WIDE.
#
# WHY source: realsense-viewer is NOT apt-installable on arm64 (Intel's binary repo is x86-only;
# the ros-noetic-librealsense2 deb ships libs only, no viewer). BUILD_GRAPHICAL_EXAMPLES=true is
# the only way to get the viewer + Advanced-Mode Depth Control sliders to tune the edge "flying
# pixel" drag.
#
# WHY system-wide (/usr/local, not an isolated prefix): isolation's only purpose would be to keep
# the CUDA SDK off the ROS pipeline (JAX GPU contention) — accepted as fine. The VIEWER links this
# CUDA lib via its rpath. NOTE (verified): the ROS node (realsense2_camera) still loads the APT
# librealsense from /opt/ros/noetic/lib — ROS setup.bash prepends that to LD_LIBRARY_PATH, which
# beats ldconfig's /usr/local. So the pipeline stays on the CPU lib by default; to move it onto
# CUDA, prepend /usr/local/lib to LD_LIBRARY_PATH in the camera launch (separate opt-in, after a
# JAX-contention check).
#
# Version pinned to 2.50.0 to MATCH the apt librealsense realsense2_camera 2.3.2 was built
# against (same SONAME/ABI, no surprises). Taeyoung96/librealsense-Docker uses 2.47.0 — changed.
# RSUSB backend: required in Docker (can't patch the host's uvc kernel modules).
# NOTE: CUDA does NOT speed the viewer's 3D render — that's llvmpipe software GL over noVNC
# (utility/gui-vnc). CUDA only accelerates the SDK's depth/pointcloud/color kernels.
#
# Runs once at image build (deps.source); BuildKit caches the layer until THIS script changes.
set -e
RS_VER="${RS_VER:-2.50.0}"
JOBS="${RS_JOBS:-6}"   # cap parallelism: nvcc is RAM-heavy on Orin's unified memory

apt-get update && apt-get install -y --no-install-recommends \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
 && rm -rf /var/lib/apt/lists/*

curl -fsSL "https://github.com/IntelRealSense/librealsense/archive/refs/tags/v${RS_VER}.tar.gz" \
  | tar xz -C /opt
cd "/opt/librealsense-${RS_VER}"

cmake -B build -S . \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_WITH_CUDA=true \
  -DCMAKE_CUDA_ARCHITECTURES=87 \
  -DBUILD_GRAPHICAL_EXAMPLES=true \
  -DBUILD_EXAMPLES=true
  # BUILD_PYTHON_BINDINGS dropped: 2.50.0's find_package(Python) grabs python2.7 (ignores the
  # legacy PYTHON_EXECUTABLE hint) and fails on missing Dev headers. pyrealsense2 isn't needed
  # for the viewer; re-add later with a proper Python3 hint if we want it.
cmake --build build -j"$JOBS" --target install
ldconfig   # /usr/local/lib librealsense2 (CUDA) takes precedence over the apt one

rm -rf "/opt/librealsense-${RS_VER}"   # source tree not needed at runtime; keep the layer small

echo "librealsense v${RS_VER} (CUDA, system-wide) + realsense-viewer installed"
command -v realsense-viewer >/dev/null && echo "  -> realsense-viewer on PATH"

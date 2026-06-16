#!/bin/bash
# Source-build librealsense 2.50.0 + realsense-viewer, CUDA-enabled, installed SYSTEM-WIDE.
#
# WHY source: realsense-viewer is NOT apt-installable on arm64 (Intel's binary repo is x86-only;
# the ros-noetic-librealsense2 deb ships libs only, no viewer). BUILD_GRAPHICAL_EXAMPLES=true is
# the only way to get the viewer + Advanced-Mode Depth Control sliders to tune the edge "flying
# pixel" drag.
#
# WHY system-wide (/usr/local): the viewer (rpath -> /usr/local) always uses this CUDA build. We
# KEEP the apt CPU librealsense too (at /opt/ros/noetic/lib) so run.sh can toggle the camera node
# between CPU and CUDA via LD_LIBRARY_PATH with no rebuild. ROS's setup.bash puts /opt/ros/noetic/lib
# first -> CPU is the default; run.sh prepends /usr/local/lib for CUDA mode (full-res). Same soname
# 2.50 -> ABI-compatible either way.
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
ldconfig   # register the /usr/local CUDA libs (the apt CPU lib stays at /opt/ros/noetic/lib)

# Both librealsense builds are kept on purpose for run.sh's CPU/CUDA toggle:
#   apt CPU  -> /opt/ros/noetic/lib/aarch64-linux-gnu  (ROS LD_LIBRARY_PATH default)
#   CUDA     -> /usr/local/lib                          (run.sh prepends for full-res)

rm -rf "/opt/librealsense-${RS_VER}"   # source tree not needed at runtime; keep the layer small

echo "librealsense v${RS_VER} (CUDA, system-wide) + realsense-viewer installed"
command -v realsense-viewer >/dev/null && echo "  -> realsense-viewer on PATH"

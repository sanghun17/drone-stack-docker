#!/bin/bash
# Source-build librealsense 2.50.0 + realsense-viewer, CUDA-enabled, installed SYSTEM-WIDE.
#
# WHY source: realsense-viewer is NOT apt-installable on arm64 (Intel's binary repo is x86-only;
# the ros-noetic-librealsense2 deb ships libs only, no viewer). BUILD_GRAPHICAL_EXAMPLES=true is
# the only way to get the viewer + Advanced-Mode Depth Control sliders to tune the edge "flying
# pixel" drag.
#
# WHY system-wide (/usr/local, not an isolated prefix): the viewer AND the ROS pipeline use this
# CUDA librealsense. ROS's setup.bash would otherwise make the node load the apt CPU lib first (it
# prepends /opt/ros/noetic/lib to LD_LIBRARY_PATH; the nodelet has no RPATH), so we DELETE the apt
# librealsense libs below — every consumer then resolves our /usr/local CUDA build via ldconfig.
# Same soname 2.50 -> ABI-compatible. JAX GPU contention on Orin's unified memory is accepted.
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

# Remove the apt librealsense SDK libs (pulled in by ros-noetic-realsense2-camera) so the ROS node
# can't load them over our CUDA build via ROS's LD_LIBRARY_PATH. With them gone, the nodelet
# resolves /usr/local through ldconfig. dpkg still marks the deb installed (harmless in a built
# image). NOTE the patterns: 'librealsense2.so*' / 'librealsense2-gl.so*' (literal dot) match ONLY
# the SDK libs — NOT 'librealsense2_camera.so', the wrapper nodelet we must keep.
find /opt/ros/noetic/lib \( -name 'librealsense2.so*' -o -name 'librealsense2-gl.so*' \) -delete 2>/dev/null || true
ldconfig

rm -rf "/opt/librealsense-${RS_VER}"   # source tree not needed at runtime; keep the layer small

echo "librealsense v${RS_VER} (CUDA, system-wide) + realsense-viewer installed"
command -v realsense-viewer >/dev/null && echo "  -> realsense-viewer on PATH"

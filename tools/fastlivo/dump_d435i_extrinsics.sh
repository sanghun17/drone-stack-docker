#!/bin/bash
# Dump the D435i FACTORY extrinsics (no calibration target needed) so they can be
# pasted into config/d435i.yaml. Run with the camera UP (sensor/realsense-d435i).
#
# fast-livo uses (vio.cpp:57): Rci = Rcl*Rli, Pci = Rcl*Pli+Pcl, where
#   extrin_calib/Rcl,Pcl  = depth(LiDAR) -> color(camera)   [already filled, factory]
#   extrin_calib/extrinsic_R,_T = Rli,Pli = depth(LiDAR) -> IMU  [CURRENTLY IDENTITY]
# The identity IMU<->depth extrinsic is the prime suspect if eval shows APE coupled
# to rotation rate (lever arm). This prints the factory value to replace it with.

if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../.." && pwd)"
  source "$__R/modules/ensure_container.sh"
  docker start "$__C" >/dev/null 2>&1
  docker exec -it "$__C" bash "/work/${__S#$__R/}" "$@"
  exit $?
fi
source /opt/ros/noetic/setup.bash
source /work/config/ros_env.sh
source /work/modules/ensure_roscore.sh

echo "=== /camera/extrinsics/depth_to_color  (-> Rcl/Pcl, depth->color) ==="
timeout 5 rostopic echo -n1 /camera/extrinsics/depth_to_color 2>/dev/null || echo "  (no msg — is the camera up?)"

echo
echo "=== tf depth_optical_frame -> imu_optical_frame  (-> extrinsic_R/_T = Rli/Pli) ==="
echo "    translation (xyz) goes to extrinsic_T; the 3x3 rotation matrix to extrinsic_R."
timeout 5 rosrun tf tf_echo /camera_depth_optical_frame /camera_imu_optical_frame 2>/dev/null \
  | sed -n '1,12p' || echo "  (tf not available — is the camera up?)"

echo
echo "=== sanity: tf depth -> color (should match Rcl/Pcl above) ==="
timeout 5 rosrun tf tf_echo /camera_depth_optical_frame /camera_color_optical_frame 2>/dev/null \
  | sed -n '1,12p'

echo
echo "NOTE: verify the matrix convention by A/B replay — copy d435i.yaml, paste the"
echo "      value into extrin_calib, then:"
echo "      replay_fastlivo.sh BAG --config /work/tools/fastlivo/<edited>.yaml"
echo "      and compare APE + the lever-arm corr against the identity baseline."

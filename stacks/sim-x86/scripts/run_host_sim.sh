#!/usr/bin/env bash
# Host half of the comparison stack; resolve assets from this checkout.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source "$ROOT/config/sim.env"
source "$ROOT/config/ros_env.sh"
case "${1:-}" in
  display)
    # Permit only the local container user; do not disable X access control.
    DISPLAY="${DISPLAY:-:0}" exec xhost +si:localuser:root
    ;;
  unreal)
    manifest="${SIM_CAPTURE_BUILD_MANIFEST:-$ROOT/.build/sim-x86/capture-time-project/build.json}"
    if [ ! -f "$manifest" ]; then
      echo 'Build the corrected simulator first: python3 stacks/sim-x86/scripts/build_capture_time_sim.py' >&2
      exit 1
    fi
    mapfile -t runtime < <(python3 - "$manifest" "$ROOT" <<'PY'
import hashlib,json,sys
from pathlib import Path
m=json.loads(Path(sys.argv[1]).read_text());root=Path(sys.argv[2])
patch=root/'modules/simulation/airsim/patches/rendered_pose_timestamp.patch'
binary=Path(m['project']).parent/'Plugins/AirSim/Binaries/Linux/libUE4Editor-AirSim.so'
assert hashlib.sha256(patch.read_bytes()).hexdigest()==m['patch_sha256'], 'Rebuild after timestamp patch changes'
assert hashlib.sha256(binary.read_bytes()).hexdigest()==m['plugin_binary_sha256'], 'Simulator binary differs from build manifest'
if m.get('timing_diagnostic_patch_sha256'):
    diagnostic=root/'modules/simulation/airsim/patches/capture_timing_diagnostics.patch'
    assert hashlib.sha256(diagnostic.read_bytes()).hexdigest()==m['timing_diagnostic_patch_sha256'], 'Timing diagnostic patch differs'
if m.get('async_readback_patch_sha256'):
    asynchronous=root/'modules/simulation/airsim/patches/capture_slot_readback.patch'
    assert hashlib.sha256(asynchronous.read_bytes()).hexdigest()==m['async_readback_patch_sha256'], 'Slot readback patch differs'
print(m['engine']);print(m['project'])
PY
)
    [ "${#runtime[@]}" = 2 ] || exit 1
    source /opt/ros/noetic/setup.bash
    rosparam set /comparison/host_simulator "$(cat "$manifest")"
    rosparam set /comparison/host_simulator/launch_arguments "$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1:]))' "${@:2}")"
    export DISPLAY="${DISPLAY:-:0}"
    "${runtime[0]}/Engine/Binaries/Linux/UE4Editor" "${runtime[1]}" \
      -game -windowed -ResX=1280 -ResY=720 -nosound -unattended \
      "-settings=$AIRSIM_SETTINGS_DIR/settings.json" "${@:2}" &
    simulator_pid=$!
    trap 'kill -TERM "$simulator_pid" 2>/dev/null || true; wait "$simulator_pid" 2>/dev/null || true; exit 130' INT TERM HUP
    wait "$simulator_pid"
    ;;
  unreal-historical)
    package="$UE_PACKAGED_ROOT/test9_vio_velocity/LinuxNoEditor/MyFirstUE4/Binaries/Linux"
    cd "$package"
    export DISPLAY="${DISPLAY:-:0}"
    source /opt/ros/noetic/setup.bash
    rosparam set /comparison/host_simulator '{backend: historical-package, timestamp_semantics: GPU-readback-completion}'
    # UE can retain the RPC port after terminal SIGINT. Forward termination
    # explicitly and wait so a subsequent run cannot collide with this process.
    ./MyFirstUE4 "-settings=$AIRSIM_SETTINGS_DIR/settings.json" "${@:2}" &
    simulator_pid=$!
    trap 'kill -TERM "$simulator_pid" 2>/dev/null || true; wait "$simulator_pid" 2>/dev/null || true; exit 130' INT TERM HUP
    wait "$simulator_pid"

    ;;
  airsim)
    source /opt/ros/noetic/setup.bash
    source "$AIRSIM_ROOT/ros/devel/setup.bash"
    api_port="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("ApiServerPort", 41451))' "$AIRSIM_SETTINGS_DIR/settings.json")"
    python3 "$ROOT/stacks/sim-x86/scripts/wait_airsim.py" --tcp-only --host "$ROS_MASTER_HOST" --port "$api_port"
    # Relocated catkin assets retain absolute source paths in devel/.catkin.
    # Override discovery without modifying preserved asset/build metadata.
    export ROS_PACKAGE_PATH="$AIRSIM_ROOT/ros/src:${ROS_PACKAGE_PATH:-}"
    export LD_LIBRARY_PATH="$AIRSIM_ROOT/ros/devel/lib:${LD_LIBRARY_PATH:-}"
    exec roslaunch "$AIRSIM_ROOT/ros/src/airsim_ros_pkgs/launch/airsim_node.launch" host:="$ROS_MASTER_HOST" "${@:2}"
    ;;
  *) echo 'usage: run_host_sim.sh unreal|unreal-historical|airsim|display [arguments]' >&2; exit 2 ;;
esac

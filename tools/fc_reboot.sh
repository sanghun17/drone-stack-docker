#!/bin/bash
# Reboot the PX4 flight controller (mavros MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1).
# param2=0 -> companion (Jetson) untouched.
__C=drone-stack-d435i-voxblox

if [ ! -f /.dockerenv ]; then
  docker start "$__C" >/dev/null 2>&1
  exec docker exec "$__C" bash /work/scripts/fc_reboot.sh
fi

source /work/config/interactive_ros.sh >/dev/null 2>&1
timeout 8 rosservice call /mavros/cmd/command \
  "{broadcast: false, command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"

#!/bin/bash
# Headless rviz over VNC. Starts a virtual display + VNC server (once), then rviz.
#
# View it from your machine (no ssh -X / local X server needed):
#   1)  ssh -L 5900:localhost:5900 hmcl@<jetson-ip>
#   2)  open any VNC client -> localhost:5900   (or a browser if noVNC is added)
#
# Pass an rviz config: run.sh -d /work/ws/risk-aware/src/.../some.rviz

# (host) auto-enter the dsd container; (inside) run the node.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox; docker start "$__C" >/dev/null 2>&1
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/../../.." && pwd)"
  __TT=$([ -t 1 ] && echo -it || echo -i)
  exec docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"
fi
set -e
export DISPLAY=:99

pgrep -x Xvfb    >/dev/null 2>&1 || { Xvfb :99 -screen 0 1920x1080x24 +extension GLX +render >/tmp/xvfb.log 2>&1 & sleep 2; }
pgrep -x fluxbox >/dev/null 2>&1 || { fluxbox >/tmp/fluxbox.log 2>&1 & sleep 1; }
pgrep -x x11vnc  >/dev/null 2>&1 || { x11vnc -display :99 -localhost -nopw -forever -shared -rfbport 5900 >/tmp/x11vnc.log 2>&1 & sleep 2; }

source /opt/ros/noetic/setup.bash
[ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
export ROS_MASTER_URI="http://localhost:${ROS_MASTER_PORT:-11399}"
export LIBGL_ALWAYS_SOFTWARE=1   # rviz GL via mesa llvmpipe on the virtual display

echo ">> rviz on :99 served via VNC localhost:5900"
echo ">> from your machine:  ssh -L 5900:localhost:5900 hmcl@<jetson>  then VNC -> localhost:5900"
exec rviz "$@"

#!/bin/bash
# Remote rviz for the dsd stack — run on the HOST (after ssh into the Jetson), exactly like
# the module run scripts: the wrapper below re-execs this same file inside the container.
#
# ssh -X gives a BLACK window here (the container has the nvidia GL for jax/torch, so rviz
# direct-renders on the Jetson GPU and the pixels never cross the X11 wire). So we render on
# a virtual display with mesa software GL and serve it over VNC (ships pixels, no GLX).
#
#   on the Jetson host:  ./scripts/rviz.sh
#   on YOUR machine:     ssh -L 5900:localhost:5900 hmcl@192.168.50.36
#                        vncviewer localhost:5900
# (the vncviewer step is on your machine because the window has to live on your screen —
#  nothing the Jetson runs can put a window there for you.)

# (host) auto-enter the dsd container; (inside) run rviz over VNC.
if [ ! -f /.dockerenv ]; then
  __C=drone-stack-d435i-voxblox; docker start "$__C" >/dev/null 2>&1
  __S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
  __R="$(cd "$(dirname "$__S")/.." && pwd)"           # scripts/ is one level under the repo root
  __TT=$([ -t 1 ] && echo -it || echo -i)
  exec docker exec $__TT "$__C" bash "/work/${__S#$__R/}" "$@"
fi
set -e

# virtual display + VNC server (idempotent)
pgrep -x Xvfb   >/dev/null 2>&1 || { nohup Xvfb :99 -screen 0 1600x900x24 >/tmp/xvfb.log 2>&1 & sleep 2; }
pgrep -x x11vnc >/dev/null 2>&1 || { nohup x11vnc -display :99 -localhost -nopw -forever -shared -rfbport 5900 >/tmp/x11vnc.log 2>&1 & sleep 2; }

source /opt/ros/noetic/setup.bash
[ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
[ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh
# pin rviz to mesa software GL (else libglvnd picks nvidia and it fails on the virtual display)
export DISPLAY=:99 __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe

echo ">> rviz on VNC :5900 — from your machine: ssh -L 5900:localhost:5900 hmcl@192.168.50.36 ; vncviewer localhost:5900"
exec rviz "$@"

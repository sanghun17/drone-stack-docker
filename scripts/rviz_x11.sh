#!/bin/bash
# Remote rviz for the dsd stack.
#
# NOTE: ssh -X gives a BLACK window on this Jetson — the container has the nvidia GL
# (runtime: nvidia for jax/torch), so rviz direct-renders on the Jetson GPU and the pixels
# never cross the X11 wire. So this launches rviz on a virtual display + serves it over VNC
# (renders on the Jetson with mesa software GL, ships pixels — no GLX transport involved).
#
#   on the host:     ./scripts/rviz_x11.sh
#   on YOUR machine: ssh -L 5900:localhost:5900 hmcl@192.168.50.36
#                    vncviewer localhost:5900
set -e
C=drone-stack-d435i-voxblox
docker start "$C" >/dev/null 2>&1

# virtual display + VNC server (idempotent)
docker exec "$C" bash -c '
  pgrep -x Xvfb   >/dev/null 2>&1 || { nohup Xvfb :99 -screen 0 1600x900x24 >/tmp/xvfb.log 2>&1 & sleep 2; }
  pgrep -x x11vnc >/dev/null 2>&1 || { nohup x11vnc -display :99 -localhost -nopw -forever -shared -rfbport 5900 >/tmp/x11vnc.log 2>&1 & sleep 2; }
'
# rviz pinned to mesa software GL (llvmpipe) on :99 — else libglvnd picks nvidia and it fails
docker exec -d "$C" bash -lc '
  pkill -x rviz 2>/dev/null
  source /opt/ros/noetic/setup.bash
  [ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
  [ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh
  export DISPLAY=:99 __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe
  exec rviz
'
sleep 3
echo ">> rviz is rendering on the Jetson, served over VNC (:5900). View it from YOUR machine:"
echo ">>   ssh -L 5900:localhost:5900 hmcl@192.168.50.36"
echo ">>   vncviewer localhost:5900"

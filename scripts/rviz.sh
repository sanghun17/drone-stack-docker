#!/bin/bash
# Remote rviz — run this ON THE JETSON HOST (after `ssh -X`), see it on YOUR screen. One command,
# nothing to run on your machine.
#
# Why it's built this way: ssh -X can't forward rviz's 3D/OpenGL on this Jetson (nvidia GL
# direct-renders -> the window is black). So rviz renders on a virtual display with mesa software
# GL, x11vnc turns it into pixels, and a 2D vnc *viewer* shows those pixels. That viewer is a plain
# 2D X app, so it forwards fine over the ssh -X you already have -> the window lands on your screen.
set -e
C=drone-stack-d435i-voxblox
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -n "${DISPLAY:-}" ] || { echo "ERROR: \$DISPLAY is empty. Reconnect with:  ssh -X hmcl@192.168.50.36"; exit 1; }
docker start "$C" >/dev/null 2>&1

# the 2D viewer must exist in the container (it draws on your ssh -X display)
if ! docker exec "$C" bash -lc 'command -v xtigervncviewer >/dev/null'; then
  echo "ERROR: xtigervncviewer not in the container. Install once:"
  echo "  docker exec $C apt-get install -y tigervnc-viewer"
  exit 1
fi

# forward this ssh -X session's X cookie into the container so the viewer can draw on YOUR screen
XA="$(mktemp)"
xauth nlist "$DISPLAY" 2>/dev/null | sed 's/^..../ffff/' | xauth -f "$XA" nmerge - 2>/dev/null
docker cp "$XA" "$C:/tmp/.rviz.xauth" >/dev/null; rm -f "$XA"

# default rviz args: load the repo's rviz.rviz (repo root -> /work in the container) unless
# the caller passes their own args (e.g. a different `-d other.rviz`).
RVIZ_ARGS=("$@")
if [ ${#RVIZ_ARGS[@]} -eq 0 ]; then
  if [ -f "$ROOT/rviz.rviz" ]; then RVIZ_ARGS=(-d /work/rviz.rviz)
  else echo ">> note: rviz.rviz not found at repo root — launching rviz with its default config"; fi
fi

# (1) render rviz on a virtual display + serve it over VNC, inside the container (idempotent)
docker exec "$C" bash -c '
  pgrep -x Xvfb   >/dev/null 2>&1 || { nohup Xvfb :99 -screen 0 1600x900x24 >/tmp/xvfb.log 2>&1 & sleep 2; }
  pgrep -x x11vnc >/dev/null 2>&1 || { nohup x11vnc -display :99 -localhost -nopw -forever -shared -rfbport 5900 >/tmp/x11vnc.log 2>&1 & sleep 2; }
  pgrep -x rviz   >/dev/null 2>&1 || {
    source /opt/ros/noetic/setup.bash
    [ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
    [ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh
    DISPLAY=:99 __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
      nohup rviz "$@" >/tmp/rviz.log 2>&1 & sleep 4
  }
' -- "${RVIZ_ARGS[@]}"

# (2) show it: 2D vnc viewer on YOUR ssh -X display -> forwards to your machine. Ctrl-C to close.
echo ">> opening rviz on your screen via ssh -X (close this window or Ctrl-C to stop viewing; rviz keeps running)"
exec docker exec -it -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.rviz.xauth "$C" \
  xtigervncviewer -PreferredEncoding=tight -RemoteResize=0 localhost:5900

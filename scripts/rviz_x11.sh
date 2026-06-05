#!/bin/bash
# Show the dsd container's rviz on YOUR machine via SSH X11 forwarding (Linux clients).
#
#   1) from your machine:   ssh -X hmcl@192.168.50.36
#   2) on the jetson:        cd ~/drone-stack && ./scripts/rviz_x11.sh [stack] [-- rviz args]
#
# (X11 forwarding sends GL to your local X server. If rviz fails with a GLX/OpenGL
#  error, either run your local X with +iglx, or use VNC instead — see scripts/rviz_vnc note.)
set -e
STACK="d435i-voxblox"
if [ -n "${1:-}" ] && [ -f "$(dirname "$0")/../stacks/$1.yml" ]; then STACK="$1"; shift; fi
C="drone-stack-${STACK}"

[ -n "${DISPLAY:-}" ] || { echo "ERROR: \$DISPLAY is empty — connect with 'ssh -X hmcl@<jetson>' (Linux + X server)."; exit 1; }
docker ps --format '{{.Names}}' | grep -qx "$C" || { echo "ERROR: container $C not running (./up.sh up $STACK)"; exit 1; }

# build an xauth file with a wildcard hostname so the cookie works inside the container,
# then copy it in (docker exec can't add -v mounts to a running container).
XA="$(mktemp)"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$XA" nmerge - 2>/dev/null
docker cp "$XA" "$C:/tmp/.dsd.xauth" >/dev/null
rm -f "$XA"

echo ">> rviz in $C on your DISPLAY=$DISPLAY (via SSH X11). Ctrl-C to close."
exec docker exec -it -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/.dsd.xauth "$C" bash -lc "
  source /opt/ros/noetic/setup.bash
  [ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
  [ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh
  export LIBGL_ALWAYS_INDIRECT=1   # render on YOUR machine, not the container's Jetson GPU
  exec rviz $*"

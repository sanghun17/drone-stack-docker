#!/bin/bash
# Shared headless-GUI-over-VNC launcher behind the scripts/utility_<gui>.sh wrappers (rviz, rqt, ...).
# Each GUI runs on its OWN virtual X display + VNC port, so they open as SEPARATE windows on your
# screen. The app renders inside the container on Xvfb with mesa software GL (the L4T nvidia GL
# can't direct-render over ssh -X), x11vnc serves that display, and a 2D tigervnc viewer — which
# forwards fine over ssh -X — shows it on YOUR machine.
#
#   _vnc_gui.sh <display-num> <vnc-port> <app-name> <app-cmd> [app-args...]
#   e.g.  _vnc_gui.sh 99 5900 rviz rviz -d /work/rviz.rviz
#         _vnc_gui.sh 98 5901 rqt  rqt
#
# Idempotency is decided by ACTUAL liveness — xdpyinfo (display up?), a socket connect (vnc port
# up?), and xwininfo (app window present?) — NOT by pgrep. The container's pid1 is `sleep infinity`
# which never reaps children, so dead Xvfb/x11vnc/app linger as zombies that would fool pgrep.
# Re-running is safe: it attaches to whatever is already up and only (re)starts what's actually dead.
set -e
C=drone-stack-d435i-voxblox
DN="$1"; P="$2"; APPNAME="$3"; shift 3      # remaining "$@" = the app command + its args
W=$(( 1000 / ${VNC_FPS:-2} ))               # x11vnc poll/defer (ms). default 2 Hz; tune e.g. VNC_FPS=10 bash scripts/utility_rviz.sh

[ -n "${DISPLAY:-}" ] || { echo "ERROR: \$DISPLAY is empty. Reconnect with:  ssh -X hmcl@192.168.50.36"; exit 1; }
docker start "$C" >/dev/null 2>&1

# viewer + app must exist in the container (baked by utility/gui-vnc + utility/<app>). source ROS
# first so the app (in /opt/ros/noetic/bin) is on PATH — a plain login shell doesn't auto-source it.
if ! docker exec "$C" bash -lc "command -v xtigervncviewer >/dev/null && { source /opt/ros/noetic/setup.bash 2>/dev/null; command -v $1 >/dev/null; }"; then
  echo "ERROR: '$1' or the vnc viewer is missing in the container."
  echo "  bake it into the image:  ./setup.sh build d435i-voxblox   (utility/gui-vnc + utility/$APPNAME)"
  exit 1
fi

# forward this ssh -X session's X cookie into the container so the viewer can draw on YOUR screen
XA="$(mktemp)"
xauth nlist "$DISPLAY" 2>/dev/null | sed 's/^..../ffff/' | xauth -f "$XA" nmerge - 2>/dev/null
docker cp "$XA" "$C:/tmp/.${APPNAME}.xauth" >/dev/null; rm -f "$XA"

# (1) ensure Xvfb:$DN + x11vnc:$P + the app are up on THIS display (liveness-checked, not pgrep)
docker exec -i "$C" env DN="$DN" P="$P" PROC="$APPNAME" W="$W" bash -s -- "$@" <<'EOSH'
D=":$DN"
# Xvfb on its own display
if ! xdpyinfo -display "$D" >/dev/null 2>&1; then
  rm -f "/tmp/.X${DN}-lock" "/tmp/.X11-unix/X${DN}" 2>/dev/null
  nohup Xvfb "$D" -screen 0 1600x900x24 >"/tmp/xvfb${DN}.log" 2>&1 &
  for i in $(seq 1 20); do xdpyinfo -display "$D" >/dev/null 2>&1 && break; sleep 0.5; done
fi
# window manager — gives the app a title bar/border so you can move/resize/maximize it. Without one
# the window is fixed & undecorated with bare-black desktop around it. One openbox per display.
if ! xprop -display "$D" -root _NET_SUPPORTING_WM_CHECK 2>/dev/null | grep -q "window id"; then
  DISPLAY="$D" nohup openbox >"/tmp/openbox${DN}.log" 2>&1 & sleep 1
fi
# x11vnc on its own port. -wait/-defer = poll/defer interval (ms); higher = lower fps, less bandwidth.
if ! python3 -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',$P));s.close()" 2>/dev/null; then
  nohup x11vnc -display "$D" -localhost -nopw -forever -shared -rfbport "$P" -wait "$W" -defer "$W" >"/tmp/x11vnc${P}.log" 2>&1 &
  for i in $(seq 1 10); do python3 -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',$P));s.close()" 2>/dev/null && break; sleep 0.5; done
fi
# the app, on this display — confirm by a real window (not pgrep, which zombies fool)
if ! xwininfo -display "$D" -root -tree 2>/dev/null | grep -qi "$PROC"; then
  source /opt/ros/noetic/setup.bash
  [ -f /work/ws/risk-aware/devel/setup.bash ] && source /work/ws/risk-aware/devel/setup.bash
  [ -f /work/config/ros_env.sh ] && source /work/config/ros_env.sh
  export XDG_RUNTIME_DIR=/tmp/runtime-root; mkdir -p "$XDG_RUNTIME_DIR"; chmod 700 "$XDG_RUNTIME_DIR"
  DISPLAY="$D" __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe \
    nohup "$@" >"/tmp/${PROC}.log" 2>&1 &
  for i in $(seq 1 20); do xwininfo -display "$D" -root -tree 2>/dev/null | grep -qi "$PROC" && break; sleep 1; done
fi
EOSH

# (2) show it: 2D vnc viewer on YOUR ssh -X display -> forwards to your machine. Ctrl-C to close.
echo ">> opening $APPNAME on your screen via ssh -X (Ctrl-C to stop viewing; $APPNAME keeps running)"
exec docker exec -it -e DISPLAY="$DISPLAY" -e XAUTHORITY="/tmp/.${APPNAME}.xauth" "$C" \
  xtigervncviewer -PreferredEncoding=tight -RemoteResize=0 "localhost:$P"

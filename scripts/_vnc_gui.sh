#!/bin/bash
# Shared headless-GUI-over-VNC launcher behind the scripts/utility_<gui>.sh wrappers (rviz, rqt, ...).
# Each GUI runs on its OWN virtual X display + VNC port + web port, so they're fully independent. The
# app renders inside the container on Xvfb with mesa software GL (the L4T nvidia GL can't direct-render
# a headless display), x11vnc turns that display into a VNC stream, and websockify serves noVNC so you
# view it in ANY BROWSER with zero install — no VNC client, no ssh -X. Just `ssh` in, run this, open URL.
#
#   _vnc_gui.sh <display-num> <vnc-port> <web-port> <app-name> <app-cmd> [app-args...]
#   e.g.  _vnc_gui.sh 99 5900 6080 rviz rviz -d /work/rviz.rviz
#         _vnc_gui.sh 98 5901 6081 rqt  rqt
#
# Idempotency is decided by ACTUAL liveness — xdpyinfo (display up?), socket connects (vnc/web ports
# up?), and xwininfo (app window present?) — NOT by pgrep. The container's pid1 is `sleep infinity`
# which never reaps children, so dead Xvfb/x11vnc/app linger as zombies that would fool pgrep.
# Re-running is safe: it attaches to whatever is already up and only (re)starts what's actually dead.
set -e
C=drone-stack-d435i-voxblox
DN="$1"; P="$2"; WP="$3"; APPNAME="$4"; shift 4   # remaining "$@" = the app command + its args
# x11vnc poll/defer (ms) = 1000/VNC_FPS. Default 10 Hz (100ms) — safe for the flaky RTL8822CE wifi so
# heavy rviz motion can't saturate the link and stall ssh/the browser. Raise for snappier interaction
# at more bandwidth (best on a solid/wired link): VNC_FPS=20 -> 50ms, VNC_FPS=30 -> 33ms.
W=$(( 1000 / ${VNC_FPS:-10} ))

docker start "$C" >/dev/null 2>&1

# app + the noVNC infra must exist in the container (baked by utility/gui-vnc + utility/<app>). source ROS
# first so the app (in /opt/ros/noetic/bin) is on PATH — a plain login shell doesn't auto-source it.
if ! docker exec "$C" bash -lc "command -v websockify >/dev/null && [ -f /opt/novnc/vnc.html ] && { source /opt/ros/noetic/setup.bash 2>/dev/null; command -v $1 >/dev/null; }"; then
  echo "ERROR: '$1', websockify, or the noVNC files are missing in the container."
  echo "  bake them into the image:  ./setup.sh build d435i-voxblox   (utility/gui-vnc + utility/$APPNAME)"
  exit 1
fi

# (1) ensure Xvfb:$DN + x11vnc:$P + websockify:$WP + the app are up (liveness-checked, not pgrep)
docker exec -i "$C" env DN="$DN" P="$P" WP="$WP" PROC="$APPNAME" W="$W" bash -s -- "$@" <<'EOSH'
D=":$DN"
# Xvfb on its own display
if ! xdpyinfo -display "$D" >/dev/null 2>&1; then
  rm -f "/tmp/.X${DN}-lock" "/tmp/.X11-unix/X${DN}" 2>/dev/null
  nohup Xvfb "$D" -screen 0 1600x900x24 >"/tmp/xvfb${DN}.log" 2>&1 &
  for i in $(seq 1 20); do xdpyinfo -display "$D" >/dev/null 2>&1 && break; sleep 0.5; done
fi
# window manager — gives the app a title bar/border so you can move/resize/maximize it. One openbox/display.
if ! xprop -display "$D" -root _NET_SUPPORTING_WM_CHECK 2>/dev/null | grep -q "window id"; then
  DISPLAY="$D" nohup openbox >"/tmp/openbox${DN}.log" 2>&1 & sleep 1
fi
# x11vnc on its own port (localhost-only; websockify is the LAN-facing door). -wait/-defer = poll/defer ms.
if ! python3 -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',$P));s.close()" 2>/dev/null; then
  nohup x11vnc -display "$D" -localhost -nopw -forever -shared -rfbport "$P" -wait "$W" -defer "$W" >"/tmp/x11vnc${P}.log" 2>&1 &
  for i in $(seq 1 10); do python3 -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',$P));s.close()" 2>/dev/null && break; sleep 0.5; done
fi
# websockify: serve noVNC + bridge the web port -> the vnc port, so a plain BROWSER can view (zero install).
# 0.0.0.0 = reachable on the LAN. NOTE: no auth (matches x11vnc -nopw); lock down with a vnc password or a
# websockify --auth-plugin/token if this host is on an untrusted network.
if ! python3 -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',$WP));s.close()" 2>/dev/null; then
  nohup websockify --web=/opt/novnc "0.0.0.0:$WP" "localhost:$P" >"/tmp/websockify${WP}.log" 2>&1 &
  for i in $(seq 1 10); do python3 -c "import socket;s=socket.socket();s.settimeout(1);s.connect(('127.0.0.1',$WP));s.close()" 2>/dev/null && break; sleep 0.5; done
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

# (2) point a browser at it. websockify listens on the LAN, so open this on any machine that can reach
# this host — no client install, no ssh -X. Closing the tab does NOT stop the app; re-run to reattach.
IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{print $7; exit}'); [ -n "$IP" ] || IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo ">> $APPNAME is up (headless). Open it in any BROWSER — zero install:"
echo ">>     http://${IP:-<this-host-ip>}:$WP/vnc.html?resize=scale&quality=0&compression=9   (then click Connect)"
echo ">> tips: quality/compression sliders live in the noVNC settings menu; rate = ${W}ms defer (~$(( 1000 / W )) Hz, wifi-safe)."
echo ">>       on a solid/wired link, run with VNC_FPS=20 (or 30) for snappier. $APPNAME keeps running after you close the tab."

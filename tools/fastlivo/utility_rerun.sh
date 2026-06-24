#!/bin/bash
# Rerun viewer for the fast-livo open-loop eval, exposed as a zero-install BROWSER view —
# same philosophy as scripts/_vnc_gui.sh (LAN-facing 0.0.0.0, open the printed http://<ip>:<port>
# URL in any browser, no client install, survives closing the tab) but WITHOUT the Xvfb/x11vnc/
# noVNC chain: Rerun ships its own web viewer. The .rrd is served from the py3.11 'evo-rerun'
# container (rerun 0.33 — the ROS container is python3.8 and can't run evo>=1.35's rerun bridge),
# and the heavy 3D rendering happens in YOUR browser (WASM) — not on the Jetson's software GL,
# so it's lighter than the rviz/realsense-viewer path, not heavier.
#
#   tools/fastlivo/utility_rerun.sh [RRD]      (default: rerun_eval/eval.rrd)
#
# Rebuild the .rrd from a replayed bag:  python3 tools/fastlivo/make_rrd_rerun.py  (in the evo-rerun container)
# Stop the server:                       tools/fastlivo/utility_rerun.sh --stop
set -e
C=evo-rerun
WEB_PORT="${RERUN_WEB_PORT:-9090}"
GRPC_PORT="${RERUN_GRPC_PORT:-9876}"

if ! docker ps -a --format '{{.Names}}' | grep -qx "$C"; then
  echo "container '$C' not found — it's created by the fast-livo rerun eval setup (py3.11 + evo + rerun-sdk)."
  exit 1
fi
docker start "$C" >/dev/null 2>&1

# stop any running rerun server (the slim image has no procps -> scan /proc, kill by cmdline)
# SIGKILL the old server (SIGTERM was too slow -> it held :9090 and the new bind failed), excluding
# our own shell, then wait for the ports to actually release before the next bind.
stop_rerun(){ docker exec "$C" bash -lc '
  me=$$
  for p in /proc/[0-9]*; do pid=${p##*/}; [ "$pid" = "$me" ] && continue
    c=$(tr "\0" " " < "$p/cmdline" 2>/dev/null) || continue
    case "$c" in *serve-web*) kill -9 "$pid" 2>/dev/null;; esac; done
  for i in $(seq 1 10); do ss -tln 2>/dev/null | grep -qE ":9090|:9876" || break; sleep 0.5; done' 2>/dev/null || true; }

if [ "$1" = "--stop" ]; then stop_rerun; echo ">> rerun server stopped."; exit 0; fi
stop_rerun

# accept one OR several .rrd (multiple => they show as separate SOURCES/recordings in the viewer)
[ $# -eq 0 ] && set -- /work/tools/fastlivo/rerun_eval/eval.rrd
RRDS=""
for r in "$@"; do
  case "$r" in /*) ;; *) r="/work/tools/fastlivo/$r";; esac
  docker exec "$C" test -f "$r" || { echo "no such .rrd in container: $r"; exit 1; }
  RRDS="$RRDS '$r'"
done

# serve LAN-facing; --cors-allow-origin '*' lets the browser (a different origin) pull the gRPC stream.
docker exec -d "$C" bash -lc "cd /work/tools/fastlivo/rerun_eval && \
  exec rerun --serve-web --bind 0.0.0.0 --web-viewer-port $WEB_PORT --port $GRPC_PORT \
             --cors-allow-origin '*' $RRDS >/tmp/rerun_serve.log 2>&1"

for i in $(seq 1 20); do
  ss -tlnp 2>/dev/null | grep -q ":$WEB_PORT" && ss -tlnp 2>/dev/null | grep -q ":$GRPC_PORT" && break; sleep 0.5
done

# rerun advertises 'localhost' in its viewer URL; substitute the LAN IP for BOTH the web host and
# the embedded gRPC proxy so a browser on another machine connects back to the Jetson, not itself.
IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{print $7; exit}'); [ -n "$IP" ] || IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo ">> rerun viewer is up (headless, served from '$C'). Open in any BROWSER — zero install:"
echo ">>     http://${IP:-<this-host-ip>}:$WEB_PORT?url=rerun%2Bhttp%3A%2F%2F${IP}%3A$GRPC_PORT%2Fproxy"
echo ">> serving:$(echo "$RRDS" | sed "s|/work/||g")   | rendering runs in your browser | close the tab anytime, re-run to reattach"
echo ">> stop:    tools/fastlivo/utility_rerun.sh --stop"

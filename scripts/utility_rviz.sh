#!/bin/bash
# Headless rviz you view in a BROWSER (zero install) — renders on Xvfb + software GL, served via
# x11vnc + noVNC/websockify. Run this ON THE JETSON HOST (plain `ssh` is fine; no ssh -X needed).
# rviz gets its OWN display :99 / VNC 5900 / web 6080 — a separate session from rqt. All the logic
# (render on Xvfb + mesa GL, serve via x11vnc, expose over noVNC) lives in scripts/_vnc_gui.sh.
ARGS=("$@"); [ ${#ARGS[@]} -eq 0 ] && ARGS=(-d /work/rviz.rviz)   # default: load the repo's rviz.rviz
# restart on re-run: kill the old rviz so _vnc_gui starts a fresh one (reloads the .rviz config).
# Separate docker exec -> pkill excludes its own pid; the VNC infra stays up so the browser reconnects.
docker exec drone-stack-d435i-voxblox pkill -f "rviz" 2>/dev/null || true; sleep 2
exec "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 99 5900 6080 rviz rviz "${ARGS[@]}"

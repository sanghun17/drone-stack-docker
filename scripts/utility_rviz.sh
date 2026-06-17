#!/bin/bash
# Headless rviz you view in a BROWSER (zero install) — renders on Xvfb + software GL, served via
# x11vnc + noVNC/websockify. Run this ON THE JETSON HOST (plain `ssh` is fine; no ssh -X needed).
# All utility GUIs SHARE this display :99 / VNC 5900 / web 6080 (rqt, realsense-viewer, diagnostic
# point here too) so they show up in ONE browser tab. openbox hosts each as its own window — scroll the empty
# desktop to switch workspace, or right-click the title bar -> Send To Desktop to split them. Each
# app stays an independent process: this re-run restarts ONLY rviz; the shared display + other apps
# stay up. The infra logic (Xvfb + mesa GL, x11vnc, noVNC) lives in scripts/_vnc_gui.sh.
ARGS=("$@"); [ ${#ARGS[@]} -eq 0 ] && ARGS=(-d /work/rviz.rviz)   # default: load the repo's rviz.rviz
# restart on re-run: kill the old rviz so _vnc_gui starts a fresh one (reloads the .rviz config).
# Separate docker exec -> pkill excludes its own pid; the VNC infra stays up so the browser reconnects.
docker exec drone-stack-d435i-voxblox pkill -f "rviz" 2>/dev/null || true; sleep 1
exec "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 99 5900 6080 rviz rviz "${ARGS[@]}"

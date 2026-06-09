#!/bin/bash
# Remote rviz on YOUR screen via ssh -X (headless render + VNC). Run this ON THE JETSON HOST.
# rviz gets its OWN display :99 / VNC port 5900 — a separate window from rqt. All the logic
# (render on Xvfb + mesa GL, serve via x11vnc, view over ssh -X) lives in scripts/_vnc_gui.sh.
ARGS=("$@"); [ ${#ARGS[@]} -eq 0 ] && ARGS=(-d /work/rviz.rviz)   # default: load the repo's rviz.rviz
exec "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 99 5900 rviz rviz "${ARGS[@]}"

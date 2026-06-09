#!/bin/bash
# Headless rqt you view in a BROWSER (zero install) — renders on Xvfb + software GL, served via
# x11vnc + noVNC/websockify. Run this ON THE JETSON HOST (plain `ssh` is fine; no ssh -X needed).
# rqt gets its OWN display :98 / VNC 5901 / web 6081 — a separate session from rviz. All the logic
# (render on Xvfb + mesa GL, serve via x11vnc, expose over noVNC) lives in scripts/_vnc_gui.sh.
exec "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 98 5901 6081 rqt rqt "$@"

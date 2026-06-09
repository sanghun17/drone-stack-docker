#!/bin/bash
# Remote rqt on YOUR screen via ssh -X (headless render + VNC). Run this ON THE JETSON HOST.
# rqt gets its OWN display :98 / VNC port 5901 — a separate window from rviz. All the logic
# (render on Xvfb + mesa GL, serve via x11vnc, view over ssh -X) lives in scripts/_vnc_gui.sh.
exec "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 98 5901 rqt rqt "$@"

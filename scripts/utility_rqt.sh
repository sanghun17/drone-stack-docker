#!/bin/bash
# Headless rqt you view in a BROWSER (zero install) — renders on Xvfb + software GL, served via
# x11vnc + noVNC/websockify. Run this ON THE JETSON HOST (plain `ssh` is fine; no ssh -X needed).
# rqt shares the GUI desktop :99 / VNC 5900 / web 6080 with rviz/realsense-viewer/diagnostic -> ONE tab.
# Liveness must single out a REAL rqt main window: plain rqt and the diagnostic's rqt_runtime_monitor
# share WM_CLASS "rqt" AND Qt spawns aux windows ("Qt Selection Owner for rqt"). So MATCH the class
# signature '"rqt" "rqt"' (real main windows only) and MATCH_NOT the monitor — else a plain rqt is
# judged "already up" when the monitor is open and never starts. (Infra logic in scripts/_vnc_gui.sh.)
exec env MATCH='"rqt" "rqt"' MATCH_NOT="rqt_runtime_monitor" "$(dirname "$(readlink -f "$0")")/_vnc_gui.sh" 99 5900 6080 rqt rqt "$@"

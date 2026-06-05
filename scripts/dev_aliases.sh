# drone-stack dev convenience functions. Add to your ~/.bashrc with:
#     source ~/drone-stack/scripts/dev_aliases.sh
# Provides: dsd (enter container), dsdrun <module> (run a module), dsdrviz (rviz).

DSD_CTR="${DSD_CTR:-drone-stack-d435i-voxblox}"

# give the container X11 access + the current display's xauth cookie
#   (works for a local display :0 AND for `ssh -X` forwarding)
_dsd_x11() {
  xhost +local:root >/dev/null 2>&1
  if [ -n "$DISPLAY" ] && command -v xauth >/dev/null 2>&1; then
    local xa; xa=$(mktemp)
    xauth nlist "$DISPLAY" 2>/dev/null | sed 's/^..../ffff/' | xauth -f "$xa" nmerge - 2>/dev/null
    docker cp "$xa" "$DSD_CTR:/tmp/.dsd.xauth" >/dev/null 2>&1
    rm -f "$xa"
  fi
}

# dsd : enter the container (ROS env + rviz/X11 ready)
dsd() {
  docker start "$DSD_CTR" >/dev/null 2>&1
  _dsd_x11
  docker exec -it -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/tmp/.dsd.xauth "$DSD_CTR" bash
}

# dsdrun <module[/script]> : run a module's run script in the container
#   dsdrun sensor/realsense-d435i            -> .../run.sh
#   dsdrun planner/risk-aware/run_voxblox.sh
dsdrun() {
  docker start "$DSD_CTR" >/dev/null 2>&1
  _dsd_x11
  local p="$1"; [[ "$p" == *.sh ]] || p="$p/run.sh"
  docker exec -it -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/tmp/.dsd.xauth "$DSD_CTR" bash -lc "bash /work/modules/$p"
}

# dsdrviz : rviz inside the container, shown on your display (after `ssh -X`)
dsdrviz() {
  _dsd_x11
  docker exec -it -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/tmp/.dsd.xauth "$DSD_CTR" bash -lc \
    "source /opt/ros/noetic/setup.bash; source /work/ws/risk-aware/devel/setup.bash; export ROS_MASTER_URI=http://localhost:11399; exec rviz"
}

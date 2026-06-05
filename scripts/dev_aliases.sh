# drone-stack dev convenience. Add to ~/.bashrc with:
#     source ~/drone-stack/scripts/dev_aliases.sh
# (or just paste the dsd function below)

# dsd : enter the drone-stack container (ROS env + rviz/X11 ready).
#   inside, run nodes with the module scripts, e.g.:
#     bash /work/modules/sensor/realsense-d435i/run.sh
#     bash /work/modules/planner/risk-aware/run_planner.sh
#   or from the host:  ./up.sh run d435i-voxblox <module>
dsd() {
  docker start drone-stack-d435i-voxblox >/dev/null 2>&1
  xhost +local:root >/dev/null 2>&1
  docker exec -it -e DISPLAY="${DISPLAY:-:0}" drone-stack-d435i-voxblox bash
}

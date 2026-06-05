#!/bin/bash
# drone-stack orchestrator. Native build per host arch (arm64 Jetson / amd64 x86).
#
#   ./up.sh clone    <stack>              # git-clone each module's source into ws/<module>/src
#   ./up.sh gen      <stack>              # generate .build/<stack>/{Dockerfile,compose.yml}
#   ./up.sh build    <stack>              # gen + docker build the single image
#   ./up.sh up       <stack>              # gen + build + start the single container (idle)
#   ./up.sh build-ws <stack>             # catkin-build each module's workspace in the container
#   ./up.sh run      <stack> <module>     # exec a module's run script inside the container
#   ./up.sh sh       <stack>              # shell into the container
#   ./up.sh down     <stack>              # stop/remove the container
#   ./up.sh ls       <stack>              # list the stack's modules + their run scripts
#
#   typical first run:  clone -> up -> build-ws -> run <camera> / <fastlivo> / <planner>
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
[ -f "$ROOT/config/stack.env" ] && source "$ROOT/config/stack.env"
PORT="${ROS_MASTER_PORT:-11399}"

ARCH=$(uname -m); case "$ARCH" in aarch64) ARCH=arm64;; x86_64) ARCH=amd64;; esac

cmd="${1:-help}"; stack="${2:-}"
need_stack(){ [ -n "$stack" ] || { echo "need <stack> (see stacks/)"; exit 1; }; }
gen(){ python3 "$ROOT/scripts/gen.py" "$stack" --arch "$ARCH"; }
DF(){ echo "$ROOT/.build/$stack/Dockerfile"; }
CF(){ echo "$ROOT/.build/$stack/compose.yml"; }

case "$cmd" in
  gen)   need_stack; gen ;;
  build) need_stack; gen
         echo ">> docker build (native $ARCH) -> drone-stack:$stack"
         docker build -f "$(DF)" -t "drone-stack:$stack" "$ROOT" ;;
  up)    need_stack; gen
         docker build -f "$(DF)" -t "drone-stack:$stack" "$ROOT"
         docker compose -f "$(CF)" up -d
         echo ">> container drone-stack-$stack up. start nodes: ./up.sh run $stack <module>" ;;
  run)   need_stack; mod="${3:?need <module> e.g. sensor/realsense-d435i}"
         [ -f "$ROOT/modules/$mod" ] || mod="$mod/run.sh"   # allow dir or explicit script
         docker exec -it "drone-stack-$stack" bash -lc \
           "source /opt/ros/noetic/setup.bash; export ROS_MASTER_URI=http://localhost:$PORT; bash /work/modules/$mod" ;;
  sh)    need_stack; docker exec -it "drone-stack-$stack" bash ;;
  down)  need_stack; docker compose -f "$(CF)" down ;;
  clone) need_stack; gen >/dev/null
         while read -r m; do
           [ -f "$ROOT/modules/$m/clone.sh" ] && { echo ">> clone: $m"; bash "$ROOT/modules/$m/clone.sh"; }
         done < "$ROOT/.build/$stack/modules.txt" ;;
  build-ws) need_stack; gen >/dev/null     # catkin-build each module's workspace in the container
         while read -r m; do
           [ -f "$ROOT/modules/$m/build_ws.sh" ] && { echo ">> build-ws: $m"; \
             docker exec "drone-stack-$stack" bash -lc "bash /work/modules/$m/build_ws.sh"; }
         done < "$ROOT/.build/$stack/modules.txt" ;;
  ls)    need_stack; gen >/dev/null; sed -n 's/^#   //p' "$(CF)" ;;
  *) sed -n '2,12p' "$0" ;;
esac

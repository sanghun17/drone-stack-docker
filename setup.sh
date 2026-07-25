#!/bin/bash
# drone-stack orchestrator. Native build per host arch (arm64 Jetson / amd64 x86).
#
#   ./setup.sh clone    <stack>              # git-clone each module's source into ws/<module>/src
#   ./setup.sh gen      <stack>              # generate .build/<stack>/{Dockerfile,compose.yml}
#   ./setup.sh build    <stack>              # gen + docker build the single image
#   ./setup.sh up       <stack>              # gen + build + start the single container (idle)
#   ./setup.sh build-ws <stack>             # catkin-build each module's workspace in the container
#   ./setup.sh run      <stack> <module>     # exec a module's run script inside the container
#   ./setup.sh sh       <stack>              # shell into the container
#   ./setup.sh down     <stack>              # stop/remove the container
#   ./setup.sh ls       <stack>              # list the stack's modules + their run scripts
#
#   typical first run:  clone -> up -> build-ws -> run <camera> / <fastlivo> / <planner>
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
# set -a: stack.env 값을 export — clone.sh 등 자식 프로세스(bash modules/*/clone.sh)가
# RISK_AWARE_BRANCH 등을 실제로 받게 한다. (export 없이는 자식이 못 봐 clone.sh 기본값이
# 조용히 이겨 왔음 — 기본값==stack.env 값이라 가려져 있던 잠재버그, 2026-07-25 수리)
[ -f "$ROOT/config/stack.env" ] && { set -a; source "$ROOT/config/stack.env"; set +a; }
PORT="${ROS_MASTER_PORT:-11311}"

ARCH=$(uname -m); case "$ARCH" in aarch64) ARCH=arm64;; x86_64) ARCH=amd64;; esac

# Extra `docker build` options. Symmetric with build_wheel.sh's DOCKER_RUN_OPTS —
# needed on a host whose docker daemon has no bridge network (e.g. im's dedicated
# DOCKER_HOST=unix:///tmp/docker-ssd.sock, `--bridge=none --iptables=false`): without
# a network, the generated Dockerfile's RUN steps (apt/pip/git) can't reach anything,
# so that host must build with DOCKER_BUILD_OPTS="--network=host" (2026-07-25,
# ete-train-4090). Empty by default = byte-identical to the pre-existing `docker
# build` invocation below.
: "${DOCKER_BUILD_OPTS:=}"

cmd="${1:-help}"; stack="${2:-}"
need_stack(){ [ -n "$stack" ] || { echo "need <stack> (see stacks/)"; exit 1; }; }
# the generated Dockerfile uses BuildKit `RUN --mount` (modules/ is bind-mounted at
# build time, never COPY-ed in) — so buildx is required for build/up.
need_buildx(){ docker buildx version >/dev/null 2>&1 || {
  echo "ERROR: 'docker buildx' missing — the build needs BuildKit (RUN --mount)."
  echo "  install the buildx CLI plugin -> ~/.docker/cli-plugins/docker-buildx"
  echo "  (releases: https://github.com/docker/buildx/releases , arm64 asset: buildx-*.linux-arm64)"
  exit 1; }; }
gen(){ python3 "$ROOT/tools/gen_dockerfile_compose.py" "$stack" --arch "$ARCH"; }
DF(){ echo "$ROOT/.build/$stack/Dockerfile"; }
CF(){ echo "$ROOT/.build/$stack/compose.yml"; }

case "$cmd" in
  gen)   need_stack; gen ;;
  build) need_stack; need_buildx; gen
         echo ">> docker build (native $ARCH) -> drone-stack:$stack"
         docker build $DOCKER_BUILD_OPTS -f "$(DF)" -t "drone-stack:$stack" "$ROOT" ;;
  up)    need_stack; need_buildx; gen
         docker build $DOCKER_BUILD_OPTS -f "$(DF)" -t "drone-stack:$stack" "$ROOT"
         docker compose -f "$(CF)" up -d
         echo ">> container drone-stack-$stack up. start nodes: ./setup.sh run $stack <module>" ;;
  run)   need_stack; mod="${3:?need <module> e.g. sensor/realsense-d435i}"
         [ -f "$ROOT/modules/$mod" ] || mod="$mod/run.sh"   # allow dir or explicit script
         # `docker exec` does NOT inherit the caller's environment, so a module run
         # script that reads env vars (training/ete-net/run.sh needs ETE_CONFIG, and
         # halts via `:?` without it — no silent default, by project convention) would
         # always halt when invoked through here. Forward the ones the caller actually
         # exported; RUN_ENV adds arbitrary extra names, e.g.
         #   ETE_CONFIG=config/ablation/v23_P1_DEPLOY1.yaml ./setup.sh run ete-train-4090 training/ete-net
         #   RUN_ENV="MY_VAR OTHER" MY_VAR=1 ./setup.sh run <stack> <module>
         # (2026-07-26: found on im — the working command had to be a hand-written
         # `docker exec -e ...`, which is exactly what this script exists to replace.)
         envargs=()
         for v in ETE_CONFIG ETE_SEED ${RUN_ENV:-}; do
           [ -n "${!v+x}" ] && envargs+=(-e "$v")
         done
         docker exec -it ${envargs[@]+"${envargs[@]}"} "drone-stack-$stack" bash -lc \
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

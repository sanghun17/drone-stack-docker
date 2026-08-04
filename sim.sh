#!/bin/bash
# MARSIM closed-loop simulation. Sibling of view.sh / replay.sh.
#
#   ./sim.sh                   # map1 = garage.pcd (기본)
#   ./sim.sh map2              # map2 = poongsan_garage.pcd
#   ./sim.sh map1 rviz:=false
#   ./sim.sh --no-trigger
#
# view.sh / replay.sh 와 달리 bag 인자 재작성이 없다 — sim 은 bag 을 안 읽는다.
# 나머지(DISPLAY, xhost)는 동일하다: MARSIM 의 렌더러와 RViz 둘 다 하드웨어 GL 이
# 필요하고, 컨테이너는 root 로 도니 X 접근 권한을 잠깐 열어줘야 한다.
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${DISPLAY:-:0}"

XHOST_ADDED=0
if command -v xhost >/dev/null 2>&1 &&
   ! xhost 2>/dev/null | grep -Fqx "SI:localuser:root"; then
  if xhost +SI:localuser:root >/dev/null 2>&1; then
    XHOST_ADDED=1
  fi
fi

cleanup_xhost() {
  if [ "$XHOST_ADDED" -eq 1 ]; then
    xhost -SI:localuser:root >/dev/null 2>&1 || true
  fi
}
trap cleanup_xhost EXIT INT TERM HUP

"$ROOT/modules/planner/epic/run_sim.sh" "$@"

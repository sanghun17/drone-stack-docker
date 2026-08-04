#!/bin/bash
# planner/epic: MARSIM closed-loop simulation (sensor sim + dynamics + EPIC).
#
#   ./sim.sh                      # map1 = garage.pcd  (기본)
#   ./sim.sh map2                 # map2 = poongsan_garage.pcd
#   ./sim.sh map1 rviz:=false     # roslaunch 인자는 그대로 통과
#   ./sim.sh --no-trigger         # /srv_start 자동 호출 안 함 (수동으로 시작)
#   ./sim.sh map1 init_x_:=10 init_y_:=5
#
# 실비행(real_flight.launch)과 달리 여기선 MARSIM 이 라이다/동역학을 대신하고
# cascadePID 가 PX4 자리를 채운다. 나머지(exploration_node, traj_server, corridor,
# MINCO)는 실기와 같은 바이너리다 — 그래서 플래너 변경의 폐루프 검증에 쓴다.
#
# 이 스크립트가 대신 챙겨주는 것 (전부 실측으로 물린 적 있는 함정):
#
#  1) /use_sim_time 되돌리기 — 가장 악질. run_view.sh / run_replay.sh 의
#     `rosbag play --clock` 이 마스터에 use_sim_time=true 를 남기는데, roscore 는
#     스크립트들이 공유한다. 그 상태로 sim 을 띄우면 모든 노드가 영영 오지 않을
#     /clock 을 기다리며 얼어붙는다. 에러는 한 줄도 안 나고 노드 목록도 멀쩡해서,
#     보이는 증상이 "[CloudCrop] WATCHDOG ... NO INPUT AT ALL" 반복뿐이다.
#     (실측: 이 원인 찾는 데 두 번의 헛 실행을 태웠다.) 매번 false 로 되돌린다.
#
#  2) rosparam delete /exploration_node — roslaunch 는 자기가 올린 파라미터를
#     지우지 않는다. roscore 를 공유한 채 다른 config 로 다시 띄우면 이전 값이
#     조용히 섞여 들어온다. 조용히 틀리는 종류라 반드시 매 실행 전에 지운다.
#
#  3) 죽은 rviz 회수 — rosnode kill 은 rviz 를 마스터에서 등록해제만 하고
#     프로세스는 살려둔다. 남으면 다음 실행의 GL 컨텍스트를 갉아먹는다.
#
#  4) use_px4_bridge:=true — cascadePID 의 px4_like_cmd 로 이어져, /position_cmd
#     의 velocity/acceleration 을 버리고 위치+yaw 만 쓰게 한다. 실기 PX4 가
#     OFFBOARD 에서 쓰는 type_mask=2552 와 같은 조건이다. 이걸 끄면(=false) sim 이
#     실기엔 없는 피드포워드를 받아 추종이 실제보다 좋게 나온다 — 즉 traj_server
#     쪽 변경을 sim 으로 판단할 수 없게 된다.
#
#  5) 스폰 기본값 — map2 의 launch 기본 스폰 (3, 8, 1) 은 장애물 여유가 0.50 m 뿐이라
#     DilateRadiusSoft(0.6) 미만이다. 드론이 팽창 장애물 안에서 시작해 "corridor
#     fallback has no guide-path point inside connected prefix" 로 교착한다.
#     map2 는 여유 2.47 m 로 측정 확인된 (21.6, 11.9, 1.0) 을 쓴다. init_x_ 등을
#     직접 넘기면 그쪽이 이긴다.
#
#  6) record_results:=false + eval 출력 tmpfs — 기본이 true 라 실험 산출물이
#     repo 의 experiments/ 를 오염시킨다.
#
#  7) /srv_start 자동 호출 — FSM 이 WAIT_TRIGGER 에 올라온 뒤 한 번. rviz 의 2D Nav
#     Goal 을 찍지 않아도 탐사가 시작된다. --no-trigger 로 끌 수 있다.
#
# 결과 확인:
#   rostopic echo -n1 /quad0_cascadePID_node/px4_like_cmd  # (rosparam) true 여야 실기 등가
#   rostopic hz /planning/trajectory
#   rostopic echo /position_cmd/velocity                   # 코너에서 크기가 변해야 정상
__MATCH="roslaunch sim_bringup garage_map|rosservice call /srv_start"
source "$(dirname "${BASH_SOURCE[0]}")/_enter.sh"

# ── in-container from here on (_enter.sh 가 ROS + ws + 네트워킹 + master 를 세팅) ──

MAP=map1
TRIGGER=1
PASS=()
for a in "$@"; do
  case "$a" in
    map1|garage)           MAP=map1 ;;
    map2|poongsan|poongsan_garage) MAP=map2 ;;
    --no-trigger)          TRIGGER=0 ;;
    -h|--help)
      sed -n '2,40p' "$0" >&2
      exit 0 ;;
    *)                     PASS+=("$a") ;;
  esac
done

if [ "$MAP" = map1 ]; then
  LAUNCH=garage_map1_mlx.launch
  SPAWN=(init_x_:=5.0 init_y_:=0.0 init_z_:=2.0)   # launch 기본값과 동일 (여유 충분)
else
  LAUNCH=garage_map2_mlx.launch
  SPAWN=(init_x_:=21.6 init_y_:=11.9 init_z_:=1.0) # (5) 참조 — 기본 스폰은 교착한다
fi

# (1) 공유 roscore 에 남은 bag-replay 잔재 제거. 이게 남아 있으면 sim 이 통째로 언다.
if [ "$(rosparam get /use_sim_time 2>/dev/null)" = "true" ]; then
  echo "[epic-sim] /use_sim_time=true 발견 (bag replay 잔재) — false 로 되돌린다" >&2
  rosparam set /use_sim_time false
fi

# (2) 이전 실행이 남긴 파라미터 제거. 없으면 실패하므로 || true.
rosparam delete /exploration_node >/dev/null 2>&1 || true

# (3) 마스터에서 빠졌지만 살아있는 rviz 회수.
pkill -9 -x rviz >/dev/null 2>&1 || true

# (7) FSM 이 뜬 뒤 한 번만 트리거.
if [ "$TRIGGER" -eq 1 ]; then
  (
    for _ in $(seq 1 120); do
      sleep 1
      rosservice list 2>/dev/null | grep -qx /srv_start || continue
      sleep 2   # 서비스 등록과 FSM 이 WAIT_TRIGGER 에 드는 시점 사이의 간격
      rosservice call /srv_start "{}" >/dev/null 2>&1 \
        && echo "[epic-sim] /srv_start 호출됨 — 탐사 시작" \
        || echo "[epic-sim] /srv_start 실패 — 수동: rosservice call /srv_start \"{}\"" >&2
      exit 0
    done
    echo "[epic-sim] /srv_start 가 120s 안에 안 떴다 — 수동 트리거 필요" >&2
  ) &
fi

echo "[epic-sim] $LAUNCH  spawn=${SPAWN[*]}  use_px4_bridge=true"

# PASS 를 마지막에 둬서 사용자 인자가 기본값을 이긴다.
exec roslaunch sim_bringup "$LAUNCH" \
  use_px4_bridge:=true \
  "${SPAWN[@]}" \
  record_results:=false eval_output_dir:=/dev/shm/epic_sim_eval \
  "${PASS[@]}"

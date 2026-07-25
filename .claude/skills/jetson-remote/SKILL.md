---
name: jetson-remote
description: Work against the jetson (Orin AGX) from the ml PC — remote ROS debugging, file/log inspection over ssh, deploy + build loop. Use when the task involves jetson hardware, real-robot nodes, or d435i sensors.
user_invocable: true
---

# Jetson Remote Workflow (single-branch: code on ml PC, execution on jetson)

**철칙: 코드 수정·커밋은 ml PC에서만. jetson은 pull-only 실행 머신, 직접 편집 금지.**
jetson clone은 pull-only이며 (`scripts/setup_jetson_guardrails.sh`로 push 차단 +
pre-commit 훅 설치됨), jetson에서 파일을 직접 고치지 않는다. jetson 위 임시 수정이
불가피했다면 diff를 ml로 가져와
(`ssh jetson "cd ~/drone-stack-docker/ws/risk-aware/src/risk_aware_planning && git diff"`)
ml 트리에 적용 후 커밋한다.

코드 정본: `/home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/`
(별도 git repo, branch main). jetson 클론: 같은 상대경로,
`~/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/` (dsd `d435i-voxblox`
스택의 `planner/risk-aware` 모듈이 여기를 `/work/ws/risk-aware` 아래로 바인드마운트).

## Connectivity

- ssh alias `jetson` (`~/.ssh/config`), key auth. 모든 원격 작업은 비대화식
  `ssh jetson "<command>"` 형태로 실행한다.
- 접속 확인: `ssh -o ConnectTimeout=3 jetson "hostname && echo OK"`
- jetson이 꺼져 있으면 사용자에게 전원을 요청하라 (원격 부팅 수단 없음).

## ROS debugging — ssh 없이 네트워크로 직접

jetson에서 roscore가 돌고 있으면 ml PC에서 토픽/노드/서비스에 바로 접근:

```bash
export ROS_MASTER_URI=http://<jetson-ip>:11311 && export ROS_IP=192.168.50.12 && source /opt/ros/noetic/setup.bash
rostopic hz /camera/depth/image_raw
rosnode info /planner/planner_node
```

- `<jetson-ip>`는 `ssh jetson "hostname -I"`로 확인.
- 주의: 이 환경변수는 셸 명령마다 다시 export해야 한다 (Claude 셸은 상태 비유지).
- sim(ml 로컬) 작업과 섞이지 않게, jetson 디버깅이 끝나면 ROS_MASTER_URI를
  로컬(192.168.50.12)로 되돌린 상태로 명령을 구성할 것.

## Files / logs / processes on jetson

```bash
ssh jetson "tail -100 ~/.ros/log/latest/<node>.log"
ssh jetson "tmux capture-pane -t <session>:<pane> -p | tail -30"
ssh jetson "docker ps"                                          # 컨테이너 조회 (기대: drone-stack-d435i-voxblox)
ssh jetson "docker exec drone-stack-d435i-voxblox <command>"    # 컨테이너 내부 일회성 실행
```

컨테이너에 대화식으로 들어가야 하면 손으로 `docker exec -it`를 조립하지 말고 dsd
스크립트를 쓴다: `ssh jetson "cd ~/drone-stack-docker && ./setup.sh sh d435i-voxblox"`.

## Deploy + build loop (커밋 없이 빠른 반복)

1. ml PC에서 코드 수정 (Edit 도구)
2. 빌드:
   - 빠른 반복(패키지 지정, catkin config는 기존 것 재사용 — 이미 RelWithDebInfo로
     구성돼 있다는 전제): `/home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/scripts/deploy_to_jetson.sh --build <pkgs>`
     — 워킹트리를 rsync(.git 제외)하고 jetson 컨테이너(`drone-stack-d435i-voxblox`)
     안에서 `catkin build <pkgs>` 실행.
   - 처음 빌드하거나 catkin config가 의심스러우면(예: voxblox가 Debug/-O0로 잘못
     구성돼 heap이 깨지는 사고 재발 방지) dsd 스크립트로 전체 재확정:
     `ssh jetson "cd ~/drone-stack-docker && ./setup.sh build-ws d435i-voxblox"`
     — 이 스택의 모든 모듈(`compute/torch`, `compute/spconv`, `compute/jax`,
     `odometry/fast-livo`, `sensor/realsense-d435i`, `planner/risk-aware`,
     `control/mavros`, `control/flight-safety`, `control/local-controller`,
     `utility/rviz`, `utility/rqt`, `odometry/optitrack`)를 `-DCMAKE_BUILD_TYPE=RelWithDebInfo`로
     재확정하며 build_ws.sh가 있는 모듈만 순서대로 빌드한다. 손으로
     `docker exec ... catkin build`를 조립하지 말 것 — sourcing 순서/cmake 재확정이
     스크립트에 이미 들어 있다.
3. jetson에서 노드 재시작: `ssh jetson "cd ~/drone-stack-docker && ./setup.sh run d435i-voxblox <module>"`
   (예: `planner/risk-aware`, `odometry/fast-livo`, `sensor/realsense-d435i`) 또는
   기존 tmux 세션이 떠 있다면 위 ssh/tmux capture-pane 패턴으로 로그만 확인. 동작 확인.
4. 검증되면 ml에서 커밋 + `git push origin main`, jetson은
   `ssh jetson "cd ~/drone-stack-docker/ws/risk-aware/src/risk_aware_planning && git stash && git pull --ff-only origin main"`
   (rsync로 dirty해진 트리는 stash/checkout으로 정리 — 어차피 ml 사본이다)

## Guardrails 재적용

jetson clone을 새로 만들었거나 훅이 사라졌으면:
`/home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/scripts/setup_jetson_guardrails.sh`

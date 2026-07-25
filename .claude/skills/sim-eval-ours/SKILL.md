---
name: sim-eval-ours
description: Run and evaluate the OURS risk-aware planner (trained ete_net injected) in simulation — checkpoint deploy preflight, full bring-up, automation experiment loop, offline evaluation, verdict checklist. Usage - /sim-eval-ours [iterations] [name]
user_invocable: true
---

# OURS Planner Sim Experiment Loop (ete_net 주입 → 구동 → 평가)

학습된 ete_net checkpoint를 planner에 주입해 시뮬레이션에서 구동·평가하는 end-to-end 루프.
부품 스킬을 재사용한다: 인프라는 `/sim-start`, VIO는 `/sim-fast-livo`, 종료는 `/sim-stop-nodes`·`/sim-kill`.
코드 근거: `automation_experiment.py`(Phase A~E), `jax_mppi_params.py`, `jax_main_node_ros_new.py:539-728`, `eval_data_node.py`, `experiment_plotter.py`. (2026-07-05 코드 검증 기준)

## CRITICAL: 컨테이너 시대 — 노드는 `drone-stack-sim-x86`에서 산다

voxblox/exploration/JAX/SO3/센서 퍼블리셔/initialize_simulator/evaluator는 전부
`modules/planner/risk-aware-sim/run_*.sh` (+ FAST-LIVO는 `modules/odometry/fast-livo-sim/run.sh`)를
통해 `drone-stack-sim-x86` 컨테이너 안에서 뜬다 — tmux pane 하나당 `bash <script>` 블로킹 하나,
기존 패턴 그대로. roscore/UE4/airsim_node(conda `airsim` env)만 호스트(window 0)에 남는다.

아래의 모든 `rostopic`/`rosservice`/`rosnode`/`rosparam` 조회·호출은 (run_*.sh가 없는 ad-hoc
명령이므로) 손으로 `docker exec`를 통해 같은 컨테이너로 들어간다 — 구 트리 `~/risk-aware_planning/devel`
호스트 워크스페이스가 없어지는 데다, `/jax/*` 등 이 스택의 서비스 상당수가 커스텀 타입이라 호스트에서
resolve가 안 된다:
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && <command>'
```
⚠ **`use_sim_time` 환경에선 `rostopic hz`가 무용지물** (실측으로 확인된 함정) — 값이 0이거나
말도 안 되게 튀어도 토픽이 실제로 안 오는 것과 구분이 안 된다. 판정은 `rostopic echo`로
메시지가 실제로 도착하는지 확인하는 방식으로 하라.

## 0. Checkpoint 배포 preflight (필수 — 여기서 실수하면 크래시가 아니라 조용한 오동작)

checkpoint 주입 지점은 **config 키가 아니라 하드코딩**: `local_planner_mpc/jax_mppi_params.py:113-121`
(경로 문자열은 :119) (`$RISK_AWARE_CHECKPOINTS`, 컨테이너에서 `/home/ml/risk_aware_assets/checkpoints` —
`config/sim.env`가 export, 호스트와 동일 절대경로 마운트라 구 트리 경로가 아니다). 모델 hyperparam
(bins/nf/hidden 등)은 `.pth` 내장 config에서 자동 로드되므로 별도 yaml 동기화 불필요.

```bash
# 1. 현재 planner가 가리키는 checkpoint 확인 (그냥 파일 읽기 — ROS/컨테이너 불필요, 호스트 절대경로로 직접)
grep -n "checkpoint_path" /home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/jax_mppi_params.py
ls -la /home/ml/risk_aware_assets/checkpoints/
```

체크리스트 (새 checkpoint 배포 시):
1. `.pth`를 `$RISK_AWARE_CHECKPOINTS/<dir>/checkpoints/`(=`/home/ml/risk_aware_assets/checkpoints/<dir>/checkpoints/`)에
   복사, `jax_mppi_params.py` 경로 갱신.
2. **`kinetic_statistics.pt`가 그 checkpoint의 학습 정규화와 일치해야 함** — 학습 쪽
   `uncertainty_predictor/.../data/kinetic_statistics.pt`와 대조. 다르면 백업 후 교체.
   (`target_statistics.pt`는 배포에서 안 씀.)
3. **세대 확인**: 현 tree는 Gen-B 전용 — Gen-A checkpoint(`nf.layers.N.net.*` 키, 예: 구 epoch_680)는
   `load_state_dict` 실패. Gen-B(`param_net`/DirectFiLM)만 로드됨.
4. **target_scale / target_normalization 정합**: planner의 margin 계산은 물리 m/rad 기준.
   checkpoint가 per-(t,axis) target_scale로 학습됐으면 planner에 역스케일이 배선돼 있어야 한다
   (2026-07-05 패치로 `jax_main_node_ros_new.py`에 checkpoint config 기반 역스케일 존재 — 로드 로그에서
   `[ETE] target_scale` 라인 확인). 없으면 margin이 최대 ~17× 과대 → 드론이 과보수/정지 (에러 없음!).
5. `aux_mean_head.*` 키는 서브모듈 prefix 재조립에서 **조용히 무시**됨(무해) — aux mean은 현재 배포에서 미소비.
6. **로드 검증 (라이브 확인된 방법)** — `run_jax.sh` 기동 후, jax 노드는 컨테이너 안에서 돌므로
   그 노드 자신의 `~/.ros`(컨테이너 HOME=`/root`)를 봐야 한다:
   ```bash
   docker exec drone-stack-sim-x86 bash -lc 'grep -E "\[ETE\]|target_scale|ckpt info|Loading checkpoint" ~/.ros/log/latest/rosout.log' | head -5
   ```
   기대 라인 4개: ① `Loading checkpoint: <의도한 경로>` ② `ckpt info: epoch=…, target_norm=vio_gt_ratio, …`
   ③ `[ETE] predictions = relative-motion drift error, body(k) frame` ④ `target_scale loaded: shape=(7, 4), range=[0.840, 17.370]`
   (tmux pane 캡처는 tiled 상태에서 폭이 좁아 불안정 — rosout.log가 확실.)

## 1. 인프라 + config (라이브 검증됨 2026-07-05)

```
/sim-start                      # roscore, UE4, AirSim, sensors, initialize_simulator (자체 검증 포함)
```
**VIO 실험 이탈 사항** (sim-start는 gt 하드코딩 — 아래처럼 바꿔 실행):
- roscore 직후:
  ```bash
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && /work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/local_planner_mpc/config/load_config.sh sim_vio'
  ```
  (`sim_vio`, `sim_gt` 아님)
- 센서 퍼블리셔는 `LOC=vio`로 다시 (기본 gt 대신):
  ```bash
  LOC=vio bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_sensor_pub.sh
  ```
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam get /system/localization'   # → vio 확인
```

라이브에서 실제로 걸렸던 것:
- **bashrc의 ROS_MASTER_URI가 다른 IP(.36 등)로 남아있으면 tmux pane 전부 접속 실패** — sim-start Step 0a가 잡아줌. `.12`로 수정 필수 (tmux pane은 bashrc를 source하므로 Claude 셸 env만 고쳐선 안 됨). 컨테이너 안 run_*.sh는 이 걱정이 없다 — `config/sim.env`가 `.12`를 명시 export한다.
- UE4 생존 확인은 `ps aux | grep MyFirstUE4`가 불안정 — **AirSim clock 수신**으로 판정하는 게 확실 (clock이 오면 UE4+AirSim 둘 다 산 것):
  ```bash
  docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rostopic echo /airsim_node/clock -n 1'
  ```

## 1.5. tmux window 1 준비 (automation_experiment.py로 갈 경우에만 — §2 참조)

automation은 **세션 `risk_aware_planning`의 window index 1** pane 0~7에 노드를 쏜다. `/sim-start`를 수동으로
진행하면 init_sim 등이 index 1을 차지해 **pane 타게팅이 충돌**한다. automation 실행 전:
```bash
tmux list-windows -t risk_aware_planning        # index 1이 nodes용 8-pane인지 확인
# 아니면 재배치:
tmux move-window -s risk_aware_planning:1 -t risk_aware_planning:5
tmux new-window -t risk_aware_planning:1 -n nodes
for i in $(seq 1 7); do tmux split-window -t risk_aware_planning:1; tmux select-layout -t risk_aware_planning:1 tiled; done
```
automation 자체는 window 1이 아닌 **별도 window**에서 실행 (pane 7은 orchestrator가 쓴다):
```bash
tmux new-window -t risk_aware_planning -n auto
tmux send-keys -t risk_aware_planning:auto "cd /home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts && python3 automation_experiment.py --iterations N --name X" C-m
```
> **UNCONFIRMED**: `automation_experiment.py`가 내부적으로 pane 0/2/3/4/5/6에 쏘는 명령이 컨테이너
> 시대 기준(`run_voxblox.sh` 등)으로 갱신됐는지는 이 마이그레이션 범위에서 확인하지 못했다 — 스크립트
> 내부 구현을 읽지 않았다. §2(automation 경로)는 원문 그대로 남기되 이 상태를 명시한다. **확실히
> 동작하는 것은 §3(수동 경로) + 아래 §2 끝의 `/eval_data_node/start_evaluation` 트리거뿐이다.**

## 2. 실험 실행 — automation 경로 (원문 절차, 컨테이너 호환성 미확인)

`automation_experiment.py`가 Phase B(드론 초기화)~E(종료 감시)를 전부 수행하고 노드도 스스로 띄운다.
**tmux 세션 `risk_aware_planning` window 1의 고정 pane 배치를 쓴다** (pane0=FAST-LIVO, 2=voxblox,
3=exploration, 4=ours_jax, 5=runtime_evaluator, 6=so3_stack, 7=automation). automation의 stop/skip
로직이 이 pane 번호에 키잉돼 있으므로 **new-window 방식 금지**.

```bash
# window 1 pane 7에서 (없으면 생성):
tmux send-keys -t risk_aware_planning:1.7 C-c   # pre-typed 잔여 입력 flush
tmux send-keys -t risk_aware_planning:1.7 "cd /home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/scripts && python3 automation_experiment.py --iterations 3 --name skill_test" Enter
```

automation이 하는 일 (개입 불필요, 알아둘 것 — §1.5의 미확인 전제하에):
- Phase A(iter≥2): FAST-LIVO/voxblox/exploration/evaluator 재시작. **jax(pane4)·so3(pane6)·orchestrator(pane7)는
  iteration 간 유지** → 매 iter `/jax/reset` 서비스로 FSM/goal 누수 정리 (Phase D에서 자동 호출).
- Phase B: teleport(기본 2,1,1.5) + GT 수렴 <0.6m + position hold.
- Phase C: 노드 기동 + 헬스 게이트(노드 존재 + `/jax/optimal_trajectory` ≥0.3Hz).
- Phase D: (vio) VIO 수렴 게이트 → bag 기록 on → planner/control toggle → **`/eval_data_node/start_evaluation`
  호출로 eval 시작**.
- Phase E: `/evaluation_running`이 False 될 때까지 수동적 대기 — **종료 판정의 주인은 eval_data_node**
  (충돌 `/collision`, time_limit, coverage 임계 중 먼저 오는 것).

산출물: `active_3d_planning_app_reconstruction/data/<name>_<ts>/iter_N/` — `iter_N_*.bag`(진실원천),
`data_log.txt`, `rosparams.yaml`. 컨테이너 안에서는
`/work/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/data/`,
호스트에서는 이 트리가 그대로 bind-mount 원본이라
`/home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/mav_active_3d_planning/active_3d_planning_app_reconstruction/data/`에서
동일하게 보인다 (컨테이너/호스트 어느 쪽에서 열어도 같은 파일). **experiment_metrics.csv는 온라인에
안 생긴다** (launch 독스트링은 stale).

⚠ **`data_log.txt`는 세션 종료(충돌 / coverage 도달 / time_limit 기본 300s) 시점까지 버퍼링된다** —
실행 중에 미리 읽으면 0바이트라 "기록 실패"로 오판하기 쉽다(실측으로 확인된 함정). 종료 전 읽기는
가짜 음성(false negative)이다 — `/evaluation_running`이 `false`가 될 때까지 기다린 뒤 읽을 것.

### 모니터링 (돌아가는 동안)
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rostopic hz /jax/optimal_trajectory'   # ≥0.3Hz — ⚠ use_sim_time 하에서 신뢰 금지, 값보다 "메시지가 오는가"만 참고
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rostopic echo /jax/optimal_trajectory -n 1'   # 실제 도착 여부 판정은 이쪽으로
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rostopic echo /planning/pos_cmd -n 1'   # 스트리밍 중
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && timeout 3 rostopic echo /gt_odom/twist/twist/linear -n 1'   # 실제 이동 중
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosparam get /evaluation_running'   # true=진행, false=iter 종료
```

## 3. 수동 경로 — 개별 노드 기동 + eval 트리거 (확정 동작, run_*.sh 기반)

automation을 안 쓰거나(§1.5 미확인 상태) 단계별로 디버그해야 할 때는 각 노드를 자기 tmux pane에서
`bash <run_*.sh>`로 블로킹 실행한다 (컨테이너 진입/정리는 스크립트가 알아서 함, `docker exec` 손으로
재현하지 말 것). 순서는 유연하지만(토픽이 늦게 바인딩됨) fast-livo(vio 시) → voxblox → exploration →
jax → so3 → eval 순이 무난:
```bash
bash /home/ml/drone-stack-docker/modules/odometry/fast-livo-sim/run.sh          # vio 실험만 필요
bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_voxblox.sh
bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_exploration.sh
bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_jax.sh      # 첫 기동 ~80s (JAX JIT)
bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_so3.sh
bash /home/ml/drone-stack-docker/modules/planner/risk-aware-sim/run_eval.sh     # algorithm:=ours 스크립트에 이미 하드코딩
```
드론 위치잡기는 `/sim-drone teleport` 사용.

이 경로에선 automation의 Phase D가 하던 eval 시작 트리거를 **직접 호출해야 한다** (필수 스텝 —
안 부르면 `eval_data_node`가 계속 "Waiting for evaluation trigger"):
```bash
docker exec drone-stack-sim-x86 bash -lc 'source /opt/ros/noetic/setup.bash && source /work/ws/risk-aware/devel/setup.bash && source /work/config/sim.env && source /work/config/ros_env.sh && rosservice call /eval_data_node/start_evaluation "{}"'
```

핵심 함정만:
- `jax_main_node_ros_new.py` 직접 실행 금지 — 반드시 `run_jax.sh`(내부적으로 `ours_jax.launch`가
  adapter 동봉; 없으면 traj_server가 굶어 hover 고정).
- SO3 스택은 이제 conda 불필요 — `run_so3.sh`는 `network_mode: host`로 AirSim RPC(127.0.0.1:41451)에
  직접 붙는다, 컨테이너 안엔 `airsim` pip 패키지만 있으면 됨(module.yml).
- voxblox는 VFE traced 모델(`sparse_vfe_traced.pt`) 없으면 시작 즉시 크래시 — `RISK_AWARE_CHECKPOINTS`가
  (컨테이너 `HOME=/root`라 `~/risk_aware_assets` fallback이 깨지는 문제 때문에) `config/sim.env`에서
  명시 export되는 걸로 해결돼 있다(2026-07-25). 크래시하면 이 env가 실제로 보이는지부터 확인.
- `run_jax.sh`도 `gpu:=1`을 하드코딩한다(automation과 동일 관례) — ml desktop 디스크리트 GPU 기준.

## 4. 평가 (오프라인 — bag에서 재생성)

> **UNCONFIRMED**: 아래 두 launch(`generate_experiment_plots.launch`, `eval_offline.launch`)는
> [공통 스펙]의 확정 변환표에 없다 — 전용 `run_*.sh`가 없다. 패키지 자체는 이제 컨테이너 안에서만
> 빌드되므로(호스트에 `~/risk-aware_planning/devel`가 더 이상 없음) 호스트에서 맨 `roslaunch`로
> 돌리는 건 이제 안 된다. 컨테이너 진입 방법을 이 마이그레이션에서 확정하지 못했으니 지어내지 않는다 —
> 오케스트레이터 판단 필요(예: 전용 run_offline_eval.sh 신설, 또는 `docker exec ... roslaunch ...`를
> 명시적으로 승인).
```bash
roslaunch active_3d_planning_app_reconstruction generate_experiment_plots.launch target_directory:=<abs>/data/<run_dir>
```
→ iter별 `experiment_metrics.csv` 재생성 + Trajectory3D/VioRmse/SimulationOverview PNG.
단일 bag 재평가: `eval_offline.launch bag:=<abs>.bag out_csv:=<abs>.csv algorithm:=ours`.

### "실험이 잘 됐는가" 판정 체크리스트
1. `iter_N/` 마다 bag 존재, 크기 수 MB 이상 (KB급이면 기록 실패).
2. `data_log.txt`의 종료 사유 확인 — coverage 도달(성공) vs collision(충돌) vs time_limit.
   **세션이 끝나기 전에 읽으면 0바이트다(위 ⚠ 참조) — `/evaluation_running` false 확인 후에 읽을 것.**
3. 비행 중 `/jax/optimal_trajectory` ≥0.3Hz 유지, GT 속도 비영이었나 (`rostopic hz` 말고 `rostopic echo`로
   실제 도착을 볼 것 — use_sim_time 함정).
4. plots 재생성이 "no .bag" 경고 없이 완료, `experiment_metrics.csv`에 iter 행 존재.
5. (모델 관점) 시작 로그에 checkpoint 경로·target_norm·역스케일 라인이 의도한 값이었나.

## 5. 종료 / 반복

- iteration 루프는 automation이 알아서 돈다(§1.5 미확인 전제). run 전체 종료 후: `/sim-stop-nodes`(인프라
  유지, 다른 checkpoint로 재실험 시) 또는 `/sim-kill`(완전 종료).
- **다른 checkpoint로 재실험**: 0번 preflight부터 다시 (경로 갱신 → pane4/6(또는 §3의 jax/so3 pane)이
  살아있으면 `/sim-stop-nodes`로 jax를 죽여야 새 checkpoint가 로드됨 — jax는 iteration 간 유지되므로
  automation만 다시 돌리면 옛 모델 그대로).

## 함정 모음 (한 번씩 다 겪은 것들)
1. checkpoint 경로는 config가 아니라 `jax_mppi_params.py` 하드코딩.
2. Gen-A/Gen-B 세대 불일치 → 시작 시 load_state_dict 에러 (이건 차라리 시끄러워서 다행).
3. target_scale 역스케일 누락 → **조용한** 과보수(margin 최대 17×). 로드 로그로 확인.
4. `kinetic_statistics.pt` 불일치 → 조용한 입력 정규화 오류.
5. jax 노드는 iteration 간 유지 — checkpoint 바꾸려면 jax를 명시적으로 재시작.
6. 종료 판정은 eval_data_node 소유 — automation을 죽여도 iter가 안 끝난 것처럼 보이면 `/evaluation_running` 확인.
7. bag이 진실원천, CSV는 오프라인 재생성 (독스트링 믿지 말 것).
8. **`data_log.txt`는 세션 종료까지 버퍼링** — 미리 읽으면 0바이트 가짜 음성 (실측 확인, 컨테이너 시대에도 동일).
9. `use_sim_time` 하에서 `rostopic hz`는 무용지물 — `rostopic echo`로 판정 (실측 확인).
10. 감사(2026-07)의 배포측 불일치들(context z-score, twist 프레임, imu_topic wz=0, setpoint OOD 등)은
    예측 품질에 영향 — 메커니즘 검증과 별개로 수치 해석 시 감안. [[project-ete-net-integrity-audit]] 참조.

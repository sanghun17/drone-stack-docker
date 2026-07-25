---
name: ete-train-loop
description: ETE-Net 학습-분석-수정 루프. uncertainty_predictor의 ete_net을 학습시키고, calibration/coverage 기준으로 분석하고, config 수정안을 제시한 뒤 승인받아 재학습을 반복할 때 사용. "학습 돌려", "ete_net 학습", "학습 결과 분석" 요청 시 호출.
---

# ETE-Net 학습-분석-수정 루프

작업 디렉토리(호스트 bare-metal 실행 시): `/home/ml/drone-stack-docker/ws/risk-aware/src/risk_aware_planning/uncertainty_predictor/src`
(모든 명령 여기서 실행). 컨테이너 모드에서는 아래 "실행 환경" 절 참고 — 컨테이너
내부 작업 디렉토리는 이것과 다르다(마운트 타겟이라 별도 경로).

## 실행 환경: 컨테이너 모드가 표준 (2026-07-14 parity 검증 완료)

학습·오프라인 평가는 **drone-stack-docker의 ete-train 스택**으로 실행하는 것이 표준이다
(호스트 bare-metal 직접 실행도 여전히 유효 — 레거시 취급이지만 병행 가능, 아래 "호스트
직접 실행" 참고).

### 컨테이너 실행 (표준)

- 스택: `stacks/ete-train-2080ti.yml`(ml 데스크톱 자신, RTX 2080 Ti x3, sm75) /
  `ete-train-4090.yml`(sm89) / `ete-train-5090.yml`(sm120) — 모듈 구성은 동일
  (`compute/torch` + `compute/spconv` + `training/ete-net`), `gpu_arch`만 다름.
  준비: `cd ~/drone-stack-docker && GPU_UUIDS=<healthy-uuid> CONTAINER_USER=$(id -u):$(id -g) ./setup.sh build ete-train-2080ti && GPU_UUIDS=<healthy-uuid> CONTAINER_USER=$(id -u):$(id -g) ./setup.sh up ete-train-2080ti`
  — `GPU_UUIDS`/`CONTAINER_USER`는 `config/stack.env`에 기본 blank로 남아 있고
  invocation마다 export하는 관례(호스트별로 다른 값이라 공유 파일에 하드코딩 안 함).
  상세 절차·GPU 지정·트러블슈팅 전체: `~/drone-stack-docker/docs/ETE_TRAIN_GPU_HOSTS.md`.
- **컨테이너 내부 작업 디렉토리**: `modules/training/ete-net/train.sh`가 `cd`하는 경로는
  호스트 실경로가 아니라 컨테이너 마운트 타겟(정확한 문자열은 `train.sh`/`module.yml`의
  `mounts:` 항목 참고)이다 — 표기가 은퇴한 구 코드 트리의 이름을 우연히 재사용하고 있을
  뿐("path of least churn", module.yml 주석 참고), 그 구 트리가 되살아난 게 아니다.
  `ete_net/utils/config.py`가 자기 `__file__` 기준 상대경로로
  형제 디렉토리 `mav_active_3d_planning/local_planner_mpc`를 찾기 때문에 이 마운트
  타겟 아래에서 `uncertainty_predictor/`와 `mav_active_3d_planning/`이 형제로
  유지돼야 한다(둘 다 `RISK_AWARE_PLANNING_SRC` 마운트 하나로 들어옴). 학습 실행은
  `./setup.sh run ete-train-2080ti training/ete-net`(train.sh가 `ETE_CONFIG`/`ETE_SEED`
  env를 읽음) 또는 전체 수동 제어가 필요하면 `./setup.sh sh ete-train-2080ti`로 셸에
  들어가 `ete_net.train`을 직접 구동.
- **데이터 마운트/DATA_ROOT**: 코드와 데이터 마운트는 분리돼 있다 — 코드는
  `RISK_AWARE_PLANNING_SRC`(`config/stack.env`, 정본 `ws/risk-aware/src/risk_aware_planning`)를
  위 경로로, 데이터는 `ETE_DATA_DIR`(`config/stack.env`, 정본 루트 **`/home/ml/data`**)를
  컨테이너의 `/home/ml/data`로 그대로 마운트하고, 컨테이너 env `DATA_ROOT=/home/ml/data`를
  세팅한다(`modules/training/ete-net/module.yml`). `ete_train_config.yaml`의
  `data.final_dir`/`intermediate_dir`/`v22_sim_windows_dir` 등은 `${DATA_ROOT}` 확장을
  쓰므로 이 값이 실제로 해당 config가 참조하는 하위 경로(`raw/`/`stage1/`/`stage2/`
  티어)를 담고 있어야 한다 — 없으면 조용히 넘어가지 않고 즉시 halt(설계된 동작,
  config를 새 tiered 레이아웃에 맞게 고칠 것이지 마운트를 우회하지 말 것).
  ⚠ **module.yml/이미지가 바뀐 뒤에는 반드시 컨테이너를 재생성**(`./setup.sh up`)해야
  마운트가 반영된다 — 이미 떠 있는 컨테이너는 자동으로 따라가지 않는다(`ensure_container.sh`가
  이미지 변경을 스스로 감지하지 않음, 최상위 CLAUDE.md 참조).
- ⚠ **GPU 고정은 반드시 UUID, 그리고 privileged 컨테이너에서는 `NVIDIA_VISIBLE_DEVICES`만으로
  안심하지 말 것**: 이 데스크톱은 죽은 카드(PCI `1a:00.0`, NVML enumerate 불가)가 섞여 있어
  `--gpus all`/인덱스 지정이 실패하거나 죽은 카드를 잡을 수 있다 — `GPU_UUIDS=GPU-<uuid>`로
  `./setup.sh {build,up}` 시점에 고정한다(`tools/gen_dockerfile_compose.py`가 amd64+GPU_UUIDS일 때
  `runtime: nvidia` + `NVIDIA_VISIBLE_DEVICES` 조합으로 생성). 다만 dsd 컨테이너는 (이
  스택 포함) **모두 `privileged: true`로 뜬다**(`tools/gen_dockerfile_compose.py`, 스택
  공통) — privileged 모드에서는 GPU 격리가 `NVIDIA_VISIBLE_DEVICES`만으로 보장되지
  않으므로, 학습/평가/스모크테스트를 실제로 `docker exec`/`./setup.sh run`으로 띄울 때는
  매번 **`CUDA_VISIBLE_DEVICES=<one-healthy-uuid>`를 그 호출에 직접 넣어 GPU를 다시
  고정**한다(`docs/ETE_TRAIN_GPU_HOSTS.md`의 GPU validation protocol이 실제로 이 이중
  고정 패턴을 쓴다). 이 데스크톱은 UE4/AirSim(`sim-x86` 스택)이 특정 GPU를 점유하는
  경우가 있으므로, 그 GPU와 겹치지 않는 healthy UUID를 `CUDA_VISIBLE_DEVICES`로 명시할 것.
- **CONTAINER_USER**: 안 하면 체크포인트/tensorboard/캐시가 root 소유로 쓰여 sudo 없이
  못 지운다 — `CONTAINER_USER=$(id -u):$(id -g)`를 build/up에 같이 export.
- **수치 혼합 금지**: 컨테이너 torch(공식 cu121 wheel)와 bare-metal torch(커스텀 빌드)는
  부동소수 연산 순서가 달라 같은 seed라도 val loss가 seed-편차 수준(±0.2)으로 다르다.
  한 비교표 안의 run들은 같은 환경에서 나와야 한다. (parity 실측: F seed42 = bare-metal
  3.87 vs 컨테이너 4.03, epoch 시간은 22.0s로 동일)
- **실측 소요시간 견적(전체 데이터셋, 단일 2080Ti)**: 1 epoch ≈ 2.75분, 600 epoch(풀
  프로덕션 스케일) ≈ 27.5시간 — ablation용 서브셋 config(v23 계열, ~22s/epoch)보다 훨씬
  크다, 스케줄링 시 감안할 것.
- 다른 GPU 호스트: `ete-train-4090`(sm89)/`ete-train-5090`(sm120) 스택 — gpu_arch만
  다른 동일 조합.

### 호스트 직접 실행 (병행 가능, 레거시 취급)

위 "작업 디렉토리"에서 아래 0~4번 명령을 그대로 bare-metal에서 실행해도 된다 — 둘 다
유효한 경로이며, 학습-분석-수정 루프의 절차/판정기준은 컨테이너/호스트 어느 쪽이든
동일하다. 단, 위 "수치 혼합 금지"를 지킬 것(같은 비교표엔 같은 환경의 run만).

## 0. 사전 게이트 — integrity 확인 (필수, 생략 금지)

```
python3 -m ete_net.utils.debug.integrity_check --checks quick --n-folders 3
```

- `targets/MODE`가 FAIL이면 **절대 학습 시작 금지**: 디스크 데이터의 target 모드와 config `model.target_normalization`이 다르다는 뜻. config를 데이터에 맞추거나 재전처리(아래 함정 #1)를 먼저.
- 데이터셋을 재생성했거나 통계가 의심되면 `--checks all`(cache 검증 포함, 36GB 로드로 수 분).
- raw 검사의 attitude/스파이크 WARN은 학습 차단 사유는 아니지만 보고서에 기록.

## 1. 학습 시작

```
python3 -m ete_net.train --config ete_net/config/ete_train_config.yaml --gpu 0
```

- **`--gpu multi` 금지** — DataParallel은 sparse points/coords가 샘플 경계 무시하고 쪼개져 미지원(ete_trainer가 raise함). GPU는 `nvidia-smi`로 빈 것 선택.
- **0번 외 GPU는 `--gpu N`으로 지정 금지** — train.py가 `cuda:N` device만 만들고 `set_device`를 안 해서 spconv가 `cudaErrorIllegalAddress`로 즉사(current device 0과 불일치). 반드시 `CUDA_VISIBLE_DEVICES=N python3 -m ete_net.train ... --gpu 0` 형태로.
- 백그라운드로 실행하고 로그 파일로 리다이렉트. 시작 직후 확인할 것:
  - `outputs/ete_net_v2_<timestamp>/` 생성 + `config.yaml` 스냅샷 존재
  - 첫 epoch 로그에 NaN 없음, `loss/total` 유한
  - 전처리 재실행이 시작되면(수 시간) 그대로 두되 사용자에게 보고 — config의 데이터 관련 키를 바꿨다는 뜻
- resume: `--resume outputs/<run>/checkpoints/<ckpt>.pth` (loss weights는 현재 YAML이 진실원천으로 재적용됨).

## 2. 모니터링

- `outputs/<run>/checkpoints/best_val.pth` mtime 갱신 여부 (val loss 개선 시에만 갱신)
- tensorboard 스칼라 추이: `loss/{total,uce,emd}`, `nf/log_n_lambda`(post-clamp 유효값), `physical/{mae_m,mae_rad}`
- 판단 기준: train loss는 내려가는데 val이 정체 → 과적합; 둘 다 정체 → lr/용량; `nf/n_lambda`가 상한(e^7)에 붙음 → evidence 포화.

## 3. 분석 (학습 종료 또는 중간 checkpoint)

```
python3 -m ete_net.evaluate_ete_net --checkpoint outputs/<run>/checkpoints/best_val.pth --mode batch --gpu 0
```

산출: `outputs/<run>/eval_results_v3/<ckpt>/{train,val}/` — 요약할 지표:
- **calibration_curve.png / reliability_temporal.png** → ECE, coverage vs nominal (핵심 판정 기준)
- **error_by_timestep.png** → horizon별 MAE 성장
- **scatter_grid.png** → 축별 예측-실측 상관 (x/y/z/yaw 중 어디가 죽었는지)
- **pmf_samples_*.png** → 분포 모양 (포화/붕괴 여부)
- train/val 지표 차이 = 일반화 갭 (split은 bag 단위 group split이라 갭이 정직함)

## 4. 진단 → 수정 제안

문제를 분류하고 config 수정 후보를 **사용자 승인용으로** 제시 (임의 적용 금지):
- 과소적합(둘 다 높은 loss): `training.num_epochs`↑(cosine T_max 연동), `lr` 조정
- 캘리브레이션 나쁨(coverage≠nominal): `dirichlet_emd_weight`, `dirichlet_entropy_weight` 조정
- evidence 문제(epistemic 무의미): `nf_adaptive_target`, NF 구조(`nf_num_layers`, `nf_latent_dim`)
- 특정 축 죽음: 데이터 분포 확인(`integrity_check --checks cache`의 bin_coverage), 축별 loss 확인
- 1변수씩 바꾸고 run 이름과 변경 내용을 기록해 비교 가능하게.

승인 후: config 수정 → **0번 게이트부터 다시** (데이터 관련 키를 건드렸으면 재전처리 발생).

## 이 프로젝트 특유의 함정 (필독)

1. **캐시 해시와 재전처리**: `data.input_duration/stride_pkls`, `model.target_normalization/n_output_steps/voxel_size_before/map_physical_*`, 또는 `dataset/data_processor/*.py` 코드 변경 → Stage1부터 전체 재생성(수 시간, data_intermediate 46GB + data_final 79GB 재작성). `training.target_clip_*` 변경 → Stage2 필터 재적용 + 통계 재계산. 의도치 않은 재전처리가 시작되면 즉시 사용자에게 알릴 것.
2. **kinetic/target statistics**: `data/data/{kinetic,target}_statistics.pt`는 데이터 변경 시 자동 재계산되지만, 배포용 `~/risk_aware_assets/checkpoints/kinetic_statistics.pt`는 **수동 복사본** — 재학습 후 배포하려면 함께 갱신해야 함.
3. **Gen-A/Gen-B 세대**: state_dict 키로 판별 — Gen-A(구, 배포 baseline): `film_cond*.N.net.*` + `nf.layers.N.net.*`(realnvp), Gen-B(신, 현 코드): `film_cond*.N.embedding`(DirectFiLM) + `nf.layers.N.param_net.*`(radial). 현 working tree는 Gen-B 전용 — Gen-A checkpoint를 로드하는 코드(planner 포함)를 이 tree에서 돌리면 load_state_dict 실패. Gen-A가 필요하면 커밋 5147a84의 dirichlet_head.py/film_modules.py.
4. **DataParallel checkpoint**: 과거 run의 checkpoint 키에 `module.` prefix가 있을 수 있음 — evaluate_ete_net의 load_model이 strip 처리하지만 직접 로드할 땐 주의.
5. **config silent default 금지**: `utils/config.py`가 학습 영향 키 누락 시 KeyError로 halt함(의도된 동작). 키를 지우지 말고 값을 바꿀 것.
6. **평가 성능**: evaluate는 train+val 전체를 순회 — `--max_samples`로 제한 가능.
7. **vio_gt_ratio target**: 현행 target은 `log(ΔVIO⁻¹ΔGT)` (lever-arm-free, body(k)축, 단위 m/rad). `absolute` 모드는 VIO 원점거리 lever-arm 오염이 있어 사용 금지 (memory: project-ete-net-integrity-audit 참조).
8. **covis/meta arm의 평가 (v2.3+)**: covis는 게이트식(`covis_poses_local` 없으면 **조용히 꺼짐**), D-메타는 fail-fast — evaluate/커스텀 평가 스크립트는 반드시 학습과 같은 kwargs를 forward에 스레딩할 것 (evaluate_ete_net.py의 `extra_kwargs` 로직 참조). 안 하면 학습과 다른 모드로 평가돼 지표가 오염된다.

## 다중-arm ablation 워크플로 (2026-07-13 확립)

여러 아키텍처 변형을 비교할 때:
- **arm-per-worktree**: arm마다 worktree-격리 에이전트 1개 (구현→단위검증→학습→eval→보고). 전제: 기반 코드 커밋 필수 (worktree는 커밋 기준 분기). 결과물은 메인 `outputs/`로 복사 후 worktree 폐기, 승자 diff만 머지.
- **체인 스크립트 패턴**: arm당 `학습→run dir을 자기 로그에서 파싱→batch eval` 원샷 스크립트 (병렬 시 "최신 outputs dir" 참조는 레이스 — 금지).
- **비교 위생**: ① 전 arm **stage-1 like-for-like** (stage-2 checkpoint 혼입 금지 — 필요 시 best_val_stage1.pth로 재평가) ② 조기 early-stop(ep<30) arm은 미학습으로 평결 유보 ③ **배치 크기·데이터·seed 동결** ④ 신규 config 키는 optional-flag 관례 (기존 arm config들이 깨지지 않게 `_required`에 추가하지 말 것) ⑤ **best-val 단일값으로 판정 금지** — best_val 체크포인트 선택이 순간 딥(노이즈 최소값)을 잡는 사례 실증됨(v2.3에서 fragility 0.23~0.50짜리 arm 3개). 판정은 지속-중앙값(best epoch ±10 윈도 median) + fragility(중앙값−best) + calibration으로 ⑥ **조합 arm은 반드시 현재 승자 config를 복사해 파생** — 다른 arm 계보에서 만들면 숨은 상속 키(attention temp 등)가 제3변수로 혼입됨(v2.3 S/R confound 실증) ⑦ **stage-1 CI/UCE 비교는 evidence 상태에 조건부** — β=1+n_λ·p_φ에서 최종 calibration은 stage-2+offset이 n_λ를 재설정하므로, stage-1 CI로 우열을 가리려면 n_λ가 arm 간 동등(예: 전부 포화 — v2.3에서 실제 그랬음)임을 확인하거나 **n_λ를 상수로 고정한 shape-CI**를 써야 한다. evidence-불변 지표(물리 MAE/r, probe 방향차등화, seed 안정성)가 스크리닝의 안전한 기본. **최종 후보 2-3개는 반드시 stage-2+offset까지 one-queue로 태워 그 레벨에서 재판정** (2026-07-14 사용자 지적으로 확립).
- **GPU 처리량**: 한 GPU에 2 run 코-스케줄은 **하지 말 것** (2026-07-14 사용자 결정 — util이 이미 77-80% 포화라 개별 run이 크게 느려져 체감상 순차보다 손해). 여러 arm은 서로 다른 GPU에 1개씩, GPU가 모자라면 순차 큐로. 배치 확대는 ablation 중 금지 — 최종 프로덕션 학습 직전에만 1회 배치+LR 캘리브레이션.
- **최종 배포 후보는 승자 arm에 stage-2(NF, sim+real 혼합) 적용 후 배포모드 스위트(M 재계산→B=1 coverage→gold 판정)로 확정** (memory: project-vfe-map-unification-v22 참조).

## 완료 보고 형식

run 이름, epochs, best_val loss(및 어느 epoch), ECE/coverage 요약, 축별 상태, 다음 수정 제안 1~3개.

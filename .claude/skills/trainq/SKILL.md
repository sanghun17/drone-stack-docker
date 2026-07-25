---
name: trainq
description: 학습 잡 큐 제출·관리 — "학습 돌려줘/큐에 추가/상태 확인" 등 ETE-Net ablation 학습을 여러 개 순차/병렬로 돌릴 때 사용. 세션마다 lane 스크립트+워치독을 수작업으로 여는 것을 대체.
---

# trainq — 학습 큐 총괄 데몬

## ✅ 2026-07-26: 도커 스택으로 복구됨

2026-07-25 진단(바레메탈 micromamba 시절 경로 3종 소실 — worktree/venv/구 `repo/`
트리)은 해소됐다. `trainq_manager.py`를 다시 썼다: job 기동을 bare
`Popen([PYTHON, '-m', 'ete_net.train', ...], cwd=REPO)`에서 `docker exec -e
ETE_CONFIG=... -e ETE_SEED=... -e ETE_OUTPUT_DIR=... drone-stack-ete-train-4090
/work/modules/training/ete-net/train.sh` (컨테이너 안 학습 실행, 이 모듈의 정석
진입점 그대로)로 교체했다. 실증: `trainq_e2e_smoke`(3-epoch config) 제출 →
`--lanes-total 3 --reserve-lanes 2`로 매니저 기동 → 배차 → 완료(checkpoint 저장,
`docker exec`로 metrics 실행 → `results.md`/`results_ext.json` 갱신) → 큐 소진
exit 0, 전 과정 실관측 완료. 그 사이 이미 돌고 있던 `DEPLOY1_s42`/`DEPLOY1_nosw`
(600-epoch 본 학습 2건)는 epoch 진행이 계속됐고 영향 없었다.

바뀐 것 요약 (상세 사유는 `trainq_manager.py` 안 주석):
- **인터프리터**: 매니저 자체는 이제 `python3`(호스트 시스템 파이썬, stdlib만
  씀) — `/media/im/ETE4090/envs/ete/bin/python`는 더 이상 존재하지 않고 불필요.
  훈련 프로세스는 컨테이너 안 `python3`(torch 2.1.2+cu121)로 돈다.
- **pinned worktree 폐기**: `/media/im/ETE4090/worktrees/trainq_exec` 재건 안 함
  — `training/ete-net` 모듈이 `/work`에 리포 전체를 라이브 바인드마운트하는 설계를
  의도적으로 유지 중이라("active training-code iteration host", module.yml 주석)
  worktree 격리와 상충. `trainq_sync_code.sh`는 은퇴(no-op 스텁, 사유는 파일 안
  주석). 남은 잔여 리스크: trainq 잡이 도는 중 `ws/risk-aware/src/risk_aware_planning`에
  `git pull`/편집을 하면 재생성되는 DataLoader worker가 디스크 최신본을 다시
  import할 수 있음 — `trainq_status.sh`로 `running` 확인 후에만 pull/편집할 것.
- **출력 디렉터리**: 이제 `.../ws/risk-aware/src/risk_aware_planning/
  uncertainty_predictor/outputs/ete_net_v2_<name>/` (구 `/media/im/ETE4090/
  outputs/ete_net_v2_<name>/`가 아님) — `train.sh`에 새 옵션 env
  `ETE_OUTPUT_DIR`을 추가해서(기존 `ETE_SEED`와 동일한 optional 패턴) 이름 고정
  경로를 되살렸다. 컨테이너 안에서 root로 생성되지만 world-readable이라 호스트
  `im` 유저가 그대로 읽어 상태 체크 가능.
- **동시 lane 수**: 하드코딩 3 → `--lanes-total`(기본 2, 2026-07-26 GPU 실측
  81%util/2-lane 근거) + `--reserve-lanes`(트레인q 밖에서 이미 돌고 있는 잡용,
  기본 0). 아래 "매니저 기동법" 참조.
- **epochs 오버라이드 제거**: `train.sh`가 `--epochs`를 안 받아서
  `trainq_add.sh`의 `[epochs]` 인자를 없앴다 — 짧은 스모크 잡은 `num_epochs`를
  낮춘 임시 config로 만들 것(`config/ablation/v23_trainq_smoke.yaml`이 패턴 예시,
  커밋 안 된 임시 파일).
- **metrics 스크립트 이전**: `z_pmf_axis_metrics_ext.py`가
  `/media/im/ETE4090/scripts/`(호스트 전용, 컨테이너에서 안 보임)에서
  `uncertainty_predictor/scripts/`(`/work` 마운트 안, 커밋 안 된 파일)로 이동 —
  `docker exec`로 컨테이너 안에서 돌려야 torch/spconv를 쓸 수 있어서. 결과는
  컨테이너 쪽 bridge 파일에 쓰고, 매니저가 호스트 쪽 `results_ext.json`(기존
  경로 그대로, 이력 보존)로 merge한다.

---

⚠ **`im` 머신(10.74.23.213)과 `im` 계정은 사용자 소유가 아니다** — 공유 접근 허가만 받은
타인 소유 장비. 사용자 소유물은 그 머신에 꽂힌 사용자의 SSD(`/media/im/ETE4090`)뿐이다.
`im`에서는 **read-only 조사**와 **사용자 SSD(`/media/im/ETE4090`) 내 파일 작업**까지만
자율로 허용된다. **apt/시스템 설정/서비스/그룹 등 호스트 자체를 바꾸는 작업은 사용자가
요청했더라도 먼저 "머신 주인 동의를 확인했는지" 되물을 것** — 이 스킬의 나머지 절차(큐
제출/매니저 기동/상태 확인)는 모두 SSD 안에서 도는 프로세스라 이 제약과 무관하게
그대로 수행 가능하다.

설정 가능한 lane 수(기본 2) 학습 큐 배차·에러 감지·축지표·결과 원장을 한 프로세스
(`trainq_manager.py`)로 통합, `docker exec`로 `drone-stack-ete-train-4090` 컨테이너
안에서 잡을 띄운다. 위치: `/media/im/ETE4090/scripts/trainq/`.

## 사용법

```
/media/im/ETE4090/scripts/trainq/trainq_add.sh v23_C7 v23_C7.yaml 42          # 잡 제출 (name config.yaml seed [prio] [mem] [--force]) -- 이름이 이미 큐에 있으면 거부(exit 1); --force로만 우회
python3 /media/im/ETE4090/scripts/trainq/trainq_manager.py --lanes-total 2 --reserve-lanes 0 > /media/im/ETE4090/outputs/trainq/manager.log 2>&1 &   # 매니저 기동 (백그라운드)
/media/im/ETE4090/scripts/trainq/trainq_status.sh                              # 상태 표
```

`--reserve-lanes`는 trainq 밖에서 이미 `docker exec`로 수동 기동된 잡(예: 2026-07-26
현재 `DEPLOY1_s42`/`DEPLOY1_nosw`)이 GPU를 점유 중일 때 그만큼 lane을 아예 배차 풀에
넣지 않기 위한 값이다 — 매니저는 그 외부 잡을 추적/kill하지 않는다(관측조차 안 함),
그냥 lane 카운트만 깎아 이중 배차를 막는다. 예: GPU에 이미 2개가 돌고 있으면
`--lanes-total 3 --reserve-lanes 2`로 기동해 trainq용 1 lane만 확보. 외부 잡이 끝나면
매니저를 `--lanes-total 2 --reserve-lanes 0`(기본값, steady state)으로 재기동할 것.

## 규율

- **짝시드 기본**: ablation 비교는 seed를 고정해 제출 (예: 42). 다른 seed로 재확인할 땐 별도 name(`_s7` 등)으로 새 잡을 추가할 것 — 기존 잡을 덮어쓰지 않는다.
- **관측전용**: 매니저는 어떤 프로세스도 kill하지 않는다(trainq가 기동한 `docker exec` 자식은 물론, `--reserve-lanes`로 명시한 외부 잡도 건드리지 않는다). 스톨(30분 무갱신)이나 연속 실패(3회)를 감지하면 스스로 exit 코드로 알릴 뿐, 개입은 사람/오케스트레이터 몫이다.
- **조기 kill 금지**: 학습 중인 잡을 임의로 죽이지 말 것. 스톨 로그를 먼저 읽고 원인(예: 데이터 재전처리, GPU 경합)을 확인한 뒤 필요하면 사람이 직접 pid를 지정해 종료한다 (pgrep 패턴 금지 — 이 repo의 자기참조 킬 사고 재발 방지).
- **raycast mem 클래스**: `mem=raycast`인 잡은 (`--lanes-total`에서 `--reserve-lanes`를 뺀) trainq 관리 lane 중 2개를 점유한다(메모리 무거운 잡, GPU 코-스케줄 금지 정책과 정합). 일반 잡은 1 lane.
- **멱등성**: 이미 `uncertainty_predictor/outputs/ete_net_v2_<name>/checkpoints/best_val.pth`가 있는 name은 재기동 시 자동 skip(done 마킹). 큐를 지우지 않고 재기동해도 안전.
- **epochs 오버라이드 없음**: `train.sh`가 `--epochs`를 안 받는다. 짧게 돌려야 하면 `training.num_epochs`/`best_val_min_epoch`/`save_every`를 낮춘 임시 config를 만들 것(커밋 금지) — `config/ablation/v23_trainq_smoke.yaml`이 예시.
- **⚠ 이름 재사용 = 조용한 폐기 (2026-07-26 실제 사고, 수정됨)**: `QueueStore.sync()`는 이름이
  이미 non-pending(done/running/failed)으로 메모리에 있으면 같은 이름의 새 disk 엔트리를
  절대 재채택하지 않는다 — 이건 "이미 학습된 이름은 재학습 안 함" 멱등성 설계 그대로다
  (바꾸지 않았다). 문제는 예전엔 이게 **완전히 조용했다**는 것: `trainq_add.sh`는 "queued"라고
  exit 0 찍고, `skipped_precompleted` 카운터도 안 오르고(그 경로는 `_skip_precompleted`가 아니라
  `sync()` 안이라), 로그에도 흔적이 없었다. 실제 사고: 옛 캠페인과 같은 이름(`CHRr27_s42` 등)으로
  arm 3개를 재제출했더니 전부 no-op — `trainq_status.sh`를 제출 목록과 diff해서 겨우 발견했다.
  지금은 **2단 방어**: ① `trainq_add.sh`가 제출 시점에 같은 이름이 이미 큐에 있으면 (상태
  무관하게, `pending`이어도) **거부**하고 exit 1 — `--force`로만 우회 가능. ② 그래도(`--force`나
  구버전 호출로) 충돌이 발생하면 `QueueStore.sync()`가 폐기하면서 매니저 stdout 로그에
  `[trainq] WARNING: ignoring resubmitted '<name>' ...`을 찍는다(최초 1회만 — 그 다음 사이클부턴
  디스크에 이미 옛 내용으로 덮어써져 있어 재경보 안 함). **동작 자체는 안 바뀌었다** — 여전히
  기존 항목이 이긴다, 다른 이름으로 다시 제출할 것.

## 매니저 기동법

**이미 떠 있으면 재기동 금지.** 먼저 생존 확인:

```
python3 -c "import json,os; s=json.load(open('/media/im/ETE4090/outputs/trainq/status.json')); print(s['manager_pid'], os.path.exists(f'/proc/{s[\"manager_pid\"]}'))"
```

또는 `trainq_status.sh`가 `manager_pid=... alive=True`를 보여주면 기존 프로세스에 잡만 `trainq_add.sh`로 추가하면 된다 — 큐 파일을 감시 폴링하므로 재기동 불필요. `alive=False`거나 status.json이 없으면(최초 기동 또는 exit 0/3/4로 종료된 상태) 새로 기동.

exit 코드로 오케스트레이터가 각성해야 하는 상황:
- `0`: 큐 소진 — 정상 종료, 결과 확인.
- `3`: 스톨 — 로그(`outputs/trainq/logs/<name>.log`) 확인 후 판단.
- `4`: 연속 3회 실패 — 체계적 문제(config/데이터/환경) 의심, 큐 계속 넣기 전에 원인 규명.

### `--reserve-lanes` 값을 바꿔야 할 때 (예: 예약해뒀던 외부 잡이 끝남/죽음)

`--lanes-total`/`--reserve-lanes`는 시작 시 고정되는 CLI 인자다 — 코드에 라이브 리로드가
없어서, 값을 바꾸려면 매니저를 죽였다 다시 띄우는 것 말고 방법이 없다(2026-07-26 실증).
**이게 trainq가 이미 배차해 돌리는 중인 잡(예: 큐에 `running`으로 찍힌 이름)이 있어도
안전한 이유**를 코드 기준으로:

1. 매니저(`trainq_manager.py`)가 job을 띄우는 방식은 `subprocess.Popen(['docker','exec',...])`
   — `docker exec` 프로세스는 매니저의 **자식**이지 매니저 프로세스 자체가 아니다. Linux에서
   `kill <매니저_pid>`(SIGTERM, **그룹이 아니라 단일 PID**)는 그 프로세스 하나만 죽이고 자식은
   안 건드린다 — 자식은 그냥 고아가 돼 init(pid 1)에게 재부모화되고 계속 돈다. `docker exec`
   자체도 컨테이너 안 학습 프로세스와는 별개 계층(dockerd/containerd가 관리)이라 로컬 클라
   이언트가 죽는 것과 컨테이너 안 프로세스 생사는 무관하다. 실증: 매니저(pid 1917656)를
   `kill -TERM`한 직후 `docker exec` 자식(1917659, ppid=1로 재부모화)과 컨테이너 안 학습
   PID 둘 다 생존, 로그도 계속 growing — 2026-07-26 `DNoff_nosw_s42` 전환 때 확인.
2. 새 매니저는 `queue.jsonl`을 처음부터 다시 읽는다(`QueueStore.sync`가 `self.jobs`가 비어
   있으면 디스크 값을 그대로 받아들임). 이미 `running`으로 찍힌 잡은 `_dispatch()`의 `pending`
   필터에서 제외되므로 **재배차되지 않는다** — 이게 "죽은 매니저가 남긴 running 엔트리는
   자동 재배차 안 함" 설계의 실제 동작이다.
3. 단, 그 살아남은 잡은 새 매니저의 `self.running`(Popen 핸들 딕셔너리)에는 없다 — 그래서
   새 매니저는 그 잡이 쓰는 lane을 스스로 카운트하지 못한다. **이걸 그냥 두면 새 매니저가
   가용 lane을 실제보다 많게 착각해 다른 잡을 과다 배차할 위험이 있다.** 해법: 살아남은 잡
   개수만큼 `--reserve-lanes`를 잡아서 기동한다 — `--reserve-lanes`가 원래 "trainq 밖에서
   도는 잡용" 슬롯이니, 방금 살아남은 잡도 새 프로세스 입장에서는 정확히 그 범주다. 예:
   외부 예약 2개가 사라지고 trainq가 배차해둔 잡 1개(`DNoff_nosw_s42`)만 남았으면
   `--lanes-total 2 --reserve-lanes 1`로 재기동(예약 1 = 그 잡, 나머지 1 lane이 다음 pending
   잡을 받는다).
4. **잔여 한계**: 이렇게 살아남은 잡은 `queue.jsonl`에서 `status: running`인 채 새 매니저의
   추적 밖에 영원히 머문다 — 완료돼도 새 매니저가 감지해 `done`으로 못 바꾼다(체크포인트
   존재만으로는 "학습 중"과 "막 끝남"을 구분 못 해서 `_skip_precompleted`도 못 씀). 실제로
   그 잡이 끝나면(로그의 "Training Complete!" 확인) `queue.jsonl`에서 그 줄을 수동으로
   `done`/`ts_end`로 고치고, 그 시점에 `--reserve-lanes`를 다시 낮춰 재기동할 것.

⚠ **2026-07-26 실수 기록 (재발 방지용)**: `--reserve-lanes` 계산에 **재기동 시점에 살아있는
모든 살아남은 잡을 다** 넣어야 한다 — 하나라도 빠뜨리면 새 매니저가 그만큼 lane을 실제보다
많게 착각해 **과다 배차**한다. 실제로 겪음: `DNoff_nosw_s42`(예약 슬롯)만 회수하려고
`--lanes-total 2 --reserve-lanes 0`으로 재기동했는데, 그 순간 `auxoff_nosw_s42`도 (이전
재기동에서 이미) trainq 추적 밖의 "살아남은 잡"이었다는 걸 깜빡했다 — `auxoff_nosw_s42`
몫도 예약했어야 하는데 `--reserve-lanes 0`으로 기동해서 새 매니저가 lane 2개를 몽땅 새
pending arm 2개(`CHRr27_gnfix_s42`, `GNAUXw05r27_gnfix_s42`)에 배차, 그 결과
`auxoff_nosw_s42` + 새 잡 2개 = **GPU에서 3-way 동시 학습**이 잠깐 벌어졌다(config 중복은
아니었음 — "같은 config 두 벌"과는 다른 문제). 학습 중이던 잡을 죽이는 건 금지라 그대로
뒀고(GPU 메모리 여유상 OOM은 없었음, 속도만 저하), `auxoff_nosw_s42`가 먼저 끝나면서
자연히 2-lane으로 수렴했다 — **restart를 한 번 더 해서 "고치려" 하지 말 것**: 이미 살아있는
CHR/GNAUX도 그 restart 시점엔 또 "추적 밖 생존 잡"이 되므로 같은 실수를 반복하기 쉽다(그리고
`--reserve-lanes`는 `--lanes-total`을 넘을 수 없어 "3개 다 예약"조차 CLI로 표현이 안 된다).
**check list**: 재기동 직전엔 반드시 `trainq_status.sh`의 `running` 목록 + (그 재기동 자체가
살려낸) 추적 밖 생존 잡까지 전부 세어서 `--reserve-lanes`에 넣을 것. 과다 배차가 이미
벌어졌다면: 죽이지 말고, 그 매니저의 `lanes_free`가 0인 동안은 추가 배차가 없다는 걸
확인하고, 초과분(가장 먼저 끝날 살아남은 잡)이 자연 종료되길 기다렸다가 그 잡의 큐 항목만
수동 패치하는 쪽이 재기동보다 안전하다.

## 결과 위치

- `/media/im/ETE4090/outputs/trainq/queue.jsonl` — 잡 상태 원장 (pending/running/done/failed). 도커 전환 후에도 같은 파일에 이어서 append(158 done+27 failed 이력 보존, 2026-07-26에 159번째로 `trainq_e2e_smoke` 추가됨).
- `/media/im/ETE4090/outputs/trainq/status.json` — 30초 주기 스냅샷 (manager_pid, `lanes_total`/`lanes_reserved`/`lanes_free`, epoch 진행).
- `/media/im/ETE4090/outputs/trainq/results.md` — 성공 잡마다 한 줄 요약(축별 pooled r/MAE) append.
- `/media/im/ETE4090/outputs/trainq/results_ext.json` — `z_pmf_axis_metrics_ext.py` 전체 출력 (tag별 key). metrics 자체는 `docker exec`로 컨테이너 안에서 돌고 결과를 `uncertainty_predictor/outputs/_trainq_bridge/results_ext.json`(컨테이너에서 보이는 경로)에 쓴 뒤, 매니저가 이 호스트 전용 canonical 경로로 merge한다.
- `/media/im/ETE4090/outputs/trainq/logs/<name>.log` — 잡별 학습 로그 (stdout+stderr, `docker exec` 표준출력을 그대로 받음).
- `.../ws/risk-aware/src/risk_aware_planning/uncertainty_predictor/outputs/ete_net_v2_<name>/` — 잡별 checkpoint/tensorboard/config.yaml (컨테이너 안 root가 생성, world-readable). 구 `/media/im/ETE4090/outputs/ete_net_v2_<name>/`가 아님.

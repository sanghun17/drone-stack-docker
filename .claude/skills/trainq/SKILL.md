---
name: trainq
description: 학습 잡 큐 제출·관리 — "학습 돌려줘/큐에 추가/상태 확인" 등 ETE-Net ablation 학습을 여러 개 순차/병렬로 돌릴 때 사용. 세션마다 lane 스크립트+워치독을 수작업으로 여는 것을 대체.
---

# trainq — 학습 큐 총괄 데몬

⚠ **`im` 머신(10.74.23.213)과 `im` 계정은 사용자 소유가 아니다** — 공유 접근 허가만 받은
타인 소유 장비. 사용자 소유물은 그 머신에 꽂힌 사용자의 SSD(`/media/im/ETE4090`)뿐이다.
`im`에서는 **read-only 조사**와 **사용자 SSD(`/media/im/ETE4090`) 내 파일 작업**까지만
자율로 허용된다. **apt/시스템 설정/서비스/그룹 등 호스트 자체를 바꾸는 작업은 사용자가
요청했더라도 먼저 "머신 주인 동의를 확인했는지" 되물을 것** — 이 스킬의 나머지 절차(큐
제출/매니저 기동/상태 확인)는 모두 SSD 안에서 도는 프로세스라 이 제약과 무관하게
그대로 수행 가능하다.

3-lane 학습 큐 배차·에러 감지·축지표·결과 원장을 한 프로세스(`trainq_manager.py`)로 통합. 위치: `/media/im/ETE4090/scripts/trainq/`.

## 사용법

```
/media/im/ETE4090/scripts/trainq/trainq_add.sh v23_C7 v23_C7.yaml 42          # 잡 제출 (name config.yaml seed [prio] [mem] [epochs])
/media/im/ETE4090/envs/ete/bin/python /media/im/ETE4090/scripts/trainq/trainq_manager.py > /media/im/ETE4090/outputs/trainq/manager.log 2>&1 &   # 매니저 기동 (백그라운드)
/media/im/ETE4090/scripts/trainq/trainq_status.sh                              # 상태 표
```

## 규율

- **짝시드 기본**: ablation 비교는 seed를 고정해 제출 (예: 42). 다른 seed로 재확인할 땐 별도 name(`_s7` 등)으로 새 잡을 추가할 것 — 기존 잡을 덮어쓰지 않는다.
- **관측전용**: 매니저는 어떤 프로세스도 kill하지 않는다. 스톨(30분 무갱신)이나 연속 실패(3회)를 감지하면 스스로 exit 코드로 알릴 뿐, 개입은 사람/오케스트레이터 몫이다.
- **조기 kill 금지**: 학습 중인 잡을 임의로 죽이지 말 것. 스톨 로그를 먼저 읽고 원인(예: 데이터 재전처리, GPU 경합)을 확인한 뒤 필요하면 사람이 직접 pid를 지정해 종료한다 (pgrep 패턴 금지 — 이 repo의 자기참조 킬 사고 재발 방지).
- **raycast mem 클래스**: `mem=raycast`인 잡은 3-lane 중 2 lane을 점유한다(메모리 무거운 잡, GPU 코-스케줄 금지 정책과 정합). 일반 잡은 1 lane.
- **멱등성**: 이미 `outputs/ete_net_v2_<name>/checkpoints/best_val.pth`가 있는 name은 재기동 시 자동 skip(done 마킹). 큐를 지우지 않고 재기동해도 안전.

## 매니저 기동법

**이미 떠 있으면 재기동 금지.** 먼저 생존 확인:

```
/media/im/ETE4090/envs/ete/bin/python -c "import json,os; s=json.load(open('/media/im/ETE4090/outputs/trainq/status.json')); print(s['manager_pid'], os.path.exists(f'/proc/{s[\"manager_pid\"]}'))"
```

또는 `trainq_status.sh`가 `manager_pid=... alive=True`를 보여주면 기존 프로세스에 잡만 `trainq_add.sh`로 추가하면 된다 — 큐 파일을 감시 폴링하므로 재기동 불필요. `alive=False`거나 status.json이 없으면(최초 기동 또는 exit 3/4로 종료된 상태) 새로 기동.

exit 코드로 오케스트레이터가 각성해야 하는 상황:
- `0`: 큐 소진 — 정상 종료, 결과 확인.
- `3`: 스톨 — 로그(`outputs/trainq/logs/<name>.log`) 확인 후 판단.
- `4`: 연속 3회 실패 — 체계적 문제(config/데이터/환경) 의심, 큐 계속 넣기 전에 원인 규명.

## 결과 위치

- `/media/im/ETE4090/outputs/trainq/queue.jsonl` — 잡 상태 원장 (pending/running/done/failed).
- `/media/im/ETE4090/outputs/trainq/status.json` — 30초 주기 스냅샷 (manager_pid, lane 점유, epoch 진행).
- `/media/im/ETE4090/outputs/trainq/results.md` — 성공 잡마다 한 줄 요약(축별 pooled r/MAE) append.
- `/media/im/ETE4090/outputs/trainq/results_ext.json` — `z_pmf_axis_metrics_ext.py` 전체 출력 (tag별 key).
- `/media/im/ETE4090/outputs/trainq/logs/<name>.log` — 잡별 학습 로그 (stdout+stderr).

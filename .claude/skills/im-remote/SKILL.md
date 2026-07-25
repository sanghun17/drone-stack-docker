---
name: im-remote
description: Work against the im 4090 host (im@10.74.23.213) from the ml PC — dedicated dockerd, ete-train-4090 stack, training launch, SSD-only writes. Use when the task involves the im machine, the RTX 4090 training host, or trainq.
user_invocable: true
---

# im Remote Workflow (shared host: im is a co-tenant machine, not ours)

**철칙: im 머신·계정은 사용자 소유가 아니다.** 공유 접근 허가만 받은 타인 소유 장비다.
사용자 소유물은 그 머신에 꽂힌 외장 SSD(`/media/im/ETE4090`)뿐이다. **호스트 수준 변경
(apt/systemd/그룹/글로벌 docker 설정 등)은 절대 금지** — 사용자가 요청했더라도 먼저
"머신 주인 동의를 확인했는지" 되물을 것. im 계정에 SSH 키를 새로 만들지 말 것(아래
Connectivity 참조). SSD 안에서 도는 것(파일 작업, SSD 전용 dockerd로 뜬 컨테이너 조작,
학습 잡)은 이 제약과 무관하게 자율로 수행 가능.

**반드시 보존**: `outputs/`(2.7G, trainq 결과 원장 포함), `claude_portable_home/`(2.4G,
이동형 Claude HOME), `torch-build` 컨테이너(exited 상태로 존재 — 커스텀 torch 빌드 로그·
산출물, 삭제 금지). 지금 **학습이 돌고 있으면(`drone-stack-ete-train-4090`, 아래 확인법)
그 컨테이너를 재시작하거나 GPU를 점유하는 작업을 하지 말 것**.

## Connectivity

- 접속: `ssh im@10.74.23.213` (BatchMode로 키 인증 됨, jetson과 같은 SSH 패턴).
- **GitHub 접근은 `ssh -A im@10.74.23.213`로 agent forwarding 필수** — im 계정에는
  `~/.ssh/`에 `authorized_keys`/`known_hosts`만 있고 **개인키가 없다**(실측 확인,
  2026-07-25). im에서 직접 `git clone git@github.com:...` 같은 걸 시도하면 인증 실패
  — im 계정용 키를 새로 만들지 말고 항상 `-A`로 ml PC(또는 그 위 체인)의 키를 빌려쓸 것.
- 접속 확인: `ssh -o ConnectTimeout=3 im@10.74.23.213 "hostname && echo OK"`.

## dockerd — 전용 소켓, 매번 DOCKER_HOST 필요 (함정)

im에는 **시스템 기본 dockerd**(`/usr/bin/dockerd -H fd://`, `docker.sock`)와, 이 스택
전용으로 띄운 **별도 dockerd**가 동시에 떠 있다:

```
sudo setsid dockerd --data-root /media/im/ETE4090/docker \
  --host unix:///tmp/docker-ssd.sock --pidfile /tmp/docker-ssd.pid \
  --bridge=none --iptables=false \
  --add-runtime nvidia=/usr/bin/nvidia-container-runtime
```

data-root가 SSD(`/media/im/ETE4090/docker`)에 있고, 네트워크 브리지 없이(`--bridge=none
--iptables=false`) 뜬다 — 호스트 iptables/네트워크 설정을 안 건드리기 위함.

⚠️ **모든 docker 명령에 `DOCKER_HOST=unix:///tmp/docker-ssd.sock`을 붙여야 한다.** 안
붙이면 시스템 기본 데몬(빈 상태)을 보게 되고 "컨테이너가 없다"고 오진하게 된다 —
실제로 이 함정에 빠진 적 있음. 매 명령 앞에 명시:

```
DOCKER_HOST=unix:///tmp/docker-ssd.sock docker ps
```

셸 세션 내내 유지하려면 `export DOCKER_HOST=unix:///tmp/docker-ssd.sock`(단, Claude
셸은 명령마다 상태 비유지이므로 매번 다시 export하거나 인라인으로 붙일 것 — jetson-remote
스킬의 ROS 환경변수 함정과 같은 패턴).

## 스택 / 컨테이너

- 이미지: `drone-stack:ete-train-4090`, 컨테이너: `drone-stack-ete-train-4090`.
- 리포(코드) 정본: `/media/im/ETE4090/drone-stack-docker` — 컨테이너 안 `/work`로 바인드
  마운트. 코드 자체는 `/work/ws/risk-aware/src/risk_aware_planning`
  (`modules/training/ete-net/module.yml`이 별도 코드 마운트 없이 dsd 루트 마운트에
  얹혀 자동 노출되는 방식 — clone.sh가 `ws/risk-aware/src/risk_aware_planning`으로
  git clone).
- 데이터 정본: `/media/im/ETE4090/data` — 컨테이너 안 `/home/ml/data`(`ETE_DATA_DIR`
  마운트, `config/stack.env`), 컨테이너 env `DATA_ROOT=/home/ml/data`
  (`modules/training/ete-net/module.yml`의 `env:`). `raw/`/`stage1/`/`stage2/` 티어
  구조는 ml 쪽과 동일 레이아웃 — `DATA_MAP.md`가 SSD에도 사본으로 있음.
- 컨테이너 상태 확인: `DOCKER_HOST=unix:///tmp/docker-ssd.sock docker ps` (기대:
  `drone-stack-ete-train-4090` — 지금 학습 중이면 절대 재기동/kill 금지).
- 처음부터 다시 세우는 경우(현재 컨테이너를 건드리지 않는 상황에서만): `cd
  /media/im/ETE4090/drone-stack-docker && DOCKER_HOST=unix:///tmp/docker-ssd.sock
  CONTAINER_USER=$(id -u):$(id -g) ./setup.sh build ete-train-4090 &&
  DOCKER_HOST=unix:///tmp/docker-ssd.sock CONTAINER_USER=$(id -u):$(id -g) ./setup.sh
  up ete-train-4090` — 4090은 불량 GPU가 없는 단일 카드라 `GPU_UUIDS`는 불필요
  (`docs/ETE_TRAIN_GPU_HOSTS.md`). **현재 떠 있는 컨테이너는 `CONTAINER_USER` 없이
  기동돼 root 소유로 파일이 쓰임** — 다음 재기동 때 반영 고려(사용자 결정 사항, 지금
  임의로 재기동하지 말 것).

## 학습 기동

`modules/training/ete-net/train.sh`는 `ETE_CONFIG`(필수, 없으면 halt)와 `ETE_SEED`(선택)
환경변수를 읽어 `python3 -m ete_net.train --config "ete_net/$ETE_CONFIG" ${ETE_SEED:+--seed
"$ETE_SEED"}`를 실행한다.

`setup.sh run`은 원래 이 두 변수를 컨테이너에 **전달하지 않았다** — `run)`이
`docker exec`에 `-e` 플래그를 하나도 안 붙였고, `docker exec`는 호출한 호스트 셸의
환경변수를 자동으로 물려주지 않는다(도커 표준 동작). 그래서 train.sh 주석이 안내하는
"`setup.sh run` 실행 전에 export 해두라"가 실제로는 `${ETE_CONFIG:?}`에서 즉시 halt했다.
**2026-07-26 수정됨** — `run)`이 `ETE_CONFIG`/`ETE_SEED`와 `RUN_ENV`에 나열한 이름들을
`-e`로 전달한다. 지금은 이게 정석이다:

```
ETE_CONFIG=config/ablation/v23_F.yaml ETE_SEED=42 ./setup.sh run ete-train-4090 training/ete-net
```

`setup.sh`를 안 거치고 직접 띄울 때(백그라운드 `-d`, 로그 리다이렉트 등)는 `-e`를 손으로:

```
DOCKER_HOST=unix:///tmp/docker-ssd.sock docker exec \
  -e ETE_CONFIG=<ete_net/ 기준 상대경로, 예: config/ablation/v23_F.yaml> \
  -e ETE_SEED=<seed> \
  drone-stack-ete-train-4090 /work/modules/training/ete-net/train.sh
```

⚠️ `setup.sh run`은 `docker exec -it`이라 **TTY가 없는 곳(백그라운드/스크립트)에서는 못 쓴다.**
장기 학습을 붙여두려면 위의 직접 `docker exec -d ... > <로그> 2>&1` 형태를 쓴다.
로그 관례: `/home/ml/data/_train_logs/<name>.log` (호스트 `/media/im/ETE4090/data/_train_logs/`).

전체 수동 제어(임의 `train.py` 플래그, `integrity_check.py`, spconv smoke test 등)가
필요하면: `DOCKER_HOST=unix:///tmp/docker-ssd.sock docker exec -it
drone-stack-ete-train-4090 bash`로 들어가 `ete_net.train`을 직접 구동
(`docs/ETE_TRAIN_GPU_HOSTS.md` 참조, 컨테이너 내부 작업 디렉토리는
`/work/ws/risk-aware/src/risk_aware_planning/uncertainty_predictor/src`).

⚠️ 같은 gap이 `ete-train-loop` 스킬(ML
2080Ti 호스트용) 문서에도 남아 있을 수 있다 — 수정(2026-07-26)은 `setup.sh` 공통이라 ML 쪽도
함께 풀렸지만, 그 스킬 문서의 설명 문구는 아직 옛 상태일 수 있다(별도 확인 대상).

## GPU

RTX 4090 1장 (sm_89), driver 535.183.01 → **CUDA ≤12.2 컨테이너만** 구동 가능
(`nvidia/cuda:12.2.2-devel-ubuntu20.04` 베이스, `stacks/ete-train-4090.yml`). 불량
카드 없음 — `GPU_UUIDS` 지정 불필요(ml 데스크톱의 죽은 2080Ti 카드 문제와 다름).

## claude_portable_home — 보안 주의

`/media/im/ETE4090/claude_portable_home/`(2.4G)는 이동형 Claude HOME(과거 두 머신을
오가며 쓰던 것) — `.credentials.json` 등 **평문 인증 토큰이 그대로 들어 있다**. 분실/유출
시 즉시 무효화 대상. 읽기(메모리 이관 등)는 되지만, 이 디렉터리 자체를 옮기거나 외부로
반출하는 작업은 하지 말고 필요하면 먼저 보고할 것.

## trainq — 현재 배차 불가 (2026-07-25 확인, 코드 미수정)

`/media/im/ETE4090/scripts/trainq/`의 스크립트 4종(`trainq_add.sh`,
`trainq_manager.py`, `trainq_status.sh`, `trainq_sync_code.sh` — 구버전 백업
`trainq_manager.py.bak_predrain` 포함 실물 파일은 5개. 스킬은
`.claude/skills/trainq/SKILL.md`)은 **파일 자체는 남아 있지만 도커 스택 전환으로
실행 기반이 사라져 지금은 잡을 실제로 돌릴 수 없다.** 상세는 그 스킬 파일의 경고
섹션과 memory `[[project-im-ml-repo-integration]]` 참조. 이 스킬(im-remote)의 위
"학습 기동" 절(`docker exec -e`)이 trainq 없이도 직접 학습을 돌리는 현재 유효한 경로다.

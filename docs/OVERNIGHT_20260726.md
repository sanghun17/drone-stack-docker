# 야간 작업 종합 보고 — 2026-07-26

집필 시각: 2026-07-26 04:1x KST (일부 학습은 이 시각에도 진행 중 — 8절 참조).
대상 기간: 2026-07-25 08시경 ~ 2026-07-26 04시경 (dsd 커밋 74f24ab ~ 3f098c6, 약 20h).
리포 2개: `drone-stack-docker`(인프라) / `risk_aware_planning`(모델·플래너·sim).

---

## 1. 한눈에

- **배포 모델의 uncertainty 폭주 원인을 확정했다.** `GroupNorm(4,4)`가 맵 feature를 항등적으로 0으로 만들던 결함(전날 밤 발견)에 이어, 그 결함을 고쳐도 안 풀리던 원인이 `dirichlet_dn_fixed_log_n: 7.0`(evidence 상수 고정)이었음을 실측했다. 이 값을 끄자 실기 홀드아웃 손실이 구 배포모델(1.2174) 대비 **0.7444로 −39%**, uncertainty 자연조건 %천장이 37.2→**25.6**으로 처음으로 MEDIAN까지 개선됐다(커밋 `1037a7d`).
- **캠페인 최종 승자는 예상 밖이었다**: GroupNorm+GNAUX 사슬 전체를 걷어내고 **평범한 BatchNorm**(`SW_r27`, GNAUX도 꺼짐)이 캠페인 57개 라벨 중 최저 손실(0.6518 @ep99)을 기록했다(`ef38afe`). B=1 배포 함정은 실측으로 기각됐다(+0.00%).
- in-domain val 과 실기 성능의 역상관이 새 arm들에서도 재현됐지만 **보편 법칙은 아니다** — `auxoff_nosw`는 in-domain 최고(3.4191)이면서 실기도 무너지지 않았다(`c8ab8d9`).
- **sim 이 밤새 세 가지 독립 결함(RRT 롤백 누락, 이륙고도, 죽은 AirSim RPC 소켓)을 거쳐 드디어 탐사 비행을 한다.** 남은 문제는 z 하한 8cm 언더슈트와 "플래너만 탐사 볼륨을 아는" 구조적 공백(미수정, 보고만).
- **밤새 세 번 틀렸다** — stack.env.local 한 줄이 3호스트에서 setup.sh를 통째로 죽였고(§6), 도달 불가능한 커버리지 게이트를 세웠다가 재설계했고(§4/§6), GNAUX가 사문화됐을 거란 가설이 실측으로 뒤집혔다(§2/§4). 셋 다 원인·발견·수정 경위를 아래에 남긴다.

> ### ⚠ 본문을 읽기 전에 — seed 분산 (보고서 작성 이후 추가, `bc899f1`)
>
> 작성 **이후** 동일 config·seed만 다른 쌍(`DNoff_nosw` s42 vs s1337)이 나왔고,
> **seed 잡음이 이 캠페인에서 "승리"라고 부른 격차 상당수보다 크다**: 에폭 매칭 차이가
> **−0.35 ~ +0.62** 로 흔들리고 방향도 일관되지 않는다. 아래 본문의 arm 간 비교는
> **전부 seed 1개(대개 42) 기준**이므로, 격차가 이 대역 안에 있는 항목은 **아직 신호로
> 확정된 게 아니다.**
>
> - **DNoff의 간판 결과 "에폭이 늘수록 실기가 좋아진다"(0.96@ep80→0.67@ep300)가
>   seed 1337에서 재현되지 않았다.** 다만 깨끗한 반증도 아니다 — s1337은 새
>   `min_delta`(1e-3)로 ep198에 조기종료돼 s42가 좋아진 구간(250~320+)에 **도달하기 전에
>   멈췄다.** "seed 탓"과 "기회를 못 얻음"을 구분할 수 없다 → **판정 불가**. §2.1의 서술
>   강도를 낮춰 읽을 것.
> - **SW_r27의 선두는 상대적으로 견고하다** — 격차(0.15~0.7+)가 seed 대역(최대 0.62)보다
>   크다. **단 SW_r27도 아직 n=1**이다(`SW_r27_s1337` 학습 중).
> - `DNoffAuxoff_nosw`는 **긴 학습에서만** 음의 상호작용이다 — ep40/80/100에서는 두 부모를
>   **다 이기다가**(0.87~0.99) ep120부터 단조 악화해 best_val(ep427)에서 **1.4307로 최악**이
>   된다. 그 ep427이 캠페인 **in-domain 최저**(3.4157)다 — 체크포인트 선택이 실기 기준
>   최악을 능동적으로 고른, 가장 극단적인 괴리 사례. 장기/best_val 배포 후보에서 **제외**.
> - 다음 라운드 확인 예정: `SW_r27_s1337`(seed 재현성), `SW_r27_long`(120이 천장인가),
>   `SWbalsmp_r27`(승자 + 분산 축소 조합).
>
> ### ✅ 그 셋이 나왔다 — 결론 갱신 (`352ff93`)
>
> - **`SW_r27`의 seed 재현성은 좋다. 위에서 낮춘 강도를 되돌린다.** s1337이 **전 에폭에서
>   s42를 이긴다**(격차 0.05~0.12, 전부 s1337 우세) 하고, 그 best **0.5893**이 캠페인 전체
>   최고 수치다. `DNoff`가 seed 간 크게 벌어지고 방향도 뒤죽박죽이던 것과 정반대다.
>   → **SW_r27 배포 후보 추천의 신뢰도는 내려간 게 아니라 올라갔다.**
> - **`SW_r27_long`(600예산)은 세 번째 패턴이다** — in-domain은 계속 좋아지는데
>   (3.4717→3.4388) 홀드아웃은 ep40~340 내내 0.76~0.95로 **평평·잡음**이고 추세가 없다.
>   자기 best(0.8673)가 짧은 런의 best(0.6757)에 못 미친다. **예산을 늘려도 더 나은 걸
>   못 찾았다.**
> - **`SWbalsmp_r27`은 평균·분산 둘 다 손해**(0.935 vs 0.686, std 0.040 vs 0.034).
>   `balsmp_nosw`에서 보였던 분산 축소가 batchnorm 베이스로 **전이되지 않는다**.
>   조합이 항상 더하기가 아니라는 **두 번째 사례**(첫째는 `DNoffAuxoff`).
>
> ### ⚠⚠ 6번째 교란 축 — 에폭 매칭이 예산 축을 중화하지 못한다 (`352ff93`)
>
> `ete_trainer.py:288`이 `CosineAnnealingLR(T_max=training_cfg['num_epochs'])`로 만들어진다.
> **LR 스케줄 주기가 arm별 예산에 묶여 있다.** 그래서 "epoch 80"은 120예산 arm에서는 코사인
> 감쇠의 67% 지점, 600예산 arm에서는 13% 지점이다 — 같은 학습의 연장이 아니라 **다른 최적화**다.
> 실증: `SW_r27`(120) vs `SW_r27_long`(600)은 그 두 줄 빼고 config·seed가 **동일**한데
> ep80에서 이미 갈린다(0.673 vs 0.776).
>
> **→ 본문 §3의 120-vs-600 에폭 매칭 비교는 전부 이 미문서화 교란을 안고 있다.**
> 같은 예산끼리의 비교(`SW_r27_long` vs `DNoff_nosw`, r27 계열 내부)는 무관하다.
> 예산이 맞는 공정 비교에서 `SW_r27_long`은 `DNoff_nosw`를 6개 매칭 중 4개에서 이기고
> best에서 진다(0.867 vs 0.744).
>
> ### 🏁 캠페인 종료 — 최종 결론 (`64468e5`, 104 라벨 / 17 arm)
>
> **최종 배포 후보: `SW_r27` — 위에 아무것도 얹지 않은 소박한 레시피.**
> batchnorm(`map_norm` 미지정) · GNAUX off(`map_token_aux_weight` 미지정) · sample_weight on ·
> **120에폭/patience 80** · bulk 단독 · chroma 없음 · balanced_sampling 없음.
> 체크포인트는 `best_val` 또는 `ep100`.
>
> - 7라운드 전 구간에서 최선 또는 근접(**0.58~0.68**), 두 독립 seed 재현(s1337이 오히려 더 좋음:
>   **0.5893**), B=1 실측 ~0% 갭.
> - **`SW_r27`의 600예산 약세는 seed 42 특유였다** — s1337의 600예산 궤적은 ep40~340 내내
>   0.61~0.72로 조밀하다. 다만 **도움도 안 된다**(긴 런 best 0.752 < 짧은 런 best 0.589).
>   120이 두 seed 모두에서 최선이고, 예산 5배는 잘해야 중립이다.
>
> ### ★ 메타 발견 — "조합은 더하기가 아니다" 3전 3패
>
> | 조합 | 결과 |
> |---|---|
> | `DNoffAuxoff` | 1.4307 (DNoff 단독 0.7444) — 장기학습에서 파국적 |
> | `SWbalsmp` | 평균 +36%, 분산도 악화 — 분산 축소가 전이 안 됨 |
> | `SWCHR` | 평균 +12% — chroma 이득이 전이 안 됨 |
>
> 셋 다 "독립적으로 이긴 두 변경"을 합친 것이고 **셋 다 실패했다.** 특히 chroma는
> groupnorm+GNAUX 베이스에서는 이 캠페인의 **유일한 깨끗한 단일변수 승리**였는데
> batchnorm 위에서는 전 매칭 에폭에서 손해다. → **어떤 조합도 "전이될 것"이라 가정하지 말고
> 재야 한다**는 상시 사전확률로 취급할 것.
>
> ### 한계 넷 (배포 전 인지할 것)
>
> 1. 실기 홀드아웃이 **단일 세트**(170윈도우/5백) — 이 세트가 특이한지 검증할 제2 세트가 없다.
> 2. 120 레시피의 s1337 ep120 이후는 직접 측정 없음(긴 런으로 간접 확인).
> 3. **하이퍼파라미터 스윕이 캠페인 전체에 하나도 없다** — sample_weight, balsmp alpha/cap,
>    chroma probability 전부 단일 설정.
> 4. LR 스케줄 발견 이전의 120-vs-600 비교는 예산 축에 한해 신뢰도를 낮춰 읽을 것.
>
> 판정 불가 7건은 각각 **필요한 실험과 함께** `report_table.py`의 `print_final_summary()`와
> 커밋 `64468e5`에 정리돼 있다 — 다음 캠페인 설계도로 쓸 것.
>
> ### 판정불가 7번 해소 — 공통 메커니즘 가설은 **반증됐다** (`3ee4f6c`)
>
> "조합 3전 3패가 맵 토큰 rank 붕괴라는 하나의 메커니즘인가?" 를 token-PR로 쟀다. **아니다.**
>
> | arm | PR (best/ep100/ep120) | 홀드아웃 악화 |
> |---|---|---|
> | `SW_r27` (베이스) | 7.342 / 7.527 / 7.181 | — |
> | `SWbalsmp_r27` | **7.649 / 7.718 / 7.709 (↑)** | **+36% (최악)** |
> | `SWCHR_r27` | 5.082 / 5.310 / 5.156 (↓28~31%) | +12% |
> | `DNoffAuxoff` (기측정) | 3.919 (↓45~48%) | +92% |
>
> SWCHR과 DNoffAuxoff는 붕괴 서사와 맞지만 **`SWbalsmp`는 PR이 오히려 오르는데 셋 중 가장
> 나쁜 실패다.** → 세 실패는 **서로 다른 이유**다.
>
> **그리고 PR은 조합 사전 스크리닝 지표로 쓸 수 없다.** (a) SWCHR을 잡는 어떤 임계값도
> SWbalsmp를 통과시킨다 — 정작 걸러야 할 걸 못 거른다. (b) 방향이 맞는 두 건에서도
> 용량-반응이 역방향이다. **조합은 여전히 끝까지 돌려서 재는 수밖에 없다.**
>
> 부수: arm 내부 시간적 한계가 더 뚜렷해졌다 — SWCHR은 ep100→ep120에서 홀드아웃이
> **좋아지는데** PR도 **같이 떨어진다**(0.767→0.743 / 5.310→5.156). "PR 낮으면 나쁘다"로
> 읽으면 정반대 결론이 나온다.

---

## 2. ★ 핵심 발견 (중요도 순)

### 2.1 DN-const 해제가 uncertainty 폭발의 진짜 답이었다 (`1037a7d`)

전날 밤 발견된 GroupNorm 결함([[project_dead_map_feature_path]])을 고쳐 맵을 살렸지만, 그것만으로는 uncertainty 천장이 풀리지 않았다 — 40에폭 시점의 개선은 미학습 과도현상이고 300~350에폭이면 원래대로 돌아왔다(`nosw`/`sw`, 아래 표). 완주 arm 3개 + 구 배포 기준선을 3축(in-domain val, 실기 홀드아웃, uncertainty %천장)으로 평가:

| arm | 에폭 | in-domain | **실기 holdout** | unc%자연 | 스윕 MEDIAN | map c2mean | n_unique |
|---|---|---|---|---|---|---|---|
| **DNoff_nosw** (dirichlet_dn_fixed_log_n: 7.0→null) | 317 | 3.5274 | **0.7444** | **25.6** | **37.3** | 0.6028 | 4 |
| nosw (맵만 수정) | 366 | 3.5376 | 1.0419 | 32.0 | 40.9 | 0.0381 | 4 |
| sw (맵 수정 + sample_weight) | 404 | 3.5197 | 1.1501 | 34.7 | 44.6 | 0.0158 | 4 |
| deploy1_s42 (구, 맵 사망) | 539 | 3.4259 | 1.2174 | 37.2 | 42.3 | 0.0000 | **1** |

`dirichlet_dn_fixed_log_n: 7.0`은 `dirichlet_log_n_clamp_max`와 같은 값이라 evidence 총량 S가 806.44로 전 샘플 고정되고, "본 적 없는 입력"을 표현할 epistemic 채널의 정보량이 애초에 0이었다. 맵을 살려도 uncertainty를 줄일 메커니즘이 PMF 샤프닝 하나뿐이었던 이유다. 곁가지: DNoff_nosw의 맵 민감도(c2 mean 0.6028)가 sw/nosw(0.016~0.038)보다 한 자릿수 크다 — evidence 채널이 정보를 나르기 시작하니 맵이 실제로 더 쓰이는 것으로 읽히나, 관찰 수준으로만 남긴다.

**에폭 방향도 뒤집혔다**: sw/nosw는 학습할수록 실기가 나빠지는데(sw ep40 1.3071→ep300 1.4424), DNoff_nosw는 좋아진다(ep40 1.2570→ep300 **0.6659**). 구 DEPLOY1 시대에 "더 돌리면 나빠진다"고 봤던 게 DN-const의 부작용이었을 가능성이 크다.

### 2.2 SW_r27(평범한 BatchNorm)이 캠페인 최고 — GN+GNAUX 사슬 전체보다 낫고 더 싸다 (`ef38afe`)

`SW_r27`(map_norm/map_token_aux_weight 키 둘 다 **없음** = 기본 batchnorm + GNAUX off, sample_weight ON, bulk 단독, 120에폭)이 **캠페인 57라벨 전체 최저 손실**:

| 에폭매칭 | SW_r27 | 차순위 |
|---|---|---|
| ep39 | 0.7421 | 0.9762 (GNAUXw05r27_gnfix) |
| ep79 | 0.6731 | 0.9265 (CHRr27_gnfix) |
| **ep99** | **0.6518** ← 캠페인 전체 최저 | 0.8798 (CHRr27_gnfix) |
| ep119 | 0.6757 | 0.8780 (CHRr27_gnfix) |

비교: 장기학습형 DNoff_nosw는 0.6659(@ep300, 600에폭) / 0.7444(best). **SW_r27은 120에폭만으로 그것보다 낫다.**

이 쌍은 config diff로 직접 확인한 결과 `map_norm`(batchnorm vs groupnorm)과 `map_token_aux_weight`(0 vs 0.05) **두 변수**가 동시에 다르다 — GNAUX가 GroupNorm의 rank collapse를 상쇄하려는 목적으로 존재하므로 "groupnorm + GNAUX 없음" 조합 arm은 설계상 만들어진 적이 없다. 그래서 이 결과는 GNAUX 단독효과가 아니라 **"GroupNorm+GNAUX 설계 사슬 전체 vs 평범한 BatchNorm"**에 답한다 — 답은 **BN 승**(0.65~0.74 vs 0.88~1.44), 게다가 에폭도 적고 aux loss도 없어 더 싸다. BN→GN 전환의 원래 명분이 "격리 샘플(B=1)에서 BN이 실패한다"는 우려였던 걸 생각하면 불편한 결과다.

**B=1 함정은 실측으로 기각됐다.** `b1_holdout_loss_probe.py`로 윈도우 단위(B=1) 재측정한 결과 SW_r27 전 체크포인트(5개) **+0.00%** 차이, groupnorm 계열만 +0.28~0.61%(부동소수 잡음 수준). `nn.BatchNorm1d`는 `eval()`에서 running mean/var를 쓰므로 추론 시 배치 크기에 수학적으로 불변이다(`load_ckpt_model`이 항상 eval 로 둠). 기록에 있던 "격리 샘플 BN 실패"는 이 층이 아니라 **배포 시 offset 캘리브/centering**이라는 다른 메커니즘이었다.

한계 셋 (`ef38afe` 자체 명시): (a) B=1 재측정은 홀드아웃 손실만 커버 — uncertainty 천장·맵 생존 축은 미측정. (b) seed 42 단일 설계, 분산 없음(§4/§8에서 s1337 진행 중). (c) **batchnorm이 왜 이기는지 원인은 미해결** — 경험적 추천이지 메커니즘 근거가 아니다.

### 2.3 in-domain/실기 역상관은 재현됐지만 보편 법칙이 아니다

1라운드: in-domain 순위 sw > DNoff_nosw > nosw인데 실기 순위는 **정확히 반대**. 2라운드(`c8ab8d9`): `auxoff_nosw`는 in-domain 최고(3.4191, 캠페인 전체 최저 val)이면서 실기 홀드아웃도 전 에폭 2~4위로 무너지지 않았다(ep119 기준 0.9267, 3위). 4라운드(`3f098c6`): `balsmp_nosw`도 in-domain은 나빠졌는데(3.6360 vs nosw 3.5376) 실기는 평평~미세우위 — **세 번째, 더 온건한 형태의 괴리**. 즉 "in-domain이 좋으면 실기가 나쁘다"는 sw/nosw 쌍에서 관측된 특수 사례이지, 전 arm에 적용되는 법칙이 아니다.

### 2.4 chroma 증강 이득 / bulk+targeted union이 오히려 손해 (`c8ab8d9`, 에폭매칭 @119)

- **chroma 증강은 깨끗한 단일변수 이득**: `CHRr27_gnfix`(chroma on) 0.8780 vs `GNAUXw05r27_gnfix`(chroma off, 나머지 동일) 1.0348.
- **bulk+targeted union이 오히려 나빴다**: `GNAUXw05r27_gnfix`(bulk 단독) 1.0348 vs `GNAUXw05r28_gnfix`(union) 1.4114. sample_weight npz가 데이터 크기에 맞춰 기계적으로 재생성돼 완전한 단일변수는 아니지만, "데이터를 더 넣으면 좋아진다"는 통념과 반대라 주목할 값이다.

### 2.5 token-PR: GNAUX는 사문화가 아니었다 — 내 가설이 실측으로 뒤집혔다 (`c8ab8d9`)

전날 밤 메모리(`project_dead_map_feature_path`)에서 "GNAUX는 GroupNorm의 rank collapse를 상쇄하려던 반창고인데, 그 GroupNorm이 사실 항등 0이었으니 상처가 없는데 반창고만 당기고 있었을 것"이라고 가설을 세웠다. 새 프로브(`token_pr_probe.py`)로 재보니 **collapse는 GroupNorm 수정 후에도 재발한다**:

| arm | token-PR @ep119 |
|---|---|
| auxoff_nosw (GNAUX off) | **3.76** (자체 best 4.29 @ep474) |
| GNAUXw05r27_gnfix (on) | 5.66 |
| GNAUXw05r28_gnfix (on) | 6.37 |
| CHRr27_gnfix (on) | 6.86 |
| 구 deploy1_s42 (맵 사망) | 11.43 |

auxoff는 에폭이 더 많은데도(474 vs 317/366) PR이 가장 낮다 — "학습 부족"으로 설명되지 않는다. **GNAUX는 자기 docstring이 주장하는 일을 실제로 하고 있었다.** 다만 이 collapse가 배포 품질에 인과적으로 영향을 주는지는 미결이다 — PR이 낮은 auxoff가 홀드아웃 2위로 멀쩡하고, GNAUX-on 600계열 3 arm은 PR 7.2~8.0으로 사실상 구분이 안 되는데 홀드아웃은 0.74~1.15로 크게 갈린다. ⚠ token-PR 수치는 원 `wave_gn_judgment/`의 12.3→5.1 재현이 아니다(그 디렉터리가 이 머신에 없어 프로토콜 복원 불가) — arm 간 상대비교로만 유효.

---

## 3. arm 결과표

**⚠ 표를 읽기 전에 — 교란 축 4개.** 이번 재학습은 이전 ablation 표가 "arm 변수"가 아니라 "맵 생존 여부"를 비교했던 오귀속([[project_all_arm_retrain_plan]])의 재발을 막는 게 목적이었는데, 그 대신 아래 4축이 arm 사이에 뒤섞여 있다. **best_val끼리 단순 비교하지 말 것 — 반드시 에폭매칭으로 읽는다.**

1. **에폭 예산/patience**: r27/r28 계열(CHRr27_gnfix, GNAUXw05r27_gnfix, GNAUXw05r28_gnfix, SW_r27) = 120/80. DEPLOY1 계열(DNoff_nosw, auxoff_nosw, nosw, sw, balsmp_nosw, DNoffAuxoff_nosw) = 600/150.
2. **데이터**: r27 계열 = `stage2/bulk` 단독(39,294). r28·DEPLOY1 계열 = `stage2/bulk ∪ targeted`(39,833). 실험 설계이며 누락이 아니다.
3. **sample_weight**: sw/SW_r27/CHRr27_gnfix/GNAUXw05r27_gnfix/GNAUXw05r28_gnfix/balsmp(대신 balanced_sampling)는 ON, nosw/DNoff_nosw/auxoff_nosw/DNoffAuxoff_nosw는 OFF.
4. **min_delta**: `balsmp_nosw`/`DNoffAuxoff_nosw`만 새 `1e-3`(`d62bd27`)로 학습, 그 이전 arm 전부는 옛 코드 기본값 `1e-4`(사실상 early stopping 무효, §6/§7 참조) — "실제로 몇 에폭 돌았는가"가 config의 num_epochs/patience만으로 안 정해진다.

### Table A — best_val 기준 (참고용, 위 축이 안 통제됨)

| arm | 예산 | 데이터 | sample_wt | GNAUX | map_norm | balsmp | min_delta | 에폭 | in-domain | **holdout** | unc%자연 | 스윕MED | map c2mean |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| SW_r27 | 120/80 | bulk | ON | off(N/A) | batchnorm | off | 1e-4 | 115 | 3.4717 | **0.6757** | 30.0 | 43.4 | 0.4155 |
| DNoff_nosw | 600/150 | union | off | ON(0.05) | groupnorm | off | 1e-4 | 317 | 3.5274 | 0.7444 | 25.6 | 37.3 | 0.6028 |
| auxoff_nosw | 600/150 | union | off | OFF | groupnorm | off | 1e-4 | 474 | **3.4191**(in-domain 최고) | 0.9041 | 34.8 | 38.0 | 0.0466 |
| CHRr27_gnfix | 120/80 | bulk | ON | ON+chroma | groupnorm | off | 1e-4 | 100 | 3.5790 | 0.9558 | 33.8 | 43.6 | 0.5017 |
| nosw | 600/150 | union | off | ON(0.05) | groupnorm | off | 1e-4 | 366 | 3.5376 | 1.0419 | 32.0 | 40.9 | 0.0381 |
| GNAUXw05r27_gnfix | 120/80 | bulk | ON | ON(0.05) | groupnorm | off | 1e-4 | 112 | 3.5923 | 1.0420 | 35.0 | 42.7 | 0.4842 |
| balsmp_nosw | 600/150(실제 240 조기종료) | union | off | ON(0.05) | groupnorm | **ON**(α0.5,cap10) | **1e-3** | 91 | 3.6360 | 1.0900 | 34.8 | 42.5 | 1.1521 |
| sw | 600/150 | union | ON | ON(0.05) | groupnorm | off | 1e-4 | 404 | 3.5197 | 1.1501 | 34.7 | 44.6 | 0.0158 |
| deploy1_s42(구, 맵 사망) | pre-fix | — | — | — | groupnorm(사망) | off | — | 539 | 3.4259 | 1.2174 | 37.2 | 42.3 | 0.0000 |
| GNAUXw05r28_gnfix | 120/80 | union | ON | ON(0.05) | groupnorm | off | 1e-4 | 111 | 3.5557 | **1.4321**(최악) | 42.9 | 45.9 | 0.5347 |

### Table B — 에폭매칭 (핵심 비교, 낮을수록 좋음)

**@ep99** (SW_r27 캠페인 바닥 지점): SW_r27 **0.6518** < CHRr27_gnfix 0.8798 < auxoff_nosw 1.0603 < balsmp_nosw 1.0698 < GNAUXw05r27_gnfix 1.0712 < nosw 1.2122 < DNoff_nosw 1.3540 < sw 1.4244 < GNAUXw05r28_gnfix 1.4412

**@ep119** (7~9-way 최대 겹침): SW_r27 **0.6757** < CHRr27_gnfix 0.8780 < auxoff_nosw 0.9267 < sw 0.9647 < nosw 0.9813 < GNAUXw05r27_gnfix 1.0348 < balsmp_nosw 1.0925 < DNoff_nosw 1.2308 < GNAUXw05r28_gnfix 1.4114

**@ep300** (DEPLOY1 예산 계열만, 장기학습 비교): DNoff_nosw **0.6659** < auxoff_nosw 0.9565 < nosw 1.0841 < sw 1.4424 — DNoff_nosw는 여기서만 유일하게 SW_r27급으로 올라온다(장기학습형).

### 어느 쌍이 단일변수인지

- **CHRr27_gnfix vs GNAUXw05r27_gnfix** — 완전 단일변수(chroma 증강만 다름, 예산/데이터/sample_weight/GNAUX 전부 동일).
- **GNAUXw05r27_gnfix vs GNAUXw05r28_gnfix** — 거의 단일변수(데이터 풀만 다름, sample_weight npz는 행수에 맞춰 기계적으로 재생성된 것이라 독립 설계변수 아님).
- **sw vs nosw vs DNoff_nosw** — 셋 다 예산/데이터/GNAUX 공유. sw는 sample_weight를, DNoff_nosw는 dirichlet_dn_fixed_log_n을 각각 단일 격리. 둘 다 깨끗함(2라운드부터).
- **balsmp_nosw vs nosw** — 완전 단일변수(config diff 확인: `training.balanced_sampling{,_alpha,_cap}` 세 줄만 추가, 나머지 0).
- **auxoff_nosw vs {CHRr27,GNAUXw05r27,GNAUXw05r28}_gnfix (에폭매칭)** — **단일변수 아님**. GNAUX on/off·sample_weight on/off·(r27쌍은) 데이터 풀까지 동시에 다르다.
- **SW_r27 vs GNAUXw05r27_gnfix** — **단일변수 아님**(§2.2). `map_norm` + `map_token_aux_weight` 2변수 동시 상이. GNAUX 단독 아이솔레이션은 이 캠페인에 존재하지 않는다(§4).

---

## 4. 판정 불가 항목 (억지로 결론 내지 않은 것들)

- **GNAUX 단독 효과**: 캠페인에 깨끗한 아이솔레이션이 없다(§2.2, §3). GNAUX가 GroupNorm의 rank collapse를 막으려는 목적으로 설계됐기 때문에 "groupnorm + GNAUX 없음" arm 자체가 만들어진 적이 없다. 판정하려면 "batchnorm + GNAUX(0.05)" 또는 "groupnorm + GNAUX(0.0)"처럼 지금 없는 조합이 필요하다.
- **balsmp 평균 우열**: `balsmp_nosw` vs `nosw`(완전 단일변수) 5개 매치포인트에서 매치-에폭 평균차가 −1.2%(1.0757 vs 1.0884)인데 이는 `nosw` 자체의 에폭간 노이즈(std 0.084)보다 작고, 5점 중 3승 2패로 방향도 일관되지 않는다(`3f098c6`). **평균 우열은 판정 불가**로 명시. 반면 **분산 축소는 분명하다** — balsmp 표준편차 0.023 vs nosw 0.084로 **3.6배** 조밀(희귀 타겟 기아를 줄인다는 설계 목적과 방향이 맞다).
- **batchnorm이 왜 이기는지**: B=1 배치통계 퇴화라는 우려는 실측으로 기각됐지만(§2.2), 그럼 왜 이기는지의 메커니즘은 미해결이다. `ef38afe`가 스스로 "경험적 추천이지 메커니즘 근거가 아니다"라고 명시.
- **seed 분산**: 캠페인 57라벨 전부 seed=42 단일 설계다. 오늘 밤 처음으로 `DNoff_nosw_s1337`(진행 중, ep45)과 `SW_r27_s1337`(대기, 큐 등록만 됨)을 투입했으나 아직 완주하지 않아 반영 불가(§8).
- **DNoffAuxoff_nosw (조합 arm)**: DNoff_nosw·auxoff_nosw가 각각 개별로 이겼길래 "둘 다 켜면 더 좋을 것"이라는 가설로 조합 arm을 만들었는데(`4e4c158`), 방금(04:07) 완주해 trainq 자동 후크가 찍은 best-checkpoint 실기 holdout 값이 **1.4307**(uce 1.0585) — DNoff_nosw(0.7444)·auxoff_nosw(0.9041) 어느 쪽보다도 나쁘고 캠페인 최악(GNAUXw05r28_gnfix 1.4321)과 동급이다. 흥미롭게도 점예측 정확도(r/mae)는 오히려 이 arm이 가장 좋다(x: r=0.488, y: r=0.447, z: r=0.446, yaw: r=0.688 — SW_r27보다도 높음) — **정확도와 불확실성 캘리브레이션이 이 arm에서 크게 어긋난다**는 신호로 보이나, 아직 단일 지점(best_val, campaign_results.json에 미병합)뿐이라 에폭매칭 없이는 "두 knob이 상호작용으로 손해를 낸다"고 단정할 수 없다. **판정 보류, 다음 라운드 과제**(§8).

---

## 5. sim — 드론이 안 움직이던 것부터 탐사 진행까지

세 가지 **독립된** 결함이 순서대로 걸려 있었다.

1. **RRT* 롤백 누락** (`46271f6`) — `rewireToBestParent`가 후보 루프에서 connectPoses 성공 여부를 알기 전에 segment를 제자리에서 변형(trajectory clear + parent = candidate)하는데, 전 후보가 실패하면 그 segment가 부모 없이 트리 밖에 고아로 남았다. 호출부는 반환값을 무시하고 `new_segment->parent != nullptr`만 성공 신호로 썼다. 증상: "[tree] nodes 1 | +15 this cycle" — 15개 만들었다는데 트리는 1개. `new_segment_tries_`가 1000에 닿아 곧바로 "Exploration DONE" → 플래너 영구 정지, 궤적 발행 0개. 수정: 전 후보 실패 경로에서 parent/trajectory 원복. 실증: connect ok 15→트리1(수정전) → 트리 성장 정상(수정후, `/planner/command/trajectory` 발행 확인, 상태 EXPLORING).
2. **잘못된 첫 시도, 되돌림** (`3c80784` → `1f345e8`) — `min_new_value: 10.0→0`으로 고쳐봤으나 실제 근본원인이 아니어서 15분 만에 되돌렸다. 진짜 원인은 다음.
3. **이륙고도가 탐사 슬래브 위였다** (`8bd515a`) — 드론 실측 호버 z=3.7076m(지령 3.0인데 +0.71 오버슛)인데 `target_bounding_volume` z∈[0,2], body_clear_radius=0.5m. 무충돌 버블 밖 첫 에지 샘플이 가장 유리한 수직하강이어도 z≈3.15>2.0이라 **모든** 에지가 out-of-bounds로 실패 → 트리 노드 0 → 궤적 0 → DONE. 회귀 시점은 2026-04-14 `940e73e`(LA-Planner 안정화로 target box z를 [-2,5]→[0,2]로 좁혔는데 sim 이륙고도 3.0은 안 내려왔었다). 이륙고도 3.0→1.2로 수정.
4. **sim 샘플링을 실기체에 정렬 + DONE 래치 조건 추가** (`bd9ef98`) — spheric→uniform 샘플링(같은 맵·정지 상태 실측: 트리 1→2→6노드, 경계밖 98.5%→0%), min_path_length 1.4→0.5(첫 비주행 샘플이 시작점에서 중앙값 0.56m인데 1.4가 그걸 다 막고 있었다). DONE 판정이 `once_leaved`라는 죽은 디버그 게이트에 걸려 있던 걸 걷어낸 이전 커밋(b10c917) 이후, sim의 첫 사이클이 항상 기아(맵이 카메라 절두체 하나뿐)라 탐사 시작 0.2초 만에 "탐사 완료"로 죽는 새 문제가 생겼다 — `ever_replanned_`(최소 1회 궤적 발행) + `starved_cycles_ >= done_starved_cycles`(연속 기아, config 신설 `done_starved_cycles: 3`, requireParam으로 halt) 두 조건을 추가해 막았다.
5. **max_density_range 1.2→0.4** (`405cf9b`) — 짧은 엣지가 살아나며 density reject가 늘어난 부작용을 실기체 값(0.4)에 맞춰 해소. 같은 맵·정지 상태 실측: 트리 6→**54**노드, edge-unobs/edge-occ 276+107(89%)→83+47.
6. **죽은 AirSim RPC 소켓** (코드 변경 없음, 재시작으로 해소) — `so3_control_bridge.py:301`의 `enableApiControl`이 `StreamClosedError`를 계속 던져 바로 다음 줄 `commands_enabled = True`에 영원히 못 도달했다. msgpackrpc는 자동 재연결을 안 한다. `run_so3.sh` 재시작으로 해소. **실기체는 mavros 경로라 이 결함과 무관** — sim 전용.
7. **MAX_THRUST 재측정** (`5737b86`, §7에도 기재) — 15.60→16.535. z~2.9m로 탐사볼륨 z[0,2] 밖에 표류하던 잔여 증상의 원인. 옛 값(2026-04-17 LSQ 주석)은 가속도 기반 피팅이었는데 AirSim SimpleFlight가 폐루프로 상승률을 제한해 구조적으로 편향됐다. 새 방법(정상상태 vz=0이 되는 throttle을 이분법 탐색)으로 h=0.5933→MAX_THRUST=9.81/0.5933=**16.535N**. 독립 교차검증: settings.json 기본 로터 4×4.1793N=16.717N(1.1% 이내 일치, 옛값은 6.0% 벗어남). 결과: 추종오차 편측 +0.204m→양측 대칭 [-0.151,+0.043], 맵 복셀 20,302→**62,269**, 수평이동 ~0→4.92m.

**남은 문제 (미수정, 보고만)**:
- z 하한 언더슈트 — 253/2251(11%) 샘플이 z<0로 ~8cm. 상한은 완전히 지킨다.
- **구조적 공백**: `/target_bounding_volume`을 아는 건 탐사 플래너뿐이다. JAX 플래너·traj_server·SO(3) 브리지는 이 볼륨을 모른다. 드론이 슬래브를 벗어나도 아무도 안 끌어오는 설계다.

---

## 6. 인프라 변경

- **setup.sh 회귀와 3호스트 영향** — `2a7ce8e`가 `config/stack.env` 끝에 `[ -f .../stack.env.local ] && . .../stack.env.local`을 넣었는데, `.local`이 없으면 `&&` 리스트 종료상태가 1이 되고 `source`가 그걸 그대로 반환해 `set -euo pipefail`인 setup.sh가 **에러 메시지 없이 exit 1**로 죽었다. `1942b57`이 `if/fi`로 수정. 실제 피해(2026-07-26 3호스트 교차검증): ml·jetson은 `.local` 없어 setup.sh 전 서브커맨드 사망(실증), im은 그날 이미 `.local`을 만들어둔 덕에 우연히 생존. **jetson은 이 수정을 pull해야 정상화된다** — 다음 접속 시 확인 필요.
- **config/stack.env.local** (`2a7ce8e`) — 호스트별 값(데이터 경로, GPU UUID, CONTAINER_USER 등)을 추적 파일 밖 untracked 파일로 분리해 pull 충돌을 제거. 셸(`source` 자기 끝에서 호출)과 파서(`load_env`가 `.local`을 먼저 읽어 first-writer-wins) 양쪽 다 대응. 직접 계기: im의 `ETE_DATA_DIR` 한 줄이 pull-only 운용과 충돌했던 것.
- **clone.sh 공용 헬퍼** (`b1c762f`) — `modules/_common/clone_repo.sh` 신설, clone.sh 7개의 "있으면 fetch+checkout, 없으면 clone -b" 반복 로직 통합. **헬퍼는 env를 직접 안 읽고 인자로만 받는다** — 호출자가 stack.env를 미리 해석해 넘기게 강제해서, `set -a`가 막으려던 "하드코딩 기본값이 조용히 이긴다" 재발을 방지. 예외 1건(fast-livo-sim의 rpg_vikit — 브랜치 아닌 커밋 핀)은 인라인 유지.
- **trainq 도커 부활 + 이름충돌 가드** (`.claude/skills/trainq/SKILL.md`, im SSD 코드 — dsd git 밖) — 2026-07-25 바레메탈 micromamba 경로 3종(worktree/venv/구 repo/) 소실 진단을 해소: 매니저가 job을 이제 `docker exec drone-stack-ete-train-4090 .../train.sh`로 기동(기존 bare `Popen(python -m ete_net.train)` 대체). lane 기본 2(GPU 실측 81%util 근거), `--reserve-lanes`로 외부 수동잡과 공존. `trainq_e2e_smoke` 풀 사이클 실증(제출→배차→완료→checkpoint→metrics→results.md/json 갱신→큐소진 exit 0), 그 사이 돌던 실제 잡(DEPLOY1_s42/nosw) 무영향 확인. **이름 재사용 조용한 폐기 사고**를 겪은 뒤(옛 캠페인과 같은 이름으로 arm 3개 재제출→전부 no-op, 흔적 없음) 2단 방어를 넣었다: ① `trainq_add.sh`가 제출 시점 중복 이름을 exit 1로 거부(`--force`로만 우회) ② 그래도 충돌하면 `QueueStore.sync()`가 `[trainq] WARNING: ignoring resubmitted '<name>'`을 최초 1회 로그.
- **평가 스위트** (`e5c3297`) — `eval_holdout_suite.py`가 스크래치패드에 흩어져 있던 프로브 3종을 `--mode {map-survival,holdout-loss,uncertainty-ceiling,all}` 하나로 통합. 체크포인트·홀드아웃 디렉터리·config·출력 경로 전부 필수 인자(기본값 없음, 프로젝트 규칙 준수). 구 체크포인트를 수정된 map_encoder로 잘못 읽는 함정을 `--old-map-encoder-labels`+`--map-encoder-orig` 상호필수 가드로 막았고, 가드 없이 재면 1.1409·가드 켜면 1.2174(기준값과 일치)로 가드가 장식이 아님을 실증. 재현성: ml 물리GPU1·일회용 컨테이너에서 기존 5개 값 전부 ~0.001 이내(spconv 비결정성 수준) 재현.
- **torch 통일 준비** (`b233de8`) — 목표는 sim 계열(자체 ABI=1 휠)과 ete-train 계열(pip ABI=0 휠)의 `build_env` 축 분기를 없애는 것. `TORCH_WHEEL_ARCHIVE_DIR`(stack.env, untracked `.local`로 호스트별 값) + `wheels/stage_from_archive.sh`(md5검증+하드링크) + 테스트 전용 스택 `ete-train-4090-abi1.yml` 신설. **실 im 4090(sm_89)에서 6종 검증 PASS**: ABI=1·LAPACK=open+CPU qr 실행·CUDA 미번들(727MB,10파일)·spconv 실 GPU forward·`import ete_net`. 기존 5개 스택 Dockerfile 재생성 diff = 바이트 동일(순가산). **계획 문서의 가정 하나가 틀렸다는 걸 발견**: "jax와 cuDNN을 공유한다"던 static readelf 판독과 달리 실제로는 이 휠이 `USE_CUDNN=OFF`다. 실사용 영향 확인: ete_net에서 cuDNN 의존 연산은 `nn.GRU` 하나뿐이고 `latent_recurrence_enabled` 게이트(기본 False, 현행 ablation 전체 False)라 이번 캠페인엔 무해하나 **기본 승격 전 확인·벤치 필수**로 문서화(`docs/ETE_TRAIN_GPU_HOSTS.md`). **오늘 밤 범위는 준비까지 — 전환은 안 함.** 빌드 전후 학습 처리량 무변화 실측(77~97ms/batch → 61~97ms/batch).
- **기타**: `ete-net`/`risk-aware-sim` 모듈의 호스트 절대경로 마운트 결함을 `clone.sh` 패턴으로 전환(`828395f`, `8e4f4a8`) — im처럼 배포 스택을 안 가진 학습 전용 호스트에서도 성립하게. `DOCKER_BUILD_OPTS`(`8ae1089`) — im의 `--bridge=none` 전용 데몬에서 이미지 빌드 가능하게(`--network=host --allow network.host`). `setup.sh run` 환경변수 전달(`14952c9`) + `ETE_OUTPUT_DIR`(`a0e9a5d`) — trainq가 컨테이너 안에 `-e ETE_CONFIG/ETE_SEED/ETE_OUTPUT_DIR`을 전달할 수 있게. `build_wheel.sh`(`c25f071`) — LAPACK 없는 torch 휠이 나왔던 사고(BLAS 개발 패키지 누락→Eigen 폴백→jax `torch.linalg.qr`에서만 뒤늦게 발각) 이후 산문 재현 절차를 실행 스크립트+4중 검증(ABI/LAPACK/CPU qr 실행/CUDA 미번들/cuDNN NEEDED)으로 정착. `sim-x86` 단일 torch(ABI=1) 아키 전환(`018dd70`/`74f24ab`/`3554156`/`73b9966`/`1bf76e3`) — 호스트 스테이징·shim·libtorch 우회물 철거, 4중 게이트(voxblox→torch conv→spconv→jax→torch.compile) 통과. `DATA_ROOT` 정본화(`ef00972`, `cd2c602`) 및 경로 동적화 4건(`a66e85c`, `0e8fb51`, `0b7dbf6`, `7685daa`) — 하드코딩 호스트 경로(`~/risk-aware_planning`, `/media/im/...`, 5090 잔재 등) 제거.

---

## 7. ⚠ 승인 없이 내가 정한 것들

사용자가 아침에 재검토할 수 있게 한 목록으로 남긴다. 각 항목에 되돌리는 커밋을 붙인다.

1. **MAX_THRUST 15.60 → 16.535** — 실측 재교정(§5). 되돌리려면 `5737b86` revert. sim 전용 파일 2줄(정의/사용)에만 존재, 실기체는 mavros 경로라 무관함을 확인함.
2. **early stopping min_delta: 코드 기본값 1e-4 → config 키 1e-3** — 되돌리려면 `d62bd27` revert(patience:150 있는 config 10개 전부에 걸쳐 있음, 한 커밋에 코드+config 동봉).
3. **balanced_sampling alpha=0.5 / cap=10.0 (bin·가중치 재설계)** — 되돌려 "기능 자체 부재" 상태로 가려면 `ac9694d` revert("이 커밋만 revert" — 그 커밋 자체 명시). 게이트 통과·최초 arm 투입은 `4e4c158`.
4. **새 arm 5종 생성**: `DNoff_nosw`/`auxoff_nosw`(`e255a15`), `balsmp_nosw`/`DNoffAuxoff_nosw`(`4e4c158`), `SW_r27_long`(`7e2e32e`, 캠페인 승자를 600에폭 예산으로). (CHRr27/GNAUXw05r27/SW_r27/GNAUXw05r28는 전날 이전부터 있던 ablation 5종을 맵 수정판으로 재학습한 것 — 새로 만든 게 아님.)
5. **sim 샘플링/DONE 정렬**: `sampling_mode: spheric→uniform` + `min_path_length: 1.4→0.5`(`bd9ef98`), `max_density_range: 1.2→0.4`(`405cf9b`), DONE 래치 조건 `done_starved_cycles: 3` 신설(`bd9ef98`, requireParam으로 halt) — 전부 "실기체 값에 맞춘다"는 판단으로 내가 결정. 잔여 리스크(memory 기록): `done_starved_cycles`는 실기체 미검증, sim만 3으로 정렬됨.
6. **sim 이륙고도 3.0 → 1.2** (`8bd515a`) — 기하학적 불일치(§5) 판단으로 결정.
7. **balanced_sampling 게이트 재정의**: 절대치 "커버리지 ≥90%"(도달 불가능, §4/§6 "틀린 것" 참조) → "균일가중 상한(1-1/e≈63.21%) 대비 격차 ≤5%p" + 4번째 게이트(다중에폭 기아) 신설(`4e4c158`). 되돌리려면 `4e4c158` revert(단, 되돌리면 balsmp_nosw arm도 근거를 잃음).
8. **C7 attention halt 가드를 warning으로 강등** (`8826f5b`) — `3a6da22`가 넣은 hard halt가 배포 체크포인트 `DEPLOY1_s42`(동결 config가 정확히 그 "위험" 조합) 로드 자체를 막는 긴급 회귀였다. 재현 확인 후 이 조합만 loud-but-non-fatal로 강등, 다른 검증은 halt 유지. 되돌리려면 `8826f5b` revert(단, 되돌리면 배포 체크포인트 로드가 다시 깨짐).
9. **gt-mode 변환 yaml 매핑 수정** (`ed2c1c2`) — `/robot/odom` 매핑 제거, `/LIVO2/imu_propagate`를 `vio_odom`으로 승격. probe 티어는 학습에 안 쓰지만([[project_probe_tier_validation_only]]) 검증 데이터 의미론을 바꾸는 결정이라 명시. probe 자체 재생성은 급하지 않다고 판단해 보류함(현행 `stage2/probe` 2,507윈도우는 원본 그대로 정상).

---

## 8. 진행 중 / 다음 (본 보고서 작성 시각 기준, 2026-07-26 04:1x KST)

trainq 큐 실시간 상태(읽기 전용 확인, 새 잡 제출 안 함):

| 잡 | 상태 | 비고 |
|---|---|---|
| `DNoff_nosw_s1337` | running (~ep45) | seed 분산 확인용 복제(§4 한계 대응) |
| `SW_r27_long_s42` | running (~ep15/600) | 캠페인 승자를 600예산으로 — "120이 천장인가, 더 돌리면 느는가" 판정용(`7e2e32e`) |
| `SW_r27_s1337` | pending (큐 등록만) | 캠페인 승자 seed 복제 |
| `DNoffAuxoff_nosw_s42` | **done**(04:07 완주) | best holdout total=1.4307 — §4에 판정보류로 기재, `campaign_results.json`(57라벨)에 아직 미병합, `merge_results.py` 실행 필요 |

**다음 할 일 (메모리/커밋에 명시된 것)**:
- `DNoffAuxoff_nosw` 결과를 `merge_results.py`로 캠페인 표에 병합하고 에폭매칭으로 재판정.
- `SW_r27_long` 완주 후 "BatchNorm이 120에폭 이후에도 개선되는가" 판정.
- `balsmp_nosw`의 B=1 재측정 — groupnorm 계열이라 갭이 거의 없을 것으로 예상되나 미실측.
- **balsmp + batchnorm(SW_r27 방식) 조합 arm은 아직 없다** — 두 축이 각각 단독으로만 검증됨.
- `balanced_sampling` config 키는 여전히 선택 키(`.get(..., False)`)다. `min_delta` 선례(`d62bd27`)대로 필수키 승격은 게이트가 더 안정화된 뒤로 미룸.
- torch 통일: 실제 스택 전환(설치 분기 제거)은 캠페인 종료 후로 유보.
- 보류 항목(메모리 명시, 이번엔 손 안 댐): 모듈명 대칭화(`risk-aware-deploy`), fast-livo 브랜치 통합, overshoot(goal 종단 구속), sim 트리 세그먼트.

---

## 미확인 (자료에서 못 찾음)

- 이 세션의 "Task 목록 11개"의 원문 자체 — 서브에이전트는 오케스트레이터의 대화 맥락(TodoWrite 상태)에 접근할 수 없다. 위 §8/전체 진행상황은 git 커밋 이력·메모리 파일·trainq 큐 실측으로 재구성한 것이며, 오케스트레이터가 실제로 관리하던 11개 항목과 1:1로 대응한다는 보장은 없다.
- `balsmp_nosw`의 B=1 손실 실측값(§8에 "미실측"으로 명시, 새로 재는 것은 지시 위반이라 하지 않음).
- `SW_r27_long_s42`/`DNoff_nosw_s1337`/`SW_r27_s1337`의 최종 결과 — 작성 시각 기준 진행 중/대기 중이라 존재하지 않음.

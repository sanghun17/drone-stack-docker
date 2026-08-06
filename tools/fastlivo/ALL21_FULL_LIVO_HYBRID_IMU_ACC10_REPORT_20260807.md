# FAST-LIVO full-LIVO hybrid-IMU audit on all 21 flights

## 결론

후보 3에서 고정한 **동일한** full-LIVO 설정을 21개 세션의 stable-hover부터
landing 직전까지 1x로 재생했다. 현재 scorer 기준 결과는 다음과 같다.

- valid **21/21**, integrity **21/21**, catastrophic **0/21**
- motion/trend valid **21/21**; 정지하거나 주행 방향이 뒤집힌 세션 없음
- no-alignment translation RMSE: 평균 **0.610 m**, 중앙값 **0.527 m**,
  범위 **0.202--1.078 m**
- 동일 flight/window의 역사적 결과와 비교하면 RMSE가 **21/21 모두 감소**
  - 역사적 평균 6.438 m -> 현재 평균 0.610 m (aggregate mean 90.5% 감소)
  - 역사적 중앙값 5.630 m -> 현재 중앙값 0.527 m (90.6% 감소)
  - paired delta 평균 -5.828 m, 95% paired-bootstrap CI
    [-7.703, -4.020] m

이 결과는 후보 두 bag에만 우연히 맞춘 설정이 나머지 bag에서 즉시 무너지는
경우는 아니라는 강한 진단 증거다. 특히 과거 문제가 집중됐던 `pure_mean`
6개도 모두 0.47--1.08 m 범위로 들어왔다.

단, 이것을 **FCU acceleration 교체 하나의 인과효과로 해석하면 안 된다.**
아래 비교는 전체 파이프라인의 paired descriptive audit이다.

## 현재 코드에서의 D435-vs-hybrid controlled A/B

역사적 결과와 달리, 현재 코드·RGB·depth·extrinsic·covariance·feature gate와
stable-hover-to-landing 구간을 모두 고정하고 `common.imu_topic`만 바꾼 대조군도
21개 전부 재생했다. Hybrid arm은 D435 timestamp와 gyro를 유지하고 acceleration만
FCU 값을 보간·좌표변환한 것으로 교체한다.

- 두 arm 모두 valid 21/21, catastrophic 0/21
- D435 IMU RMSE 평균/중앙값: **0.861 / 0.711 m**
- Hybrid IMU RMSE 평균/중앙값: **0.610 / 0.527 m**
- paired mean delta (hybrid - D435): **-0.251 m**, bootstrap 95% CI
  **[-0.442, -0.080] m**
- hybrid가 개선된 세션은 **14/21**, 악화된 세션은 **7/21**
- `pure_mean`은 평균 0.952 -> 0.654 m, 5/6 개선
- `nominal`은 평균 0.632 -> 0.632 m로 실질적인 이득이 없었다

따라서 FCU acceleration 교체는 이 21개 replay에서 평균과 tail을 개선한
유의미한 요인이지만, 모든 세션을 단조롭게 개선하는 해법은 아니다. 특히 설정이
p1/pm4를 보고 post-hoc으로 선택됐으므로 p-value와 CI는 descriptive evidence이며
locked-test confirmation으로 보고하면 안 된다. 상세 session row, parameter parity,
플롯과 재생성 코드는 아래에 있다.

```text
tools/fastlivo/_campaign_20260805/summary/
  full_livo_hybrid_imu_acc10_hover_r1/paired_d435_control/
```

## 비교 설계와 중요한 제한

- 매칭 키: `flight_id` (21/21 one-to-one)
- 구간: 각 세션의 동일한 stable-hover-to-landing window
- 지표: `/aft_mapped_to_optitrack` 대 VRPN, spatial alignment 없음
- 현재 입력/설정: RGB + depth + IMU를 모두 사용하는
  `mock_candidate3_full_livo_hybrid_imu.yaml`, 1x replay
- 현재 FAST-LIVO: `ee166f508ecef3d87566edf5fb6fc25206febfe5`
- 역사적 FAST-LIVO: `904980fd78ee2d86f00c9260ddcb8828080c99fa`

역사적 결과는 당시의 project-specific `health_guard`를 포함한다. 현재 결과는
그 guard를 제거한 `0d96bcb` 이후 코드이고, startup propagation 수정, hybrid
acceleration, 현재 RGB/extrinsic/gating 및 covariance 설정도 함께 사용한다.
따라서 아래 delta에는 여러 변경이 결합되어 있다. 또한 현재 overlay는 p1과
pm4를 보고 post-hoc으로 선택했고 두 세션 모두 원래 validation split이므로,
이후의 development/validation 표시는 데이터 식별용일 뿐 locked-test 의미를
잃었다. 논문용 독립 검증이라고 주장해서는 안 된다.

## 조건별 paired 결과

| Condition | n | Historical RMSE mean / median / range (m) | Current RMSE mean / median / range (m) | Paired mean delta (m) | Improved |
|---|---:|---:|---:|---:|---:|
| pure_wodz | 4 | 2.178 / 1.739 / 1.252--3.983 | **0.533 / 0.529 / 0.202--0.871** | -1.646 | 4/4 |
| pure | 7 | 7.451 / 5.822 / 5.368--10.689 | **0.603 / 0.488 / 0.422--0.900** | -6.849 | 7/7 |
| pure_mean | 6 | 11.563 / 11.524 / 9.913--12.983 | **0.654 / 0.547 / 0.472--1.078** | -10.909 | 6/6 |
| nominal | 4 | 1.236 / 1.160 / 1.097--1.527 | **0.632 / 0.594 / 0.305--1.035** | -0.604 | 4/4 |

현재 RMSE 최저는 `pw4` 0.202 m이고 최고는 `pm3` 1.078 m이다. 절대
개선량이 가장 큰 세션은 `pm0` (-12.491 m), 가장 작은 세션은 `n0`
(-0.137 m)이다. 후자는 여전히 개선됐지만 11.7%에 그쳐, 모든 조건에서 같은
크기의 효과라고 볼 수는 없다.

## Coverage와 움직임 건전성

| Metric | Mean | Median | Min | Max |
|---|---:|---:|---:|---:|
| association coverage | 0.9950 | 0.9961 | 0.9785 | 1.0000 |
| estimated/GT path ratio | 1.5248 | 1.5150 | 1.3335 | 1.9341 |
| 1 s weighted direction cosine | 0.9077 | 0.9113 | 0.8746 | 0.9644 |
| progress correlation | 0.9878 | 0.9903 | 0.9626 | 0.9984 |
| reverse-distance fraction | 0.0073 | 0.0033 | 0.0000 | 0.0384 |
| stall-window fraction | 0.0022 | 0.0020 | 0.0000 | 0.0078 |

모든 세션의 path ratio가 validity limit 2.0 아래다. 최소 coverage 0.9785는
`pm1`, 최대 reverse fraction 0.0384는 `pw1`, 최대 stall fraction 0.0078은
`p1`이다. 즉 낮은 RMSE가 estimator 정지나 반대 방향 주행으로 만들어진 것은
아니다. 다만 path ratio 중앙값 1.515는 추정 경로 길이가 GT보다 여전히
상당히 길다는 뜻이므로 다음 개선 대상으로 남는다.

## 산출물

모든 재현 입력 hash, 공식, session row, condition aggregation과 fixed-seed
bootstrap 결과는 아래 JSON/CSV에 있다.

```text
tools/fastlivo/_campaign_20260805/summary/
  full_livo_hybrid_imu_acc10_hover_r1/paired_historical/
    paired_sessions.csv
    paired_conditions.csv
    paired_summary.json
    paired_rmse_by_session.png
    paired_rmse_scatter.png
    condition_rmse_comparison.png
    current_quality_diagnostics.png
    generate_paired_historical.py
```

재생성 명령:

```bash
python3 tools/fastlivo/_campaign_20260805/summary/full_livo_hybrid_imu_acc10_hover_r1/paired_historical/generate_paired_historical.py \
  --current tools/fastlivo/_campaign_20260805/runs/full_livo_hybrid_imu_acc10_hover_r1/results.json \
  --historical tools/fastlivo/_campaign_20260805/timeseries/production_primary/plots/session_summary_hover.csv \
  --overlay tools/fastlivo/mock_candidate3_full_livo_hybrid_imu.yaml \
  --output tools/fastlivo/_campaign_20260805/summary/full_livo_hybrid_imu_acc10_hover_r1/paired_historical
```

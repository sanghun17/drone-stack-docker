# FAST-LIVO 21-flight safety-mode campaign (2026-08-05)

## 결론

현재 production FAST-LIVO 설정은 선정된 실제 비행 21회 모두에서
catastrophic 발산을 막았다. 사전 정의한 기준은 translation APE가 전체
VRPN 이동 거리의 2배를 한 번이라도 넘는 경우이며 결과는 **0/21**이다.
출력 coverage와 gap도 정상 범위였다.

그러나 accuracy 목표는 완료되지 않았다. estimated/GT path ratio가 2를
넘는 secondary integrity 실패가 **10/21**이고, 전부 `pure` 4회와
`pure_mean` 6회에 집중됐다. 전체 no-alignment RMSE는 평균 6.748 m,
중앙값 5.928 m이다. 이것은 estimator가 죽거나 멈춘 결과가 아니라,
health guard가 catastrophic 폭주를 막는 동안 자세 오차와 추정 경로
길이가 누적된 결과다.

이번 캠페인에서는 production parameter를 변경하지 않았다. development
split에서 production과 LIO-only의 순위가 반복 재생 사이에 뒤집혔기
때문에, 사전등록한 surrender condition에 따라 추가 threshold search를
중단하고 기존 production을 고정했다. 현재 증거로 임의의 파라미터를
배포하는 것보다 이 결정이 안전하다.

## 데이터와 평가 규율

- 조건/횟수: `pure_wodz` 4, `pure` 7, `pure_mean` 6, `nominal` 4
- development 8 / locked validation 13은 estimator 결과를 보기 전에
  acquisition ordinal로 고정했다.
- 원본 bag은 읽기 전용이다. canonical bag은 compressed RGB를 point cloud와
  **header timestamp가 정확히 같은 frame만** raw BGR로 복원한다. 시간 이동,
  보간, pose/outcome 선택은 없다.
- 21개 모두 RGB, 10 Hz point cloud, IMU, CameraInfo, VRPN을 포함하고 exact
  RGB/cloud pair retention은 98.86--100%다.
- 점수는 `/aft_mapped_to_optitrack` 대 `/vrpn_client_node/pure/pose`이며
  spatial alignment는 하지 않는다. 일정한 초기 anchor 이후의 실제 drift를
  측정한다.
- pass가 estimator 조기 종료로 만들어지지 않도록 coverage >= 95%, 최대
  output gap <= 0.5 s, output/input >= 0.90을 확인한다.
- secondary integrity는 위 조건과 함께 estimated/GT associated path ratio가
  `[0.5, 2.0]`이어야 한다.

사전등록 원문은 `CAMPAIGN_20260805_PREREGISTRATION.md`, session catalog는
`campaign_20260805_sessions.json`이다.

## Frozen production 21회 결과

| 조건 | n | catastrophic | integrity 실패 | RMSE 평균 m | RMSE 중앙값 m | orientation RMSE 평균 deg | body-up RMSE 평균 deg |
|---|---:|---:|---:|---:|---:|---:|---:|
| pure_wodz | 4 | 0 | 0 | 2.227 | 1.725 | 27.71 | 4.78 |
| pure | 7 | 0 | 4 | 7.630 | 6.075 | 51.78 | 10.66 |
| pure_mean | 6 | 0 | 6 | 12.369 | 12.291 | 78.14 | 16.07 |
| nominal | 4 | 0 | 0 | 1.293 | 1.167 | 17.40 | 5.95 |

위 자세 수치는 mapped estimate와 VRPN quaternion의 geodesic error이고,
body-up은 yaw를 제거하고 두 body z-axis의 각도만 측정한다. 위치 RMSE와
orientation RMSE의 session-level Pearson correlation은 0.864, 위치 RMSE와
body-up RMSE는 0.907이다. 따라서 큰 위치 오차가 yaw만의 현상이 아니라
tilt 오차와 강하게 함께 발생한다.

반대로 VRPN에서 측정한 실제 기체 tilt p90의 조건별 평균은 4.18--5.45도에
불과하고 20도 초과 비율도 거의 0%다. health guard의 `bad_tilt`는 실제
기체가 과격하게 기울어서 생긴 정상 상태가 아니라 estimator state가
물리적으로 벗어난 신호다.

## 요청한 `pure < pure_mean < nominal` 순서

이 데이터와 공통 production parameter에서는 성립하지 않는다.

- 조건별 mean: `nominal 1.293 < pure 7.630 < pure_mean 12.369 m`
- 조건별 median도 동일한 방향이다.
- 20,000회 stratified bootstrap에서 요청 순서의 mean/median 확률은 모두
  0이었다.
- `pure × pure_mean × nominal` 개별 조합 168개 중 요청 순서를 만족한 것은
  0개다.
- 전체 평균 GT path와 duration으로 보정한 RMSE도 nominal 1.536,
  pure 7.455, pure_mean 12.534 m로 순서가 유지된다.

따라서 공통 estimator parameter로 요청 순서를 만들겠다는 목표는 현재
데이터에서는 지원되지 않는다. 조건별 parameter를 달리하거나 nominal을
의도적으로 악화시키면 숫자는 만들 수 있지만 estimator 개선이 아니며
허용하지 않았다. 네 조건은 같은 trajectory의 paired 반복이 아니므로 이
결과는 planner mode의 인과 효과가 아니라 session/trajectory와 연관된
관찰 결과다.

## 파라미터 후보와 중단 근거

8개 development session의 1차 결과:

| 후보 | catastrophic | integrity 실패 | RMSE 평균 m | RMSE 중앙값 m |
|---|---:|---:|---:|---:|
| production r1 | 0/8 | 3/8 | 5.707 | 4.806 |
| LIO-only r1 | 0/8 | 3/8 | 6.330 | 3.963 |
| LIO fallback 40 | 0/8 | 4/8 | 6.694 | 6.498 |
| LIO fallback 48 | 0/8 | 4/8 | 6.091 | 4.075 |

동률인 production/LIO-only를 반복했을 때 production median은 3.698 m,
LIO-only는 7.008 m가 되어 1차의 median 순위가 뒤집혔다. 두 설정 모두
반복에서 catastrophic 0, integrity 실패 2/8이었다. 이 replay
non-determinism 때문에 작은 candidate 차이를 실제 개선으로 승격할 수
없다. production과 LIO-only를 합친 각 16회에서도 production이 더 낫지만,
사전등록은 ranking reversal 시 tuning을 중단하도록 정했다.

또한 네 predeclared 후보 모두 development에서 nominal RMSE가 약
1.27--1.30 m로 가장 작았고 pure/pure_mean은 더 컸다. 후보를 바꿔도 요청한
mode 순서가 나타날 조짐은 없었다.

## 원인 localization

동일 production 설정으로 21회를 한 번 더 재생하며 각 sensor epoch의
fusion/health CSV를 저장했다. 진단 수치는 그 진단 재생에서 나온 estimate와
짝지었고, replay variance가 있으므로 frozen primary 점수와 섞지 않았다.

| 조건 | LIO 유효 feature median | LIO reject | bad motion | bad tilt | VIO feature median | VIO reject |
|---|---:|---:|---:|---:|---:|---:|
| pure_wodz | 695 | 1.7% | 0.4% | 0.3% | 66 | 18.9% |
| pure | 579 | 1.7% | 1.3% | 0.1% | 64 | 25.2% |
| pure_mean | 428 | 30.7% | 10.1% | 23.0% | 43 | 45.7% |
| nominal | 698 | 1.2% | 0.3% | 0.2% | 68 | 25.3% |

진단 재생의 session-level RMSE와 가장 강한 동반 지표는 다음과 같다.

- LIO 유효 feature median: r = -0.929
- estimator tilt p90: r = +0.899
- LIO minimum information / feature: r = -0.821
- bad-motion reject fraction: r = +0.780
- LIO rotation information ratio: r = -0.778
- 전체 LIO reject fraction: r = +0.725

중요한 구분은 raw point 수와 유효 LIO constraint 수다. 원본 입력 1 Hz
sampling 결과 `pure_mean`의 raw point median은 오히려 가장 많았다
(약 9,735; nominal 약 9,427). RGB 밝기/대비도 소실되지 않았다. 하지만
point-plane matching 뒤의 유효 LIO feature와 rotation information은 가장
약했다. 즉 센서 message가 빠지거나 depth point가 사라진 것이 아니라,
해당 관측 geometry가 pose constraint로 잘 변환되지 못했다.

현재 `vio.max_lio_features_for_fusion=50`은 LIO가 거의 붕괴한 순간에만 VIO
EKF correction을 허용한다. `pure_mean`의 LIO는 나쁘지만 median 약 428이라
이 gate를 대부분 통과하지 못하고, 동시에 VIO support도 약하다. 따라서
40/48/50 같은 작은 threshold 변경이나 LIO-only 전환이 이 중간 품질
영역을 해결하지 못한 것이 결과와 일치한다.

이 분석은 새 threshold를 바로 선택할 근거는 아니다. session-level
상관은 failure localization에는 강하지만, 각 frame에서 VIO correction이
LIO보다 실제로 나은지를 판별하는 per-frame causal gate는 아니다. 이전
캠페인에서도 일반적인 scalar VIO quality 지표는 helpful/harmful correction을
분리하지 못했다.

## 개선 판단과 다음 실험

1. **현재 production 유지.** 0/21 catastrophic 안전 목표는 달성했고,
   검증되지 않은 parameter 변경은 배포하지 않는다.
2. **재현성부터 고친다.** 같은 bag의 실시간 replay가 candidate ranking을
   바꿀 정도로 달라진다. ROS sensor callback/epoch ordering, OpenMP/Eigen
   numeric ordering, 초기 map insertion ordering을 하나씩 고정하고 동일 bag
   반복의 estimate hash 또는 RMSE 범위를 acceptance criterion으로 둬야 한다.
3. **medium-support degeneracy를 별도 상태로 취급한다.** `<=50` collapse와
   정상 LIO 사이에, feature 수는 수백 개지만 rotation/minimum information이
   약한 구간이 있다. 다음 development hypothesis는 count 하나가 아니라
   normalized information과 attitude consistency를 함께 사용해야 한다.
4. **VIO를 무조건 켜면 안 된다.** pure_mean에서 VIO support도 낮으므로
   threshold를 500처럼 올리는 것만으로는 안전이 증명되지 않는다. 새로운
   독립 development flight에서 per-frame GT benefit을 먼저 확인하고,
   도움이 되는 경우에만 continuous weighting/constrained update를 시험한다.
5. **locked 13개로 추가 튜닝하지 않는다.** 새 rule은 새 split 또는 새 비행
   데이터로 개발하고 이 13개는 최종 회귀 확인에만 사용한다.

## 재현 명령

```bash
# Canonical input 생성/검증
python3 tools/fastlivo/campaign_20260803.py \
  --root tools/fastlivo/_campaign_20260805 \
  --spec tools/fastlivo/campaign_20260805_sessions.json \
  prepare --group all

# Frozen production replay
python3 tools/fastlivo/campaign_20260803.py \
  --root tools/fastlivo/_campaign_20260805 \
  --spec tools/fastlivo/campaign_20260805_sessions.json \
  run --group all --tag production_recheck --rate 1.0

# 원본 RGB/cloud 품질 요약
python3 tools/fastlivo/campaign_20260803.py \
  --root tools/fastlivo/_campaign_20260805 \
  --spec tools/fastlivo/campaign_20260805_sessions.json \
  inspect-inputs --out-dir tools/fastlivo/_campaign_20260805/input_summary \
  --stride 10
```

ML x86에서는 `replay_fastlivo.sh`가 이미 실행 중인
`drone-stack-sim-x86`를 재사용하고 loopback ROS master를 사용한다. Jetson
또는 실제 비행 ROS master에는 연결하지 않는다.

## 생성 산출물과 해시

생성물은 약 9.5 GB이며 `.gitignore`의 `tools/fastlivo/_campaign_*/` 아래에
있다. 원본 bag은 수정하지 않았다.

- manifest: `_campaign_20260805/manifest.json`
  - SHA-256 `7f43ae5b0cd6bb3e6c61eeb802d4c2ec012c8e9fbb4b10811ff5eb12b3655f96`
- frozen primary rows: `_campaign_20260805/runs/production_primary/results.json`
  - SHA-256 `42bf0ae4599c00d9a71087acc6f466382eaa3657ffacac15f7bdc33aa47eea2a`
- aggregate summary/figure: `_campaign_20260805/summary/`
- paired fusion diagnostics: `_campaign_20260805/diagnostic_summary/`
- raw input quality: `_campaign_20260805/input_summary/`
- 21-session EVO PDF: `_campaign_20260805/evo/production_21_evo_report.pdf`
  - SHA-256 `7d0c1002447824cf170588851a44560f214924644be6e446506df306f6a1b748`

Production config SHA-256는
`5d3863f8546082089eab647f76b43ebf9344269fa8c567b7d258718a33579514`이다.
FAST-LIVO source commit은 `904980fd78ee2d86f00c9260ddcb8828080c99fa`이다.

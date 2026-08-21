# 전체 실비행 bag 기반 VIO 튜닝·비행 준비성 감사

작성일: 2026-08-14 (Asia/Seoul)

## 결론

현재 상태는 **legacy objective와 correction surrogate 기준 잠정 개발 1위 확인,
실비행 NO-GO**다.

- Phase-B의 20개 config×session 그룹에서 지상 정지 초기화, legacy 10 Hz
  post-LIO body trajectory, post-VIO correction stream이 세 fresh-process repeat에서
  exact였다. Phase-A는 arm×session당 1회라 반복결정성 증거에 포함하지 않는다.
- 현재 최선의 개발 후보는 다음과 같다.
  - `imu.acc_cov: 10.0`
  - `vio.img_point_cov: 1000.0`
  - `vio.outlier_threshold: 600.0`
- 하지만 60개 Phase-B primary readiness report가 모두 FAIL이다.
- 기존 200 Hz propagation payload는 20/20 config×session 그룹에서 세 반복이
  모두 서로 달랐다. 따라서 controller/planner용 고율 상태는 아직 사용할 수 없다.
- 정확도 ranking에 사용한 `/aft_mapped_to_body`는 같은 epoch의 VIO correction
  전 상태다. 최종 VIO posterior용 저율 topic이 없으므로 Phase-B 수치를 곧바로
  '최종 VIO 정확도'라고 부를 수 없다.
- FCU firmware/parameter/timesync/EKF2 외부비전 설정 receipt가 없고, 기존
  mocap 중심 mux/safety policy도 standalone VIO-local 비행과 호환되지 않는다.

`phase_b_report.json`의 `status=pass`는 **개발용 저율 반복결정성 protocol을
통과했다**는 뜻이다. `flight_ready=false`, `candidate_promotion_allowed=false`,
`global_high_rate_interface_remains_no_go=true`가 최종 비행 판정이다.

## 데이터와 실험 계약

- 개발용 실제 비행 세션 5개: `pw1`, `pm2`, `p0`, `n0`, `pw3`
- 각 세션은 full bag 시작부터 cached landing까지 재생했다.
- 최초 full-sync 지상 epoch 뒤 정확히 30개의 IMU를 사용했다.
- GT/mocap은 estimator에 보이지 않게 차단하고 평가에만 사용했다.
- Phase-A/B 모두 `validation_data_accessed=false`; validation split은 열지 않았다.
- primary는 full-start→landing safety report, secondary는 primary의 동일 고정
  alignment를 재사용한 hover ranking report다. secondary는 재정렬·재정합하지
  않는다.
- 동결 evaluator의 local accuracy source는 legacy `/aft_mapped_to_body`, 즉
  post-LIO/pre-VIO body pose다. VIO tuning 값은 다음 epoch의 prior/map을 통해 이
  stream에도 영향을 주지만, 같은 epoch의 최종 visual posterior는 아니다.
- Phase-B는 4개 설정 × 5개 세션 × 3개 fresh process = 60개 셀을 균형·순차
  실행했다.
- 결과: 60/60 완료, 성공 UUID 60개, 실패·재시도 0, 540개 artifact의 SHA/size
  검증 통과.

정본:

- orchestration identity:
  `25c985dc551a55555d28cb2c3cf9d6391381859207cc0cebab1bf7bcfc68f0f9`
- sequence receipt identity:
  `7c9f780b2a1d090c937419d00260bf98eb6f5f97b56babdfa616a712379df4f8`
- report identity:
  `548f23b20822adf3d4d56a50d18611ba130469b3b1553f96a81a1ff29a24e7de`
- report file SHA-256:
  `1560677834dd44c909e1a2a5687106f1e908d8db88333ba29115bb8121024a70`

## Phase-B 결과

동결된 순위 규칙은 reliability failure, 저율 repeat failure, worst normalized
session loss, mean loss, plan order 순이다. 낮을수록 좋다.

| 순위 | 설정 | worst | mean |
|---:|---|---:|---:|
| 1 | acc10 / img1000 / out600 | 6.504428 | 3.942702 |
| 2 | acc5 / img1000 / out1000 | 7.133930 | 4.696793 |
| 3 | acc10 / img1000 / out1000 | 7.915460 | 4.657592 |
| 4 | acc5 / img1000 / out600 | 13.176492 | 5.452022 |

최선 후보의 5-session 요약:

| 지표 | 평균 | 최악 |
|---|---:|---:|
| translation APE RMSE | 0.4380 m | 0.8450 m |
| 1 s translation RPE RMSE | 0.1334 m | 0.1543 m |
| orientation RMSE | 7.458 deg | 11.401 deg |
| path ratio | 1.3937 | 1.6504 |

기존 acc10/img1000/out1000 대비 outlier 600은 평균 APE, RPE, orientation,
path ratio와 worst normalized loss를 모두 개선했다. 반면 `acc5 + out600`
상호작용은 특히 `pw1`에서 크게 악화됐다. 두 factor의 단독 이득을 합산해
동시에 적용하면 안 된다.

Path ratio 과다의 성격을 확인하기 위해 최선 후보 r1의 동일 local/GT 궤적을
정규 시간격자로 보간해 sampling 간격만 바꿨다. 이 진단은 동결 ranking을
대체하지 않으며 원인을 분리하기 위한 것이다.

| 세션 | 동결 10 Hz 부근 ratio | 0.5 s 간격 | 2.0 s 간격 |
|---|---:|---:|---:|
| pw1 | 1.335 | 1.130 | 1.039 |
| pm2 | 1.328 | 1.110 | 1.015 |
| p0 | 1.650 | 1.284 | 1.109 |
| n0 | 1.343 | 1.131 | 0.980 |
| pw3 | 1.312 | 1.199 | 1.125 |

0.5 s/2.0 s 열은 local과 GT를 따로 재보간한 값이 아니라 동결 evaluator가 만든
동일 `AssociatedTrajectory` pair를 규칙적으로 subsample한 값이다. 간격을 늘리면
비율이 빠르게 1에 가까워지므로 단순 전역 scale보다 legacy 10 Hz pose의 고주파
variation과 일치한다. LIO/visual active-set 변화는 아직 원인 확정이 아니라 후속
log/A-B 가설이다.
따라서 다음 정확도 실험은 path ratio gate를 완화하는 대신 correction step,
같은 segment 내 연속성, visual residual/accepted-set 안정성을 별도 목표로 삼아야
한다. 특히 p0는 저주파 성분도 가장 많이 남아 추가 원인 분리가 필요하다.

### 저율 출력 계약 버그

소스 흐름을 다시 대조한 결과 `handleLIO()`는 LIO update 직후
`publish_body_optitrack()`을 호출하지만, 이어지는 같은-epoch `handleVIO()`는
visual update 뒤 `/aft_mapped_to_body`를 다시 publish하지 않는다. 따라서 이
topic은 최종 VIO posterior가 아니라 pre-VIO LIO posterior다.

같은 함수의 no-point guard도 `pcl_w_wait_pub->empty() || ptr==nullptr` 순서라서
포인터가 null이면 먼저 역참조하는 latent null-dereference가 있다. 현재 metric
원인으로 관측되지는 않았지만 null check를 먼저 하도록 별도 안전 수정과 단위시험이
필요하다.

Post-VIO 상태는 현재 `imu_rate_odom=true`일 때 correction queue를 통해
`/aft_mapped_to_body_correction_pose_cov`에 늦게 나타날 뿐이다. 이 topic도 고율
worker에 종속돼 있어 `imu_rate_odom=false` 개발 후보에서는 최종 posterior를
일반 소비자에게 제공하지 않는다. 기본 mux는 mocap이지만, mux를 `source=vio`로
전환하면 현재 MAVROS 경로가 `/aft_mapped_to_body`를 사용하므로 그 경우 FCU는
visual correction 이전 pose를 받는다.

현재 60개 Phase-B result bag/cell 안에서는 correction topic이 finite/PSD/unique이고,
각 bag의 초기화 correction 1개를 제외한 hover-window correction이 final VIO 또는
no-point/gated LIO fallback을 담은 terminal-state surrogate임을 확인했다.
동일 exact hover window와 primary yaw+translation을 그대로 재사용해 최선 후보
r1을 읽기 전용 재계산한 결과는 다음과 같다. scale fit, time shift, 재정합은 없다.

| 세션 | legacy APE / ori / path | correction APE / ori / path |
|---|---:|---:|
| pw1 | 0.845 m / 7.925° / 1.335 | 0.845 m / 7.931° / 1.278 |
| pm2 | 0.319 m / 5.710° / 1.328 | 0.316 m / 5.707° / 1.275 |
| p0 | 0.456 m / 11.401° / 1.650 | 0.452 m / 11.366° / 1.502 |
| n0 | 0.264 m / 3.433° / 1.343 | 0.265 m / 3.371° / 1.236 |
| pw3 | 0.306 m / 8.821° / 1.312 | 0.307 m / 8.859° / 1.288 |

Correction 기준 normalized loss의 평균/최악은 `3.277884 / 5.019263`이다
(legacy `3.942702 / 6.504428`). 네 설정을 모두 재계산해도
`acc10/img1000/out600`은 1위를 유지했지만 2·3위는 바뀌었다. 따라서 후보
선택은 잠정 유지할 수 있으나, 기존 accuracy objective와 수치를 final-VIO
qualification으로 사용할 수는 없다. Correction topic 역시 timer/queue/drop에
종속되고 stage/session identity가 없으므로 새 canonical topic의 대체물은 아니다.

향후에는 legacy topic을 바꾸지 말고, VIO-applied, gate-off, zero-valid, no-point,
pure LIO/LO/IMU-only의 모든 terminal branch에서 main estimator thread가 epoch당
정확히 한 번 발행하는 별도 final-posterior 저율 topic을
pose/covariance/source/session identity와 함께 추가해야 한다. Evaluator와 mux는 이
topic으로 재등록하고 Phase-A 40셀과 Phase-B 60셀 전체를 새 semantics에서
재baseline해야 한다. 기존 Phase-B 순위는 개발 방향 증거로만 보존하며 최종 VIO
성능 claim으로 승격하지 않는다.

이 문제의 severity는 estimator 자체의 즉시 발산 증거가 아니라
**qualification-blocking measurement-contract defect**다. Phase-A도 같은 legacy
local objective로 pruning했으므로 final topic 도입 뒤 Phase-A/Phase-B objective
dependent ranking을 모두 재baseline하고, 그 전에는 Phase-C나 validation으로
진행하지 않는다.

### 다음 정확도 가설

Final-posterior objective를 고친 뒤에만 다음 개발 A/B를 수행한다.

- 현재 초기화는 약 0.15 s의 30개 샘플에서 gyro mean을 계산하지만
  `imu.init_estimate_gyr_bias=false`라 bias를 0으로 둔다. 더 긴 known-disarmed
  stationarity receipt를 먼저 만들고 bias false/true만 비교한다.
- Hybrid bridge는 FCU base acceleration을 camera frame으로 회전하지만 camera-IMU
  lever-arm의 각가속도/구심가속도는 보정하지 않는다. 단일 authoritative rigid
  transform을 고정한 뒤 angular-rate 구간에서 lever-arm on/off residual을 비교한다.
- Visual pre-gate는 patch SSE threshold이고 EKF update는 선택된 pixel을 동일 가중해
  사용한다. Per-patch robust kernel/innovation gate가 없다. 먼저 visual-quality log로
  실패 frame을 stratify한 뒤 isolated Huber/innovation A/B를 수행한다. 특히
  `acc5+out600`의 pw1 악화는 interaction 신호이지 아직 원인 증명은 아니다.

## 코드·설정에서 확인하고 고친 문제

이번 작업에서 확인하고, 해당 campaign/runtime 계약에서 수정·검증한 주요 항목은
다음과 같다. 아직 공유 build에 적용하지 않은 항목은 아래 NO-GO 목록에 따로 둔다.

- replay rate에 따라 초기 IMU 3개가 유실되던 pre-LiDAR callback admission
- callback batching에 따라 달라지던 IMU 초기화 window
- explicit full-sync sensor-time anchor와 exact 30-sample provenance
- 초기화 overflow, duplicate/backward timestamp의 fail-closed 처리
- 초기 state와 first correction의 IEEE-754 fingerprint
- LIVO에서 같은 epoch의 LIO/VIO correction이 중복 적용되던 경로
- correction covariance의 중복 publication
- IMU FIFO/history/queue의 bounded 처리와 오류 counter
- low-rate pose/correction의 repeat determinism 검사
- evaluator의 local gap/freeze/frame/quaternion/covariance/GT isolation gate
- startup subscriber transport race를 exact anchor가 fail-fast로 검출하는 경로

## 아직 비행을 막는 문제

### 1. 고율 상태 인터페이스

기존 구현은 main estimator loop의 `spinOnce()` 뒤에서 IMU FIFO를 burst로
drain한다. sensor header는 약 200 Hz지만 실제 delivery는 burst이며 correction
적용 시점에 따라 이미 publish된 payload가 달라진다.

다음-build 전용 callback queue/worker를 격리 구현했지만 공유 소스에는 적용하지
않았다. 최신 45-file content archive는 base/result SHA와 mode 기준 review artifact
무결성만 GO이며, Stage 1 apply와 Stage 2/flight는 독립 감사 NO-GO다.

- shutdown과 correction dequeue가 원자적이지 않아 candidate가 APPLIED/SHUTDOWN
  outcome 없이 소실될 수 있음
- legacy output queue 10000(약 50 s), input/history 4096(약 20.5 s)로 stale backlog를
  허용하며 freshness budget에 맞춘 overload fail-close 시험이 없음
- Stage 2의 session/generation이 `MixTraj→traj_server→PositionCommand`까지 전달되지
  않고 active spline purge/heartbeat deadline이 완결되지 않음
- Stage 2의 `UNAPPLYABLE` 표기는 아직 verifier/deployer가 기계적으로 강제하지 않음
- full catkin/ARM/runtime/replay, live D435 clock/header sequence, 200 Hz deadline과
  covariance 수치 검증, SITL/HIL이 미실행

격리 archive 정본:

- `/tmp/fastlivo_highrate_worker_20260814.s9h2aM/PATCH_MANIFEST.json`
- SHA-256:
  `51d1d9a6fa9d68a50b6ffc84c434a6841def70163e6c99cb101d751a77a3bc2a`

stats/reset session ordering, typed aggregate, PSD/gyro-noise covariance와 terminal branch
denominator는 정적·단위시험에서 개선을 확인했다. 그러나 위 Stage 1 blocker를 닫고
결정론 stop-race/queue-overload 시험을 통과하기 전에는 archive를 적용하지 않는다.

### 2. Hybrid IMU의 live admission

선정 후보는 `/camera/imu_hybrid`를 사용한다. 이 topic은 D435의 stamp/gyro와
MAVROS FCU acceleration을 causal interpolation한 결과라서 두 clock/frame의
결속도 비행 계약의 일부다.

현재 bridge는 extrapolation을 하지 않고 rate/stale/queue 진단을 제공하는 장점이
있다. 5개 개발 세션의 원본 full bag과 derived hybrid bag을 전 메시지 대조한
결과, online causal matcher는 stamp/gyro를 정확히 보존하고 acceleration 및
covariance를 절대오차 `1e-12` 이내로 재현했다. 즉 수학적 변환과 causal
interpolation의 read-only diagnostic은 통과했다. 다만 별도 hash-bound receipt가
없으므로 이를 qualification evidence로 승격하지 않는다.

다만 다음 live admission 항목은 아직 비행용 fail-closed가 아니다.

- `d435i.launch`의 `enable_hybrid_imu` 기본값은 false이고 현재 표준 실행 경로가
  이를 자동으로 true로 결속하지 않는다. 후보 overlay만 로드하면 estimator는
  존재하지 않는 topic을 기다린다.
- FAST-LIVO 시작이 `/camera/imu_hybrid/ready`와 결속되지 않는다.
- 동일 D435 stamp가 strict duplicate로 거부되지 않는다.
- drop/overflow 뒤 1초 정상 스트림만 보이면 같은 estimator session에서 ready가
  다시 켜질 수 있다.
- FCU input frame 및 D435↔FCU clock-domain/timesync receipt가 없다.
- covariance의 unknown marker, symmetry, PSD를 admission gate로 검사하지 않는다.

Extrinsic을 다시 frame-aware하게 감사한 결과, `body_calib`은 color-optical→body,
hybrid/static TF는 depth-optical→base이므로 둘을 직접 비교한 값은 의미가 없었다.
`body<-color`와 frozen `color<-depth`(`Rcl/Pcl`)를 합성하면 hybrid matrix와의 차이는
회전 약 `8.4e-5 deg`, translation 약 `5.7e-4 mm`로 반올림 수준이다. 따라서 static
extrinsic 불일치 blocker는 취소한다. 다만 FCU accelerometer의 실제 sensing point와
`base_link` 원점 사이 lever arm, 그리고 angular/centripetal acceleration 보정의
물리·캘리브레이션 provenance는 여전히 없다.

따라서 live shadow에서는 ready와 diagnostics를 별도 prerequisite로 검증하고,
post-ready fault는 estimator session을 닫아야 한다. 이 계약 전에는 bag에서 만든
hybrid stream과 live hybrid stream을 동등하다고 볼 수 없다.

### 3. PX4/MAVROS/FCU 증거

현재 repository에는 실제 FCU firmware/hash와 전체 PX4 parameter receipt가 없다.
따라서 `EKF2_EV_CTRL`/legacy `EKF2_AID_MASK`, EV delay/noise/gate, height/yaw
source, position offsets, GPS policy와 offboard/RC/geofence failsafe를 결정할 수 없다.

현재 FCU 경로도 10 Hz `PoseStamped`를 MAVROS vision_pose로 보내며 frame_id를
변환 계약으로 사용하지 않고 covariance/velocity/reset identity를 보내지 않는다.
이 경로를 30–50 Hz 외부비전 입력으로 간주하면 안 된다.

새 read-only FCU receipt와 subscriber-only shadow tool의 fail-closed 코드/receipt
계약은 독립 감사 GO다. 전체 `66/66` 테스트와 adversarial empty/malformed/rollback
probe를 통과했고, bundle manifest는 package 21개, exact role 8개, external 20개와
MAVROS source tree 93개를 결속한다.

- `ws/flight-safety/src/flight_safety/config/fcu_shadow_bundle_manifest.json`
- SHA-256:
  `5d47e0d537632ad489342d4872b74c0f620961c67950eb250c325978fd3a6340`

다만 stock 배포에는 qualified timesync accepted-sequence publisher와 estimator/hybrid
typed producer identity/source/binary/build가 없다. 기본 profile의 해당 path도 비어 있어
실제 receipt는 의도대로 INCOMPLETE/FAIL이다. 실제 disarmed/on-ground FCU에서 새
manifest와 receipt PASS를 얻기 전에는 비행 qualification NO-GO다. 이번 작업에서는
ROS master/FCU를 호출하지 않았다. Force pull은 FCU persistent parameter를 쓰지는
않지만 MAVROS ROS parameter cache를 갱신하므로 그 부작용도 receipt에 기록한다.

### 4. 평가 계약

현재 evaluator는 replay bag의 `bag record time - sensor header`를 계산해 여전히
strict latency gate에 사용한다. replay `/clock`, callback scheduling, recorder callback
순서가 섞인 값이므로 live flight latency를 증명할 수 없다. Correction/reset step은
검출하지만 pose-gradient twist consistency 계산에서 해당 interval을 아직 mask하지
않고, angular consistency도 interval의 right-end IMU가 아니라 left sample과 비교해
off-by-one이다. 따라서 개별 primary의 propagated age/twist FAIL을 정량적인 live
latency 또는 연속구간 kinematics 판정으로 해석하면 안 된다. 저율 정확도 초과와 고율
payload 비결정성이라는 별도 NO-GO 근거는 이 evaluator 결함과 무관하게 유지된다.

### 5. mux와 safety policy

- estimator mux 기본은 mocap/mocap이다.
- VIO-only일 때도 VRPN geofence/consistency 입력을 요구하는 policy가 있다.
- planner의 VIO-local frame과 PX4 local frame의 고정 transform/원점 receipt가 없다.
- `/robot/odom_planning`을 소비하는 global planner/voxblox도 typed high-rate gate를
  통과하지 않는다.
- MAVROS `local_position/tf/send=true`와 FAST-LIVO의 단일 TF authority 주석이
  충돌한다.

## 개발용 후보 파일

`tools/fastlivo/vio_ground_phase_b_candidate_development.yaml`

- SHA-256:
  `8c651f57273ce1d3a8a5f1b5d5cf057cbf578f08491e34b09bfb74963b625310`

이 overlay는 다음 원칙으로 만들었다.

- Phase-B에서 선택된 tuning factor 값을 정확히 고정
- live explicit anchor는 비우고, estimator 시작 전 disarmed/stationary 외부 admission을
  요구(현재 estimator가 armed/ground state를 machine-enforce하지 않음)
- ground qualification과 같은 엄격한 init gate 사용
- mocap anchor와 runtime reinit 비활성화
- 미검증 200 Hz output은 비활성화

현재는 replay/new-bag overlay 전용이다. Base→candidate→static camera 파일의 SHA와
namespace load를 검증하는 별도 shadow launcher 전에는 live estimator에도 사용할 수
없으며 flight launch에는 넣으면 안 된다. `imu_rate_odom=false`에서는 correction
surrogate도 생성되지 않으므로 새 final-posterior topic 전에는 final-VIO
qualification용 output source가 아니다.

현재 shared `config/d435i.yaml`과의 중요한 차이는 다음과 같다. 즉, 선택된
세 숫자만 기존 live 파일에 복사하면 Phase-B에서 검증한 estimator가 되지 않는다.

| ROS parameter | 현재 live | 개발 후보 |
|---|---:|---:|
| `common.imu_topic` | `/camera/imu` | `/camera/imu_hybrid` |
| `common.online_intrinsics_en` | true | false |
| `imu.acc_cov` | 0.1 | 10.0 |
| `imu.init_max_gyr_mean` | 0.30 | 0.01 |
| `imu.init_max_gyr_std` | 0.25 | 0.04 |
| `imu.init_max_acc_std` | 1.50 | 0.25 |
| `imu.init_acc_norm_tolerance` | 3.00 | 0.10 |
| `vio.max_lio_features_for_fusion` | 50 | -1 |
| `vio.img_point_cov` | 100 | 1000 |
| `vio.outlier_threshold` | 1000 | 600 |
| `mocap.anchor_enable` | true | false |
| `uav.imu_rate_odom` | true | false |

이 차이는 의도적으로 shared live 설정에 적용하지 않았다. 신규 build의 저율 parity,
고율 interface, VIO-only mux/safety를 먼저 통과한 뒤 하나의 hash-bound flight profile로
승격해야 한다.

특히 Phase-B는 static intrinsics(`online_intrinsics_en=false`)로 실행됐으므로 후보도
그 값을 명시적으로 고정한다. 이 overlay는 단독 launch가 아니며 frozen
`camera_d435i.yaml`(SHA-256
`99c13f2cb2fa7cbcfa744138814f52c73ff20ee230ec065e1bf117767f0d2492`)을
`mapping_d435i_replay.launch`처럼 `/laserMapping` private
namespace에 함께 load해야 한다. 그렇지 않으면 표준 live launch에서 camera model이
없는 채 시작할 수 있다. Live CameraInfo를 쓰는 `true` profile은 D435 serial, topic,
640×480 mode, distortion model과 `fx/fy/cx/cy`를 receipt로 결속한 별도 shadow
변형이어야 하며, effective intrinsics가 달라지면 같은 objective를 새로 rebaseline한다.

## 다음 승인 순서

1. Additive final-posterior 저율 topic/null guard와 evaluator terminal-epoch contract
2. 공유 source에 patch를 적용한 새 build 생성
3. legacy topic의 old/new canonical bit parity 및 새 final topic exactly-once 검증
4. final-posterior objective로 Phase-A/Phase-B 개발 grid 재baseline
5. 격리 high-rate worker blocker 수정과 unit/static/integration test
6. 새 worker의 callback-stall, correction-dropout, invalid IMU, reset/session,
   overflow/history-miss integration tests
7. bag 0.5×/1× 반복 및 full-session high-rate typed status/correction evaluation
8. live disarmed clock/latency shadow: consumer callback-to-use latency와 exact
   pose+twist+status atomic coverage
9. read-only FCU firmware/parameter/timesync/EKF receipt
10. standalone VIO mux/safety profile와 VIO↔PX4-local transform receipt
11. props-off axis/setpoint/failsafe test
12. tethered low-speed hover, 즉시 RC takeover/kill 조건

1–10의 통과는 필요조건일 뿐 arming 권한이 아니다. 11과 12는 각각 별도 위험 검토와
명시적 승인 아래 수행해야 하며, 이 보고서는 어떤 arming/free-flight authority도
부여하지 않는다.

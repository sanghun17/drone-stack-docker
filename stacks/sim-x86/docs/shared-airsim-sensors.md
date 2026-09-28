# AirSim 센서 공통 구조 점검 (2026-09-28)

## 이력과 결론

RGB·Depth 좌표 불일치는 최근 리팩터링에서 새로 생긴 값이 아니다.
Risk 원본 launch의 `base_link -> camera_left_optical_frame` 위치
`(.25, .5, .25)`와 RGB/Depth 동일 위치 TF는 2025-09-08 `11a01de`부터
존재한다. 보관된 `settings_260203.json`에도 RGB의 FRD 위치는
`(.25, .15, -.25)`, Depth는 `(.25, 0, -.25)`이다. 현재 실행 서버 설정과
직접 측정한 camera/body 상대 위치도 이 값에 해당한다.

2026-09-14 `7980fae` 리팩터링은 `simulation/airsim` 공통 클라이언트와
설정 기반 카메라 진입점을 만들었지만, Risk의 RGB+Depth 실행은 기존
`risk_aware_planning` 발행기를 계속 사용했다. ArUco는 같은 공통 진입점에서
스택이 지정한 C++/mmap 구현을 선택했다. **전체 센서 실행이 하나로 통합된
상태는 아니었다.**

직전 GT 진단에서 추가한 Risk 원본의 선택적 좌표/Depth 수정은 공통 모듈이나
ArUco를 변경하지 않았다. 다만 공통 구조를 활용하지 못한 수정이므로,
이번에는 공통 모듈로 기능을 옮기고 해당 원본 변경은 `63f74af`에서 되돌렸다.
과거 진단의 원본 커밋 `7c000a4`와 기록은 그대로 보존한다.

## 현재 소유 관계

| 기능 | 소유 위치 / 실행 경로 |
| --- | --- |
| AirSim client, 설정 기반 카메라 진입점 | `simulation/airsim`, 원본 `drone-runtime-modules` |
| 기본 Scene 발행, RGB·Depth 다중 스트림 | 위 공통 모듈의 Python 구현 |
| 카메라 내부 파라미터·FRD/FLU/optical 변환 | 위 공통 모듈의 `camera_geometry.py` |
| Risk 카메라 종류·장착 위치·해상도·ROS 이름 | `stacks/sim-x86/config/airsim_cameras.yaml` |
| Risk GT relay 및 ROS Depth nodelet 조합 | `config/launch/airsim_sensor_pipeline.launch` |
| ArUco 60 Hz C++/mmap 구현·전용 UE 플러그인 | 기존 ArUco 스택의 구현 및 패치, 변경 없음 |
| ArUco 카메라 프로필 | 기존 `landing_camera.yaml`, 변경 없음 |

두 체크아웃은 같은 공통 모듈 revision을 사용한다. `run_camera.sh`의 기존
`AIRSIM_CAMERA_BRIDGE_BIN` 선택 방식도 유지한다. 맵, 차량 배치, 장착 좌표,
렌더링 정책은 플랫폼별 프로필이어야 하며 서로 같은 숫자로 통일하지 않는다.
ArUco mmap 플러그인은 Scene byte 이미지용이다. 현재 Risk 패키지의 별도
DepthPlanar float 카메라에 이 mmap 프로토콜을 그대로 적용하지 않는다.

## 반영한 공통 기능

- 하나의 요청으로 여러 카메라를 취득하되 **각 응답의 실제 타임스탬프**를
  보존한다. 이전 Risk 발행기는 Depth 시각을 RGB에도 복사했다. 실측 응답은
  약 수 ms 차이가 있어 동일 시각으로 가정하면 안 된다.
- RGB와 Depth의 CameraInfo 및 optical frame을 구분한다. 이동된 두 카메라를
  등록된 RGBD로 간주하지 않고 Depth 자체 K와 TF로 XYZ를 투영한다.
- 카메라 장착값·해상도·FOV를 실행 중인 `getSettingsString()`과 대조한다.
  값이 다르면 발행 전에 실패한다.
- 병렬 RPC와 최대 한 배치 버퍼, 증가하는 캡처 시각만 발행하는 정책으로
  오래된 요청이 늦게 끝나도 시간 순서가 뒤집히지 않게 한다.
- ArUco와 같은 렌더링 명령 설정 기능을 다중 스트림에도 제공한다. Risk에서는
  motion blur/DoF와 temporal AA를 제거하되 맵 텍스처 품질은 낮추지 않는다.
- 선택적 `capture_pose_ned` 토픽으로 영상 응답의 원본 카메라 위치·자세를
  같은 시각에 보관한다. AirSim local NED/카메라 FRD의 진단 자료이며,
  제어 odometry나 optical TF로 사용하지 않는다.
- 실험마다 카메라 프로필, 실행 조합, 모듈 lock, CameraInfo, TF, 캡처 자세와
  발행 통계를 기록한다. 모듈 원본 수정은 소유 저장소에 commit/push 후 sync했다.
- 공통 실행기의 종료 패턴이 기존 `airsim_landing_camera_bridge` 이름도
  처리하도록 수정했다. 기존 패턴은 Python 이름만 일치시켜 ArUco C++ 프로세스가
  남을 수 있었다. C++ 영상 처리 구현이나 mmap 프로토콜은 변경하지 않았다.

## 실행과 재현

일반 Risk sensor 진입점 `run_sensor_pub.sh`는 공통 발행기를 사용한다.
과거 비교 실행의 기본 `--sensor-calibration historical`은 기록 재현을 위해
기존 코드를 사용한다. 새 GT 진단은 아래 옵션을 명시한다.

```bash
bash stacks/sim-x86/scripts/run_experiments.sh \
  --planner rhem --iterations 1 --planning-source gt --control-source gt \
  --sensor-calibration airsim --control-max-thrust 16.535 \
  --rhem-gt-conservative --rhem-belief-mode rovio --rhem-diagnostics \
  --time-limit 90 --output /work/flight_logs/rhem-shared-NEW
```

`historical`을 선택하는 것만으로 UE console 설정이 되돌아가지는 않는다.
렌더링 명령은 엔진 프로세스 동안 유지되므로 과거 조건 비교 전에 Unreal을
원래 settings 파일로 재시작해야 한다. 새 설정은 현재 GT 경로에서 검증했으며,
FAST-LIVO의 VIO 성능이나 원래 논문 비교 조건과 동등하다고 주장하지 않는다.

## 검증 자료와 한계

`data/results/sim-sensor-audit-20260928/`에 입력 통계와 GT/ROVIO audit를,
`flight_logs/rhem-shared-{sensors,render,cold}-20260928-rovio/`에 원본을 보관한다.
전방/하향 optical 축, FRD→FLU 장착 위치, 비중앙 K, RGB/Depth 개별 시각,
float depth 단위와 잘못된 프로필 거부를 검사했다. ArUco의 프로필, C++ 소스,
플러그인과 선택 진입점이 그대로인 것도 확인했다. 이번 점검에서 ArUco의 실제
맵을 다시 띄워 60 Hz 비행 성능을 재측정한 것은 아니다.

ROVIO의 JPL quaternion은 Eigen/ROS와 기호를 직관적으로 비교하면 안 된다.
실제 사용 중인 kindr로 전방 X→optical Z, 좌측 Y→optical -X,
위쪽 Z→optical -Y 변환을 C++로 확인했으며 이 회전 설정은 뒤집지 않았다.

입력 개선과 ROVIO 안정화는 별도로 평가한다. 관측된 프레임 간격 개선만으로
ROVIO가 정상화됐다고 간주하지 않는다. 카메라 품질 변경 및 엔진 재시작의
결과는 결과 폴더의 최종 요약과 함께 확인한다.

## 최종 측정

| 카메라 경로 | RGB Hz | 간격 p95 | 간격 p99 |
| --- | ---: | ---: | ---: |
| 이전 RGB+Depth 발행기 | 9.64 | 711 ms | 787 ms |
| 공통 병렬 RPC | 17.34 | 93 ms | 716 ms |
| 공통 RPC + 렌더 설정 | 17.72 | 94 ms | 722 ms |
| 위 설정 + Unreal/ROS bridge 재시작 | 17.69 | 92 ms | 725 ms |

서로 다른 경로를 비행한 진단 결과이므로 엄밀한 알고리즘 A/B 성능 비교는
아니다. 마지막 세 GT+ROVIO 실행은 각각 90초 시간 제한까지 충돌 없이
종료됐으며 cleanup 오류는 없었다. 그러나 raw ROVIO의 자체 좌표계 내 최대
변위는 각각 803m, 925m, 440m였고, 대응 GT 최대 변위는 3.23m, 2.29m,
2.18m였다. **영상 발행 개선으로 ROVIO가 안정화되지는 않았다.**

마지막 bag에는 각 카메라의 영상/CameraInfo/캡처 자세 1,732개씩을 기록했다.
기존 recorder는 녹화 시작 전 들어온 latched `/tf_static`을 버리고 있었으며,
토픽 목록에 추가하는 것만으로는 저장되지 않는 문제도 발견했다. 시작 전의
각 publisher별 최신 static TF를 원본 시각·내용 그대로 보관하는 수정과
회귀 테스트를 추가했다. 이 세 bag을 소급 수정하지는 않았으며 당시 TF는
보관한 카메라 프로필과 소스 revision으로 복원할 수 있다.
영상 시각은 역행하지 않았지만 최대 캡처 간격은 845ms였다. GT 각속도와
IMU 자이로의 RMSE는 축별 약 .0074/.0049/.0019 rad/s로 작았다. 반면
렌더된 자세를 차분한 값은 다르게 나타났으며, AirSim 소스에서도 차량
렌더 pose와 물리 kinematics가 별도로 갱신됨을 확인했다. 이것만으로 발산의
단일 원인이라고 단정하지 않는다.

캡처 응답 자세의 프레임 간 회전과 IMU 적분의 차이는 중앙값 .12°, p95
2.78°, RMSE 3.27°였다. -200~+100ms 시간 이동을 시험해도 최선 RMSE는
3.17°로 변화가 작았다. 따라서 임의의 타임스탬프 보정은 적용하지 않았다.
남은 점검 대상은 패키지 내부 캡처/물리 상태의 시간 정합과 ROVIO 필터의
입력·초기화·잡음 모델이다. 공통 센서 구조 점검 결과를 ROVIO 해결 완료나
논문용 RHEM 성능 검증으로 해석하면 안 된다.

공통 모듈 최종 revision은 `b48b604`이며 두 프로젝트 lock에 동일하게
반영했다. 소유 저장소의 `fix/shared-airsim-streams-20260928` 브랜치에
commit/push했다. Risk 원본은 `63f74af`를 가리키고, 카메라 관련 소스는
기존 상태로 복원되어 있다. 스택 실행 조합·분석 스크립트와 lock 변경은
각 프로젝트 체크아웃의 작업 트리에 남겨 두었다.

## 후속 ROVIO 진단에서 확인한 촬영 시각 오류

위 점검에서 응답의 `time_stamp`를 캡처 시각으로 부른 것은 정확하지 않았다.
기존 AirSim은 GPU readback 완료 시각을 넣고 있었다. 자세와 영상의 물리
상태는 그보다 중앙값 31ms, 일부 약 0.7초 이전 상태였다. 타임스탬프가
단조 증가하고 영상 Hz가 개선되는 것만으로는 이 불일치를 발견할 수 없다.

후속 공통 모듈 `2b88686`는 동기 캡처에서 렌더링한 물리 상태의 시각을 응답까지 전달한다.
두 카메라가 같은 물리 프레임을 촬영하면 같은 시각을 가지는 것이 정상이며,
이전 응답 간 수 ms 차이는 readback 순서에서 생긴 차이였다. ArUco 패치와의
텍스트 병합은 가능했으나, 비동기 GPU 버퍼가 이전 프레임을 반환하는 동안
현재 자세/시각을 붙이면 잘못된 측정이 된다. 해당 경로는 버퍼별 촬영 메타데이터가
필요하므로 적용 도구가 변경 전에 거부한다. 기존 ArUco 고성능 경로·바이너리는
그대로이며, ArUco에서 촬영 시각이 검증됐다는 뜻은 아니다.

ROVIO 초기 공분산·예측 잡음 복원도 함께 필요했다. 상세 실험, 실제 실행
결과와 현재 실행법은 [ROVIO_DIVERGENCE.md](../../../data/analysis/risk-aware/planner-runtime/ROVIO_DIVERGENCE.md)에
기록했다. `run_host_sim.sh unreal`은 이제 별도 빌드한 수정 엔진을 사용하며,
과거 패키지 재현은 `unreal-historical`을 명시한다.

## 선택적 공통 비동기 readback (후속 미션 진단)

Risk lock의 공통 모듈 `38b27c4`는 RGB BGRA8와 Depth RGBA16F GPU 버퍼마다
촬영 시각·자세를 보관하는 `capture_slot_readback.patch`를 추가한다.
빌더의 `--async-readback`과 엔진의 `-AirSimAsyncCapture`를 모두 명시해야
사용된다. 기본 동기 경로도 유지한다. 기존 ArUco async 패치의 자동 병합은
계속 거부하며, ArUco 체크아웃·전용 바이너리는 이 후속 작업에서 변경하지
않았다. 따라서 위의 '두 체크아웃 동일 revision'은 이전 점검 시점의 기록이다.

Risk 렌더 프로필은 전용 GPU의 offscreen 렌더링, RPC worker 1개와
`r.Vulkan.FlushOnMapStaging=0`을 사용한다. 후자는 완료된 GPU fence 뒤에
중복된 장치 전체 대기를 피한다. 공통 모듈 자체가 렌더링 정책을 강제하지 않는다.

140초 이동 입력 검사에서 RGB/Depth 3,792프레임을 확인했다. 캡처 자세와
영상 optical flow 잔차 중앙값은 0.238px, 카메라 회전과 IMU 적분의 RMS는
0.00213도이며 시간 이동 0에서 가장 잘 맞았다. 영상 간격 최대 0.687초의
stalls는 아직 남아 있다. 장시간 탐색과 raw ROVIO 오차 결과는
[RHEM_MISSION.md](../../../data/analysis/risk-aware/planner-runtime/RHEM_MISSION.md)에
별도로 기록한다. 새 공통 backend의 검증을 ArUco의 비행 성능 검증으로
해석하면 안 된다.

640×480 RGB 실험도 같은 다중 스트림 발행기에 명시적 프로필을 전달한다.
Depth 해상도는 320×240으로 유지하며, RHEM과 GT용 FAST-LIVO 진단의 K는
실제 CameraInfo에서 생성한다. `SIM_AIRSIM_SETTINGS`는 호스트 엔진/bridge의
설정 파일을 선택하고, `--airsim-camera-profile`은 대응하는 공통 발행기 YAML을
선택한다. 기본 YAML과 원본 simulator JSON은 320×240 조건을 유지한다.
이 선택 기능은 별도 Risk 영상 발행기나 ArUco 구현 변경을 만들지 않는다.

최종 GT/AirSim FAST-LIVO 실행은 별도 K 변환 도구 대신 필터에 이미 있는
CameraInfo one-shot 로더를 사용한다. 새 노드가 ROS 시각 0에서 유한 시간
대기를 시작하면 첫 절대 `/clock` 수신으로 즉시 timeout이 발생하는 문제를
실행 로그에서 확인했다. 이 명시적 모드에서는 기존 로더의 timeout을 0으로
설정해 CameraInfo를 기다리며, 센서 시작·실패 감시는 스택에서 담당한다.
실제 cold start에서 640×480 K를 수신한 것까지 검증했다.

GT 탐색 3시간 진단은 미션 성공 없이 종료했다. 촬영 시각 변경이 FAST-LIVO
정확도에도 도움이 되는지는 같은 영상·점군·IMU의 헤더 시각만 바꾸는 비교로
분리해 측정하며, 결과는
[FAST_LIVO_TIMING.md](../../../data/analysis/risk-aware/planner-runtime/FAST_LIVO_TIMING.md)에
기록한다. 입력 정합 개선 수치를 필터 위치 정확도 개선으로 대신하지 않는다.

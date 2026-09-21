# ArUco Unreal 맵 작업 맥락 복원

2026-09-18에 사용자의 요청으로 로컬 세션 원문과 현재 소스를 대조했다.
과거 실행 결과는 당시 기록이며, 이번 복원 작업에서 엔진이나 시뮬레이터를 실행하지는 않았다.

## 원본 세션

- 세션 ID: `01a075e2-082c-7ad1-8ead-d1433700e054`
- 시작일: 2026-09-06. 맵 생성 및 환경 스크린샷 작업일: **2026-09-07 (KST)**.
- 원문: `/home/ml/.codex/sessions/2026/09/06/rollout-2026-09-06T17-42-22-01a075e2-082c-7ad1-8ead-d1433700e054.jsonl`
- 주요 JSONL 행: 1412(사용자의 맵 요청), 1739/1753(엔진 불일치와 2,158개 빌드),
  2294(엔진 복구 확인), 2352(헤드리스 스폰 크래시), 2433(맵 생성 성공),
  2535/2571(736개 게임 타깃 빌드 및 재사용 범위), 2792(수정 후 4개만 증분 빌드),
  2835(AirSim 콘텐츠 cook 누락), 4830(환경 캡처 완료), 4996(PNG 영구 경로 이동).
- 후속 모듈화 세션: `01a09dfd-8feb-7bb2-a51d-54e520741275` (2026-09-14).
  현재 경로는 당시 최초 작업 경로와 다르므로 아래 현재 경로를 사용한다.

## 오래 걸렸던 실제 이유

1. 당시 `UE4Editor` 실행 파일은 2025년 빌드이고 핵심 `.so` 일부는 2026-08-31 빌드였다.
   바이너리 불일치로 undefined-symbol 오류가 발생했다. 기존 엔진 소스 수정은 유지하고
   `UE4Editor Linux Development`를 다시 빌드/링크했으며, 총 **2,158개 액션**이 잡혔다.
   원문에는 20-way parallel 빌드와 이후 오류 해소가 기록되어 있다.
2. Linux 독립 실행 패키지를 처음 만들면서 셰이더 컴파일러를 빌드하고,
   에디터와 별개의 게임 타깃 런타임/AirSim **736개 액션**을 처음 컴파일했다.
3. 최초 DDC/asset 초기화도 있었지만, 단순한 셰이더 캐시 변경만으로 설명할 일이 아니었다.
4. 프로젝트 헬퍼 수정 뒤에는 736개 전체가 아니라 4개만 증분 빌드된 기록이 있다.

현재 `Intermediate`, `Binaries`, `Saved`, `DerivedDataCache`와 패키지 실행 스크립트는
존재한다. 캐시의 완전성이나 현재 엔진 실행 가능성까지 이번에 검증한 것은 아니다.
새 맵 작업에서 기존 엔진/툴체인/빌드 산출물을 보존하고, 먼저 실행 가능성을 확인한다.
새 맵을 만든다는 이유만으로 엔진 전체 재빌드를 시작할 필요는 없다.

## 기존 환경과 현재 파일

- UE4/AirSim은 **ml 호스트**에서 실행한다. ROS bridge/estimator/controller는 Docker 쪽이다.
  착륙 시뮬레이션 제어는 AirSim API를 사용하며 MAVROS를 사용하지 않는다.
- 엔진 기본 경로: `/home/ml/UnrealEngine`.
- AirSim 원본: `config/sim.env`의 `AIRSIM_ROOT`.
- 최초 환경은 0.8 m 패드와 8×8 m 회색 바닥, 벽/주변 물체 없는 공간이었다.
  현재 환경 설정은 **0.70 m 패드, 7×7 m 바닥, 중앙 고도 2 m 시작**이다.
- 자산 루트: `stacks/aruco-landing-sim-x86/`.
  과거의 `modules/simulation/airsim-landing/` 명령을 그대로 사용하지 않는다.
- UE 프로젝트: `stacks/aruco-landing-sim-x86/unreal/ArucoLandingBaseline/`.
- 생성 맵: 프로젝트 아래 `Content/Maps/BaselineMap.umap`.
- 맵 생성기: 프로젝트 아래 `Scripts/generate_baseline_map.py`.
- C++ 헬퍼: 프로젝트 아래 `Source/ArucoLandingBaseline/BaselineEditorTools.{h,cpp}`.
- 환경/외부 카메라: 자산 루트 아래 `config/baseline_environment.yaml`.
- 하향 카메라: 자산 루트 아래 `config/landing_camera.yaml`.
- 재사용 가능한 공통 AirSim 코드: `modules/simulation/airsim/`.
- 생성 설정: `.build/aruco-landing-sim-x86/baseline/settings.json`.
- 독립 실행 패키지: `~/drone-data/aruco/assets/simulator/package/LinuxNoEditor/`.

## 다시 밟지 않을 구현상 함정

- UE4.27 헤드리스 commandlet에서 `EditorLevelLibrary.spawn_actor_from_class()`가
  viewport를 참조하다 크래시했다. 프로젝트 C++ 헬퍼의 직접 `UWorld::SpawnActor`
  호출로 우회했다. 현재 생성기는 `unreal.BaselineEditorTools.spawn_actor_direct()`를 쓴다.
- 게임 타깃에서는 editor 전용 헬퍼 헤더/코드를 분리해야 했다. 기존 guard를 보존한다.
- 첫 패키지에서 AirSim `BP_FlyingPawn`이 빠졌다. `/AirSim` 콘텐츠 cook 설정을 보존한다.
- 현재 AirSim 플러그인은 프로젝트 로컬 복사본에 landing 전용 패치를 적용한다.
  `tools/prepare_airsim_plugin.sh`가 실제 구현이며, README의 vanilla 플러그인 symlink
  설명은 현재 구현과 맞지 않는다. risk-aware가 사용하는 원본 플러그인에 덮어쓰지 않는다.
- 현재 패드 생성기는 unlit, no mipmaps, **bilinear** filtering을 사용한다.
  과거 기록/README의 nearest-neighbour 설명보다 현재 코드를 기준으로 한다.
- 현재 바닥/패드 충돌은 꺼져 있고 추정 카메라 높이로 touchdown을 판정한다.
  새 맵에 충돌체를 추가하면 기존 착륙 평가 조건과의 관계를 검토해야 한다.
- 맵/텍스처 변경은 맵 재생성과 cook/package가 필요하다. 기존 패키지가 존재하면
  `run_packaged_sim.sh`가 자동으로 변경된 맵을 재패키징하지 않으므로 명시적으로 갱신한다.
- 카메라 설정만 변경하면 settings 재생성 및 재시작으로 반영할 수 있다.
  ROS 코드만 변경한 경우 UE 재빌드는 필요하지 않다.

## 재사용할 명령과 스크린샷

저장소 루트에서, 새 맵 요구사항을 반영한 뒤 필요한 단계만 실행한다.
현재 generator는 `/Game/Maps/BaselineMap`을 갱신하므로 별도 맵을 보존해야 한다면
먼저 asset 경로와 출력 경로를 분리해야 한다.

```bash
./stacks/aruco-landing-sim-x86/scripts/build_baseline_map.sh
./stacks/aruco-landing-sim-x86/scripts/package_baseline_sim.sh
./stacks/aruco-landing-sim-x86/scripts/run_packaged_sim.sh
# 시뮬레이터 실행 후 별도 터미널에서:
.//home/ml/drone-data/aruco/archive/offline-tools/aruco-landing-sim-x86/tools/capture_environment_figure.sh
```

- 기존 환경 PNG: `flight_logs/aruco-landing/paper/simulation_environment.png`.
  파일 존재 확인 완료. 당시 캡처 해상도는 1920×1080.
- 기존 `.build/.../paper/simulation_environment.png`는 9월 7일 위 영구 경로로 이동됐다.
- 외부 카메라 `paper_overview`: NED 위치 (-2.5, -2.5, -3.8) m,
  pitch -37°, yaw 45°, 1920×1080, HFOV 75°.
- 캡처 구현: `/home/ml/drone-data/aruco/archive/offline-tools/aruco-landing-sim-x86/scripts/capture_environment_figure.py`.
- `simulation_environment_10_trials.png`는 환경 위 궤적 overlay로 만들었으나,
  사용자는 이후 별도 Matplotlib 3D plot을 원한다고 정정했다. 논문 그림 재사용 시 구분한다.

이번 요청의 완료 범위는 이전 작업 맥락 복원이다. 새 맵 요구사항은 아직 받지 않았다.

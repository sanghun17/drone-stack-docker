# FAST-LIVO2 D435i — 자율 디버깅 저널 (2026-06-22 야간, ~6h)

목표(사용자 지시): Tier1(cov 튜닝 재검증) + Tier3(gravity_align A/B, climb scale) + 방법론(리플레이 비결정성).
원래 Tier1은 핸드헬드 GT bag 재평가였으나 **그 bag들 시스템에서 삭제됨** → flight bag `2026-06-19-17-17-03.bag`(316s, GT vrpn 포함)으로 대체. pre-yaw(0-195s)=깨끗한 정확도 구간, yaw(200-260s)=발산 구간.

## 확정된 베이스라인 (이 세션 앞부분)
- **gt_odom→odom 광학flip 버그 FIX 완료** (LIVMapper.cpp ~1437, q_vrpn*q_o2r + geoQuat*q_o2r⁻¹). pre-yaw 위치 0.19m·자세 3.3°, RAW≈align(프레임 진짜 맞음).
- **yaw 발산 = 진짜 관측성 한계** (6각도 검증: GT깨끗/odom10Hz/내부발산/자세선행/출력시프트무관/imu_offset스윕무관). 튜닝으로 못 고침.
- climb scale 0.78~0.87 (수직 15~20% 덜 봄), run마다 0.46~0.64로 흔들림.
- 리플레이 **비결정적**: 같은 config가 18m~7.7km로 발산양상 다름.

## 실험 계획
- **A. 비결정성 특성화** (먼저 — 신뢰성 전제): 동일 config N회 반복 → pre-yaw/yaw 스프레드 측정. rate 0.5 효과. → A/B에 필요한 반복수 결정.
- **B. cov 검증** (Tier1 피벗): baseline(b 0.0001) vs biashi(b 0.001) vs 변형, fixed 바이너리 + 올바른 메트릭(aft_odom RAW)로 flight bag pre-yaw A/B. biashi 여전히 최적? 옛 메트릭 오염됐었나? cov가 yaw onset에 영향?
- **C. gravity_align A/B**: false(현재)→true. 단 gravity_align=on이면 내부가 z-up이라 광학flip이 틀려짐 → 이 A/B는 **aft_init(내부) Umeyama**로 프레임무관 평가. 추정기 자체 개선되는지.
- **D. climb scale 디버그**: config별 climb scale. gravity_align/extrinsic/cov 영향?
- **E. 종합 + 메모리 갱신.**

평가 메트릭: pre-yaw(0-190s) 위치 RAW APE + 자세 평균, yaw(200-240s) 위치/자세, climb scale.

---
# 진행 로그 (incremental)

## [setup] 2026-06-22
- 핸드헬드 bag 부재 확인 → flight bag으로 피벗.
- fast-livo git = `ws/fast-livo/src` (branch jetson-orin-agx); top repo는 `/ws/` 전체 gitignore.
- 커밋 진행 예정: ① 컴포넌트(LIVMapper fix) ② top repo(도구/모듈).

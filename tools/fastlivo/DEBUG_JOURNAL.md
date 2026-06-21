# FAST-LIVO2 D435i — 자율 디버깅 저널 (2026-06-22 야간, ~6h)

## ★ 최종 요약 (먼저 읽기)
밤새 한 일 + 결론. 자세한 건 아래 Wave 로그.

**1. cov 튜닝 검증 (Tier1) → biashi 안전, 재튜닝 불필요.**
- 핸드헬드 GT bag이 삭제돼 flight bag으로 검증. b_*_cov 0.0001/0.001(현재)/0.01 모두 flight pre-yaw 동일(initUmey~0.10), yaw 카오스. gentle flight선 cov 무차별. biashi가 공격적 핸드헬드서 발산 막은 건 프레임무관(프레임버그가 cov 랭킹 오염 안 함). **네가 걱정한 "튜닝 다 틀림" → 아님. cov 랭킹 유효, biashi 유지.**

**2. 🎯 새 발견: gravity_align ON이 climb scale 0.74→0.92 개선 (수직 미관측 23%→8%).**
- 현재 `gravity_align_en: false`. ON으로 켜면 accel이 중력("up")을 직접 구속 → 수직 관측 개선, N=4 견고(0.92/0.92/0.98/0.84), 편차도 감소. pre 내부오차도 0.11→0.088.
- **단 미완(WIP)**: ON이면 내부가 z-up이라 odom 매핑 새로 필요. 조건부 매핑 구현해서 **자세는 고쳤는데(89°→4°) 위치 RAW가 ~1m 오프셋 남음**(pos_end latch≈0이라 그게 아님, 원인 미해결). commit 8e09209에 WIP로 보존. **기본은 off 유지(검증 0.159m, 무회귀).**
- → climb 개선 가치 있으니, 깨어나서 **위치 anchor 마저 풀면 gravity_align ON이 완전한 개선**이 됨. 같이 보자.

**3. 방법론: 리플레이 비결정성 특성화.**
- pre-yaw 재현성 TIGHT(±5%) but **yaw 발산은 카오스**(run마다 18m~7.7km). → yaw A/B는 통계로만, pre-yaw A/B는 적은반복 OK.

**4. yaw 발산은 여전히 진짜 관측성 한계** (cov·gravity_align·time-offset 다 못 고침, 앞 세션서 6각도 검증). 해법은 외부 yaw 융합/느린 yaw/360° LiDAR.

**커밋**: ws/fast-livo/src — 5387508(off 광학flip fix, 핵심)·a7c63a0(biashi 체크포인트)·8e09209(gravity_align 조건부 WIP). top repo 8cb77d7(도구/저널).
**기본 동작 = 검증된 off 경로(pre-yaw 0.19m/3.3°)**, 변경 없음. gravity_align ON은 dormant WIP.

---

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
- 커밋 완료: ① 컴포넌트 `ws/fast-livo/src` 5387508(LIVMapper fix)+체크포인트, ② top repo 8cb77d7(도구/저널, .gitignore에 *.bag 추가). bag은 커밋 안 함.

## [Wave1] 비결정성 + gravity_align on/off — 실행 중
- 평가 도구 `_eval_run.py`: aft_odom RAW(pos/att) + **aft_init Umeyama(프레임무관 추정기품질)** + climb scale. gravity_align=on이면 내부가 z-up이라 aft_odom 매핑이 틀려짐 → **gravity_align 비교는 aft_init Umeyama로** (프레임무관).
- 구성: fixed(현재 config) 기존2 + 신규2 = N4(비결정성), gravity_align_on N2. 각 --duration 245(pre-yaw+yaw 포함).
### Wave1 결과 (03:54)
fixed N=4 (현재 config, gravity_align off):
```
fixed_full   pre RAW0.191 att3.3 initUmey0.113 climb0.64 | yaw pos3.26/6.23 att18.5
toff_base0   pre RAW0.191 att3.1 initUmey0.099 climb0.78 | yaw pos4.11/6.40 att16.0
fixed1       pre RAW0.212 att3.0 initUmey0.133 climb0.79 | yaw pos1.61/2.50 att28.8
fixed2       pre RAW0.196 att3.0 initUmey0.108 climb0.76 | yaw pos1.91/5.18 att21.5
```
gravity_align ON N=2:
```
gravon1      pre RAW1.093 att89.0 initUmey0.085 climb0.92 | yaw pos2.76/6.26 att89.0
gravon2      pre RAW1.099 att89.3 initUmey0.093 climb0.92 | yaw pos6.87/8.22 att87.9
```

**발견 1 — 비결정성**: pre-yaw 재현성 TIGHT(RAW ±5%, att ±0.3°) → cov A/B는 pre-yaw서 적은반복 OK. **yaw는 카오스**(pos 1.6~4.1m, att 16~29°, ±100%) → 발산크기는 초기조건 민감(나비효과), 단일run 비교 무의미. **yaw onset은 통계적으로만 봐야.**

**발견 2 — gravity_align ON이 추정기 개선** (프레임무관 initUmey/climb로 측정; aft_odom RAW1.09/att89는 z-up 내부에 광학flip이 틀어진 예상된 아티팩트):
- **climb scale 0.77→0.92** (수직 미관측 23%→8%), 게다가 **편차 사라짐**(0.92/0.92 vs 0.64~0.79). 중력정렬이 accel로 "up"을 직접 구속 → 수직 관측성 개선. **climb scale 문제의 진짜 레버.**
- pre initUmey 0.11→0.088 (~20% 내부궤적 개선).
- yaw: initUmey 2.5~2.7로 fixed와 비슷(카오스라 불명확).
- ⚠️ **gravity_align ON 쓰려면 gt_odom 매핑을 조건부로**: z-up 내부엔 광학flip 빼고 yaw-정렬만 해야. → 다음: 올바른 매핑 오프라인 규명 후 코드 조건부화.

## [Wave2] gravity_align 확인(N→4) + cov 스윕(b 0.0001/0.001/0.01) — 실행 중 (03:58~)
- 리플레이 중 동시 분석은 비결정성 오염 우려 → `_diag_gravon_map.py`(gravon aft_init→GT Umeyama R 분해; pure yaw면 gravity_align=on 매핑=Rz(yaw)만)는 작성만, Wave2 후 실행.
### Wave2 결과 (04:28)
gravity_align ON 확인 (N=4 누적): climb 0.92/0.92/0.98/0.84(~0.92), pre initUmey 0.085/0.093/0.082/0.090(~0.088). **견고하게 fixed(climb~0.74, initUmey~0.113) 능가.** yaw initUmey 1.26~3.50=fixed와 비슷(카오스).
cov 스윕:
```
b0.0001  pre initUmey 0.097/0.109  climb 0.76/0.81   (=baseline)
b0.001   pre initUmey 0.099~0.133  climb 0.64~0.79   (현재/biashi, wave1)
b0.01    pre initUmey 0.126/0.109  climb 0.81/0.75
```
**cov 결론(Tier1)**: flight pre-yaw서 b 0.0001/0.001/0.01 **전부 동일**(initUmey~0.10), yaw 카오스. → gentle flight선 cov 무차별. biashi(0.001)는 **공격적 핸드헬드 bag**(baseline 440~1640m 발산)서 검증된 거고, 발산 랭킹은 프레임무관(프레임버그가 cov 랭킹 오염 안 함). **biashi 안전, 재튜닝 불필요. cov는 yaw 레버 아님.**

### gravon 올바른 odom 매핑 (오프라인 규명)
Umeyama R(aft_init→GT, gravon) = 거의 순수 yaw **−179°**(pitch6°/roll2.5°=중력정렬잔차), scale0.88. vrpn init yaw −96°와 −83° 차이=센서↔body 마운트. z-up 내부라 광학flip 불필요. **일반 정답식: `T_odom_camera_init = vrpn_init · inv(geoQuat_init)`** (init서 1회 latch; off의 q_o2r은 optical 컨벤션이 geoQuat에 안 담겨서 별도 필요, on은 z-up이라 이 식으로 충분).

## [Wave3] gravity_align=on odom 매핑 조건부 구현 + 검증
- 목표: gravity_align ON의 climb 0.92 개선을 **실사용 가능**하게 — aft_odom이 on에서도 GT와 맞도록.
- 안전: **gravity_align_en 조건부** → off 경로(검증된 fix) 완전 무손상 = 회귀 0. 기본값은 일단 off 유지, on-path 검증 후 권장.

### 구현 (LIVMapper, 빌드 성공 1m12s)
- `include/LIVMapper.h`: `grav_anchor_latched`, `odom_to_camera_init_grav` 멤버 추가.
- `publish_odometry_odom` gt 경로 **조건부화**: off=기존 q_vrpn*q_o2r(무변경), **on=`vrpn_init·inv(geoQuat_init)`를 gravity_align_finished 후 1회 latch**(그 전 지면정적엔 vrpn 포즈 임시). 자세 켤레변환 자동(geoQuat 그대로).
- `reinit_cbk`: `grav_anchor_latched=false` 리셋.
### Wave3 결과 (04:46)
```
wv3_off     pre RAW0.159 att2.6 initUmey0.091 climb0.77 | yaw pos2.70 att17.2   ← OFF 회귀 없음 ✓
wv3_gravon  pre RAW1.084 att3.9 initUmey0.105 climb0.97 | yaw pos6.35 att96.8
```
- **OFF 무회귀 확인** (0.159m, 검증된 fix 무손상).
- **ON: 자세 고쳐짐(89°→3.9°)·climb 0.97 ✓, 근데 위치 RAW 1.08m**(initUmey 0.105=shape 좋음인데 RAW만 1m=상수 오프셋 → anchor 위치 문제).
- 원인: anchor를 vrpn_pos로만 잡고 latch 순간 pos_end(≠0 가능) 안 뺌. 

### Wave3b — full-anchor 수정 (vrpn·inv(전체 T_ci_body) = vrpn_pos − R(qr)·pos_end_latch)
- 한 항 추가 + latch 시 |pos_end| 로그. 리빌드+gravon 1회 검증 실행 중.

#!/usr/bin/env python3
"""오프라인: goal 근처 제동 시 vel_scale 대칭 cap vs 비대칭(멀어지는 방향 full).
t=20 상태 재현: goal까지 0.45m, toward-goal 속도 1.39. 최강 제동 primitive로 forward-sim(trapezoidal).
end 위치(goal=0.45 기준 오버슛)와 v_start 비교. 비행 불필요."""
import numpy as np

dt, tau, H = 0.3, 0.67, 7
alpha = 1 - np.exp(-dt / tau)
v0 = 1.39          # toward-goal 속도 (body forward)
goal = 0.45        # goal까지 거리 (m), 드론 x0=0
ratio = 0.5        # vel_scale at dist<min_dist

# v_x_options (toward-goal축) base
base = np.array([-1.5, -0.75, 0.0, 0.75, 1.5])
sym = base * ratio                       # 대칭: 전부 ×0.5
asym = np.where(base < 0, base, base * ratio)  # 비대칭: 멀어지는(음수=제동) full, 접근(양수) ×0.5

def rollout(target):
    v = [v0]; x = [0.0]
    for _ in range(H):
        vn = v[-1] + (target - v[-1]) * alpha
        x.append(x[-1] + 0.5 * (v[-1] + vn) * dt)   # trapezoidal
        v.append(vn)
    x = np.array(x); v = np.array(v)
    vstart = (x[1] - x[0]) / dt                      # b-spline v_start (finite-diff)
    peak = x.max()                                   # 최대 전진(오버슛 지점)
    return vstart, peak, x[-1], v

print(f"goal={goal}m 앞, 시작속도 {v0} toward-goal, ratio={ratio}")
print(f"{'config':14} {'최강제동 target':>14} {'v_start':>8} {'peak(오버슛)':>14} {'end_x':>8}")
for name, opts in (('대칭(cap)', sym), ('비대칭(brake full)', asym)):
    brake_target = opts.min()          # 가장 강한 제동 (가장 음수)
    vs, peak, endx, v = rollout(brake_target)
    print(f"{name:14} {brake_target:>14.2f} {vs:>+8.2f} {peak:>+9.2f}(+{peak-goal:.2f}) {endx:>+8.2f}")
print(f"\n옵션: 대칭 sym={np.round(sym,2)}  비대칭 asym={np.round(asym,2)}")
print("기대: 대칭은 제동 −0.75라 한참 오버슛, 비대칭은 −1.5로 goal 근처서 멈춤(오버슛↓)")
#!/usr/bin/env python3
"""오프라인 검증: forward-sim 위치적분 방식이 b-spline v_start/accel에 미치는 영향.
motion_primitives 1D forward-sim을 재현 (감속 중인 patrol 접근 상태):
  현재(backward-Euler, 위치=v_new·dt) / 내 (a)(v_start=실제 강제, 위치는 그대로) / trapezoidal(위치=평균·dt)
각각 b-spline fit 후 시작속도와 max|accel| 비교. 비행 불필요."""
import numpy as np
from scipy.interpolate import BSpline


# --- jax_to_mixtraj의 b-spline 빌더 복사 (pure numpy) ---
def parameterize_to_bspline(ts, points, sed, degree=3):
    K = len(points); n = K + degree - 1
    pp = np.array([1, 4, 1]) / 6.0
    pv = np.array([-1, 0, 1]) / (2 * ts)
    pa = np.array([1, -2, 1]) / (ts * ts)
    A = np.zeros((K + 4, n))
    for i in range(K): A[i, i:i + 3] = pp
    A[K, 0:3] = pv; A[K + 1, K - 1:K + 2] = pv
    A[K + 2, 0:3] = pa; A[K + 3, K - 1:K + 2] = pa
    b = np.zeros((K + 4, points.shape[1]))
    b[:K] = points
    for i in range(4): b[K + i] = sed[i]
    return np.linalg.lstsq(A, b, rcond=None)[0]


def knots(n, deg, iv):
    m = n - 1 + deg + 1
    u = np.zeros(m + 1)
    for i in range(m + 1):
        u[i] = (-deg + i) * iv if i <= deg else u[i - 1] + iv
    return u


# --- forward-sim (1D forward axis), 감속 접근 ---
dt, tau, H = 0.3, 0.67, 7
alpha = 1 - np.exp(-dt / tau)
v0 = 1.3        # 실제 현재속도 (body forward, goal로)
target = 0.0    # 정지 primitive (goal 접근)
x0 = -0.58      # 시작 위치 (goal -1.0서 0.42m)

# 속도 시퀀스 (state는 둘 다 동일: v_new)
v = [v0]
for _ in range(H):
    v.append(v[-1] + (target - v[-1]) * alpha)
v = np.array(v)            # v[0..H], v[0]=현재, v[k]=k스텝 후

# 위치 시퀀스 두 방식
def integrate(scheme):
    x = [x0]
    for k in range(H):
        if scheme == 'backward':   step = v[k + 1]              # 끝속도 (현재 코드)
        elif scheme == 'trapz':    step = 0.5 * (v[k] + v[k + 1])  # 평균 (제안)
        x.append(x[-1] + step * dt)
    return np.array(x)

for scheme in ('backward', 'trapz'):
    x = integrate(scheme)
    pts = np.stack([x, np.zeros(H + 1), np.zeros(H + 1)], axis=1)  # 3D (y,z=0)
    vw = np.zeros_like(pts)
    vw[:-1] = (pts[1:] - pts[:-1]) / dt; vw[-1] = vw[-2]
    vstart_fd = vw[0, 0]                      # finite-diff 시작속도
    a_end = (vw[-1] - vw[-2]) / dt
    # 세 v_start 케이스로 b-spline fit
    cases = {'finite-diff(현코드)': vstart_fd,
             'force 실제(내 a)': v0,
             'trapz-fd': vstart_fd}   # trapz는 fd가 곧 평균
    if scheme == 'backward':
        runs = [('현재 backward+finite-diff', vstart_fd),
                ('내 (a): backward+force1.3', v0)]
    else:
        runs = [('trapezoidal+finite-diff', vstart_fd)]
    for label, vs in runs:
        sed = [np.array([vs, 0, 0]), vw[-1], np.zeros(3), a_end]
        ctrl = parameterize_to_bspline(dt, pts, sed, 3)
        kn = knots(len(ctrl), 3, dt)
        spl = BSpline(kn, ctrl, 3, axis=0)
        uu = np.linspace(kn[3], kn[len(ctrl)], 80)
        vel = spl.derivative()(uu)[:, 0]
        acc = spl.derivative(2)(uu)[:, 0]
        x_end = spl(uu)[-1, 0]
        print(f"[{label:32}] v_start={vel[0]:+.2f}  |accel|max={np.abs(acc).max():.2f}  끝점={x_end:+.2f}")

print(f"\n참고: 실제 현재속도 v0={v0}, backward 끝속도 v_new[0]={v[1]:.2f}, trapz 평균={0.5*(v[0]+v[1]):.2f}")
print("기대: 현코드 v_start 낮음(과소), 내(a) accel 폭발(불일치), trapz v_start 중간+accel 정상")
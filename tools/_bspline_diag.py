#!/usr/bin/env python3
"""b-spline 속도 ripple 진단: 현재(LA v+a 경계 interp) vs 경계accel제거 vs smoothing.
사용: python3 _bspline_diag.py <bag> [out]"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag
from scipy.interpolate import splrep, splev, BSpline

def param_bspline(ts, pts, derivs, degree=3):
    K = len(pts); n_ctrl = K + degree - 1
    pp = np.array([1., 4., 1.]) / 6
    pv = np.array([-1., 0., 1.]) / (2 * ts)
    pa = np.array([1., -2., 1.]) / (ts * ts); w = 3
    A = np.zeros((K + 4, n_ctrl))
    for i in range(K): A[i, i:i + w] = pp
    A[K, 0:w] = pv; A[K + 1, K - 1:K - 1 + w] = pv
    A[K + 2, 0:w] = pa; A[K + 3, K - 1:K - 1 + w] = pa
    b = np.zeros((K + 4, 3)); b[:K] = pts
    for i in range(4): b[K + i] = derivs[i]
    ctrl, *_ = np.linalg.lstsq(A, b, rcond=None)
    return ctrl

def knots_clamped(n_ctrl, degree, iv):
    m = n_ctrl - 1 + degree + 1; u = np.zeros(m + 1)
    for i in range(m + 1):
        u[i] = (-degree + i) * iv if i <= degree else u[i - 1] + iv
    return u

BAG = sys.argv[1]; OUT = sys.argv[2] if len(sys.argv) > 2 else '.'
raws = []
b = rosbag.Bag(BAG)
for tp, m, t in b.read_messages(topics=['/jax/optimal_trajectory']):
    pos = np.array([[p.transforms[0].translation.x, p.transforms[0].translation.y,
                     p.transforms[0].translation.z] for p in m.points])
    ts = np.array([p.time_from_start.to_sec() for p in m.points])
    raws.append((pos, ts))
b.close()
raws.sort(key=lambda r: -np.linalg.norm(r[0][-1] - r[0][0]))
pos, rt = raws[0]; dt = float(rt[1] - rt[0]); deg = 3
rvel = np.zeros_like(pos); rvel[:-1] = np.diff(pos, axis=0) / np.diff(rt)[:, None]; rvel[-1] = rvel[-2]
v0, v1 = rvel[0], rvel[-1]
a0 = (rvel[1] - rvel[0]) / dt; a1 = (rvel[-1] - rvel[-2]) / dt
print(f"K={len(pos)} dt={dt:.3f} a0={a0} a1={a1}")

# 1) LA 현재 (v+a 경계)
ctrl = param_bspline(dt, pos, [v0, v1, a0, a1], deg); kn = knots_clamped(len(ctrl), deg, dt)
tt = np.linspace(kn[deg], kn[-deg - 1], 120); t0 = kn[deg]
velL = BSpline(kn, ctrl, deg, axis=0).derivative()(tt)
# 2) LA 경계 accel 제거 (a=0)
ctrl2 = param_bspline(dt, pos, [v0, v1, np.zeros(3), np.zeros(3)], deg)
velL2 = BSpline(kn, ctrl2, deg, axis=0).derivative()(tt)
# 3) scipy smoothing spline (s>0, 점 근사)
velS = np.zeros((len(tt), 3))
S = 1e-4
for ax in range(3):
    tck = splrep(rt, pos[:, ax], k=3, s=S)
    velS[:, ax] = splev(tt, tck, der=1)

def ripple(v):  # vel 2차차분 크기 합 = 출렁임 척도
    return float(np.sum(np.abs(np.diff(v, 2, axis=0))))
print(f"ripple(작을수록 매끈): LA현재 {ripple(velL):.3f} | a=0 {ripple(velL2):.3f} | smooth(s={S}) {ripple(velS):.3f}")

fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
for r, lbl in enumerate('XYZ'):
    axs[r].plot(rt, rvel[:, r], 'k.--', ms=8, lw=1, label='raw diff')
    axs[r].plot(tt, velL[:, r], 'r-', lw=2.2, label='LA current (v+a bc)')
    axs[r].plot(tt, velL2[:, r], 'g-', lw=1.6, label='LA, accel bc=0')
    axs[r].plot(tt, velS[:, r], 'b-', lw=1.6, label=f'smoothing spline s={S}')
    axs[r].set_ylabel(f'v{lbl.lower()} [m/s]'); axs[r].grid(alpha=.3)
# vz: vx와 동일한 0.2 m/s 폭으로 고정 — autoscale 확대 왜곡 제거
cz = float(rvel[:, 2].mean()); axs[2].set_ylim(cz - 0.1, cz + 0.1)
axs[0].legend(fontsize=8); axs[2].set_xlabel('t [s]')
fig.suptitle('velocity ripple diagnosis: current vs accel-bc=0 vs smoothing')
fig.tight_layout()
p = f'{OUT}/bspline_diag.png'; fig.savefig(p, dpi=110); print(p)

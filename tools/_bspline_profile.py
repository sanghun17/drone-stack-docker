#!/usr/bin/env python3
"""경로별 b-spline 전/후 위치·속도 프로파일.
raw = /jax/optimal_trajectory (점 + time_from_start, vel=finite-diff).
b-spline = /planning/trajectory (MixTraj ctrl_pts/knots) -> scipy BSpline pos + derivative(vel).
변위 큰 대표 N개를 경로당 한 장(3행 x 2열: pos | vel). 사용: python3 _bspline_profile.py <bag> [out] [N]"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag
from scipy.interpolate import BSpline

BAG = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else '.'
N = int(sys.argv[3]) if len(sys.argv) > 3 else 4

raws, mixs = [], []
b = rosbag.Bag(BAG)
for tp, m, t in b.read_messages(topics=['/jax/optimal_trajectory', '/planning/trajectory']):
    if tp == '/jax/optimal_trajectory':
        pos = np.array([[p.transforms[0].translation.x, p.transforms[0].translation.y,
                         p.transforms[0].translation.z] for p in m.points])
        ts = np.array([p.time_from_start.to_sec() for p in m.points])
        raws.append((t.to_sec(), pos, ts))
    else:
        ctrl = np.array([[p.x, p.y, p.z] for p in m.pos_pts])
        mixs.append((t.to_sec(), ctrl, np.array(m.knots), m.bspline_degree, m.real_traj_duration))
b.close()
if not mixs:
    print("no /planning/trajectory (MixTraj) — 2단계 bag 아님?"); sys.exit(1)

def nearest_raw(ts):
    return min(raws, key=lambda r: abs(r[0] - ts))

items = []
for mix in mixs:
    raw = nearest_raw(mix[0])
    items.append((float(np.linalg.norm(raw[1][-1] - raw[1][0])), raw, mix))
items.sort(key=lambda x: -x[0])
N = min(N, len(items))

for k in range(N):
    disp, raw, mix = items[k]
    rpos, rt = raw[1], raw[2]
    rvel = np.zeros_like(rpos)
    rvel[:-1] = np.diff(rpos, axis=0) / np.diff(rt)[:, None]
    rvel[-1] = rvel[-2]
    ctrl, knots, deg, dur = mix[1], mix[2], mix[3], mix[4]
    bs = BSpline(knots, ctrl, deg, axis=0)
    bsv = bs.derivative()
    t0, t1 = knots[deg], knots[-deg - 1]
    tt = np.linspace(t0, t1, 120)
    bpos, bvel = bs(tt), bsv(tt)
    tt0 = tt - t0

    fig, axs = plt.subplots(3, 2, figsize=(12, 9), sharex=True)
    for r, lbl in enumerate('XYZ'):
        axs[r, 0].plot(rt, rpos[:, r], 'r.--', ms=8, lw=1.2, label='raw JAX')
        axs[r, 0].plot(tt0, bpos[:, r], 'b-', lw=2, label='b-spline')
        axs[r, 0].set_ylabel(f'{lbl} [m]'); axs[r, 0].grid(alpha=.3)
        axs[r, 1].plot(rt, rvel[:, r], 'r.--', ms=8, lw=1.2, label='raw diff')
        axs[r, 1].plot(tt0, bvel[:, r], 'b-', lw=2, label='b-spline')
        axs[r, 1].set_ylabel(f'v{lbl.lower()} [m/s]'); axs[r, 1].grid(alpha=.3)
    axs[0, 0].set_title('POSITION'); axs[0, 1].set_title('VELOCITY')
    axs[0, 0].legend(fontsize=8); axs[0, 1].legend(fontsize=8)
    axs[2, 0].set_xlabel('t [s]'); axs[2, 1].set_xlabel('t [s]')
    fig.suptitle(f'traj {k+1}/{N}  disp {disp:.2f}m  dur {dur:.2f}s  ({len(rpos)} raw pts) - b-spline pos/vel profile')
    fig.tight_layout()
    p = f'{OUT}/bspline_profile_{k+1}.png'; fig.savefig(p, dpi=110); print(p, f'disp={disp:.2f}')

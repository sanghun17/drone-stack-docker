#!/usr/bin/env python3
"""b-spline 전(raw JAX waypoints) vs 후(b-spline curve) 비교.
/jax_to_mixtraj/raw_path (점) vs /jax_to_mixtraj/bspline_path (곡선) — 동시 발행 쌍.
사용: python3 _bspline_compare.py <bag> [out_dir]"""
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag

BAG = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else '.'

def poses_xyz(m):
    return np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in m.poses])

raws, bsps = [], []
b = rosbag.Bag(BAG)
for tp, m, t in b.read_messages(topics=['/jax_to_mixtraj/raw_path', '/jax_to_mixtraj/bspline_path']):
    (raws if tp.endswith('raw_path') else bsps).append((t.to_sec(), poses_xyz(m)))
b.close()
print(f"raw_path {len(raws)}  bspline_path {len(bsps)}")
n = min(len(raws), len(bsps))
if n == 0:
    print("no path msgs"); sys.exit(1)

def turning(p):  # 누적 방향변화(rad) — 꺾임 척도
    d = np.diff(p[:, :2], axis=0)
    a = np.arctan2(d[:, 1], d[:, 0])
    return float(np.sum(np.abs((np.diff(a) + np.pi) % (2 * np.pi) - np.pi))) if len(a) > 1 else 0.0

disp = [np.linalg.norm(raws[i][1][-1] - raws[i][1][0]) for i in range(n)]
idx = int(np.argmax(disp))
raw, bsp = raws[idx][1], bsps[idx][1]
tr = sum(turning(raws[i][1]) for i in range(n)) / n
tb = sum(turning(bsps[i][1]) for i in range(n)) / n
print(f"평균 누적방향변화(꺾임): raw {tr:.2f} rad -> bspline {tb:.2f} rad")
print(f"대표 궤적 #{idx}: 변위 {disp[idx]:.2f}m, raw {len(raw)}pts, bspline {len(bsp)}pts")

fig, axs = plt.subplots(1, 2, figsize=(13, 6))
for i in range(n):
    axs[0].plot(raws[i][1][:, 0], raws[i][1][:, 1], 'r-', lw=0.6, alpha=0.3)
    axs[0].plot(bsps[i][1][:, 0], bsps[i][1][:, 1], 'b-', lw=0.6, alpha=0.3)
axs[0].plot([], [], 'r-', label='raw JAX (pre)')
axs[0].plot([], [], 'b-', label='b-spline (post)')
axs[0].set_title(f'all {n} trajs XY  (turning raw {tr:.2f} -> bspline {tb:.2f} rad)')
axs[0].set_xlabel('X [m]'); axs[0].set_ylabel('Y [m]'); axs[0].axis('equal')
axs[0].grid(alpha=0.3); axs[0].legend(fontsize=9)
axs[1].plot(raw[:, 0], raw[:, 1], 'r.--', ms=9, lw=1.3, label='raw JAX pts (pre)')
axs[1].plot(bsp[:, 0], bsp[:, 1], 'b-', lw=2.0, label='b-spline curve (post)')
axs[1].set_title(f'representative traj #{idx} (disp {disp[idx]:.2f} m)')
axs[1].set_xlabel('X [m]'); axs[1].set_ylabel('Y [m]'); axs[1].axis('equal')
axs[1].grid(alpha=0.3); axs[1].legend(fontsize=9)
fig.suptitle(f'b-spline pre/post compare  -  {BAG.split("/")[-1]}')
fig.tight_layout()
p = f'{OUT}/bspline_compare.png'; fig.savefig(p, dpi=120); print(p)

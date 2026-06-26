#!/usr/bin/env python3
"""반환점 추종 분리 진단: 정지 계획한 궤적(b-spline) vs 실제 reference(pos_cmd) vs odom.
계획은 끝점에서 vy→0(정지)인데 odom이 그 위치에서 속도가 큰지 = tracking 실패 검증.
usage: _track_zoom.py <bag> <traj_id1> [traj_id2 ...]"""
import sys, math, rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline

bagf = sys.argv[1]
tids = [int(x) for x in sys.argv[2:]]
OUTDIR = '/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad'
out = bagf.rsplit('/', 1)[-1].replace('.bag', '')

planned = {}      # tid -> (t_abs, y, vy)
pc_t, pc_y, pc_vy, pc_id = [], [], [], []
od_t, od_y, od_vy = [], [], []

b = rosbag.Bag(bagf)
for topic, m, t in b.read_messages(topics=['/planning/trajectory', '/planning/pos_cmd', '/robot/odom']):
    if topic == '/planning/trajectory':
        if m.traj_id not in tids:
            continue
        deg = m.bspline_degree; kn = np.array(m.knots)
        cp = np.array([[p.x, p.y, p.z] for p in m.pos_pts])
        spl = BSpline(kn, cp, deg, axis=0); dspl = spl.derivative()
        uu = np.linspace(kn[deg], kn[len(cp)], 120)
        planned[m.traj_id] = (m.start_time.to_sec() + uu, spl(uu)[:, 1], dspl(uu)[:, 1])
    elif topic == '/planning/pos_cmd':
        pc_t.append(m.header.stamp.to_sec()); pc_y.append(m.position.y)
        pc_vy.append(m.velocity.y); pc_id.append(m.trajectory_id)
    elif topic == '/robot/odom':
        ts = m.header.stamp.to_sec(); q = m.pose.pose.orientation; v = m.twist.twist.linear
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        od_t.append(ts); od_y.append(m.pose.pose.position.y)
        od_vy.append(math.sin(yaw) * v.x + math.cos(yaw) * v.y)
b.close()

pc_t = np.array(pc_t); pc_y = np.array(pc_y); pc_vy = np.array(pc_vy); pc_id = np.array(pc_id)
od_t = np.array(od_t); od_y = np.array(od_y); od_vy = np.array(od_vy)

n = len(tids)
fig, axs = plt.subplots(2, n, figsize=(6.5 * n, 8), squeeze=False)
for c, tid in enumerate(tids):
    pt, py, pvy = planned[tid]
    t0, t1 = pt[0] - 0.3, pt[-1] + 0.8
    mpc = (pc_t >= t0) & (pc_t <= t1)
    mod = (od_t >= t0) & (od_t <= t1)
    tref = t0
    # position Y
    ax = axs[0][c]
    ax.plot(pt - tref, py, '-', color='tab:blue', lw=2.2, label=f'planned traj {tid} (b-spline)')
    ax.plot(pc_t[mpc] - tref, pc_y[mpc], ':', color='tab:orange', lw=1.8, label='reference (pos_cmd)')
    ax.plot(od_t[mod] - tref, od_y[mod], '-', color='k', lw=1.6, label='odom (actual)')
    ax.plot(pt[-1] - tref, py[-1], 'b*', ms=14)   # 계획 끝점
    ax.axhline(round(py[-1]), color='0.7', ls='--', lw=0.8)
    ax.set_title(f'traj {tid}: 계획 끝점 y={py[-1]:+.2f}, vy={pvy[-1]:+.2f}')
    ax.set_ylabel('y [m]'); ax.grid(alpha=0.3); ax.legend(fontsize=8)
    # velocity Y
    ax = axs[1][c]
    ax.plot(pt - tref, pvy, '-', color='tab:blue', lw=2.2, label='planned vy')
    ax.plot(pc_t[mpc] - tref, pc_vy[mpc], ':', color='tab:orange', lw=1.8, label='reference vy (pos_cmd)')
    ax.plot(od_t[mod] - tref, od_vy[mod], '-', color='k', lw=1.6, label='odom vy (actual)')
    ax.axhline(0, color='0.7', ls='--', lw=0.8)
    ax.plot(pt[-1] - tref, pvy[-1], 'b*', ms=14)
    ax.set_xlabel('t since window start [s]'); ax.set_ylabel('vy [m/s]')
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    # 진단 수치: 계획 끝점 시각에서 odom의 위치/속도
    te = pt[-1]
    iod = np.argmin(np.abs(od_t - te))
    print(f"traj {tid}: 계획끝(t={te-tref:.2f}) plan(y={py[-1]:+.2f},vy={pvy[-1]:+.2f})  "
          f"| 동시각 odom(y={od_y[iod]:+.2f},vy={od_vy[iod]:+.2f})")
    # odom이 계획 끝점 y에 실제 도달한 순간의 odom vy
    wp = py[-1]
    sign = 1 if pvy[max(0, len(pvy)//2)] > 0 else -1
    reach = np.where((od_t >= pt[0]) & ((od_y >= wp) if sign > 0 else (od_y <= wp)))[0]
    if len(reach):
        j = reach[0]
        print(f"         odom이 y={wp:+.2f} 도달 순간: odom_vy={od_vy[j]:+.2f}  (계획은 vy={pvy[-1]:+.2f})")

fig.suptitle(f'{out}  TRACKING: planned(stop) vs reference vs odom')
fig.tight_layout()
p = f'{OUTDIR}/{out}_trackzoom.png'
fig.savefig(p, dpi=120); print('saved', p)
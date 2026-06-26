#!/usr/bin/env python3
"""각 b-spline 궤적(/planning/trajectory MixTraj)을 자신의 전체 시간 horizon으로
de Boor 평가해 시간축에 '겹쳐서' 그린다 (실행 전에 replan으로 잘린 감속 꼬리까지 보이게).
traj_server: t_cur = now - start_time, 평가구간 [0, real_traj_duration].
  - 각 궤적: 점선, traj_id별 색. 시작점=o, 끝점=x 마커.
  - current(odom): 검은 실선.
Figure 2개: Position(x,y,z,yaw) / Velocity(vx,vy,vz,yaw_rate), 각 1열 4행.
usage: _traj_full_plot.py <bag> [out_prefix]"""
import sys, math, rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from scipy.interpolate import BSpline

bagf = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else bagf.rsplit('/', 1)[-1].replace('.bag', '')
OUTDIR = '/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad'


def eval_yaw(coef, dur, t):
    """piecewise order-5 poly yaw. coef flat [c5,c4,c3,c2,c1,c0] per piece."""
    acc = 0.0
    for i, d in enumerate(dur):
        if t <= acc + d or i == len(dur) - 1:
            tau = max(0.0, min(t - acc, d))
            c = coef[6 * i:6 * i + 6]
            pos = ((((c[0] * tau + c[1]) * tau + c[2]) * tau + c[3]) * tau + c[4]) * tau + c[5]
            vel = (((5 * c[0] * tau + 4 * c[1]) * tau + 3 * c[2]) * tau + 2 * c[3]) * tau + c[4]
            return pos, vel
        acc += d
    return 0.0, 0.0


trajs = []        # list of dicts: t_abs, pos(N,3), vel(N,3), yaw(N), yawrate(N), tid
od_t, od_pos, od_vel, od_yaw, od_wz = [], [], [], [], []

b = rosbag.Bag(bagf)
for topic, m, t in b.read_messages(topics=['/planning/trajectory', '/robot/odom']):
    if topic == '/planning/trajectory':
        deg = m.bspline_degree
        knots = np.array(m.knots)
        ctrl = np.array([[p.x, p.y, p.z] for p in m.pos_pts])
        if len(ctrl) < deg + 1:
            continue
        dur = m.real_traj_duration
        spl = BSpline(knots, ctrl, deg, axis=0)
        dspl = spl.derivative()
        N = 60
        uu = np.linspace(knots[deg], knots[len(ctrl)], N)   # [0, dur]
        pos = spl(uu); vel = dspl(uu)
        yy = np.array([eval_yaw(m.coef_yaw, m.duration_yaw, u) for u in uu])
        t0 = m.start_time.to_sec()
        trajs.append(dict(t=t0 + uu, pos=pos, vel=vel, yaw=yy[:, 0], yr=yy[:, 1],
                          tid=m.traj_id, dur=dur, t0=t0))
    elif topic == '/robot/odom':
        ts = m.header.stamp.to_sec()
        p = m.pose.pose.position; q = m.pose.pose.orientation
        v = m.twist.twist.linear; w = m.twist.twist.angular
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        od_t.append(ts); od_pos.append((p.x, p.y, p.z))
        od_vel.append((math.cos(yaw) * v.x - math.sin(yaw) * v.y,
                       math.sin(yaw) * v.x + math.cos(yaw) * v.y, v.z))
        od_yaw.append(yaw); od_wz.append(w.z)
b.close()

t_ref = min(trajs[0]['t0'], od_t[0])
od_rel = [x - t_ref for x in od_t]
cmap = plt.get_cmap('jet')
nT = len(trajs)
for i, tr in enumerate(trajs):
    tr['c'] = cmap(i / max(1, nT - 1))
    tr['rel'] = tr['t'] - t_ref

print(f"궤적 {nT}개  traj_id={[tr['tid'] for tr in trajs]}")
print(f"평균 duration={np.mean([tr['dur'] for tr in trajs]):.2f}s  "
      f"평균 발행간격={np.mean(np.diff([tr['t0'] for tr in trajs])):.2f}s")


AXC = ['tab:red', 'tab:green', 'tab:blue', 'tab:purple']   # current 축색 (x/y/z/yaw)


def draw(ax, getter, od_vals, ylabel, axc, mark=True):
    ax.plot(od_rel, od_vals, '-', color=axc, lw=1.6, zorder=5, label='current(odom)')
    for tr in trajs:
        v = getter(tr)
        ax.plot(tr['rel'], v, ':', color=tr['c'], lw=1.3, zorder=3)
        if mark:
            ax.plot(tr['rel'][0], v[0], 'o', color=tr['c'], ms=4, zorder=4)   # start
            ax.plot(tr['rel'][-1], v[-1], 'x', color=tr['c'], ms=6, mew=1.6, zorder=4)  # end
    ax.set_ylabel(ylabel); ax.grid(alpha=0.3)


def make_fig(title, getters, od_series, ylabels, fname):
    fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    for r in range(4):
        draw(axs[r], getters[r], od_series[r], ylabels[r], AXC[r])
    axs[0].set_title(title)
    axs[-1].set_xlabel('time [s]')
    handles = [Line2D([0], [0], color='k', ls='-', label='current(odom)'),
               Line2D([0], [0], color='0.4', ls=':', label='trajectory (full horizon)'),
               Line2D([0], [0], marker='o', color='0.4', ls='', label='traj start'),
               Line2D([0], [0], marker='x', color='0.4', ls='', label='traj end')]
    axs[0].legend(handles=handles, ncol=4, fontsize=9, loc='upper right')
    fig.tight_layout()
    p = f'{OUTDIR}/{fname}'
    fig.savefig(p, dpi=110); print('saved', p)


make_fig(f'{out}  POSITION  each b-spline traj over FULL horizon (dotted) vs odom (solid)',
         [lambda tr: tr['pos'][:, 0], lambda tr: tr['pos'][:, 1],
          lambda tr: tr['pos'][:, 2], lambda tr: tr['yaw']],
         [[v[0] for v in od_pos], [v[1] for v in od_pos], [v[2] for v in od_pos], od_yaw],
         ['x [m]', 'y [m]', 'z [m]', 'yaw [rad]'], f'{out}_trajfull_pos.png')

make_fig(f'{out}  VELOCITY  each b-spline traj over FULL horizon (dotted) vs odom (solid)',
         [lambda tr: tr['vel'][:, 0], lambda tr: tr['vel'][:, 1],
          lambda tr: tr['vel'][:, 2], lambda tr: tr['yr']],
         [[v[0] for v in od_vel], [v[1] for v in od_vel], [v[2] for v in od_vel], od_wz],
         ['vx [m/s]', 'vy [m/s]', 'vz [m/s]', 'yaw_rate [rad/s]'], f'{out}_trajfull_vel.png')

# 진단: 각 궤적의 끝점(Y, vy) — "끝점이 경계면 vy=0 명령" 검증
print("\ntraj  start_y   end_y   end_vy   dur  (끝점 vy≈0이면 정지 계획)")
for tr in trajs:
    print(f"  {tr['tid']:>3}  {tr['pos'][0,1]:+.2f}  {tr['pos'][-1,1]:+.2f}  "
          f"{tr['vel'][-1,1]:+.2f}  {tr['dur']:.2f}")
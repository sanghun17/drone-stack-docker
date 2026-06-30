#!/usr/bin/env python3
"""각 b-spline 궤적(/planning/trajectory MixTraj)을 자신의 전체 시간 horizon으로
de Boor 평가해 시간축에 '겹쳐서' 그린다 (실행 전에 replan으로 잘린 감속 꼬리까지 보이게).
  - 각 궤적: 점선, traj_id별 색. 시작점=o, 끝점=x 마커.
  - current(odom): 검은 실선.
  - goal(/local_planner/goal_pose): magenta step 라인 (목표 x/y/z/yaw — odom 수렴 비교용).
  - 배경: goal 변경마다 다른 파스텔색. OFFBOARD 경계=회색 점선.
Figure 2개: Position(x,y,z,yaw) / Velocity(vx,vy,vz,yaw_rate), 각 1열 4행 (goal 라인은 POSITION만).
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
ms = []           # /mavros/state (t_recv, mode) — OFFBOARD 경계용
goals = []        # (t_abs, (gx, gy, gz), gyaw) — /local_planner/goal_pose
sp_t, sp_pos, sp_vel, sp_yaw, sp_wz = [], [], [], [], []   # /local_controller/setpoint_raw/local (명령)

b = rosbag.Bag(bagf)
for topic, m, t in b.read_messages(topics=[
        '/planning/trajectory', '/robot/odom', '/mavros/state',
        '/local_planner/goal_pose', '/local_controller/setpoint_raw/local']):
    if topic == '/mavros/state':
        ms.append((t.to_sec(), m.mode)); continue
    if topic == '/local_controller/setpoint_raw/local':
        sp_t.append(t.to_sec())
        sp_pos.append((m.position.x, m.position.y, m.position.z))
        sp_vel.append((m.velocity.x, m.velocity.y, m.velocity.z))
        sp_yaw.append(m.yaw); sp_wz.append(m.yaw_rate)
        continue
    if topic == '/local_planner/goal_pose':
        if m.markers:
            sph = next((mk for mk in m.markers if mk.type == 2), m.markers[0])  # SPHERE = goal pos
            p = sph.pose.position
            arr = next((mk for mk in m.markers                                 # ARROW = goal yaw (tail->tip)
                        if mk.type == 0 and len(mk.points) >= 2), None)
            gyaw = (math.atan2(arr.points[1].y - arr.points[0].y,
                               arr.points[1].x - arr.points[0].x) if arr else 0.0)
            goals.append((t.to_sec(), (p.x, p.y, p.z), gyaw))
        continue
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
sp_rel = [x - t_ref for x in sp_t]

# OFFBOARD 경계 (t_ref 상대시각) — 세로 점선용
ob_spans = []
_open = None
for tt, mode in ms:
    rel = tt - t_ref
    if mode == 'OFFBOARD' and _open is None:
        _open = rel
    elif mode != 'OFFBOARD' and _open is not None:
        ob_spans.append((_open, rel)); _open = None
if _open is not None and od_rel:
    ob_spans.append((_open, od_rel[-1]))
print(f"OFFBOARD 구간: {[(round(s,1), round(e,1)) for s, e in ob_spans]}")

# goal 세그먼트 (변경마다 분할) — 배경 음영 + step 라인용
goal_segs = []    # (rel_start, rel_end, goal_xyz, goal_yaw)
if goals:
    cur_g, cur_yaw, cur_s = goals[0][1], goals[0][2], goals[0][0] - t_ref
    for ta, g, gy in goals[1:]:
        if any(abs(a - c) > 0.02 for a, c in zip(g, cur_g)):
            goal_segs.append((cur_s, ta - t_ref, cur_g, cur_yaw)); cur_g, cur_yaw, cur_s = g, gy, ta - t_ref
    goal_segs.append((cur_s, od_rel[-1] if od_rel else cur_s, cur_g, cur_yaw))
print(f"goal {len(goal_segs)}개: {[(round(s,1), tuple(round(v,2) for v in g), round(y,2)) for s, e, g, y in goal_segs]}")

# goal step 라인 (각 세그먼트 [start,end]에서 일정 -> 변경시 수직 점프)
g_rel, g_x, g_y, g_z, g_yaw = [], [], [], [], []
for s, e, g, gy in goal_segs:
    g_rel += [s, e]; g_x += [g[0], g[0]]; g_y += [g[1], g[1]]; g_z += [g[2], g[2]]; g_yaw += [gy, gy]

cmap = plt.get_cmap('jet')
nT = len(trajs)
for i, tr in enumerate(trajs):
    tr['c'] = cmap(i / max(1, nT - 1))
    tr['rel'] = tr['t'] - t_ref

print(f"궤적 {nT}개  평균 duration={np.mean([tr['dur'] for tr in trajs]):.2f}s  "
      f"평균 발행간격={np.mean(np.diff([tr['t0'] for tr in trajs])):.2f}s")


AXC = ['tab:red', 'tab:green', 'tab:blue', 'tab:purple']   # current 축색 (x/y/z/yaw)
GPAL = ['#ffdede', '#deffde', '#dedeff', '#fff3cf', '#f3deff', '#defff3', '#ffe7cf', '#e6e6e6']
SPC = 'magenta'    # setpoint 색


def draw(ax, getter, od_vals, goal_vals, sp_vals, ylabel, axc, mark=True):
    if goal_segs:                                          # goal 세그먼트별 배경 (OFFBOARD 구간만)
        for i, (s, e, g, gy) in enumerate(goal_segs):
            for os, oe in ob_spans:
                a, b = max(s, os), min(e, oe)
                if b > a:
                    ax.axvspan(a, b, color=GPAL[i % len(GPAL)], alpha=0.6, zorder=0)
    else:                                                  # goal 없으면 OFFBOARD 음영(구버전)
        for s, e in ob_spans:
            ax.axvspan(s, e, color='0.80', alpha=0.45, zorder=0)
    for s, e in ob_spans:                                  # OFFBOARD 경계 = 세로 점선
        ax.axvline(s, color='0.25', ls='--', lw=1.0, alpha=0.7, zorder=1)
        ax.axvline(e, color='0.25', ls='--', lw=1.0, alpha=0.7, zorder=1)
    ax.plot(od_rel, od_vals, '-', color=axc, lw=1.7, zorder=5, label='current(odom)')
    if sp_vals is not None and sp_rel:                     # 명령 setpoint = 축색 점선
        ax.plot(sp_rel, sp_vals, ':', color=axc, lw=1.8, zorder=4.5, label='setpoint(cmd)')
    if goal_vals is not None and g_rel:                    # goal step 라인
        ax.plot(g_rel, goal_vals, '--', color=SPC, lw=1.6, zorder=4, label='goal')
    for tr in trajs:
        v = getter(tr)
        ax.plot(tr['rel'], v, ':', color=tr['c'], lw=1.0, zorder=3)
        if mark:
            ax.plot(tr['rel'][0], v[0], 'o', color=tr['c'], ms=3, zorder=3.5)
            ax.plot(tr['rel'][-1], v[-1], 'x', color=tr['c'], ms=5, mew=1.4, zorder=3.5)
    ax.set_ylabel(ylabel); ax.grid(alpha=0.3)


def make_fig(title, getters, od_series, goal_series, sp_series, ylabels, fname):
    fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    for r in range(4):
        draw(axs[r], getters[r], od_series[r], goal_series[r], sp_series[r], ylabels[r], AXC[r])
    top = axs[0].get_ylim()[1]                             # goal 값 라벨 (OFFBOARD 보이는 구간 중심)
    for s, e, g, gy in goal_segs:
        vis = [(max(s, os), min(e, oe)) for os, oe in ob_spans if min(e, oe) > max(s, os)]
        if not vis:
            continue
        axs[0].text((vis[0][0] + vis[-1][1]) / 2, top, f'({g[0]:.1f},{g[1]:.1f},{g[2]:.1f})',
                    ha='center', va='top', fontsize=7, color='0.25')
    axs[0].set_title(title)
    axs[-1].set_xlabel('time [s]')
    handles = [Line2D([0], [0], color='k', ls='-', label='current(odom)'),
               Line2D([0], [0], color='k', ls=':', lw=1.8, label='setpoint(cmd, axis color)'),
               Line2D([0], [0], color=SPC, ls='--', label='goal'),
               Line2D([0], [0], color='0.4', ls=':', label='trajectory (full horizon)'),
               Line2D([0], [0], marker='o', color='0.4', ls='', label='traj start'),
               Line2D([0], [0], marker='x', color='0.4', ls='', label='traj end'),
               Line2D([0], [0], color='0.25', ls='--', label='OFFBOARD edge')]
    axs[0].legend(handles=handles, ncol=7, fontsize=7, loc='upper right')
    fig.tight_layout()
    p = f'{OUTDIR}/{fname}'
    fig.savefig(p, dpi=110); print('saved', p)


make_fig(f'{out}  POSITION  traj(dotted) / odom(solid) / goal(dashed) — bg=goal seg',
         [lambda tr: tr['pos'][:, 0], lambda tr: tr['pos'][:, 1],
          lambda tr: tr['pos'][:, 2], lambda tr: tr['yaw']],
         [[v[0] for v in od_pos], [v[1] for v in od_pos], [v[2] for v in od_pos], od_yaw],
         [g_x, g_y, g_z, g_yaw],
         [[v[0] for v in sp_pos], [v[1] for v in sp_pos], [v[2] for v in sp_pos], sp_yaw],
         ['x [m]', 'y [m]', 'z [m]', 'yaw [rad]'], f'{out}_trajfull_pos.png')

make_fig(f'{out}  VELOCITY  traj(dotted) / odom(solid) — bg=goal seg',
         [lambda tr: tr['vel'][:, 0], lambda tr: tr['vel'][:, 1],
          lambda tr: tr['vel'][:, 2], lambda tr: tr['yr']],
         [[v[0] for v in od_vel], [v[1] for v in od_vel], [v[2] for v in od_vel], od_wz],
         [None, None, None, None],
         [[v[0] for v in sp_vel], [v[1] for v in sp_vel], [v[2] for v in sp_vel], sp_wz],
         ['vx [m/s]', 'vy [m/s]', 'vz [m/s]', 'yaw_rate [rad/s]'], f'{out}_trajfull_vel.png')

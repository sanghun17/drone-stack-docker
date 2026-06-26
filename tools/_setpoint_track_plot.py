#!/usr/bin/env python3
"""control_bridge가 PX4로 내보내는 setpoint(/local_controller/setpoint_raw/local) vs odom 추종 플롯.
stage-2 B(full feedforward) 검증용: pos+vel+accel+yaw가 의도한 프레임·부호로 실리는지.
setpoint = PositionTarget (FRAME_LOCAL_NED = world ENU 가정; coordinate_frame/type_mask 새니티 출력).
Figure 3개: POSITION(x,y,z,yaw) / VELOCITY(vx,vy,vz,yaw_rate) / ACCELERATION(ax,ay,az), 1열.
  - current(odom): 실선, 축별 빨/초/파/보라 (accel은 odom world-vel 수치미분)
  - setpoint: 점선, trajectory_id별 색 (pos_cmd에서 시각 매칭). yaw_rate행 setpoint는 IGNORE라 생략.
usage: _setpoint_track_plot.py <bag> [out_prefix]"""
import sys, math, rosbag
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

bagf = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else bagf.rsplit('/', 1)[-1].replace('.bag', '')
OUTDIR = '/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad'
SP_TOPIC = '/local_controller/setpoint_raw/local'

sp_t, sp_pos, sp_vel, sp_acc, sp_yaw, sp_frames, sp_masks = [], [], [], [], [], set(), set()
pc_t, pc_id = [], []
od_t, od_pos, od_vel, od_yaw, od_wz = [], [], [], [], []

b = rosbag.Bag(bagf)
for topic, m, t in b.read_messages(topics=[SP_TOPIC, '/planning/pos_cmd', '/robot/odom']):
    if topic == SP_TOPIC:
        ts = m.header.stamp.to_sec() or t.to_sec()
        sp_t.append(ts); sp_frames.add(m.coordinate_frame); sp_masks.add(m.type_mask)
        sp_pos.append((m.position.x, m.position.y, m.position.z))
        sp_vel.append((m.velocity.x, m.velocity.y, m.velocity.z))
        sp_acc.append((m.acceleration_or_force.x, m.acceleration_or_force.y, m.acceleration_or_force.z))
        sp_yaw.append(m.yaw)
    elif topic == '/planning/pos_cmd':
        pc_t.append(m.header.stamp.to_sec() or t.to_sec()); pc_id.append(m.trajectory_id)
    elif topic == '/robot/odom':
        ts = m.header.stamp.to_sec() or t.to_sec()
        p = m.pose.pose.position; q = m.pose.pose.orientation
        v = m.twist.twist.linear; w = m.twist.twist.angular
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        od_t.append(ts); od_pos.append((p.x, p.y, p.z))
        od_vel.append((math.cos(yaw) * v.x - math.sin(yaw) * v.y,
                       math.sin(yaw) * v.x + math.cos(yaw) * v.y, v.z))
        od_yaw.append(yaw); od_wz.append(w.z)
b.close()

if not sp_t:
    print(f"!! {SP_TOPIC} 메시지 없음 — commands enable(toggle_running) 안 했거나 stage-2 미기동")
    sys.exit(1)

sp_t = np.array(sp_t); od_t = np.array(od_t)
sp_acc_a = np.array(sp_acc)
print(f"setpoint coordinate_frame={sorted(sp_frames)} (1=LOCAL_NED world, 8=BODY_NED)  type_mask={sorted(sp_masks)}")
print(f"setpoint accel |max|: x={np.abs(sp_acc_a[:,0]).max():.2f} y={np.abs(sp_acc_a[:,1]).max():.2f} "
      f"z={np.abs(sp_acc_a[:,2]).max():.2f}  (≈0이면 ff 안 실림)")

# trajectory_id: pos_cmd에서 setpoint 시각에 가장 가까운 것 매칭
pc_t = np.array(pc_t); pc_id = np.array(pc_id)
if len(pc_t):
    idx = np.clip(np.searchsorted(pc_t, sp_t), 0, len(pc_id) - 1)
    sp_traj = pc_id[idx]
else:
    sp_traj = np.zeros(len(sp_t), dtype=int)
ids = sorted(set(int(x) for x in sp_traj))
cmap = plt.get_cmap('jet')
id_color = {tid: cmap(i / max(1, len(ids) - 1)) for i, tid in enumerate(ids)}

# odom world accel = world velocity 수치미분 (moving-avg 평활)
od_vel_a = np.array(od_vel)
od_acc = np.zeros_like(od_vel_a)
if len(od_t) > 2:
    dt = np.gradient(od_t)
    for ax in range(3):
        od_acc[:, ax] = np.gradient(od_vel_a[:, ax]) / np.where(dt > 1e-6, dt, 1e-6)
    k = 7
    ker = np.ones(k) / k
    for ax in range(3):
        od_acc[:, ax] = np.convolve(od_acc[:, ax], ker, mode='same')

t0 = min(sp_t[0], od_t[0])
sp_rel = sp_t - t0
od_rel = od_t - t0
AXC = ['tab:red', 'tab:green', 'tab:blue', 'tab:purple']


def split_by_id(rel, vals):
    segs = []
    if not len(rel):
        return segs
    s = 0
    for i in range(1, len(sp_traj) + 1):
        if i == len(sp_traj) or sp_traj[i] != sp_traj[s]:
            segs.append((int(sp_traj[s]), rel[s:i], vals[s:i]))
            s = i
    return segs


def draw(ax, row, sp_vals, od_vals, ylabel, sp_on=True):
    ax.plot(od_rel, od_vals, '-', color=AXC[row], lw=1.3, zorder=3)
    if sp_on:
        for tid, ts, vs in split_by_id(sp_rel, np.asarray(sp_vals)):
            ax.plot(ts, vs, ':', color=id_color[tid], lw=1.6, zorder=4)
    ax.set_ylabel(ylabel); ax.grid(alpha=0.3)


def make_fig(title, sp_series, od_series, ylabels, fname, sp_mask=None):
    n = len(ylabels)
    fig, axs = plt.subplots(n, 1, figsize=(13, 2.8 * n), sharex=True)
    if n == 1:
        axs = [axs]
    for r in range(n):
        on = True if sp_mask is None else sp_mask[r]
        draw(axs[r], r, sp_series[r] if sp_series[r] is not None else [], od_series[r], ylabels[r], on)
    axs[0].set_title(title)
    axs[-1].set_xlabel('time [s]')
    handles = [Line2D([0], [0], color='0.3', ls='-', label='current(odom)'),
               Line2D([0], [0], color='0.3', ls=':', label='setpoint(sent)')]
    handles += [Line2D([0], [0], color=id_color[t], ls=':', label=f'traj {t}') for t in ids[:10]]
    axs[0].legend(handles=handles, ncol=4, fontsize=8, loc='upper right')
    fig.tight_layout()
    p = f'{OUTDIR}/{fname}'
    fig.savefig(p, dpi=110); print('saved', p)


sp_p = [[v[0] for v in sp_pos], [v[1] for v in sp_pos], [v[2] for v in sp_pos], sp_yaw]
od_p = [[v[0] for v in od_pos], [v[1] for v in od_pos], [v[2] for v in od_pos], od_yaw]
make_fig(f'{out}  POSITION  setpoint(sent, :, traj색) vs odom(-, 축색)',
         sp_p, od_p, ['x [m]', 'y [m]', 'z [m]', 'yaw [rad]'], f'{out}_pos.png')

sp_v = [[v[0] for v in sp_vel], [v[1] for v in sp_vel], [v[2] for v in sp_vel], None]
od_v = [od_vel_a[:, 0], od_vel_a[:, 1], od_vel_a[:, 2], od_wz]
make_fig(f'{out}  VELOCITY  setpoint(sent, :) vs odom(-); yaw_rate행 setpoint=IGNORE',
         sp_v, od_v, ['vx [m/s]', 'vy [m/s]', 'vz [m/s]', 'yaw_rate [rad/s]'], f'{out}_vel.png',
         sp_mask=[True, True, True, False])

sp_a = [sp_acc_a[:, 0], sp_acc_a[:, 1], sp_acc_a[:, 2]]
od_a = [od_acc[:, 0], od_acc[:, 1], od_acc[:, 2]]
make_fig(f'{out}  ACCELERATION  setpoint ff(sent, :) vs odom(-, vel 수치미분)',
         sp_a, od_a, ['ax [m/s²]', 'ay [m/s²]', 'az [m/s²]'], f'{out}_acc.png')
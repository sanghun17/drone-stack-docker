#!/usr/bin/env python3
"""stage1 vs stage2 patrol overlay. 각 stage 같은 색: actual=실선, setpoint=점선.
 위치: goal(검정 step) + actual + position setpoint(=pos_cmd.position or jax pt[1]).
 속도: actual + velocity setpoint(=setpoint_raw.velocity), 둘 다 world frame.
사용: python3 _track_compare.py <s1.bag> <s2.bag> [out]"""
import sys, math, bisect
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag

def yawf(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def load(bag):
    ot, ox, oy, oz, oyaw, avx, avy, avz = [], [], [], [], [], [], [], []
    gt, gx, gy, gz = [], [], [], []
    sp_raw = []                 # (ts, body vx,vy,vz)
    pc, jp = [], []             # pos_cmd.position ; jax point[1]
    states = []
    b = rosbag.Bag(bag)
    for tp, m, t in b.read_messages(topics=['/robot/odom', '/planner/command/trajectory',
            '/local_controller/setpoint_raw/local', '/planning/pos_cmd',
            '/jax/optimal_trajectory', '/mavros/state']):
        ts = t.to_sec()
        if tp == '/robot/odom':
            p = m.pose.pose.position; tw = m.twist.twist.linear; yaw = yawf(m.pose.pose.orientation)
            ot.append(ts); ox.append(p.x); oy.append(p.y); oz.append(p.z); oyaw.append(yaw)
            avx.append(math.cos(yaw) * tw.x - math.sin(yaw) * tw.y)
            avy.append(math.sin(yaw) * tw.x + math.cos(yaw) * tw.y); avz.append(tw.z)
        elif tp == '/planner/command/trajectory' and len(m.points) >= 2:
            tr = m.points[1].transforms[0].translation
            gt.append(ts); gx.append(tr.x); gy.append(tr.y); gz.append(tr.z)
        elif tp == '/local_controller/setpoint_raw/local':
            sp_raw.append((ts, m.velocity.x, m.velocity.y, m.velocity.z))
        elif tp == '/planning/pos_cmd':
            pc.append((ts, m.position.x, m.position.y, m.position.z))
        elif tp == '/jax/optimal_trajectory' and len(m.points) >= 2:
            tr = m.points[1].transforms[0].translation
            jp.append((ts, tr.x, tr.y, tr.z))
        else:
            states.append((ts, m.armed and m.mode == 'OFFBOARD'))
    b.close()
    st, svx, svy, svz = [], [], [], []
    for ts, vx, vy, vz in sp_raw:
        i = min(bisect.bisect_left(ot, ts), len(oyaw) - 1) if oyaw else 0
        yaw = oyaw[i] if oyaw else 0.0
        st.append(ts); svx.append(math.cos(yaw) * vx - math.sin(yaw) * vy)
        svy.append(math.sin(yaw) * vx + math.cos(yaw) * vy); svz.append(vz)
    ps = pc if pc else jp   # position setpoint: stage2=pos_cmd, stage1=jax pt[1]
    pst = [r[0] for r in ps]; psx = [r[1] for r in ps]; psy = [r[2] for r in ps]; psz = [r[3] for r in ps]
    t0 = next((ts for ts, on in states if on), ot[0])
    t1 = next((ts for ts, on in states if ts > t0 and not on), ot[-1])
    return dict(t0=t0, t1=t1, ot=ot, ox=ox, oy=oy, oz=oz, avx=avx, avy=avy, avz=avz,
                gt=gt, gx=gx, gy=gy, gz=gz, st=st, svx=svx, svy=svy, svz=svz,
                pst=pst, psx=psx, psy=psy, psz=psz)

S1 = load(sys.argv[1]); S2 = load(sys.argv[2]); OUT = sys.argv[3] if len(sys.argv) > 3 else '.'
rel = lambda d, k: [t - d['t0'] for t in d[k]]
xmax = max(S1['t1'] - S1['t0'], S2['t1'] - S2['t0']) + 1

# ---- POSITION: actual 실선 + position setpoint 점선 (같은 색) + goal ----
fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
pa = ['ox', 'oy', 'oz']; ps = ['psx', 'psy', 'psz']
for r in range(3):
    axs[r].plot(rel(S2, 'gt'), S2[['gx', 'gy', 'gz'][r]], 'k-', lw=0.8, drawstyle='steps-post', label='goal')
    axs[r].plot(rel(S1, 'ot'), S1[pa[r]], 'b-', lw=1.4, label='stage1 actual')
    axs[r].plot(rel(S1, 'pst'), S1[ps[r]], 'b--', lw=0.9, alpha=.7, label='stage1 setpoint')
    axs[r].plot(rel(S2, 'ot'), S2[pa[r]], 'r-', lw=1.4, label='stage2 actual')
    axs[r].plot(rel(S2, 'pst'), S2[ps[r]], 'r--', lw=0.9, alpha=.7, label='stage2 setpoint')
    axs[r].set_ylabel(['X', 'Y', 'Z'][r] + ' [m]'); axs[r].grid(alpha=.3); axs[r].set_xlim(-1, xmax)
axs[0].legend(fontsize=7, loc='upper right', ncol=3); axs[-1].set_xlabel('t since OFFBOARD [s]')
fig.suptitle('Position (world): actual=solid, setpoint=dashed | stage1=blue, stage2=red')
fig.tight_layout(); fig.savefig(f'{OUT}/cmp_position.png', dpi=120)

# ---- VELOCITY: actual 실선 + velocity setpoint 점선 (같은 색), world ----
fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
av = ['avx', 'avy', 'avz']; sv = ['svx', 'svy', 'svz']
for r in range(3):
    axs[r].plot(rel(S1, 'st'), S1[sv[r]], 'b--', lw=0.9, alpha=.7, label='stage1 setpoint')
    axs[r].plot(rel(S1, 'ot'), S1[av[r]], 'b-', lw=1.5, label='stage1 actual')
    axs[r].plot(rel(S2, 'st'), S2[sv[r]], 'r--', lw=0.9, alpha=.7, label='stage2 setpoint')
    axs[r].plot(rel(S2, 'ot'), S2[av[r]], 'r-', lw=1.5, label='stage2 actual')
    axs[r].axhline(0, color='k', lw=.4); axs[r].set_ylabel(['vx', 'vy', 'vz'][r] + '(world) [m/s]')
    axs[r].grid(alpha=.3); axs[r].set_xlim(-1, xmax)
axs[0].legend(fontsize=7, loc='upper right', ncol=2); axs[-1].set_xlabel('t since OFFBOARD [s]')
fig.suptitle('Velocity (world): actual=solid, setpoint=dashed | stage1=blue, stage2=red')
fig.tight_layout(); fig.savefig(f'{OUT}/cmp_velocity.png', dpi=120)
print('done')

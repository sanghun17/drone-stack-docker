#!/usr/bin/env python3
"""4행 두 장: (1) 위치+yaw goal(setpoint) vs 실제  (2) 속도+yawrate cmd vs 실제.
pos setpoint = /planner/command/trajectory point[1] (goal). 실제 = /robot/odom.
yaw setpoint = goal point[1] rotation (goal yaw). control이 실제 추종하는 target_yaw=0 점선.
vel setpoint = /local_controller/setpoint_raw/local (FLU body). 실제 = odom twist.
yawrate: cmd = setpoint yaw_rate, 실제 = odom angular.z.
사용: python3 _track_plot.py <bag> [out_dir]"""
import sys, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag

BAG = sys.argv[1]
OUT = sys.argv[2] if len(sys.argv) > 2 else '.'
deg = math.degrees
def yaw_q(q): return deg(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

ot, ox, oy, oz, oyaw, oyr = [], [], [], [], [], []      # odom pose+yaw + yawrate(angular.z)
otw, ovx, ovy, ovz = [], [], [], []                      # odom twist (FLU body)
gt, gx, gy, gz, gyaw = [], [], [], [], []                # goal (setpoint pos+yaw)
st, svx, svy, svz, syr = [], [], [], [], []              # setpoint vel (FLU body) + yaw_rate cmd
off = []

b = rosbag.Bag(BAG)
for topic, m, t in b.read_messages(topics=[
        '/robot/odom', '/planner/command/trajectory',
        '/local_controller/setpoint_raw/local', '/mavros/state']):
    ts = t.to_sec()
    if topic == '/robot/odom':
        p = m.pose.pose.position; tw = m.twist.twist.linear
        ot.append(ts); ox.append(p.x); oy.append(p.y); oz.append(p.z)
        oyaw.append(yaw_q(m.pose.pose.orientation)); oyr.append(deg(m.twist.twist.angular.z))
        otw.append(ts); ovx.append(tw.x); ovy.append(tw.y); ovz.append(tw.z)
    elif topic == '/planner/command/trajectory' and len(m.points) >= 2:
        tf = m.points[1].transforms[0]
        gt.append(ts); gx.append(tf.translation.x); gy.append(tf.translation.y)
        gz.append(tf.translation.z); gyaw.append(yaw_q(tf.rotation))
    elif topic == '/local_controller/setpoint_raw/local':
        st.append(ts); svx.append(m.velocity.x); svy.append(m.velocity.y)
        svz.append(m.velocity.z); syr.append(deg(m.yaw_rate))
    elif topic == '/mavros/state':
        off.append((ts, m.armed and m.mode == 'OFFBOARD'))
b.close()

t0 = min([x[0] for x in (ot, gt, st) if x] or [0])
rel = lambda a: [x - t0 for x in a]
ivals = []; cur = None
for ts, on in off:
    if on and cur is None: cur = ts
    if not on and cur is not None: ivals.append((cur, ts)); cur = None
if cur is not None: ivals.append((cur, ot[-1] if ot else cur))
def shade(ax):
    for a, c in ivals: ax.axvspan(a - t0, c - t0, color='tab:green', alpha=0.08, lw=0)

# ---------- (1) POSITION + YAW ----------
fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for ax, lbl, oa, ga in zip(axs[:3], ('X', 'Y', 'Z'), (ox, oy, oz), (gx, gy, gz)):
    shade(ax)
    ax.plot(rel(gt), ga, 'r.-', lw=1.4, ms=3, label='goal (setpoint)', drawstyle='steps-post')
    ax.plot(rel(ot), oa, 'b-', lw=1.2, label='actual (odom)')
    ax.set_ylabel(f'{lbl}  [m]'); ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8)
ax = axs[3]; shade(ax)
ax.plot(rel(gt), gyaw, 'r.-', lw=1.4, ms=3, label='goal yaw (setpoint)', drawstyle='steps-post')
ax.plot(rel(ot), oyaw, 'b-', lw=1.2, label='actual yaw (odom)')
ax.axhline(0, color='gray', ls='--', lw=1.0, label='target_yaw=0 (what control tracks)')
ax.set_ylabel('yaw  [deg]'); ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8)
axs[0].set_title(f'Position + Yaw: goal(setpoint) vs actual  —  {BAG.split("/")[-1]}  (green=OFFBOARD)')
axs[-1].set_xlabel('time [s]'); fig.tight_layout()
p1 = f'{OUT}/track_position.png'; fig.savefig(p1, dpi=120); print(p1)

# ---------- (2) VELOCITY + YAWRATE ----------
fig, axs = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
for ax, lbl, sa, oa in zip(axs[:3], ('vx', 'vy', 'vz'), (svx, svy, svz), (ovx, ovy, ovz)):
    shade(ax)
    ax.plot(rel(st), sa, 'r-', lw=1.2, label='cmd (setpoint)')
    ax.plot(rel(otw), oa, 'b-', lw=1.0, alpha=0.85, label='actual (odom twist)')
    ax.axhline(0, color='k', lw=0.5, alpha=0.4)
    ax.set_ylabel(f'{lbl}  [m/s]'); ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8)
ax = axs[3]; shade(ax)
ax.plot(rel(st), syr, 'r-', lw=1.2, label='cmd yaw_rate (setpoint)')
ax.plot(rel(ot), oyr, 'b-', lw=1.0, alpha=0.85, label='actual yaw_rate (odom)')
ax.axhline(0, color='k', lw=0.5, alpha=0.4)
ax.set_ylabel('yawrate [deg/s]'); ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8)
axs[0].set_title(f'Velocity + Yawrate (FLU body): cmd(setpoint) vs actual  —  {BAG.split("/")[-1]}  (green=OFFBOARD)')
axs[-1].set_xlabel('time [s]'); fig.tight_layout()
p2 = f'{OUT}/track_velocity.png'; fig.savefig(p2, dpi=120); print(p2)

#!/usr/bin/env python3
"""brake leg(13~18s) y(t): odom vs pos_cmd reference vs brake b-spline + goal −1.0.
odom-read / publish / live 마커. 오버슛이 (plan terminal) + (lag offset) 임을 시각화."""
import sys, math
import numpy as np, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline

bagf = sys.argv[1] if len(sys.argv) > 1 else \
    '/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag'
b = rosbag.Bag(bagf); t0 = b.get_start_time()

ot, oy = [], []
for _, m, t in b.read_messages(topics=['/robot/odom']):
    ot.append(m.header.stamp.to_sec()-t0); oy.append(m.pose.pose.position.y)
ct, cy = [], []
for _, m, t in b.read_messages(topics=['/planning/pos_cmd']):
    ct.append((m.header.stamp.to_sec() if m.header.stamp.to_sec()>0 else t.to_sec())-t0)
    cy.append(m.position.y)
brake = None
for _, m, t in b.read_messages(topics=['/planning/trajectory']):
    if abs((t.to_sec()-t0)-15.43) < 0.3:
        deg=m.bspline_degree; kn=np.array(m.knots)
        ctrl=np.array([[p.x,p.y,p.z] for p in m.pos_pts])
        spl=BSpline(kn,ctrl,deg,axis=0)
        uu=np.linspace(kn[deg],kn[len(ctrl)],60)
        brake=dict(t=m.start_time.to_sec()-t0+uu, y=spl(uu)[:,1])
b.close()

fig, ax = plt.subplots(figsize=(11,6))
ax.plot(ot, oy, 'b-', lw=2, label='odom_y (실제)')
ax.plot(ct, cy, color='orange', ls=':', lw=1.5, label='pos_cmd_y (드론이 받는 reference)')
if brake is not None:
    ax.plot(brake['t'], brake['y'], 'r-', lw=2.5, alpha=0.8,
            label='brake b-spline (15.47 GOAL_PROX plan)')
ax.axhline(-1.0, color='green', ls='--', lw=1, label='goal y=−1.0')
ax.axhline(-1.673, color='red', ls=':', lw=0.8, alpha=0.5)
for tt, lab, c in [(15.29,'odom-read','purple'),(15.43,'publish','brown'),(15.47,'live','red')]:
    ax.axvline(tt, color=c, ls='-', lw=0.8, alpha=0.6)
    ax.text(tt, 0.15, lab, rotation=90, fontsize=8, color=c, va='bottom')
ax.annotate(f'overshoot −1.673 @16.93', xy=(16.93,-1.673), xytext=(17.2,-1.3),
            fontsize=9, arrowprops=dict(arrowstyle='->',color='red'))
ax.annotate('plan terminal −1.43\n(plan 자체가 goal 넘김:\ncoast 0.90m > 남은거리 0.44m)',
            xy=(16.0,-1.43), xytext=(13.1,-1.55), fontsize=8,
            arrowprops=dict(arrowstyle='->',color='gray'))
ax.set_xlim(13,18); ax.set_ylim(-1.8,0.3)
ax.set_xlabel('t (s)'); ax.set_ylabel('y (m)')
ax.set_title('1.5 m/s −Y brake: 오버슛 = stopping-distance(주) + exec lag(부)\n'
             'odom-read 15.29 (y=−0.56, vy=−1.34) → live 15.47 (y=−0.80, 0.24m 이동)')
ax.legend(loc='lower left', fontsize=8); ax.grid(alpha=0.3)
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/lag_verify_15-57-36.png'
plt.tight_layout(); plt.savefig(out, dpi=110); print(out)

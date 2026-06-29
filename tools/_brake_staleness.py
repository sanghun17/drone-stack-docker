#!/usr/bin/env python3
"""-Y overshoot: goal flips to +1.0 at 15.52, JAX replans reverse at 15.64/16.26 but
jax_to_mixtraj min_pub_interval=1.5 DROPS them -> drone tracks the 15.43 coast b-spline
for 2.4s -> overshoot. y vs time. (figure text in English.)"""
import numpy as np, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline
b=rosbag.Bag('/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag');t0=b.get_start_time()
ot,oy=[],[]
for _,m,t in b.read_messages(topics=['/robot/odom']):
    tr=m.header.stamp.to_sec()-t0
    if 13.0<tr<18.6:ot.append(tr);oy.append(m.pose.pose.position.y)
mix=[]
for _,m,t in b.read_messages(topics=['/planning/trajectory']):
    tr=t.to_sec()-t0
    if 13.0<tr<18.6:
        deg=m.bspline_degree;kn=np.array(m.knots);ctrl=np.array([[p.x,p.y,p.z] for p in m.pos_pts])
        spl=BSpline(kn,ctrl,deg,axis=0);uu=np.linspace(kn[deg],kn[len(ctrl)],50)
        mix.append((m.start_time.to_sec()-t0,uu,spl(uu)[:,1]))
jax=[]
for _,m,t in b.read_messages(topics=['/jax/optimal_trajectory']):
    tr=t.to_sec()-t0
    if 15.3<tr<18.0:
        ts=np.array([p.time_from_start.to_sec() for p in m.points])
        ys=np.array([p.transforms[0].translation.y for p in m.points])
        jax.append((tr,m.header.stamp.to_sec()-t0+ts,ys))
b.close()

fig,ax=plt.subplots(figsize=(11.5,6.8))
ax.plot(ot,oy,'b-',lw=2.4,label='odom y (actual, overshoots to -1.67)',zorder=6)
for st,uu,yy in mix:
    fwd = (st<15.6) or (st>17.0)   # forwarded b-splines
    ax.plot(st+uu,yy,'-',color='red' if st<15.6 and st>15 else ('green' if st>17 else '0.6'),
            lw=2.4,alpha=0.9,zorder=5)
# JAX dropped reverse plans
for tr,tt,yy in jax:
    dropped = 15.6<tr<17.0
    ax.plot(tt,yy,ls='--',marker='^',ms=4,lw=1.6,
            color='darkorange' if dropped else '0.5',alpha=0.9,zorder=4,
            label=('JAX reverse plan DROPPED by min_pub_interval' if (dropped and tr<15.8) else None))
ax.axhline(-1.0,color='green',ls='--',lw=1.2,label='goal y=-1.0')
ax.axvline(15.30,color='purple',lw=1);ax.text(15.30,-0.32,'GOAL_PROXIMITY\n(0.5m) 15.30',fontsize=7,color='purple',ha='center')
ax.axvline(15.52,color='brown',lw=1);ax.text(15.52,-1.85,'goal flips ->+1.0\n(0.26m) 15.52',fontsize=7,color='brown',ha='center')
ax.annotate('coast b-spline (->-1.45) tracked 15.43..17.81 (2.4s, stale)',xy=(16.6,-1.5),xytext=(13.2,-1.9),
            fontsize=8,color='red',arrowprops=dict(arrowstyle='->',color='red'))
ax.annotate('reverse b-spline finally live 17.81',xy=(17.81,-1.6),xytext=(17.0,-0.5),
            fontsize=8,color='green',arrowprops=dict(arrowstyle='->',color='green'))
ax.set_xlabel('time (s)');ax.set_ylabel('y (m)')
ax.set_title('-Y overshoot compounded by staleness: planner decided to reverse at 15.64,\n'
             'but min_pub_interval=1.5 withheld it 2.4s; drone tracked the coast plan -> -1.67')
ax.set_xlim(14.8,18.5);ax.set_ylim(-2.0,-0.2);ax.grid(alpha=0.3);ax.legend(loc='lower right',fontsize=8)
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_staleness.png'
plt.tight_layout();plt.savefig(out,dpi=115);print(out)

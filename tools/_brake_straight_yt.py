#!/usr/bin/env python3
"""직진 제동 패밀리(vy=0 & wz=0, vx 변화)를 세로=진행축 위치 y, 가로=시간으로.
goal=-1.0, cost 채점점(2.1s), 실제 odom 같이. (이전 XY를 시간전개)"""
import numpy as np, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
DT,H=0.3,7;TAU_XY=0.67;VX=[-1.5,-0.75,0.0,0.75,1.5];VS_MAX,VS_MIN,VS_RATIO=2.0,1.0,0.5
state=np.array([0.035,-0.555,0.776,-1.671,1.337,0.061,0.027,0.0]);goal=np.array([0.0,-1.0,1.0])
a_xy=1-np.exp(-DT/TAU_XY)
x0,y0,z0,yaw0=state[:4];vx0=state[4]
dist0=np.linalg.norm(state[:3]-goal);tn=(dist0-VS_MIN)/max(VS_MAX-VS_MIN,1e-6)
vsf=np.clip(VS_RATIO+tn*(1-VS_RATIO),VS_RATIO,1.0)
cy=np.cos(yaw0);sy=np.sin(yaw0)
gx_b=(cy*(goal[0]-x0)+sy*(goal[1]-y0));gx_b/=max(abs(gx_b),1e-6)
def roll_y(vxt):
    y,yaw,vx=y0,yaw0,vx0;Y=[y]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy;vxm=0.5*(vx+vxn)
        y+=vxm*np.sin(yaw)*DT;vx=vxn;Y.append(y)
    return np.array(Y)
tt=np.arange(H+1)*DT
b=rosbag.Bag('/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag');t0=b.get_start_time()
ot,oy=[],[]
for _,m,t in b.read_messages(topics=['/robot/odom']):
    tr=m.header.stamp.to_sec()-t0-15.29
    if -0.2<tr<2.6:ot.append(tr);oy.append(m.pose.pose.position.y)
b.close()

fig,ax=plt.subplots(figsize=(10,6.5))
col={-1.5:'indigo',-0.75:'magenta',0.0:'red',0.75:'darkorange',1.5:'saddlebrown'}
note={-1.5:'역추진강',-0.75:'역추진(goal통과→되돌아옴)',0.0:'coast(선택,넘김)',0.75:'전진',1.5:'전진강'}
for vx in VX:
    vxs=vx*(vsf if vx*gx_b>0 else 1.0)
    Y=roll_y(vxs)
    ax.plot(tt,Y,marker='o',ms=5,lw=2.4,color=col[vx],label=f'vx={vx:+.2f} 끝y={Y[-1]:+.2f} — {note[vx]}')
ax.plot(ot,oy,'b-',lw=2,alpha=0.6,label='실제 odom')
ax.axhline(-1.0,color='green',ls='--',lw=1.3,label='goal y=-1.0')
ax.axvline(2.1,color='gray',ls=':',lw=1.2);ax.text(2.1,-0.42,'cost 채점점\n(horizon 끝 2.1s)',fontsize=8,color='gray',ha='center')
ax.annotate('역추진은 여기서 goal 통과',xy=(0.9,-1.0),xytext=(0.95,-0.55),fontsize=9,color='magenta',
            arrowprops=dict(arrowstyle='->',color='magenta'))
ax.set_xlabel('시간 (s, replan=15.29부터)');ax.set_ylabel('y (진행축 위치, m)')
ax.set_title('직진 제동 패밀리: 세로=y, 가로=시간\n역추진(자홍)은 ~0.9s에 goal 통과 후 되돌아감 → 끝점(2.1s)이 멀어 탈락 / coast(빨강) 선택')
ax.set_ylim(-2.0,-0.3);ax.grid(alpha=0.3);ax.legend(fontsize=8,loc='lower left')
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_straight_yt.png'
plt.tight_layout();plt.savefig(out,dpi=115);print(out)

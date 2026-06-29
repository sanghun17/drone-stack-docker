#!/usr/bin/env python3
"""재현된 후보로 '왜 cost가 coast를 고르나' 시각화.
coast#157(선택,끝 -1.43) vs 역추진#99(goal 찍고 되돌아감,끝 -0.54) + goal + 실제 odom.
cost는 2.1s 끝점(terminal_dist×10)만 보니 역추진이 페널티 → coast가 최소."""
import numpy as np, math, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

DT,H=0.3,7; TAU_XY,TAU_Z,TAU_WZ=0.67,0.075,0.264
VX=[-1.5,-0.75,0.0,0.75,1.5];VY=[-0.5,0.0,0.5];VZ=[-0.5,0.0,0.5];NYAW=7;MAXYAW=1.5
VS_MAX,VS_MIN,VS_RATIO=2.0,1.0,0.5
state=np.array([0.035,-0.555,0.776,-1.671,1.337,0.061,0.027,0.0]); goal=np.array([0.0,-1.0,1.0])
a_xy=1-np.exp(-DT/TAU_XY);a_z=1-np.exp(-DT/TAU_Z);a_wz=1-np.exp(-DT/TAU_WZ)
x0,y0,z0,yaw0=state[:4];vx0,vy0,vz0,wz0=state[4:]
dist0=np.linalg.norm(state[:3]-goal);tn=(dist0-VS_MIN)/max(VS_MAX-VS_MIN,1e-6)
vs=np.clip(VS_RATIO+tn*(1-VS_RATIO),VS_RATIO,1.0)
cy,sy=np.cos(yaw0),np.sin(yaw0)
gx_b=cy*(goal[0]-x0)+sy*(goal[1]-y0);gy_b=-sy*(goal[0]-x0)+cy*(goal[1]-y0)
gn=max(np.hypot(gx_b,gy_b),1e-6);gx_b,gy_b=gx_b/gn,gy_b/gn
VXg,VYg,VZg,WZg=np.meshgrid(VX,VY,VZ,np.linspace(-MAXYAW,MAXYAW,NYAW),indexing='ij')
tw=VXg*gx_b+VYg*gy_b;xs=np.where(tw>0,vs,1.0)
P=np.stack([(VXg*xs).ravel(),(VYg*xs).ravel(),(VZg*vs).ravel(),(WZg*vs).ravel()],1)
RAW=np.stack([VXg.ravel(),VYg.ravel(),VZg.ravel(),WZg.ravel()],1)
def roll(p):
    vxt,vyt,vzt,wzt=p;x,y,z,yaw,vx,vy,vz,wz=x0,y0,z0,yaw0,vx0,vy0,vz0,wz0;T=[[x,y,z]]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy;vyn=vy+(vyt-vy)*a_xy;vzn=vz+(vzt-vz)*a_z;wzn=wz+(wzt-wz)*a_wz
        nyaw=yaw+wzn*DT;vxm,vym,vzm=0.5*(vx+vxn),0.5*(vy+vyn),0.5*(vz+vzn)
        if abs(wzn)<1e-6:dx=(vxm*np.cos(yaw)-vym*np.sin(yaw))*DT;dy=(vxm*np.sin(yaw)+vym*np.cos(yaw))*DT
        else:
            iw=1/wzn;dx=iw*(vxm*(np.sin(nyaw)-np.sin(yaw))+vym*(np.cos(nyaw)-np.cos(yaw)));dy=iw*(vxm*(np.cos(yaw)-np.cos(nyaw))+vym*(np.sin(nyaw)-np.sin(yaw)))
        x,y,z=x+dx,y+dy,z+vzm*DT;yaw=np.arctan2(np.sin(nyaw),np.cos(nyaw));vx,vy,vz,wz=vxn,vyn,vzn,wzn;T.append([x,y,z])
    return np.array(T)
trs=[roll(p) for p in P]
tt=np.arange(H+1)*DT+15.29

# 실제 odom
b=rosbag.Bag('/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag');t0=b.get_start_time()
ot,oy=[],[]
for _,m,t in b.read_messages(topics=['/robot/odom']):
    tr=m.header.stamp.to_sec()-t0
    if 15.0<tr<18.0: ot.append(tr);oy.append(m.pose.pose.position.y)
b.close()

fig,ax=plt.subplots(figsize=(11,6.5))
# 역추진 후보 전부 옅게
for i in np.where(RAW[:,0]<0)[0]:
    ax.plot(tt,trs[i][:,1],color='0.85',lw=0.4,alpha=0.5,zorder=1)
ax.plot(tt,trs[157][:,1],'r-',lw=2.8,marker='o',ms=4,label='선택=coast #157 (끝 -1.43, cost 최소)',zorder=5)
ax.plot(tt,trs[99][:,1],color='magenta',lw=2.8,marker='s',ms=4,
        label='역추진 #99 (goal -0.94 찍고 되돌아감→끝 -0.54)',zorder=5)
ax.plot(ot,oy,'b-',lw=2,alpha=0.7,label='실제 odom (끝 -1.67)',zorder=4)
ax.axhline(-1.0,color='green',ls='--',lw=1.2,label='goal y=-1.0')
ax.axhline(-1.43,color='red',ls=':',lw=0.7,alpha=0.5)
ax.axvline(15.29+2.1,color='gray',ls=':',lw=1)
ax.text(15.29+2.1,-0.45,'cost 채점점\n(2.1s 끝)',fontsize=8,color='gray',ha='center')
ax.annotate('역추진은 goal 찍고\n되돌아가 끝점이 멀어짐\n→ cost 페널티',xy=(15.29+1.2,-0.7),
            xytext=(16.5,-0.62),fontsize=9,color='magenta',
            arrowprops=dict(arrowstyle='->',color='magenta'))
ax.set_xlabel('t (s)');ax.set_ylabel('y (m)')
ax.set_title('왜 cost가 coast(오버슛)를 고르나: 2.1s 끝점만 채점\n'
             '역추진은 goal 통과하지만 끝점이 되돌아가 페널티 → coast(-1.43)가 cost 최소')
ax.legend(loc='lower left',fontsize=8);ax.grid(alpha=0.3);ax.set_xlim(15.2,17.5);ax.set_ylim(-1.8,-0.3)
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_cost_endpoint.png'
plt.tight_layout();plt.savefig(out,dpi=110);print(out)

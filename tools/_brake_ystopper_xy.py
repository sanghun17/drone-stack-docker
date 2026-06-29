#!/usr/bin/env python3
"""y(t)로는 -1.0에 멈춘 듯 보이는 후보가 XY로는 옆으로 새는 것을 보인다.
coast#157(선택) vs 역추진#99(되돌아옴) vs y-stopper#134,#113(X로 샘) + goal + odom."""
import numpy as np, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
DT,H=0.3,7;TAU_XY,TAU_Z,TAU_WZ=0.67,0.075,0.264
VX=[-1.5,-0.75,0.0,0.75,1.5];VY=[-0.5,0.0,0.5];VZ=[-0.5,0.0,0.5];NYAW=7;MAXYAW=1.5
VS_MAX,VS_MIN,VS_RATIO=2.0,1.0,0.5
state=np.array([0.035,-0.555,0.776,-1.671,1.337,0.061,0.027,0.0]);goal=np.array([0.0,-1.0,1.0])
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
def roll(p):
    vxt,vyt,vzt,wzt=p;x,y,z,yaw,vx,vy,vz,wz=x0,y0,z0,yaw0,vx0,vy0,vz0,wz0;T=[[x,y,z,yaw]]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy;vyn=vy+(vyt-vy)*a_xy;vzn=vz+(vzt-vz)*a_z;wzn=wz+(wzt-wz)*a_wz
        nyaw=yaw+wzn*DT;vxm,vym,vzm=0.5*(vx+vxn),0.5*(vy+vyn),0.5*(vz+vzn)
        if abs(wzn)<1e-6:dx=(vxm*np.cos(yaw)-vym*np.sin(yaw))*DT;dy=(vxm*np.sin(yaw)+vym*np.cos(yaw))*DT
        else:
            iw=1/wzn;dx=iw*(vxm*(np.sin(nyaw)-np.sin(yaw))+vym*(np.cos(nyaw)-np.cos(yaw)));dy=iw*(vxm*(np.cos(yaw)-np.cos(nyaw))+vym*(np.sin(nyaw)-np.sin(yaw)))
        x,y,z=x+dx,y+dy,z+vzm*DT;yaw=np.arctan2(np.sin(nyaw),np.cos(nyaw));vx,vy,vz,wz=vxn,vyn,vzn,wzn;T.append([x,y,z,yaw])
    return np.array(T)
trs=[roll(p) for p in P]
b=rosbag.Bag('/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag');t0=b.get_start_time()
ox,oy=[],[]
for _,m,t in b.read_messages(topics=['/robot/odom']):
    tr=m.header.stamp.to_sec()-t0
    if 15.0<tr<18.0:ox.append(m.pose.pose.position.x);oy.append(m.pose.pose.position.y)
b.close()

def arrow(ax,tr,c):  # 끝 yaw 화살표
    ex,ey,eyaw=tr[-1,0],tr[-1,1],tr[-1,3]
    ax.arrow(ex,ey,0.18*np.cos(eyaw),0.18*np.sin(eyaw),head_width=0.05,color=c,zorder=8)

fig,ax=plt.subplots(figsize=(8.5,8))
for tr in trs: ax.plot(tr[:,0],tr[:,1],color='0.88',lw=0.4,alpha=0.4,zorder=1)
for i,c,lab in [(157,'red','선택 coast #157 (끝 -1.43, yaw정렬)'),
                (99,'orange','역추진 #99 (goal찍고 되돌아옴 -0.54)'),
                (134,'magenta','y-stopper #134 (X로 -0.76 샘, yaw -152°)'),
                (113,'purple','y-stopper #113 (X로 +0.99 샘)')]:
    ax.plot(trs[i][:,0],trs[i][:,1],color=c,lw=2.5,marker='o',ms=3,label=lab,zorder=5);arrow(ax,trs[i],c)
ax.plot(ox,oy,'b-',lw=2,alpha=0.6,label='실제 odom',zorder=4)
ax.plot(0.035,-0.555,'bo',ms=11,label='start (0.04,-0.56)',zorder=7)
ax.plot(0,-1.0,'g*',ms=24,label='goal (0,-1.0)',zorder=7)
ax.axhline(-1.0,color='green',ls='--',lw=0.7,alpha=0.5)
ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.axis('equal');ax.grid(alpha=0.3)
ax.set_title('XY: y로 -1.0 근처 멈춘 듯한 후보는 실은 X로 옆으로 샌다\n'
             'cost는 3D위치+yaw로 정당히 버림 (화살표=끝 heading)')
ax.legend(loc='upper left',fontsize=8)
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_ystopper_xy.png'
plt.tight_layout();plt.savefig(out,dpi=115);print(out)

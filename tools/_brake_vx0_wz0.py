#!/usr/bin/env python3
"""yawrate=0(회전없음) 후보만. 왼쪽: vx=0 & wz=0 (9개, vy·vz만). 오른쪽: vy=0 & wz=0,
vx 변화(직진 제동축 5개). goal에 멈추는 직진 후보가 있나 확인."""
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
vsf=np.clip(VS_RATIO+tn*(1-VS_RATIO),VS_RATIO,1.0)
cy,sy=np.cos(yaw0),np.sin(yaw0)
gx_b=cy*(goal[0]-x0)+sy*(goal[1]-y0);gy_b=-sy*(goal[0]-x0)+cy*(goal[1]-y0)
gn=max(np.hypot(gx_b,gy_b),1e-6);gx_b,gy_b=gx_b/gn,gy_b/gn
def scale_vx(vx): return vx*(vsf if (vx*gx_b)>0 else 1.0)  # toward-goal만 감속(비대칭)
def roll(vxt,vyt,vzt,wzt):
    x,y,z,yaw,vx,vy,vz,wz=x0,y0,z0,yaw0,vx0,vy0,vz0,wz0;T=[[x,y,z,yaw]]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy;vyn=vy+(vyt-vy)*a_xy;vzn=vz+(vzt-vz)*a_z;wzn=wz+(wzt-wz)*a_wz
        nyaw=yaw+wzn*DT;vxm,vym,vzm=0.5*(vx+vxn),0.5*(vy+vyn),0.5*(vz+vzn)
        dx=(vxm*np.cos(yaw)-vym*np.sin(yaw))*DT;dy=(vxm*np.sin(yaw)+vym*np.cos(yaw))*DT
        x,y,z=x+dx,y+dy,z+vzm*DT;yaw=nyaw;vx,vy,vz,wz=vxn,vyn,vzn,wzn;T.append([x,y,z,yaw])
    return np.array(T)
b=rosbag.Bag('/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag');t0=b.get_start_time()
ox,oy=[],[]
for _,m,t in b.read_messages(topics=['/robot/odom']):
    tr=m.header.stamp.to_sec()-t0
    if 15.0<tr<18.0:ox.append(m.pose.pose.position.x);oy.append(m.pose.pose.position.y)
b.close()

fig,(axL,axR)=plt.subplots(1,2,figsize=(15,7.2))
# 왼쪽: vx=0 & wz=0 (vy,vz 변화) — 9개
print("=== vx=0 & wz=0 (요청) — 끝점 ===")
for vy in VY:
    for vz in VZ:
        tr=roll(0.0, scale_vx(0.0)*0+vy*1.0 if False else vy, vz, 0.0)  # vx=0
        tr=roll(0.0, vy, vz, 0.0)
        lab=f'vy={vy:+.1f} vz={vz:+.1f}'
        axL.plot(tr[:,0],tr[:,1],marker='o',ms=3,lw=1.6,label=lab)
        if vz==0.0: print(f"  {lab}: 끝=({tr[-1,0]:+.2f},{tr[-1,1]:+.2f},{tr[-1,2]:+.2f})")
axL.plot(ox,oy,'b-',lw=2,alpha=0.5,label='odom',zorder=2)
axL.plot(0.035,-0.555,'ks',ms=10,label='start');axL.plot(0,-1.0,'g*',ms=22,label='goal')
axL.axhline(-1.0,color='green',ls='--',lw=0.7,alpha=0.5)
axL.set_title('요청: vx=0 & yawrate=0 (9개, vy·vz만)\n모두 coast→y≈-1.43 (제동 권한 없음)')
axL.set_xlabel('x');axL.set_ylabel('y');axL.axis('equal');axL.grid(alpha=0.3);axL.legend(fontsize=7,loc='upper left')

# 오른쪽: vy=0 & wz=0, vx 변화 (직진 제동축) — 5개
print("\n=== vy=0 & wz=0, vx 변화 (직진 제동) — 끝점/최근접 ===")
for vx in VX:
    vxs=scale_vx(vx)
    tr=roll(vxs,0.0,0.0,0.0)
    mind=np.linalg.norm(tr[:,:3]-goal,axis=1).min()
    c='red' if vx==0 else ('purple' if vx<0 else 'gray')
    axR.plot(tr[:,0],tr[:,1],marker='o',ms=3,lw=2.2,color=c,
             label=f'vx={vx:+.2f}(scaled {vxs:+.2f}) 끝y={tr[-1,1]:+.2f} 최근접{mind:.2f}')
    print(f"  vx={vx:+.2f}(scaled{vxs:+.2f}): 끝=({tr[-1,0]:+.2f},{tr[-1,1]:+.2f}) 최근접goal={mind:.3f}")
axR.plot(ox,oy,'b-',lw=2,alpha=0.5,label='odom',zorder=2)
axR.plot(0.035,-0.555,'ks',ms=10);axR.plot(0,-1.0,'g*',ms=22,label='goal')
axR.axhline(-1.0,color='green',ls='--',lw=0.7,alpha=0.5)
axR.set_title('직진 제동축: vy=0 & yawrate=0, vx만 변화\n역추진(보라)=goal 찍고 되돌아옴 / coast(빨강)=넘김')
axR.set_xlabel('x');axR.set_ylabel('y');axR.axis('equal');axR.grid(alpha=0.3);axR.legend(fontsize=7,loc='upper left')
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_vx0_wz0.png'
plt.tight_layout();plt.savefig(out,dpi=115);print('\n'+out)

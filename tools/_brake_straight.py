#!/usr/bin/env python3
"""직진 제동 패밀리만: vy=0 & wz=0, vx 변화(5개). body축 표시.
vx=진행/제동축(-Y), vy=측면(+X), wz=회전. vx=0=coast(넘김), vx<0=역추진(goal통과 후 되돌아옴)."""
import numpy as np, rosbag
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
DT,H=0.3,7;TAU_XY,TAU_Z,TAU_WZ=0.67,0.075,0.264
VX=[-1.5,-0.75,0.0,0.75,1.5];VS_MAX,VS_MIN,VS_RATIO=2.0,1.0,0.5
state=np.array([0.035,-0.555,0.776,-1.671,1.337,0.061,0.027,0.0]);goal=np.array([0.0,-1.0,1.0])
a_xy=1-np.exp(-DT/TAU_XY)
x0,y0,z0,yaw0=state[:4];vx0,vy0,vz0,wz0=state[4:]
dist0=np.linalg.norm(state[:3]-goal);tn=(dist0-VS_MIN)/max(VS_MAX-VS_MIN,1e-6)
vsf=np.clip(VS_RATIO+tn*(1-VS_RATIO),VS_RATIO,1.0)
cy,sy=np.cos(yaw0),np.sin(yaw0)
gx_b=cy*(goal[0]-x0)+sy*(goal[1]-y0);gy_b=-sy*(goal[0]-x0)+cy*(goal[1]-y0)
gx_b/=max(np.hypot(gx_b,gy_b),1e-6)
def roll(vxt):
    x,y,z,yaw,vx=x0,y0,z0,yaw0,vx0;T=[[x,y]]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy;vxm=0.5*(vx+vxn)
        x+=vxm*np.cos(yaw)*DT;y+=vxm*np.sin(yaw)*DT;vx=vxn
        T.append([x,y])
    return np.array(T)
b=rosbag.Bag('/home/hmcl/drone-stack-docker/flight_logs/safety_2026-06-26-15-57-36.bag');t0=b.get_start_time()
ox,oy=[],[]
for _,m,t in b.read_messages(topics=['/robot/odom']):
    tr=m.header.stamp.to_sec()-t0
    if 15.0<tr<18.0:ox.append(m.pose.pose.position.x);oy.append(m.pose.pose.position.y)
b.close()

fig,ax=plt.subplots(figsize=(8.5,8.5))
for vx in VX:
    vxs=vx*(vsf if vx*gx_b>0 else 1.0)
    tr=roll(vxs);mind=np.hypot(tr[:,0]-goal[0],tr[:,1]-goal[1]).min()
    c={-1.5:'indigo',-0.75:'magenta',0.0:'red',0.75:'darkorange',1.5:'saddlebrown'}[vx]
    note={-1.5:'역추진강(되돌아옴)',-0.75:'역추진(goal통과→되돌아옴)',0.0:'coast(선택,넘김)',
          0.75:'전진',1.5:'전진강'}[vx]
    ax.plot(tr[:,0],tr[:,1],marker='o',ms=4,lw=2.4,color=c,
            label=f'vx={vx:+.2f}(scaled{vxs:+.2f}) 끝y={tr[-1,1]:+.2f} 최근접{mind:.2f} — {note}')
ax.plot(ox,oy,'b-',lw=2,alpha=0.5,label='실제 odom')
ax.plot(x0,y0,'ks',ms=11,zorder=7);ax.plot(0,-1.0,'g*',ms=26,zorder=7,label='goal(0,-1.0)')
ax.axhline(-1.0,color='green',ls='--',lw=0.7,alpha=0.5)
# body 축 화살표 (start에서)
ax.annotate('',xy=(x0+0.0*np.cos(yaw0)*1.5,y0+np.sin(yaw0)*0.45),xytext=(x0,y0),
            arrowprops=dict(arrowstyle='-|>',color='k',lw=2))
ax.text(x0+np.cos(yaw0)*0.5,y0+np.sin(yaw0)*0.5,'vx(전방=-Y,진행/제동축)',fontsize=8,color='k')
ax.annotate('',xy=(x0-np.sin(yaw0)*0.45,y0+np.cos(yaw0)*0.45),xytext=(x0,y0),
            arrowprops=dict(arrowstyle='-|>',color='gray',lw=1.6))
ax.text(x0-np.sin(yaw0)*0.5+0.05,y0+np.cos(yaw0)*0.5,'vy(측면=+X, 불필요)',fontsize=8,color='gray')
ax.set_title('직진 제동만: vy=0 & yawrate=0, vx 변화\n역추진(vx-0.75)=goal 통과하나 2.1s엔 되돌아감→끝점 멀어 탈락 / coast(vx0)=넘김→선택')
ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.axis('equal');ax.grid(alpha=0.3);ax.legend(fontsize=8,loc='upper left')
out='/tmp/claude-1000/-home-hmcl-drone-stack-docker/6e8e0ebb-070b-4ea5-8e53-219ea330a722/scratchpad/brake_straight.png'
plt.tight_layout();plt.savefig(out,dpi=115);print(out)

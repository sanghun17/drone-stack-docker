#!/usr/bin/env python3
"""Verify Approach B offline: 0.5m GOAL_PROXIMITY brake replan (start odom -0.56,
goal -1.0, next +1.0). Compare single-goal (old) vs multi-goal (new code) cost.
Mirrors motion_primitives + planner_base._goal_cost. Expect argmin flips
coast(-1.43) -> brake-reverse (reaches -1.0, heads +1.0, less overshoot)."""
import numpy as np
DT,H=0.3,7;TAU_XY,TAU_Z,TAU_WZ=0.67,0.075,0.264
VX=[-1.5,-0.75,0.0,0.75,1.5];VY=[-0.5,0.0,0.5];VZ=[-0.5,0.0,0.5];NYAW=7;MAXYAW=1.5
VS_MAX,VS_MIN,VS_RATIO=2.0,1.0,0.5
S_RPOS,S_TDIST,S_RYAW,S_TYAW,S_VDIR=1.0,10.0,1.0,5.0,3.0
CAPTURE=0.3
state=np.array([0.035,-0.555,0.776,-1.671, 1.337,0.061,0.027,0.0])
goal=np.array([0.0,-1.0,1.0]);goal_yaw=-1.570;next_wp=np.array([0.0,1.0,1.0])
a_xy=1-np.exp(-DT/TAU_XY);a_z=1-np.exp(-DT/TAU_Z);a_wz=1-np.exp(-DT/TAU_WZ)
x0,y0,z0,yaw0,vx0,vy0,vz0,wz0=state
dist0=np.linalg.norm(state[:3]-goal);tn=(dist0-VS_MIN)/max(VS_MAX-VS_MIN,1e-6)
vs=np.clip(VS_RATIO+tn*(1-VS_RATIO),VS_RATIO,1.0)
cy,sy=np.cos(yaw0),np.sin(yaw0)
gx_b=cy*(goal[0]-x0)+sy*(goal[1]-y0);gy_b=-sy*(goal[0]-x0)+cy*(goal[1]-y0)
gn=max(np.hypot(gx_b,gy_b),1e-6);gx_b,gy_b=gx_b/gn,gy_b/gn
VXg,VYg,VZg,WZg=np.meshgrid(VX,VY,VZ,np.linspace(-MAXYAW,MAXYAW,NYAW),indexing='ij')
tw=VXg*gx_b+VYg*gy_b;xs=np.where(tw>0,vs,1.0)
P=np.stack([(VXg*xs).ravel(),(VYg*xs).ravel(),(VZg*vs).ravel(),(WZg*vs).ravel()],1)
RAW=np.stack([VXg.ravel(),VYg.ravel(),VZg.ravel(),WZg.ravel()],1)
def wrap(a):return np.arctan2(np.sin(a),np.cos(a))
def roll(p):
    vxt,vyt,vzt,wzt=p;x,y,z,yaw,vx,vy,vz,wz=x0,y0,z0,yaw0,vx0,vy0,vz0,wz0;T=[[x,y,z,yaw,vx,vy,vz,wz]]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy;vyn=vy+(vyt-vy)*a_xy;vzn=vz+(vzt-vz)*a_z;wzn=wz+(wzt-wz)*a_wz
        nyaw=yaw+wzn*DT;vxm,vym,vzm=0.5*(vx+vxn),0.5*(vy+vyn),0.5*(vz+vzn)
        if abs(wzn)<1e-6:dx=(vxm*np.cos(yaw)-vym*np.sin(yaw))*DT;dy=(vxm*np.sin(yaw)+vym*np.cos(yaw))*DT
        else:
            iw=1/wzn;dx=iw*(vxm*(np.sin(nyaw)-np.sin(yaw))+vym*(np.cos(nyaw)-np.cos(yaw)));dy=iw*(vxm*(np.cos(yaw)-np.cos(nyaw))+vym*(np.sin(nyaw)-np.sin(yaw)))
        x,y,z=x+dx,y+dy,z+vzm*DT;yaw=np.arctan2(np.sin(nyaw),np.cos(nyaw));vx,vy,vz,wz=vxn,vyn,vzn,wzn
        T.append([x,y,z,yaw,vx,vy,vz,wz])
    return np.array(T)
trs=[roll(p) for p in P]

def eff_goal(pos):  # branchless multi-goal (== planner_base._effective_goal_per_point)
    captured=np.linalg.norm(pos-goal,axis=1)<CAPTURE
    reached=np.cumsum(captured)>0
    return np.where(reached[:,None], next_wp, goal)

def cost(tr, multigoal):
    pos=tr[:,:3];yaws=tr[:,3];tp=tr[-1,:3];ty=tr[-1,3];tv=tr[-1,4:8]
    ge=eff_goal(pos) if multigoal else np.broadcast_to(goal,pos.shape)
    d=np.linalg.norm(pos-ge,axis=1);rpos=np.mean(d-d[0]);tpos=np.linalg.norm(tp-ge[-1])
    ryaw=np.mean(np.abs(wrap(yaws-goal_yaw)));tyaw=np.abs(wrap(ty-goal_yaw))
    cyt,syt=np.cos(ty),np.sin(ty);wvx=tv[0]*cyt-tv[1]*syt;wvy=tv[0]*syt+tv[1]*cyt
    dirn=next_wp[:2]-tp[:2];dn=max(np.linalg.norm(dirn),1e-6);vn=np.hypot(wvx,wvy)
    cs=np.dot([wvx,wvy],dirn)/(max(vn,1e-6)*dn);vdir=(1-cs)*min(vn,1.0)
    return S_RPOS*rpos+S_TDIST*tpos+S_RYAW*ryaw+S_TYAW*tyaw+S_VDIR*vdir

for mode in [False, True]:
    tot=np.array([cost(tr,mode) for tr in trs]);bi=int(np.argmin(tot))
    miny=trs[bi][:,1].min()
    tag="MULTI-goal (new, B)" if mode else "SINGLE-goal (old)"
    print(f"\n[{tag}] argmin #{bi}: raw_vx={RAW[bi,0]:+.2f} vy={RAW[bi,1]:+.1f} vz={RAW[bi,2]:+.1f} wz={RAW[bi,3]:+.2f}")
    print(f"  plan y(t)={np.round(trs[bi][:,1],2)}")
    print(f"  end_y={trs[bi][-1,1]:+.2f}  min_y(최대전진)={miny:+.2f}  goal(-1.0) 기준 오버슛={abs(miny+1.0):.2f}")

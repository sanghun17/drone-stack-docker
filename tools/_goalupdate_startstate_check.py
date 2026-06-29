#!/usr/bin/env python3
"""fix(b) offline check: 15.64 GOAL_UPDATED replan (goal flips to +1.0) start-state 비교.
 ODOM(fix b): real odom y=-1.02, vy=-1.33 (실제 운동량).
 LOOKAHEAD(old): coast plan eval y=-0.68, vel=0 (저장 target=0, 정지로 오판).
motion_primitives+_goal_cost 재현으로 argmin plan의 예측 min-y(=−Y 최대전진=오버슛)·끝점 비교.
figure 없음, 텍스트만."""
import numpy as np
DT,H=0.3,7;TAU_XY,TAU_Z,TAU_WZ=0.67,0.075,0.264
VX=[-1.5,-0.75,0.0,0.75,1.5];VY=[-0.5,0.0,0.5];VZ=[-0.5,0.0,0.5];NYAW=7;MAXYAW=1.5
VS_MAX,VS_MIN,VS_RATIO=2.0,1.0,0.5
S_RPOS,S_TDIST,S_RYAW,S_TYAW,S_TVEL,S_VDIR=1.0,10.0,1.0,5.0,5.0,3.0
goal=np.array([0.0,1.0,1.0]);goal_yaw=-1.570;next_wp=np.array([0.0,-1.0,1.0])
a_xy=1-np.exp(-DT/TAU_XY);a_z=1-np.exp(-DT/TAU_Z);a_wz=1-np.exp(-DT/TAU_WZ)
def wrap(a):return np.arctan2(np.sin(a),np.cos(a))

def run(label, st):
    x0,y0,z0,yaw0,vx0,vy0,vz0,wz0=st
    dist0=np.linalg.norm(st[:3]-goal);tn=(dist0-VS_MIN)/max(VS_MAX-VS_MIN,1e-6)
    vs=np.clip(VS_RATIO+tn*(1-VS_RATIO),VS_RATIO,1.0)
    cy,sy=np.cos(yaw0),np.sin(yaw0)
    gx_b=cy*(goal[0]-x0)+sy*(goal[1]-y0);gy_b=-sy*(goal[0]-x0)+cy*(goal[1]-y0)
    gn=max(np.hypot(gx_b,gy_b),1e-6);gx_b,gy_b=gx_b/gn,gy_b/gn
    VXg,VYg,VZg,WZg=np.meshgrid(VX,VY,VZ,np.linspace(-MAXYAW,MAXYAW,NYAW),indexing='ij')
    tw=VXg*gx_b+VYg*gy_b;xs=np.where(tw>0,vs,1.0)
    P=np.stack([(VXg*xs).ravel(),(VYg*xs).ravel(),(VZg*vs).ravel(),(WZg*vs).ravel()],1)
    RAW=np.stack([VXg.ravel(),VYg.ravel(),VZg.ravel(),WZg.ravel()],1)
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
    def cost(tr):
        pos=tr[:,:3];yaws=tr[:,3];tp=tr[-1,:3];ty=tr[-1,3];tv=tr[-1,4:8]
        d=np.linalg.norm(pos-goal,axis=1);rpos=np.mean(d-d[0]);tpos=np.linalg.norm(tp-goal)
        ryaw=np.mean(np.abs(wrap(yaws-goal_yaw)));tyaw=np.abs(wrap(ty-goal_yaw))
        cyt,syt=np.cos(ty),np.sin(ty);wvx=tv[0]*cyt-tv[1]*syt;wvy=tv[0]*syt+tv[1]*cyt
        dirn=next_wp[:2]-tp[:2];dn=max(np.linalg.norm(dirn),1e-6);vn=np.hypot(wvx,wvy)
        cos=np.dot([wvx,wvy],dirn)/(max(vn,1e-6)*dn);vdir=(1-cos)*min(vn,1.0)
        return S_RPOS*rpos+S_TDIST*tpos+S_RYAW*ryaw+S_TYAW*tyaw+S_VDIR*vdir
    trs=[roll(p) for p in P]
    tot=np.array([cost(tr) for tr in trs])
    bi=int(np.argmin(tot))
    miny=trs[bi][:,1].min()
    print(f"\n[{label}] start=(y={y0:+.2f}, vy_body={vx0:+.2f}→world≈{np.sin(yaw0)*vx0:+.2f}), vel_scale={vs:.2f}")
    print(f"  argmin: raw_vx={RAW[bi,0]:+.2f} vy={RAW[bi,1]:+.1f} vz={RAW[bi,2]:+.1f} wz={RAW[bi,3]:+.2f}")
    print(f"  plan y(t)={np.round(trs[bi][:,1],2)}")
    print(f"  예측 min_y(=-Y 최대전진/오버슛)={miny:+.2f}  end_y={trs[bi][-1,1]:+.2f}")
    print(f"  → goal(-1.0 기준) 예측 오버슛 = {abs(miny+1.0):.2f} m")
    return miny

# yaw -96° = -1.672. body vx>0 = -Y forward.
odom = np.array([0.003,-1.020,0.774,-1.672, 1.335,0.047,-0.048,0.003])      # fix(b): 실제
look = np.array([0.003,-0.680,0.774,-1.672, 0.0,0.0,0.0,0.0])               # old: lookahead, vel=0
print("="*70)
print("실제 −Y 오버슛(odom)은 −1.67 였음 (이 plan들이 드론에 갔다면 얼마였을지 비교)")
m_o=run("fix(b) ODOM-start", odom)
m_l=run("OLD lookahead-start (vel=0)", look)
print("\n"+"="*70)
print(f"요약: ODOM-start 예측 오버슛 {abs(m_o+1.0):.2f}m vs lookahead {abs(m_l+1.0):.2f}m")
print("lookahead는 '정지'로 봐서 오버슛 거의 0 예측(낙관/비현실), ODOM은 운동량 보고 실제 제동 거리 예측")

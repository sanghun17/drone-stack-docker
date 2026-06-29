#!/usr/bin/env python3
"""brake replan(15.29) 후보를 motion_primitives + _goal_cost 그대로 numpy 재현(jax 불필요).
검증: 재현 argmin == 기록된 선택(vx_target≈0, terminal_y≈-1.43, t_pos≈4.82, total≈5.67).
그 뒤 dissect: goal에 3D로 멈추는 후보가 cost로 왜 탈락하나 / terminal_velocity_cost(정지유인)를
켜면(is_last_wp=True) stopper가 이기나.  소스: motion_primitives.py, planner_base.py _goal_cost.
"""
import numpy as np

# ---- params (jax_mppi_params.py 그대로) ----
DT, H = 0.3, 7
TAU_XY, TAU_Z, TAU_WZ = 0.67, 0.075, 0.264
VX=[-1.5,-0.75,0.0,0.75,1.5]; VY=[-0.5,0.0,0.5]; VZ=[-0.5,0.0,0.5]
NYAW=7; MAXYAW=1.5
VS_MAX,VS_MIN,VS_RATIO = 2.0,1.0,0.5
S_RPOS,S_TDIST,S_RYAW,S_TYAW,S_TVEL,S_LOS,S_PROG,S_VDIR = 1.0,10.0,1.0,5.0,5.0,0.0,0.0,3.0

# ---- recorded state ----
state = np.array([0.035,-0.555,0.776,-1.671, 1.337,0.061,0.027,0.0])
goal  = np.array([0.0,-1.0,1.0,-1.570])         # x,y,z,yaw
next_wp = np.array([0.0,1.0,1.0])
goal_xyz, goal_yaw = goal[:3], goal[3]

a_xy=1-np.exp(-DT/TAU_XY); a_z=1-np.exp(-DT/TAU_Z); a_wz=1-np.exp(-DT/TAU_WZ)
x0,y0,z0,yaw0 = state[:4]; vx0,vy0,vz0,wz0 = state[4:]

# ---- vel_scale (asymmetric, motion_primitives.py:114-144) ----
dist0 = np.linalg.norm(state[:3]-goal_xyz)
tnorm = (dist0-VS_MIN)/max(VS_MAX-VS_MIN,1e-6)
vel_scale = np.clip(VS_RATIO + tnorm*(1.0-VS_RATIO), VS_RATIO, 1.0)
gx_w,gy_w = goal_xyz[0]-x0, goal_xyz[1]-y0
cy,sy=np.cos(yaw0),np.sin(yaw0)
gx_b=cy*gx_w+sy*gy_w; gy_b=-sy*gx_w+cy*gy_w
gn=max(np.hypot(gx_b,gy_b),1e-6); gx_b,gy_b=gx_b/gn,gy_b/gn
print(f"dist0={dist0:.3f} vel_scale={vel_scale:.3f} goal_body_dir=({gx_b:+.2f},{gy_b:+.2f})")

# meshgrid ij
VXg,VYg,VZg,WZg = np.meshgrid(VX,VY,VZ,np.linspace(-MAXYAW,MAXYAW,NYAW),indexing='ij')
toward = VXg*gx_b + VYg*gy_b
xy_scale = np.where(toward>0.0, vel_scale, 1.0)
VXs=VXg*xy_scale; VYs=VYg*xy_scale; VZs=VZg*vel_scale; WZs=WZg*vel_scale
params = np.stack([VXs.ravel(),VYs.ravel(),VZs.ravel(),WZs.ravel()],1)  # (315,4)
raw    = np.stack([VXg.ravel(),VYg.ravel(),VZg.ravel(),WZg.ravel()],1)  # 원래 grid값(coast/reverse 식별용)

def rollout(p):
    vxt,vyt,vzt,wzt=p
    x,y,z,yaw,vx,vy,vz,wz=x0,y0,z0,yaw0,vx0,vy0,vz0,wz0
    traj=[[x,y,z,yaw,vx,vy,vz,wz]]
    for _ in range(H):
        vxn=vx+(vxt-vx)*a_xy; vyn=vy+(vyt-vy)*a_xy
        vzn=vz+(vzt-vz)*a_z;  wzn=wz+(wzt-wz)*a_wz
        nyaw=yaw+wzn*DT
        vxm,vym,vzm=0.5*(vx+vxn),0.5*(vy+vyn),0.5*(vz+vzn)
        if abs(wzn)<1e-6:
            dx=(vxm*np.cos(yaw)-vym*np.sin(yaw))*DT; dy=(vxm*np.sin(yaw)+vym*np.cos(yaw))*DT
        else:
            iw=1.0/wzn
            dx=iw*(vxm*(np.sin(nyaw)-np.sin(yaw))+vym*(np.cos(nyaw)-np.cos(yaw)))
            dy=iw*(vxm*(np.cos(yaw)-np.cos(nyaw))+vym*(np.sin(nyaw)-np.sin(yaw)))
        x,y,z=x+dx,y+dy,z+vzm*DT
        yaw=np.arctan2(np.sin(nyaw),np.cos(nyaw)); vx,vy,vz,wz=vxn,vyn,vzn,wzn
        traj.append([x,y,z,yaw,vx,vy,vz,wz])
    return np.array(traj)

def wrap(a): return np.arctan2(np.sin(a),np.cos(a))

def cost(tr, last_wp):
    pos=tr[:,:3]; yaws=tr[:,3]; tp=tr[-1,:3]; ty=tr[-1,3]; tv=tr[-1,4:8]
    d=np.linalg.norm(pos-goal_xyz,axis=1)
    rpos=np.mean(d-d[0])
    tpos=np.linalg.norm(tp-goal_xyz)
    ryaw=np.mean(np.abs(wrap(yaws-goal_yaw)))
    tyaw=np.abs(wrap(ty-goal_yaw))
    # world terminal vel xy from body
    cyt,syt=np.cos(ty),np.sin(ty)
    wvx=tv[0]*cyt-tv[1]*syt; wvy=tv[0]*syt+tv[1]*cyt
    tdist=np.linalg.norm(tp-goal_xyz); prox=np.clip(1-tdist/1.0,0,1)
    tvel = prox*np.linalg.norm(tv[:3]) if last_wp else 0.0
    if last_wp: vdir=0.0
    else:
        dirn=next_wp[:2]-tp[:2]; dn=max(np.linalg.norm(dirn),1e-6)
        vn=np.hypot(wvx,wvy); vh=np.array([wvx,wvy])/max(vn,1e-6)
        cos=np.dot(vh,dirn/dn); vdir=(1-cos)*min(vn,1.0)
    total=S_RPOS*rpos+S_TDIST*tpos+S_RYAW*ryaw+S_TYAW*tyaw+S_TVEL*tvel+S_VDIR*vdir
    return dict(total=total,rpos=rpos,tpos=tpos,ryaw=ryaw,tyaw=tyaw,tvel=tvel,vdir=vdir,
                term=tp, tvxy=(wvx,wvy))

trajs=[rollout(p) for p in params]

# ===== 검증: is_last_wp=False (경유점) =====
C=[cost(tr,last_wp=False) for tr in trajs]
tot=np.array([c['total'] for c in C])
bi=int(np.argmin(tot))
print(f"\n=== 재현 argmin (경유점, t_vel OFF) — 기록된 선택과 대조 ===")
def show(i,c):
    print(f"  #{i} raw_vx={raw[i,0]:+.2f} scaled_vx={params[i,0]:+.2f} vz={raw[i,2]:+.1f} wz={raw[i,3]:+.2f} "
          f"| term=({c['term'][0]:+.2f},{c['term'][1]:+.2f},{c['term'][2]:+.2f}) "
          f"total={c['total']:.2f} t_pos={S_TDIST*c['tpos']:.2f} vdir={S_VDIR*c['vdir']:.2f} "
          f"t_yaw={S_TYAW*c['tyaw']:.2f} r_pos={c['rpos']:+.2f}")
show(bi,C[bi])
print(f"  (기록: vx_target≈0, term_y≈-1.43, total≈5.67, t_pos≈4.82, vdir≈0.45)")

# goal y 에 가장 가깝게 멈추는(3D term_dist 작고 |term vy| 작은) 후보
termy=np.array([c['term'][1] for c in C])
tdist3=np.array([c['tpos'] for c in C])
near=np.argsort(tdist3)[:5]
print(f"\n=== 3D terminal_dist 최소 후보 5개 ===")
for i in near: show(i,C[i])

# ===== 가정: terminal_velocity_cost ON (정지 유인 켜면) =====
C2=[cost(tr,last_wp=True) for tr in trajs]
tot2=np.array([c['total'] for c in C2])
bi2=int(np.argmin(tot2))
print(f"\n=== 만약 t_vel ON (정지유인) 이면 argmin ===")
show(bi2,C2[bi2])
print(f"  → terminal y={C2[bi2]['term'][1]:+.2f} (goal -1.0 에 {'근접' if abs(C2[bi2]['term'][1]+1.0)<0.2 else '여전히 넘김'})")

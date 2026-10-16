#!/usr/bin/env python3
"""Test plane-normal-constrained marker localization from recorded IMU attitude.
GT is used only to learn the initial placement and evaluate poses, except for
an explicitly labelled optional scale-fit experiment on takeoff samples.
"""
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
spec=importlib.util.spec_from_file_location('cv2','/usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so');cv2=importlib.util.module_from_spec(spec);sys.modules['cv2']=cv2;spec.loader.exec_module(cv2)
import rosbag
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent));from audit import interpolate,quantile
CORNERS=np.array([[-.5,.5,0],[.5,.5,0],[.5,-.5,0],[-.5,-.5,0]])

def normal_pose(obj,pix,K,D,R0):
 """Known plane normal turns planar PnP into a 2D similarity fit."""
 rays=np.c_[cv2.undistortPoints(pix.reshape(-1,1,2),K,D).reshape(-1,2),np.ones(len(pix))]@R0
 if np.any(abs(rays[:,2])<.1):return None
 q=rays[:,:2]/rays[:,2,None]
 a=obj[:,:2]-obj[:,:2].mean(0);b=q-q.mean(0)
 U,s,Vt=np.linalg.svd(a.T@b);R2=Vt.T@U.T
 if np.linalg.det(R2)<0:Vt[-1]*=-1;R2=Vt.T@U.T
 scale=np.sum((a@R2.T)*b)/np.sum(a*a)
 sign=np.sign(np.median(rays[:,2]));R2*=sign;z=sign/scale
 shift=q.mean(0)-(obj[:,:2]@R2.T).mean(0)/z
 # q = R2*p/z + t_xy/z, including negative z when normal faces camera.
 T=np.eye(4);rot=np.eye(3);rot[:2,:2]=R2;T[:3,:3]=R0@rot;T[:3,3]=R0@np.r_[shift*z,z]
 if np.any((obj@T[:3,:3].T+T[:3,3])[:,2]<=0):return None
 return T

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('replay',type=Path);args=p.parse_args();d=json.loads(args.replay.read_text());K=np.array(d['crop_K']);D=np.array(d['D']);X=np.array(d['body_from_camera']);XI=np.linalg.inv(X)
 models={}
 for m in d['layout']['markers']:
  models[str(m['id'])]=CORNERS*m['side_m']@Rotation.from_euler('z',m['yaw_deg'],degrees=True).as_matrix().T+[m['center_m']['x'],m['center_m']['y'],0]
 imu=[]
 with rosbag.Bag(str(args.bag)) as b:
  for _,m,t in b.read_messages(topics=['/mavros/imu/data']):
   q=m.orientation;T=np.eye(4);T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();imu.append((m.header.stamp.to_sec(),T))
 rows=[r for r in d['rows'] if 'gt' in r];att,ok=interpolate(imu,np.array([r['stamp'] for r in rows]));out=[]
 pairs=[(r,I) for r,I,valid in zip(rows,att,ok) if valid and r['bag_s']<1.2 and r['gt_body_height_m']<.4]
 g0=np.mean([np.array(r['gt'])[:3,:3].T@np.array([0,0,1]) for r,I in pairs],axis=0);i0=np.mean([I[:3,:3].T@np.array([0,0,1]) for r,I in pairs],axis=0)
 g0/=np.linalg.norm(g0);i0/=np.linalg.norm(i0);cross=np.cross(i0,g0);angle=np.arccos(np.clip(i0@g0,-1,1));Q=Rotation.from_rotvec(cross/np.linalg.norm(cross)*angle).as_matrix() if np.linalg.norm(cross)>1e-10 else np.eye(3)
 print('initial attitude reference tilt correction deg',np.rad2deg(angle),flush=True)
 for row,I,valid in zip(rows,att,ok):
  if not valid:continue
  ids=[str(mid) for mid in row['ids'] if str(mid) in models and np.linalg.norm(np.array(row['corners'][str(mid)])-np.roll(np.array(row['corners'][str(mid)]),-1,axis=0),axis=1).mean()>=12]
  if not ids:continue
  pix=np.concatenate([row['corners'][mid] for mid in ids]);obj=np.concatenate([models[mid] for mid in ids]);R0=X[:3,:3].T@Q@I[:3,:3].T
  Z=normal_pose(obj,pix,K,D,R0)
  if Z is None:continue
  rv=cv2.Rodrigues(Z[:3,:3])[0];project=cv2.projectPoints(obj,rv,Z[:3,3],K,D)[0].reshape(-1,2);rms=float(np.sqrt(np.mean(np.sum((project-pix)**2,axis=1))))
  out.append(dict(row,gravity_Z=Z.tolist(),gravity_rms_px=rms))
 initial=[r for r in out if r['bag_s']<1.2 and r['gt_body_height_m']<.4]
 if len(initial)<5:raise RuntimeError('Too few initial low-altitude samples')
 asc=[r for r in out if r['bag_s']<7.7 and r['gt_body_height_m']>.4]
 az=[];zz=[]
 for r in asc:
  Z=np.array(r['gravity_Z']);az.append((-Z[:3,:3].T@Z[:3,3])[2]);zz.append(np.array(r['gt'])[2,3]-(Z[:3,:3].T@XI[:3,3])[2])
 scale=float(np.linalg.lstsq(np.c_[az,np.ones(len(az))],zz,rcond=None)[0][0])
 results={}
 for mode,s in [('gravity_level',1.),('gravity_level_scale_fitted_on_takeoff',scale)]:
  pairs=[]
  for r in initial:
   Z=np.array(r['gravity_Z']);Z[:3,3]*=s;P=np.linalg.inv(Z)@XI;A=np.array(r['gt']);Y=A@np.linalg.inv(P);pairs.append((A,P,np.arctan2(Y[1,0],Y[0,0])))
  yaw=np.arctan2(np.mean([np.sin(p[2]) for p in pairs]),np.mean([np.cos(p[2]) for p in pairs]));Y=np.eye(4);Y[:3,:3]=Rotation.from_euler('z',yaw).as_matrix();Y[:3,3]=np.median([A[:3,3]-Y[:3,:3]@P[:3,3] for A,P,_ in pairs],axis=0)
  sample=[]
  for r in out:
   Z=np.array(r['gravity_Z']);Z[:3,3]*=s;P=np.linalg.inv(Z)@XI;G=Y@P;A=np.array(r['gt']);sample.append({'bag_s':r['bag_s'],'height':r['gt_body_height_m'],'error_m':float(np.linalg.norm(G[:3,3]-A[:3,3])),'angle_deg':float(np.rad2deg(Rotation.from_matrix(A[:3,:3].T@G[:3,:3]).magnitude())),'rms_px':r['gravity_rms_px']})
  groups={}
  for name,sel in [('all',sample),('heldout_descent',[v for v in sample if v['bag_s']>=9.72]),('high',[v for v in sample if v['height']>=1.7])]:
   groups[name]={'n':len(sel),'error_m_p50_p95_max':quantile([v['error_m'] for v in sel]),'angle_deg_p50_p95_max':quantile([v['angle_deg'] for v in sel]),'rms_px_p50_p95_max':quantile([v['rms_px'] for v in sel])}
  results[mode]={'metric_scale':s,'initial_ground_samples':len(initial),'attitude_reference_rotation':Q.tolist(),'Y':Y.tolist(),'metrics':groups,'samples':sample};print(mode,s,groups,flush=True)
 (args.replay.parent/'gravity_reference_replay.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':main()

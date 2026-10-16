#!/usr/bin/env python3
"""Diagnostic GT-assisted static-corner fit with takeoff/descent separation.
No runtime values are written. Effective rigid-body/camera calibration cannot
separate a changed OptiTrack body frame from a changed camera mount.
"""
import argparse,importlib.util,json,sys
from pathlib import Path
import numpy as np
spec=importlib.util.spec_from_file_location('cv2','/usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so');cv2=importlib.util.module_from_spec(spec);sys.modules['cv2']=cv2;spec.loader.exec_module(cv2)
import rosbag
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
sys.path.insert(0,str(Path(__file__).resolve().parent));from audit import matrix

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('replay',type=Path);args=p.parse_args();d=json.loads(args.replay.read_text());K=np.array(d['crop_K']);D=np.array(d['D']);X0=np.array(d['body_from_camera']);Y0=np.array(d['frozen_global_pad'])
 A=[]
 with rosbag.Bag(str(args.bag)) as b:
  for _,m,t in b.read_messages(topics=['/vrpn_client_node/pure/pose']):A.append((m.header.stamp.to_sec(),matrix(m.pose)))
 times=np.array([v[0] for v in A]);epoch=times[0];times=times-epoch;mats=np.array([v[1] for v in A]);unique=np.r_[True,np.diff(times)>0];times=times[unique];mats=mats[unique];slerp=Slerp(times,Rotation.from_matrix(mats[:,:3,:3]))
 def body(t):
  t=np.clip(t,times[0],times[-1]);T=np.tile(np.eye(4),(len(t),1,1));T[:,:3,:3]=slerp(t).as_matrix();T[:,:3,3]=np.column_stack([np.interp(t,times,mats[:,i,3]) for i in range(3)]);return T
 corners=np.array([[-.5,.5,0],[.5,.5,0],[.5,-.5,0],[-.5,-.5,0]])
 objects=[]
 for mid in [0,1]:
  m=next(m for m in d['layout']['markers'] if m['id']==mid);R=Rotation.from_euler('z',m['yaw_deg'],degrees=True).as_matrix();pts=corners*m['side_m']@R.T+[m['center_m']['x'],m['center_m']['y'],0];objects.extend(pts@Y0[:3,:3].T+Y0[:3,3])
 objects=np.array(objects)
 rows=[]
 for r in d['rows']:
  if 'gt' not in r or 'combined' not in r['poses']:continue
  keys=[];pixels=[]
  for mid in [0,1]:
   if str(mid) not in r['corners']:continue
   c=np.array(r['corners'][str(mid)])
   if np.linalg.norm(c-np.roll(c,-1,axis=0),axis=1).mean()<12:continue
   keys.extend(range(mid*4,mid*4+4));pixels.extend(c)
  if keys:rows.append(dict(r,point_ids=np.array(keys),pixels=np.array(pixels)))
 train=[r for r in rows if r['bag_s']<7.7][::2];held=[r for r in rows if r['bag_s']>=9.72]
 print('static ray fit',len(train),'train',len(held),'heldout',flush=True)
 def arrays(rs):
  return np.array([r['stamp']-epoch for r in rs]),np.concatenate([np.full(len(r['point_ids']),i) for i,r in enumerate(rs)]),np.concatenate([r['point_ids'] for r in rs]),np.concatenate([r['pixels'] for r in rs])
 def decode(x,mode):
  points=x[:24].reshape(8,3);X=X0.copy();dt=0
  if mode!='points_only':X[:3,:3]=Rotation.from_rotvec(x[24:27]).as_matrix()@X0[:3,:3];dt=x[27]
  return points,X,dt
 def errors(x,mode,arr):
  points,X,dt=decode(x,mode);ts,fi,pi,pix=arr;C=body(ts+dt)@X
  pc=np.einsum('nij,nj->ni',np.swapaxes(C[fi,:3,:3],1,2),points[pi]-C[fi,:3,3]);image=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
  return (image-pix).ravel()
 outputs={};atrain=arrays(train);aheld=arrays(held)
 for mode in ['points_only','points_and_effective_rotation_time']:
  x0=objects.ravel();bounds=np.full(24,.15)
  if mode!='points_only':x0=np.r_[x0,0,0,0,0];bounds=np.r_[bounds,[.15,.15,.15,.1]]
  def residual(x):
   e=errors(x,mode,atrain)
   # Very weak yaw gauge regularization; the nearly vertical flight weakly
   # observes mount yaw independently of map yaw.
   return np.r_[e,x[26]/np.deg2rad(10)] if mode!='points_only' else e
  fit=least_squares(residual,x0,bounds=(x0-bounds,x0+bounds),loss='soft_l1',f_scale=.5,x_scale='jac',max_nfev=150)
  points,X,dt=decode(fit.x,mode);e=errors(fit.x,mode,aheld)
  result={'success':bool(fit.success),'nfev':fit.nfev,'rotation_delta_xyz_deg':Rotation.from_matrix(X[:3,:3]@X0[:3,:3].T).as_euler('xyz',degrees=True).tolist(),'time_change_s':float(dt),'effective_body_from_camera':X.tolist(),'world_corners':points.tolist(),'train_corner_rmse_px':float(np.sqrt(np.mean(errors(fit.x,mode,atrain)**2))),'heldout_corner_rmse_px':float(np.sqrt(np.mean(e**2))),'near_bounds':np.flatnonzero(abs(fit.x-x0)>bounds*.97).tolist()}
  result['markers']=[]
  for mid in [0,1]:
   pts=points[mid*4:mid*4+4];_,s,v=np.linalg.svd(pts-pts.mean(0));normal=v[-1]
   if normal[2]<0:normal=-normal
   result['markers'].append({'id':mid,'edge_lengths_m':np.linalg.norm(pts-np.roll(pts,-1,axis=0),axis=1).tolist(),'normal_global':normal.tolist(),'tilt_from_world_up_deg':float(np.rad2deg(np.arccos(normal[2]))),'nonplanarity_singular_value_m':float(s[-1])})
  perframe=[]
  for r in held:
   v=np.array(r['poses']['combined']['camera_from_pad']);initial_C_G=v@np.linalg.inv(Y0)
   rv=cv2.Rodrigues(initial_C_G[:3,:3])[0];tv=initial_C_G[:3,3].reshape(3,1)
   try:rv,tv=cv2.solvePnPRefineLM(points[r['point_ids']],r['pixels'],K,D,rv,tv)
   except cv2.error:continue
   C_G=np.eye(4);C_G[:3,:3]=cv2.Rodrigues(rv)[0];C_G[:3,3]=tv.ravel();G_B=np.linalg.inv(C_G)@np.linalg.inv(X);GT=body(np.array([r['stamp']-epoch+dt]))[0]
   perframe.append({'bag_s':r['bag_s'],'gt_height_m':r['gt_body_height_m'],'position_error_m':float(np.linalg.norm(G_B[:3,3]-GT[:3,3])),'angle_error_deg':float(np.rad2deg(Rotation.from_matrix(GT[:3,:3].T@G_B[:3,:3]).magnitude()))})
  result['heldout_pose_error_m_p50_p95_max']=np.percentile([r['position_error_m'] for r in perframe],[50,95,100]).tolist();result['heldout_angle_error_deg_p50_p95_max']=np.percentile([r['angle_error_deg'] for r in perframe],[50,95,100]).tolist();result['heldout_per_frame']=perframe;outputs[mode]=result
  print(mode,{k:v for k,v in result.items() if k not in ['world_corners','effective_body_from_camera','heldout_per_frame']},flush=True)
 (args.replay.parent/'static_ray_fit.json').write_text(json.dumps({'method':'fit static world corners to raw image pixels and GT; takeoff train/descent heldout; mount translation and intrinsics fixed','limitations':'Single flight. Effective mount rotation confounds GT body axes and camera mount; free corner geometry can absorb intrinsic errors. Not a deployable calibration.','models':outputs},indent=2)+'\n')

if __name__=='__main__':main()

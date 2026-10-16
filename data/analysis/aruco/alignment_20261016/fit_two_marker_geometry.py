#!/usr/bin/env python3
"""Image-only relative two-marker calibration; diagnostic, never auto-deployed.
Train on takeoff and validate on descent. Anchor ID1 is fixed in metric scale;
absolute scale/intrinsic validity is not established by this experiment.
"""
import argparse,json,sys
from pathlib import Path
import importlib.util
import numpy as np
spec=importlib.util.spec_from_file_location('cv2','/usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so')
cv2=importlib.util.module_from_spec(spec);sys.modules['cv2']=cv2;spec.loader.exec_module(cv2)
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation

CORNERS=np.array([[-.5,.5,0],[.5,.5,0],[.5,-.5,0],[-.5,-.5,0]])

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('replay',type=Path);args=p.parse_args();d=json.loads(args.replay.read_text());K=np.array(d['crop_K']);D=np.array(d['D']);layout={m['id']:m for m in d['layout']['markers']}
 def nominal(mid):
  m=layout[mid];R=Rotation.from_euler('z',m['yaw_deg'],degrees=True).as_matrix();center=np.array([m['center_m']['x'],m['center_m']['y'],0])
  return CORNERS*m['side_m']@R.T+center
 anchor=nominal(1);m0=layout[0];r0=Rotation.from_euler('z',m0['yaw_deg'],degrees=True).as_matrix();c0=np.array([m0['center_m']['x'],m0['center_m']['y'],0])
 def geometry(v,mode):
  if mode=='nominal':return np.r_[nominal(0),anchor]
  if mode=='planar':dx,dy,logs,rz=v;delta=np.array([dx,dy,0]);rot=Rotation.from_rotvec([0,0,rz]).as_matrix()
  else:delta=v[:3];logs=v[3];rot=Rotation.from_rotvec(v[4:7]).as_matrix()
  return np.r_[CORNERS*(m0['side_m']*np.exp(logs))@(rot@r0).T+c0+delta,anchor]
 eligible=[]
 for row in d['rows']:
  if not all(str(mid) in row['corners'] for mid in [0,1]) or 'combined' not in row['poses'] or 'gt' not in row:continue
  corners={mid:np.array(row['corners'][str(mid)]) for mid in [0,1]}
  side={mid:np.linalg.norm(v-np.roll(v,-1,axis=0),axis=1).mean() for mid,v in corners.items()}
  if side[1]<35 or side[0]<100:continue
  row=dict(row);row['pixels']=np.r_[corners[0],corners[1]];eligible.append(row)
 train=[r for r in eligible if r['bag_s']<7.7];test=[r for r in eligible if r['bag_s']>=9.72]
 if len(train)<10 or len(test)<10:raise RuntimeError('Insufficient resolved takeoff/descent frames')
 train=train[::max(1,len(train)//70)]
 print('geometry observations',len(train),'train',len(test),'heldout',flush=True)
 def posevec(row):
  T=np.array(row['poses']['combined']['camera_from_pad']);return np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]]
 def project(points,v):return cv2.projectPoints(points,v[:3],v[3:],K,D)[0].reshape(-1,2)
 def evaluate(rows,points):
  allerr=[];bodyerr=[];angles=[];perframe=[]
  X=np.array(d['body_from_camera']);Y=np.array(d['frozen_global_pad'])
  for row in rows:
   v=posevec(row);rv,tv=cv2.solvePnPRefineLM(points,row['pixels'],K,D,v[:3].reshape(3,1),v[3:].reshape(3,1))
   v=np.r_[rv.ravel(),tv.ravel()];e=project(points,v)-row['pixels'];allerr.extend(e.ravel());Z=np.eye(4);Z[:3,:3]=cv2.Rodrigues(rv)[0];Z[:3,3]=tv.ravel();body=Y@np.linalg.inv(Z)@np.linalg.inv(X);gt=np.array(row['gt']);bodyerr.append(float(np.linalg.norm(body[:3,3]-gt[:3,3])));Yim=gt@X@Z;angles.append(Rotation.from_matrix(Yim[:3,:3]).as_euler('xyz',degrees=True));perframe.append({'bag_s':row['bag_s'],'pixel_rmse_per_coordinate':float(np.sqrt(np.mean(e**2))),'position_error_m':bodyerr[-1],'inferred_pad_euler_xyz_deg':angles[-1].tolist()})
  return {'frames':len(rows),'corner_rmse_per_coordinate_px':float(np.sqrt(np.mean(np.array(allerr)**2))),
   'position_error_m_p50_p95_max':np.percentile(bodyerr,[50,95,100]).tolist(),
   'median_inferred_pad_euler_xyz_deg':np.median(angles,axis=0).tolist(),'per_frame':perframe}
 all_both=[]
 for row in d['rows']:
  if 'gt' not in row or 'combined' not in row['poses'] or not all(str(mid) in row['corners'] for mid in [0,1]):continue
  pixels={mid:np.array(row['corners'][str(mid)]) for mid in [0,1]}
  if any(np.linalg.norm(v-np.roll(v,-1,axis=0),axis=1).mean()<12 for v in pixels.values()):continue
  r=dict(row);r['pixels']=np.r_[pixels[0],pixels[1]];all_both.append(r)
 outputs={}
 for mode in ['nominal','planar','nonplanar']:
  n={'nominal':0,'planar':4,'nonplanar':7}[mode]
  if n:
   initial=np.r_[np.zeros(n),np.concatenate([posevec(r) for r in train])]
   def residual(x):
    points=geometry(x[:n],mode);poses=x[n:].reshape(-1,6)
    return np.concatenate([(project(points,v)-r['pixels']).ravel() for r,v in zip(train,poses)])
   sparse=lil_matrix((len(train)*16,len(initial)),dtype=int)
   for i in range(len(train)):sparse[i*16:(i+1)*16,:n]=1;sparse[i*16:(i+1)*16,n+i*6:n+(i+1)*6]=1
   bounds=np.array([.03,.03,.12,.25] if mode=='planar' else [.03,.03,.03,.12,.25,.25,.25])
   low=np.r_[-bounds,np.full(6*len(train),-np.inf)];high=np.r_[bounds,np.full(6*len(train),np.inf)]
   result=least_squares(residual,initial,jac_sparsity=sparse.tocsr(),bounds=(low,high),loss='soft_l1',f_scale=.3,x_scale='jac',max_nfev=250,ftol=1e-8,xtol=1e-8)
   value=result.x[:n];points=geometry(value,mode)
   model={'parameters':value.tolist(),'success':bool(result.success),'cost':float(result.cost),'nfev':result.nfev,'near_bounds':np.flatnonzero(abs(value)>bounds*.97).tolist()}
   if mode=='nonplanar':model['relative_tilt_deg']=float(np.rad2deg(np.linalg.norm(value[4:6])))
   print(mode,model,flush=True)
  else:points=geometry([],mode);model={}
  model['marker0_corners_m']=points[:4].tolist();model['train']=evaluate(train,points);model['heldout']=evaluate(test,points);outputs[mode]=model
  model['all_heights']=[]
  for low,high in [(.4,1.),(1.,1.7),(1.7,2.4)]:
   selected=[r for r in all_both if low<=r['gt_body_height_m']<high]
   if selected:model['all_heights'].append({'height_m':[low,high],**evaluate(selected,points)})
  print(mode,'train_rmse',model['train']['corner_rmse_per_coordinate_px'],'heldout_rmse',model['heldout']['corner_rmse_per_coordinate_px'],'heldout_pose',model['heldout']['position_error_m_p50_p95_max'],flush=True)
 result={'opencv':cv2.__version__,'train':'takeoff, bag_s<7.7; both markers resolved, ID1 edge >=35px, ID0>=100px', 'heldout':'descent, bag_s>=9.72, same resolution gates', 'method':'image-only bundle adjustment; per-frame pose plus shared ID0 relative geometry; ID1 fixed; no GT in fitting','limitations':'One flight; mostly frontal views. Anchor scale and intrinsic distortion are assumed; shape improvements do not uniquely identify all physical causes. Diagnostic only.', 'models':outputs}
 (args.replay.parent/'relative_geometry_fit.json').write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()

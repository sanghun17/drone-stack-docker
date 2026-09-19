#!/usr/bin/env python3
"""Fit one fixed Body<-Camera extrinsic across recorded Pure conventions.

Current Pure is Body by the user's definition. Origins are assumed identical.
Earlier Pure axes are mapped to current Body using independent recorded gyros.
Only writes offline results, never runtime configuration or vehicle state.
"""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
import yaml
import audit_coordinate_frames as a

OUT=a.OUT/'fixed-body-refit'
NAMES=('initial_calibration','handcarried','manual_flight')


def load_data():
    data=[]
    mount=yaml.safe_load((a.CAL/'base_link_to_see3cam_optical_frame.yaml').read_text())
    offset=mount['image_to_body_time_offset_s']
    rows=json.loads((a.CAL/'camera_poses.json').read_text())
    # Original bag poses were previously verified against this lossless extraction.
    p=np.load(a.OLD.with_suffix('.body_poses.npz'))
    A=np.tile(np.eye(4),(len(p['timestamps']),1,1));A[:,:3,3]=p['positions'];A[:,:3,:3]=R.from_quat(p['quaternions_xyzw']).as_matrix()
    old={'pure':(p['timestamps'],A),'camera':(np.array([r['stamp']+offset for r in rows]),np.array([r['T_C_P']for r in rows]))}
    for name,d in zip(NAMES,[old,a.read(a.HAND)[0],a.read(a.FLIGHT)[0]]):
        epoch=d['pure'][0][0];pt,P=d['pure'];ct,C=d['camera'];pt=pt-epoch;ct=ct-epoch
        keep=(ct>pt[0]+.25)&(ct<pt[-1]-.25);ct=ct[keep];C=C[keep]
        train=np.arange(0,int(len(ct)*.65),3);test=np.arange(int(len(ct)*.65),len(ct),2)
        data.append(dict(name=name,pure=(pt,P),time=ct,Z=C,train=train,test=test))
    return data,np.array(mount['matrix_row_major']).reshape(4,4),offset


def fit(data,H,Xold):
    for i,d in enumerate(data):d['old_pure_from_body']=np.linalg.inv(H) if i<2 else np.eye(4)
    X0=H@Xold
    def unpack(v):return a.unpack(v[:6]),[a.unpack(v[6+i*7:12+i*7])for i in range(3)],[v[12+i*7]for i in range(3)]
    def residual(v,selection='train'):
        X,Y,dt=unpack(v);out=[]
        for i,d in enumerate(data):
            ix=d[selection];A=a.interp(d['pure'],d['time'][ix]+dt[i])@d['old_pure_from_body']
            c=A@X@d['Z'][ix]
            # Equal dataset weight; all sensor/map noise remains correlated.
            err=np.c_[(c[:,:3,3]-Y[i][:3,3])/.03,R.from_matrix(Y[i][:3,:3].T@c[:,:3,:3]).as_rotvec()/np.radians(3)]
            out.extend((err*np.sqrt(120/len(ix))).ravel())
        return np.array(out)
    initial=list(a.pack(X0))
    for d in data:
        ix=d['train'];A=a.interp(d['pure'],d['time'][ix])@d['old_pure_from_body']
        initial.extend(a.pack(a.mean(A@X0@d['Z'][ix])));initial.append(0.)
    lower=np.full(27,-np.inf);upper=np.full(27,np.inf)
    lower[[12,19,26]]=-.15;upper[[12,19,26]]=.15
    f=least_squares(residual,initial,bounds=(lower,upper),loss='soft_l1',max_nfev=200)
    X,Y,dt=unpack(f.x)
    results={}
    for i,d in enumerate(data):
        results[d['name']]={'additional_time_offset_s':float(dt[i])}
        for selection in ('train','test'):
            ix=d[selection];A=a.interp(d['pure'],d['time'][ix]+dt[i])@d['old_pure_from_body']
            est=Y[i]@np.linalg.inv(d['Z'][ix])@np.linalg.inv(X)
            dp=np.linalg.norm(A[:,:3,3]-est[:,:3,3],axis=1)
            dr=np.degrees((R.from_matrix(A[:,:3,:3]).inv()*R.from_matrix(est[:,:3,:3])).magnitude())
            results[d['name']][selection]={'n':len(ix),'position_m':a.stats(dp),'orientation_deg':a.stats(dr)}
        allA=a.interp(d['pure'],d['time']+dt[i])@d['old_pure_from_body']
        allB=Y[i]@np.linalg.inv(d['Z'])@np.linalg.inv(X)
        np.savez_compressed(OUT/(d['name']+'.npz'),time_s=d['time'],true_xyz=allA[:,:3,3],estimated_xyz=allB[:,:3,3],
            position_error_m=np.linalg.norm(allA[:,:3,3]-allB[:,:3,3],axis=1),heldout_start_s=d['time'][d['test'][0]])
    return X,results,{'optimizer_success':bool(f.success),'cost':float(f.cost),'scaled_jacobian_singular_values':np.linalg.svd(f.jac,compute_uv=False).tolist()}


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    audit=json.loads((a.OUT/'audit.json').read_text())
    Qold=np.array(audit['gyro']['handcarried']['rotation_matrix']);Qnow=np.array(audit['gyro']['manual_flight']['rotation_matrix'])
    H=np.eye(4);H[:3,:3]=Qnow@Qold.T
    data,Xold,offset=load_data()
    X,results,diagnostics=fit(data,H,Xold)
    report={'frame_contract':'Current Pure is Body (FLU); current Pure origin is vehicle centre, per user. Same mechanical origin across earlier recordings is assumed.',
            'method':'A single fixed Body<-Camera transform, three independent session Global<-Pad poses and three timing corrections. Historical Pure rotation is from independent gyro alignment, not forced to 90 degrees.',
            'old_pure_to_current_body':H.tolist(),'old_pure_to_body_rpy_deg':R.from_matrix(H[:3,:3]).as_euler('xyz',degrees=True).tolist(),
            'body_from_camera':X.tolist(),'translation_m':X[:3,3].tolist(),'rpy_deg':R.from_matrix(X[:3,:3]).as_euler('xyz',degrees=True).tolist(),
            'results':results,'diagnostics':diagnostics,
            'historical_scope':'First calibration has no IMU; shares old Pure convention inferred from matching camera extrinsics in first and handcarried recordings.',
            'timing_scope':'Original image time offset is added to fitted additional offsets. Session-dependent corrections are diagnostic, not a universal clock measurement.',
            'original_image_to_body_offset_s':offset,'deployment':'offline only until validation is reviewed; no global-pad runtime calibration is saved'}
    (OUT/'refit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Review a shared fixed camera mount from recorded A X Z = Y observations.

Uses the existing stack calibration solver's transform conventions and helpers.
Outputs diagnostic candidates only; never modifies runtime calibration.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
import rosbag
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[3]
SOLVER = ROOT/'stacks/aruco-landing-jetson/scripts/calibrate_pad_extrinsics.py'
spec = importlib.util.spec_from_file_location('existing_pad_extrinsic_solver', SOLVER)
solver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(solver)


def pose(p):
    q, t = p.orientation, p.position
    return solver.transform(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(), [t.x,t.y,t.z])


def read_bag(path):
    body, camera = [], []
    with rosbag.Bag(str(path)) as bag:
        for topic, message, receipt in bag.read_messages(topics=['/vrpn_client_node/pure/pose','/landing/target_pose_camera']):
            # These camera headers already include the runtime image/body offset.
            stamp = message.header.stamp.to_sec()
            if topic == '/vrpn_client_node/pure/pose':
                body.append((stamp,pose(message.pose)))
            else:
                camera.append((stamp,pose(message.pose.pose)))
    epoch=body[0][0]
    times=np.array([r[0]-epoch for r in body]); matrices=np.array([r[1]for r in body])
    unique=np.r_[True,np.diff(times)>1e-7];times=times[unique];matrices=matrices[unique]
    slerp=Slerp(times,Rotation.from_matrix(matrices[:,:3,:3]))
    selected=[];last=-1e30
    for stamp,Z in camera:
        t=stamp-epoch
        if times[0]<=t<=times[-1] and t-last>=.095:
            selected.append((t,Z));last=t
    t=np.array([r[0]for r in selected]);Z=np.array([r[1]for r in selected])
    A=np.tile(np.eye(4),(len(t),1,1));A[:,:3,:3]=slerp(t).as_matrix()
    A[:,:3,3]=np.column_stack([np.interp(t,times,matrices[:,j,3])for j in range(3)])
    held=(np.floor(t/2).astype(int)%5)==4
    rel=(Rotation.from_matrix(A[0,:3,:3]).inv()*Rotation.from_matrix(A[:,:3,:3])).as_rotvec()
    excitation=np.linalg.svd(rel-rel.mean(0),compute_uv=False)
    return dict(name=path.stem,A=A,Z=Z,t=t,train=np.flatnonzero(~held),test=np.flatnonzero(held),
                excitation_singular_values_rad=excitation.tolist(),
                body_rotation_span_xyz_deg=np.ptp(np.degrees(rel),axis=0).tolist())


def summarize(dp,dr):
    return dict(n=len(dp),position_m_p50_p95_max=np.percentile(dp,[50,95,100]).tolist(),
                position_rms_m=float(np.sqrt(np.mean(dp**2))),rotation_deg_p50_p95_max=np.percentile(dr,[50,95,100]).tolist())


def fit(datasets,old,mode,initial_delta=None):
    nmount={'unchanged':0,'rotation_only':3,'full':6}[mode]
    def unpack_mount(v):
        X=old.copy()
        if nmount:
            X[:3,:3]=Rotation.from_rotvec(v[:3]).as_matrix()@old[:3,:3]
        if nmount==6:X[:3,3]=old[:3,3]+v[3:6]
        return X
    initial=np.zeros(nmount) if initial_delta is None else np.asarray(initial_delta)
    for d in datasets:
        ix=d['train'];initial=np.r_[initial,solver.pack(solver.mean_transform(d['A'][ix]@old@d['Z'][ix]))]
    def unpack(v):
        return unpack_mount(v),[solver.unpack(v[nmount+6*i:nmount+6*(i+1)])for i in range(len(datasets))]
    def residual(v):
        X,Y=unpack(v);rows=[]
        for d,y in zip(datasets,Y):
            ix=d['train'];closed=d['A'][ix]@X@d['Z'][ix]
            tr=(closed[:,:3,3]-y[:3,3])/.02
            rr=Rotation.from_matrix(y[:3,:3].T@closed[:,:3,:3]).as_rotvec()/np.radians(2)
            rows.extend((np.c_[tr,rr]*np.sqrt(200/len(ix))).ravel())
        return np.asarray(rows)
    lower=np.full(len(initial),-np.inf);upper=-lower
    if nmount:lower[:3]=-np.radians(15);upper[:3]=np.radians(15)
    if nmount==6:lower[3:6]=-.15;upper[3:6]=.15
    result=least_squares(residual,initial,bounds=(lower,upper),loss='soft_l1',max_nfev=300,x_scale='jac')
    X,Y=unpack(result.x);metrics={}
    for d,y in zip(datasets,Y):
        predicted=y@np.linalg.inv(d['Z'])@solver.inverse(X)
        dp=np.linalg.norm(predicted[:,:3,3]-d['A'][:,:3,3],axis=1)
        dr=np.degrees((Rotation.from_matrix(d['A'][:,:3,:3]).inv()*Rotation.from_matrix(predicted[:,:3,:3])).magnitude())
        metrics[d['name']]={split:summarize(dp[d[split]],dr[d[split]])for split in ['train','test']}
        metrics[d['name']]['session_global_pad']=y.tolist()
    delta=Rotation.from_matrix(X[:3,:3]@old[:3,:3].T)
    return dict(mode=mode,success=bool(result.success),cost=float(result.cost),
                rotation_change_vector_deg=np.degrees(delta.as_rotvec()).tolist(),
                rotation_change_angle_deg=float(np.degrees(delta.magnitude())),
                translation_change_m=(X[:3,3]-old[:3,3]).tolist(),
                body_from_camera=X.tolist(),jacobian_singular_values=np.linalg.svd(result.jac,compute_uv=False).tolist(),metrics=metrics)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',action='append',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    config=ROOT/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml'
    old=np.array(yaml.safe_load(config.read_text())['matrix_row_major']).reshape(4,4)
    datasets=[read_bag(p)for p in args.bag]
    report={'scope':'offline review only; pose estimates, fixed runtime time correction, temporal 2-second heldout blocks',
            'existing_solver':str(SOLVER),'current_mount':old.tolist(),
            'motion':[{k:d[k]for k in ['name','excitation_singular_values_rad','body_rotation_span_xyz_deg']}for d in datasets],
            'shared_fits':[fit(datasets,old,mode)for mode in ['unchanged','rotation_only','full']],
            'per_flight_rotation_fits':[fit([d],old,'rotation_only')for d in datasets]}
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Offline metric pad reconstruction and joint T_body_camera/T_global_pad fit.

T_A_B maps B coordinates into A; observations satisfy A(t) X Z(t) = Y.
Raw bag input and reports are never installed as deployment calibration here.
"""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from scipy.sparse import lil_matrix

CORNERS = np.array([[-.5, .5], [.5, .5], [.5, -.5], [-.5, -.5]])


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=lambda a: a.tolist() if isinstance(a, np.ndarray) else a.item()) + '\n')


def intrinsic(path):
    d = yaml.safe_load(Path(path).read_text())
    return np.array(d['camera_matrix']['data']).reshape(3, 3), np.array(d['distortion_coefficients']['data'])


def square(p):
    x, y, log_side, yaw = p
    c, s = np.cos(yaw), np.sin(yaw)
    return CORNERS @ np.array([[c, s], [-s, c]]) * np.exp(log_side) + [x, y]


def project_h(H, points):
    p = np.column_stack((points, np.ones(len(points)))) @ H.T
    return p[:, :2] / p[:, 2:3]


def H_unpack(x):
    return np.r_[x, 1.].reshape(3, 3)


def transform(r, t):
    T = np.eye(4); T[:3, :3] = r; T[:3, 3] = np.asarray(t).flatten()
    return T


def inverse(T):
    r = T[:3, :3].T
    return transform(r, -r @ T[:3, 3])


def pack(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def unpack(v):
    return transform(Rotation.from_rotvec(v[:3]).as_matrix(), v[3:6])


def detector():
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMax = 101
    p.adaptiveThreshWinSizeStep = 4
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 3
    p.cornerRefinementMinAccuracy = .01
    p.cornerRefinementMaxIterations = 50
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50), p)


def extract(args):
    import rosbag
    out = Path(args.output); out.mkdir(parents=True, exist_ok=True)
    (out / 'frames').mkdir(exist_ok=True)
    K, D = intrinsic(args.intrinsic)
    det = detector(); records = []; last = -1e30; start = time.monotonic()
    with rosbag.Bag(args.bag) as bag:
        t0 = bag.get_start_time()
        for _, msg, bt in bag.read_messages(topics=['/landing/camera/image_raw']):
            stamp = msg.header.stamp.to_sec()
            if stamp - last < args.sample_period - .002:
                continue
            last = stamp
            if (msg.width, msg.height) != (1280, 720):
                raise ValueError('Calibration requires original 1280x720 frames')
            a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width*3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
            elif msg.encoding != 'bgr8':
                raise ValueError(msg.encoding)
            gray = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
            c, ids, _ = det.detectMarkers(gray)
            record = {'index': len(records), 'stamp': stamp, 'relative_s': stamp-t0,
                      'sharpness': float(cv2.Laplacian(gray, cv2.CV_64F).var()), 'markers': {}}
            if ids is not None:
                for pts, mid in zip(c, ids.flatten()):
                    pts = pts.reshape(4, 2)
                    if str(int(mid)) in record['markers']:
                        record['duplicate_ids'] = True
                    undist = cv2.undistortPoints(pts.reshape(-1,1,2), K, D, P=K).reshape(4,2)
                    record['markers'][str(int(mid))] = {'raw': pts.tolist(), 'undistorted': undist.tolist(),
                        'side_px': float(np.linalg.norm(pts-np.roll(pts,-1,axis=0),axis=1).mean())}
            records.append(record)
            if record['markers']:
                cv2.imwrite(str(out/'frames'/('%04d.png'%record['index'])), a)
            if len(records) % 50 == 0:
                print('Extracted',len(records),'t',round(stamp-t0,1),'wall',round(time.monotonic()-start,1),flush=True)
    save_json(out/'detections.json', {'bag': args.bag, 'intrinsic': args.intrinsic, 'sample_period_s':args.sample_period,'records':records})
    counts={}
    for r in records:
        for mid in r['markers']:counts[mid]=counts.get(mid,0)+1
    print('DETECTIONS',len(records),counts,flush=True)


def fit_pad(records, anchor, side, max_views=160):
    eligible = [r for r in records if not r.get('duplicate_ids') and len(r['markers']) >= 3
                and int(r['relative_s']//2)%5 != 4]
    anchor_par=np.array([0.,0.,np.log(side),0.])
    pars={anchor:anchor_par}
    def fit_square(pts):
        edge=((pts[1]-pts[0])+(pts[2]-pts[3]))*.5
        return np.r_[pts.mean(0),np.log(np.linalg.norm(pts-np.roll(pts,-1,axis=0),axis=1).mean()),np.arctan2(edge[1],edge[0])]
    # Grow the connected map from well-resolved anchor views. A tiny anchor's
    # four-corner homography is too ill-conditioned to extrapolate the full pad.
    estimates={}
    for r in eligible:
        a=r['markers'].get(str(anchor))
        if a is None or a['side_px']<65:continue
        H=cv2.getPerspectiveTransform(square(anchor_par).astype(np.float32),np.array(a['undistorted'],np.float32))
        for k,v in r['markers'].items():
            estimates.setdefault(int(k),[]).append(project_h(np.linalg.inv(H),v['undistorted']))
    for k,values in estimates.items():
        if k!=anchor and len(values)>=3:pars[k]=fit_square(np.median(values,axis=0))
    def frame_h(r):
        keys=[int(k) for k,v in r['markers'].items() if int(k) in pars and v['side_px']>=18]
        if len(keys)<2:return None
        obj=np.concatenate([square(pars[k]) for k in keys])
        pix=np.concatenate([r['markers'][str(k)]['undistorted'] for k in keys])
        H,mask=cv2.findHomography(obj,pix,cv2.RANSAC,3.,maxIters=2000,confidence=.999)
        if H is None or mask.sum()<8:return None
        return H/H[2,2]
    for iteration in range(6):
        estimates={}
        for r in eligible:
            H=frame_h(r)
            if H is None:continue
            for k,v in r['markers'].items():
                if int(k) in pars or v['side_px']<18:continue
                estimates.setdefault(int(k),[]).append(project_h(np.linalg.inv(H),v['undistorted']))
        added=[]
        for k,values in estimates.items():
            if len(values)<3:continue
            p=fit_square(np.median(values,axis=0))
            if np.linalg.norm(p[:2])<2 and .003<np.exp(p[2])<.8:pars[k]=p;added.append(k)
        if not added:break
    if len(pars)<3:raise ValueError('Need several close views of the anchor and at least two neighboring markers')
    pairs=[(r,frame_h(r)) for r in eligible]
    pairs=[(r,h) for r,h in pairs if h is not None]
    if len(pairs)>max_views:pairs=[pairs[i] for i in np.linspace(0,len(pairs)-1,max_views).astype(int)]
    eligible=[r for r,h in pairs];Hs=[h for r,h in pairs];ids=sorted(pars)
    print('PAD init',len(eligible),'views',ids,flush=True)
    variable=[k for k in ids if k!=anchor];n=len(eligible);mi={k:i for i,k in enumerate(ids)}
    index={k:8*n+4*i for i,k in enumerate(variable)}
    obs=[(i,int(k),np.array(v['undistorted'])) for i,r in enumerate(eligible) for k,v in r['markers'].items() if int(k) in ids and v['side_px']>=12]
    oi=np.array([v[0] for v in obs]);om=np.array([mi[v[1]] for v in obs]);pixels=np.array([v[2] for v in obs])
    # Normalize pixel coordinates to improve homography parameter conditioning.
    N=np.array([[1/700,0,-640/700],[0,1/700,-360/700],[0,0,1.]])
    x=np.r_[np.concatenate([(N@h).ravel()[:8] for h in Hs]),np.concatenate([pars[k] for k in variable])]
    sparsity=lil_matrix((len(obs)*8,len(x)),dtype=int)
    for j,(i,k,_) in enumerate(obs):
        sparsity[j*8:(j+1)*8,i*8:i*8+8]=1
        if k!=anchor:sparsity[j*8:(j+1)*8,index[k]:index[k]+4]=1
    def unpack_map(v):
        hs=np.column_stack((v[:8*n].reshape(n,8),np.ones(n))).reshape(n,3,3)
        ps=np.array([anchor_par if k==anchor else v[index[k]:index[k]+4] for k in ids])
        return hs,ps
    def residual(v):
        hs,ps=unpack_map(v)
        c=np.cos(ps[:,3]);s=np.sin(ps[:,3])
        rots=np.stack((c,-s,s,c),axis=1).reshape(-1,2,2)
        pts=np.einsum('mij,kj->mki',rots,CORNERS)*np.exp(ps[:,2,None,None])+ps[:,:2,None].transpose(0,2,1)
        ph=np.concatenate((pts[om],np.ones((len(obs),4,1))),axis=2)
        pred=np.einsum('nij,nkj->nki',hs[oi],ph)
        pred=700*pred[:,:,:2]/pred[:,:,2:3]+[640,360]
        return (pred-pixels).ravel()
    lower=np.full(len(x),-np.inf);upper=np.full(len(x),np.inf)
    for k in variable:
        j=index[k];lower[j:j+3]=[-2,-2,np.log(.003)];upper[j:j+3]=[2,2,np.log(.8)]
    mask=np.ones(len(obs)*8,dtype=bool)
    for pass_index in range(2):
        fit=least_squares(lambda v:residual(v)[mask],x,jac_sparsity=sparsity.tocsr()[mask],bounds=(lower,upper),loss='soft_l1',f_scale=1.,x_scale='jac',max_nfev=500,ftol=1e-8,xtol=1e-8,gtol=1e-8,tr_options={'atol':1e-8,'btol':1e-8,'maxiter':1500})
        x=fit.x
        errors=np.linalg.norm(residual(x).reshape(-1,4,2),axis=2)
        obs_error=np.sqrt(np.mean(errors**2,axis=1))
        print('PAD pass',pass_index,'RMS',np.sqrt(np.mean(errors**2)),'median',np.median(obs_error),'nfev',fit.nfev,flush=True)
        if pass_index==0:mask=np.repeat(obs_error<4.,8)
    hs,ps=unpack_map(x);counts={k:sum(o[1]==k for o in obs) for k in ids}
    return dict(zip(ids,ps)),{'training_frame_indices':[r['index'] for r in eligible],'observations':len(obs),'rms_px':float(np.sqrt(np.mean(errors**2))),'inlier_rms_px':float(np.sqrt(np.mean(residual(x)[mask]**2)*2)),'median_px':float(np.median(obs_error)),'max_px':float(obs_error.max()),'rejected_marker_observations':int((~mask).sum()/8),'optimizer_success':bool(fit.success),'optimizer_message':fit.message,'marker_observation_counts':counts}

def camera_poses(records, pars, K, D):
    output=[]
    for r in records:
        if r.get('duplicate_ids'):continue
        objects=[];pixels=[];ids=[]
        for mid,obs in r['markers'].items():
            mid=int(mid)
            if mid not in pars or obs['side_px']<12:continue
            objects.extend(np.column_stack((square(pars[mid]),np.zeros(4))))
            pixels.extend(obs['raw']);ids.append(mid)
        if len(ids)<2:continue
        obj=np.array(objects,np.float64);pix=np.array(pixels,np.float64)
        # All points are coplanar. Solve both planar hypotheses using the whole
        # board before trimming corner outliers; unconstrained 4-point RANSAC
        # can fit a mirrored attitude to a small subset and reject the real board.
        solutions=cv2.solvePnPGeneric(obj,pix,K,D,flags=cv2.SOLVEPNP_IPPE)
        choices=[]
        for rv,tv in zip(solutions[1],solutions[2]):
            rv,tv=cv2.solvePnPRefineLM(obj,pix,K,D,rv,tv)
            pred=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
            error=np.linalg.norm(pred-pix,axis=1)
            if tv[2,0]>0:choices.append((float(np.median(error)+np.mean(np.minimum(error,10))),rv,tv,error))
        if not choices:continue
        _,rv,tv,error=min(choices,key=lambda v:v[0])
        good=np.flatnonzero(error<4.)
        if len(good)<8 or len(good)<.75*len(obj):continue
        rv,tv=cv2.solvePnPRefineLM(obj[good],pix[good],K,D,rv,tv)
        pred=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
        error=np.linalg.norm(pred-pix,axis=1)
        if np.sqrt(np.mean(error[good]**2))>2.5 or tv[2,0]<=0:continue
        RR=cv2.Rodrigues(rv)[0]
        output.append({**r,'T_C_P':transform(RR,tv),'reprojection_rms_px':float(np.sqrt(np.mean(error[good]**2))),
                       'inlier_corners':len(good),'inlier_ids':[mid for i,mid in enumerate(ids) if all(j in good for j in range(i*4,i*4+4))],'detected_ids':ids,'reprojection_all_px':error.tolist()})
    return output


def mean_transform(values):
    return transform(Rotation.from_matrix(values[:, :3, :3]).mean().as_matrix(), np.median(values[:, :3, 3], axis=0))


def yaml_transform(parent, child, T):
    q=Rotation.from_matrix(T[:3,:3]).as_quat()
    return {'parent_frame':parent,'child_frame':child,
            'convention':'T_parent_child maps child coordinates into parent coordinates',
            'translation_m':dict(zip('xyz',map(float,T[:3,3]))),
            'rotation_xyzw':dict(zip(['x','y','z','w'],map(float,q))),
            'matrix_row_major':T.ravel().tolist()}


def solve(args):
    out=Path(args.output);K,D=intrinsic(args.intrinsic)
    records=json.loads((out/'detections.json').read_text())['records']
    pars,map_report=fit_pad(records,args.anchor_id,args.anchor_side_m)
    manifest={'name':'physical_landing_pad','dictionary':'DICT_7X7_50',
              'frame_convention':'origin at anchor center; +X c0-to-c1; +Y c3-to-c0; +Z outward from paper',
              'anchor':{'id':args.anchor_id,'side_m':args.anchor_side_m},
              'markers':[{'id':k,'center_m':dict(zip('xy',map(float,v[:2]))),'side_m':float(np.exp(v[2])),
                          'yaw_deg':float((np.degrees(v[3])+180)%360-180)} for k,v in sorted(pars.items())]}
    (out/'physical_landing_pad_DICT_7X7_50.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
    save_json(out/'map_report.json',map_report)
    camera=camera_poses(records,pars,K,D)
    save_json(out/'camera_poses.json',camera)
    print('Camera PnP observations',len(camera),flush=True)
    data=np.load(args.poses);times=data['timestamps'];positions=data['positions'];quats=data['quaternions_xyzw']
    epoch=float(times[0]);times=times-epoch
    unique=np.r_[True,np.diff(times)>1e-7];times=times[unique];positions=positions[unique];quats=quats[unique]
    quats=quats/np.linalg.norm(quats,axis=1)[:,None]
    slerp=Slerp(times,Rotation.from_quat(quats))
    def body_at(t):
        result=np.tile(np.eye(4),(len(t),1,1))
        result[:,:3,:3]=slerp(t).as_matrix()
        result[:,:3,3]=np.column_stack([np.interp(t,times,positions[:,j]) for j in range(3)])
        return result
    angles=np.degrees(2*np.arccos(np.clip(np.abs((quats[1:]*quats[:-1]).sum(1)),0,1)))
    jumps=(np.linalg.norm(np.diff(positions,axis=0),axis=1)>.08)|(angles>15)
    bad_times=times[1:][jumps]
    candidates=[r for r in camera if times[0]+.12<r['stamp']-epoch<times[-1]-.12
                and (len(bad_times)==0 or np.min(np.abs(bad_times-(r['stamp']-epoch)))>.25)]
    # Keep diverse poses, not hundreds of redundant frames of a stationary drone.
    selected=[];last_T=None;last_t=-1e30
    for r in candidates:
        A=body_at([r['stamp']-epoch])[0]
        if last_T is not None:
            delta=inverse(last_T)@A
            if np.linalg.norm(delta[:3,3])<.02 and np.linalg.norm(Rotation.from_matrix(delta[:3,:3]).as_rotvec())<np.radians(2) and r['stamp']-last_t<3:
                continue
        selected.append(r);last_T=A;last_t=r['stamp']
    if len(selected)<40:raise ValueError('Insufficient diverse image/body pairs: %d'%len(selected))
    t=np.array([r['stamp']-epoch for r in selected]);Z=np.array([r['T_C_P'] for r in selected]);A=body_at(t)
    held=np.array([int(r['relative_s']//2)%5==4 for r in selected])
    train=np.flatnonzero(~held);test=np.flatnonzero(held)
    if len(test)<8:raise ValueError('Insufficient held-out pairs')
    print('Fit pairs train/heldout',len(train),len(test),'rejected motion jumps',len(bad_times),flush=True)
    def closure(X,Y,offset,indices):
        estimate=body_at(t[indices]+offset)@X@Z[indices]
        trans=estimate[:,:3,3]-Y[:3,3]
        rot=Rotation.from_matrix(Y[:3,:3].T@estimate[:,:3,:3]).as_rotvec()
        return trans,rot
    rng=np.random.default_rng(20260919);starts=[]
    methods=[cv2.CALIB_HAND_EYE_PARK,cv2.CALIB_HAND_EYE_TSAI,cv2.CALIB_HAND_EYE_HORAUD]
    for attempt in range(63):
        sample=train if attempt<3 else rng.choice(train,min(12,len(train)),replace=False)
        try:
            R,x=cv2.calibrateHandEye(list(A[sample,:3,:3]),list(A[sample,:3,3]),list(Z[sample,:3,:3]),list(Z[sample,:3,3]),method=methods[attempt%3])
            X=transform(R,x)
            if not np.isfinite(X).all() or np.linalg.norm(x)>2 or np.linalg.det(R)<.99:continue
            Y=mean_transform(A[train]@X@Z[train])
            dt,dr=closure(X,Y,0,train)
            score=np.linalg.norm(dt,axis=1)/.03+np.linalg.norm(dr,axis=1)/np.radians(3)
            starts.append((float(np.median(score)+.2*np.percentile(score,75)),X,Y))
        except (cv2.error,ValueError):continue
    if not starts:raise ValueError('All hand-eye initializers failed')
    starts.sort(key=lambda a:a[0]);print('Initializer scores',[round(v[0],3) for v in starts[:5]],flush=True)
    def optimize(X,Y,offset,indices,loss='soft_l1'):
        x0=np.r_[pack(X),pack(Y),offset]
        def residual(v):
            tr,rr=closure(unpack(v[:6]),unpack(v[6:12]),v[12],indices)
            return np.column_stack((tr/.02,rr/np.radians(2))).ravel()
        lower=np.r_[np.full(12,-np.inf),-.1];upper=np.r_[np.full(12,np.inf),.1]
        return least_squares(residual,x0,bounds=(lower,upper),loss=loss,f_scale=1.,x_scale='jac',max_nfev=300,ftol=1e-9,xtol=1e-9,gtol=1e-9)
    fits=[]
    for _,X,Y in starts[:3]:
        fit=optimize(X,Y,0,train);fits.append(fit)
    fit=min(fits,key=lambda f:f.cost)
    X,Y,offset=unpack(fit.x[:6]),unpack(fit.x[6:12]),float(fit.x[12])
    dt,dr=closure(X,Y,offset,train)
    inliers=train[(np.linalg.norm(dt,axis=1)<.05)&(np.linalg.norm(dr,axis=1)<np.radians(5))]
    if len(inliers)>=30:
        fit=optimize(X,Y,offset,inliers)
        X,Y,offset=unpack(fit.x[:6]),unpack(fit.x[6:12]),float(fit.x[12])
    else:inliers=train
    def metric(indices):
        dt,dr=closure(X,Y,offset,indices);tr=np.linalg.norm(dt,axis=1);rr=np.degrees(np.linalg.norm(dr,axis=1))
        return {'count':len(indices),'translation_rms_m':float(np.sqrt(np.mean(tr**2))),'translation_median_m':float(np.median(tr)),'translation_max_m':float(tr.max()),
                'rotation_rms_deg':float(np.sqrt(np.mean(rr**2))),'rotation_median_deg':float(np.median(rr)),'rotation_max_deg':float(rr.max())}
    train_metrics=metric(inliers);test_metrics=metric(test)
    # Closing the loop at the pad and evaluating the reconstructed body origin
    # are different metrics: an angular error creates a range-dependent lever arm.
    reconstructed=Y@np.linalg.inv(Z)@inverse(X)
    reference=body_at(t+offset)
    body_errors=np.linalg.norm(reconstructed[:,:3,3]-reference[:,:3,3],axis=1)
    def body_metric(indices):
        e=body_errors[indices]
        if not len(e):return {'count':0,'rms_m':None,'median_m':None,'p95_m':None,'max_m':None}
        return {'count':len(e),'rms_m':float(np.sqrt(np.mean(e**2))),'median_m':float(np.median(e)),
                'p95_m':float(np.percentile(e,95)),'max_m':float(e.max())}
    body_metrics={'train_inlier':body_metric(inliers),'heldout':body_metric(test),
                  'heldout_at_least_3_inlier_markers':body_metric(np.array([i for i in test if len(selected[i]['inlier_ids'])>=3],dtype=int))}
    # Conditional uncertainty: resample complete training time blocks, keeping
    # the map/intrinsic fixed. This does not measure scale or intrinsic bias.
    block_ids=np.array([int(selected[i]['relative_s']//2) for i in inliers])
    blocks=np.unique(block_ids);boot=[]
    for _ in range(30):
        sample=np.concatenate([inliers[block_ids==b] for b in rng.choice(blocks,len(blocks),replace=True)])
        bf=optimize(X,Y,offset,sample)
        if bf.success:
            bx,by=unpack(bf.x[:6]),unpack(bf.x[6:12])
            boot.append(np.r_[bx[:3,3]-X[:3,3],Rotation.from_matrix(X[:3,:3].T@bx[:3,:3]).as_rotvec(),by[:3,3]-Y[:3,3],Rotation.from_matrix(Y[:3,:3].T@by[:3,:3]).as_rotvec(),bf.x[12]-offset])
    if len(boot)<10:raise ValueError('Too few converged bootstrap fits to assess stability')
    boot=np.array(boot)
    uncertainty={'method':'30 temporal-block bootstrap fits; fixed intrinsic and pad map; conditional, excludes systematic/scale error',
                 'successful_fits':len(boot),'X_translation_component_std_m':boot[:,:3].std(0).tolist(),
                 'X_rotation_vector_component_std_deg':np.degrees(boot[:,3:6].std(0)).tolist(),
                 'Y_translation_component_std_m':boot[:,6:9].std(0).tolist(),
                 'time_offset_std_s':float(boot[:,12].std())}

    motion=body_at(t[inliers]+offset);rel=(Rotation.from_matrix(motion[0,:3,:3]).inv()*Rotation.from_matrix(motion[:,:3,:3])).as_rotvec()
    excitation_singular=np.linalg.svd(rel-rel.mean(0),compute_uv=False)
    checks={'enough_training_pairs':len(inliers)>=30,'enough_heldout_pairs':len(test)>=8,
            'heldout_translation_rms_under_2cm':test_metrics['translation_rms_m']<.02,
            'heldout_rotation_rms_under_2deg':test_metrics['rotation_rms_deg']<2,
            'offset_not_at_bound':abs(offset)<.095,'mount_translation_under_50cm':bool(np.linalg.norm(X[:3,3])<.5),
            'optical_z_down_in_body':float(X[2,2])<-.70,
            'multiple_rotation_axes_excited':float(excitation_singular[-1])>.1,
            'optimizer_success':bool(fit.success),'map_optimizer_success':map_report['optimizer_success'],
            'map_inlier_rms_under_2px':map_report['inlier_rms_px']<2}
    result='PASS' if all(checks.values()) else 'FAIL'
    for filename,parent,child,T in [('base_link_to_see3cam_optical_frame.yaml','base_link','see3cam_optical_frame',X),
        ('see3cam_optical_frame_to_base_link_check.yaml','see3cam_optical_frame','base_link',inverse(X))]:
        value=yaml_transform(parent,child,T);value['validation_result']=result
        value['validation_scope']='offline held-out AXZY closure; live estimator and PX4 transition not validated'
        value['source_bag']=json.loads((out/'detections.json').read_text()).get('bag')
        value['anchor_id']=args.anchor_id;value['anchor_side_m']=args.anchor_side_m
        value['image_to_body_time_offset_s']=offset
        (out/filename).write_text(yaml.safe_dump(value,sort_keys=False))
    dt_all,dr_all=closure(X,Y,offset,np.arange(len(t)))
    rows=[{'frame_index':r['index'],'relative_s':r['relative_s'],'split':'heldout' if held[i] else ('train_inlier' if i in inliers else 'train_outlier'),
           'body_position_error_m':float(body_errors[i]),'inlier_marker_count':len(r['inlier_ids']),
           'body_position_opti_m':reference[i,:3,3].tolist(),'body_position_pad_m':reconstructed[i,:3,3].tolist(),
           'translation_error_m':float(np.linalg.norm(dt_all[i])),'rotation_error_deg':float(np.degrees(np.linalg.norm(dr_all[i])))} for i,r in enumerate(selected)]
    report={'result':result,'validation_scope':'offline held-out AXZY closure','body_position_validation':body_metrics,'conditional_uncertainty':uncertainty,'quality_checks':checks,'train':train_metrics,'heldout':test_metrics,
            'time_offset_s':offset,'time_offset_convention':'body pose is interpolated at image header stamp + offset',
            'body_camera_translation_m':X[:3,3].tolist(),'body_camera_rotation_xyzw':Rotation.from_matrix(X[:3,:3]).as_quat().tolist(),
            'global_pad_translation_m':Y[:3,3].tolist(),'global_pad_rotation_xyzw':Rotation.from_matrix(Y[:3,:3]).as_quat().tolist(),
            'anchor_id':args.anchor_id,'anchor_side_m':args.anchor_side_m,'motion_jump_times_relative_s':(bad_times-times[0]).tolist(),
            'rotation_excitation_singular_values_rad':excitation_singular.tolist(),'map_report':map_report,'residuals':rows,
            'candidates_before_pose_subsampling':len(candidates),'fit_success':bool(fit.success),'fit_message':fit.message}
    save_json(out/'extrinsic_report.json',report)
    print('RESULT',json.dumps({k:v for k,v in report.items() if k not in ['residuals','map_report','motion_jump_times_relative_s']}),flush=True)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('stage',choices=['extract','solve'])
    p.add_argument('--bag')
    p.add_argument('--intrinsic',required=True)
    p.add_argument('--poses')
    p.add_argument('--output',required=True)
    p.add_argument('--anchor-id',type=int,default=21)
    p.add_argument('--anchor-side-m',type=float,default=.030)
    p.add_argument('--sample-period',type=float,default=.2)
    args=p.parse_args()
    if args.anchor_side_m<=0:p.error('--anchor-side-m must be positive')
    if args.stage=='extract' and not args.bag:p.error('extract requires --bag')
    if args.stage=='solve' and not args.poses:p.error('solve requires --poses')
    cv2.setNumThreads(2)
    if args.stage=='extract':extract(args)
    else:solve(args)


if __name__=='__main__':main()

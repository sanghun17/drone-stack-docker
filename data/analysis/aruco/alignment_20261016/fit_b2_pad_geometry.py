#!/usr/bin/env python3
"""Bounded B2 square-marker geometry fit; takeoff train, descent holdout.

Native OpenCV production replay. Camera/mount/timing fixed, global pad RP=0.
Coarse unresolved corners are excluded from fitting, but retained for an
independent evaluation with and without the same observation-quality guard.
No deployment changes. Effective visual calibration is not a physical survey.
"""
import argparse,copy,json,sys,time
from pathlib import Path
import cv2,numpy as np,rosbag,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from audit import matrix,interpolate,quantile
CORNERS=np.array([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]])
IDS=[1,2,3,4,19,43]


def extract(bag,cache):
    gt=[];K=D=Y=None;offset=None;descend=None;events=[];old={}
    if cache and cache.exists():old={round(r['stamp'],6):r for r in json.loads(cache.read_text())['rows']}
    with rosbag.Bag(str(bag)) as b:
        start=b.get_start_time()
        for topic,m,bt in b.read_messages(topics=['/vrpn_client_node/pure/pose','/landing/camera/camera_info','/landing/pad_pose_global','/landing/estimator/status','/landing/trial/status']):
            if topic=='/vrpn_client_node/pure/pose':gt.append((m.header.stamp.to_sec(),matrix(m.pose)))
            elif topic.endswith('camera_info'):K=np.array(m.K).reshape(3,3);D=np.array(m.D)
            elif topic.endswith('pad_pose_global'):
                if Y is None:Y=matrix(m.pose)
            elif topic.endswith('estimator/status'):offset=json.loads(m.data)['image_time_offset_s']
            else:
                d=json.loads(m.data)
                if d.get('phase')=='DESCEND' and descend is None:descend=bt.to_sec()-start
    detector=PhysicalPadDetector(LAYOUT,min_markers=1)
    detector.params.adaptiveThreshWinSizeMin=3;detector.params.adaptiveThreshWinSizeMax=23;detector.params.adaptiveThreshWinSizeStep=10
    rows=[];cv2.setNumThreads(2)
    with rosbag.Bag(str(bag)) as b:
        for _,m,bt in b.read_messages(topics=['/landing/camera/image_raw']):
            stamp=m.header.stamp.to_sec()+offset;G,ok=interpolate(gt,np.array([stamp]));G=G[0]
            if not ok[0] or G[2,3]<.25:continue
            previous=old.get(round(stamp,6))
            if previous:corners=previous['corners']
            else:
                ch=1 if m.encoding=='mono8' else 3
                a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*ch].reshape(m.height,m.width,ch)
                gray=a[:,:,0] if ch==1 else cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
                gray,ck=center_crop(gray,K,720,720)
                _,cs,ids=detector.detect(gray,ck,D)
                corners={str(mid):np.array(c).reshape(4,2).tolist() for c,mid in zip(cs,ids)}
            if corners:rows.append(dict(bag_s=bt.to_sec()-start,stamp=stamp,gt=G.tolist(),corners=corners,gt_height_m=float(G[2,3])))
            if len(rows)%200==0:print('observations',len(rows),flush=True)
    ck=K.copy();ck[0,2]-=280
    return dict(bag=str(bag),K=ck.tolist(),D=D.tolist(),Y0=Y.tolist(),offset=offset,descend_s=descend,rows=rows)


def coarse(c):return bool(np.max(np.abs(c-np.round(c)))<1e-5)


def marker_points(markers):
    result=[]
    for m in markers:
        if 'corners_m' in m:
            result.append(np.array(m['corners_m']));continue
        a=np.radians(m['yaw_deg']);c,s=np.cos(a),np.sin(a)
        xy=CORNERS@np.array([[c,s],[-s,c]])*m['side_m']+[m['center_m']['x'],m['center_m']['y']]
        result.append(np.c_[xy,np.zeros(4)])
    return np.array(result)


def fit_model(data,mode,training,output,tag=None):
    nominal=sorted(copy.deepcopy(LAYOUT['markers']),key=lambda m:IDS.index(m['id']))
    basepoints=marker_points(nominal);Y0=np.array(data['Y0']);K=np.array(data['K']);D=np.array(data['D'])
    yaw0=np.arctan2(Y0[1,0],Y0[0,0])
    def decode(x):
        Y=np.eye(4);Y[:3,:3]=Rotation.from_euler('z',yaw0+x[3]).as_matrix();Y[:3,3]=Y0[:3,3]+x[:3]
        markers=copy.deepcopy(nominal)
        if mode=='uniform_scale':
            scale=np.exp(x[4])
            for m in markers:
                m['side_m']*=scale;m['center_m']['x']*=scale;m['center_m']['y']*=scale
                if 'corners_m' in m:m['corners_m']=(np.array(m['corners_m'])*scale).tolist()
        elif mode=='per_marker':
            for j,m in enumerate(markers[:5]):
                dx,dy,logs,rz=x[4+j*4:8+j*4]
                m['center_m']['x']+=dx;m['center_m']['y']+=dy;m['side_m']*=np.exp(logs);m['yaw_deg']+=np.degrees(rz)
            markers[5]['side_m']*=np.exp(x[24])
        return Y,markers,marker_points(markers)
    bounds=np.array([.15,.15,.10,np.radians(10.)]+([.20] if mode=='uniform_scale' else ([.04,.04,.20,np.radians(8.)]*5+[.20] if mode=='per_marker' else [])))
    C=[];fi=[];mi=[];ci=[];pix=[]
    for i,row in enumerate(training):
        C.append(np.array(row['gt'])@X)
        for j,mid in enumerate(IDS):
            if str(mid) not in row['corners']:continue
            c=np.array(row['corners'][str(mid)])
            if coarse(c) or np.linalg.norm(c-np.roll(c,-1,axis=0),axis=1).mean()<18:continue
            fi.extend([i]*4);mi.extend([j]*4);ci.extend(range(4));pix.extend(c)
    C=np.array(C);fi=np.array(fi);mi=np.array(mi);ci=np.array(ci);pix=np.array(pix)
    Rinv=np.swapaxes(C[fi,:3,:3],1,2);origin=C[fi,:3,3]
    def residual(x,regularize=True):
        Y,markers,points=decode(x);global_points=points[mi,ci]@Y[:3,:3].T+Y[:3,3]
        pc=np.einsum('nij,nj->ni',Rinv,global_points-origin)
        projected=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
        error=(projected-pix).ravel()
        if regularize and mode=='per_marker':
            priors=np.array([.02,.02,.08,np.radians(5.)]*5+[.08]);error=np.r_[error,x[4:]/priors]
        return error
    t=time.monotonic();result=least_squares(residual,np.zeros(len(bounds)),bounds=(-bounds,bounds),loss='soft_l1',f_scale=.6,x_scale='jac',max_nfev=180)
    Y,markers,points=decode(result.x)
    report=dict(success=bool(result.success),nfev=result.nfev,wall_s=time.monotonic()-t,
        parameters=result.x.tolist(),near_bounds=np.flatnonzero(abs(result.x)>.97*bounds).tolist(),
        global_pad=Y.tolist(),markers=markers,fit_frames=len(training),fit_points=len(pix),
        train_rmse_per_coordinate_px=float(np.sqrt(np.mean(residual(result.x,False)**2))))
    if mode=='uniform_scale':report['uniform_scale']=float(np.exp(result.x[4]))
    report['marker_changes']=[]
    for old,new in zip(nominal,markers):
        report['marker_changes'].append(dict(id=new['id'],side_m=new['side_m'],side_change_pct=100*(new['side_m']/old['side_m']-1),
            center_delta_m=[new['center_m'][a]-old['center_m'][a] for a in ['x','y']],yaw_delta_deg=new['yaw_deg']-old['yaw_deg']))
    (output/((tag or mode)+'_fit.json')).write_text(json.dumps(report,indent=2)+'\n')
    return report


def evaluate(data,model,rows,drop_coarse=False):
    layout=copy.deepcopy(LAYOUT);layout['markers']=model['markers'];detector=PhysicalPadDetector(layout,min_markers=1)
    detector.models={m['id']:points for m,points in zip(model['markers'],marker_points(model['markers']))}
    K=np.array(data['K']);D=np.array(data['D']);Y=np.array(model['global_pad']);records=[];rays=[]
    points={m['id']:pts for m,pts in zip(model['markers'],marker_points(model['markers']))}
    for row in rows:
        pairs=[(np.array(c,np.float32).reshape(1,4,2),int(mid)) for mid,c in row['corners'].items() if int(mid) in IDS]
        if drop_coarse:pairs=[(c,mid) for c,mid in pairs if not coarse(c)]
        corners=[c for c,_ in pairs];ids=[mid for _,mid in pairs];res=detector.estimate(corners,ids,K,D)
        G=np.array(row['gt']);C=G@X
        for c,mid in pairs:
            c=c.reshape(4,2)
            if coarse(c) or np.linalg.norm(c-np.roll(c,-1,axis=0),axis=1).mean()<18:continue
            g=points[mid]@Y[:3,:3].T+Y[:3,3];pc=(g-C[:3,3])@C[:3,:3]
            rays.extend((cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)-c).ravel())
        if res is None:continue
        estimate=Y@inverse(res['camera_from_pad'])@inverse(X)
        records.append(dict(bag_s=row['bag_s'],gt_height_m=row['gt_height_m'],
            position_error_m=float(np.linalg.norm(estimate[:3,3]-G[:3,3])),
            rotation_error_deg=float(np.degrees(Rotation.from_matrix(estimate[:3,:3]@G[:3,:3].T).magnitude())),
            inliers=res['inlier_ids'],rms_px=res['rms_px']))
    return dict(frames=len(rows),valid=len(records),position_m_p50_p95_max=quantile([r['position_error_m'] for r in records]),
        rotation_deg_p50_p95_max=quantile([r['rotation_error_deg'] for r in records]),
        over15cm=sum(r['position_error_m']>.15 for r in records),over20cm=sum(r['position_error_m']>.2 for r in records),
        reprojection_rmse_per_coordinate_px=float(np.sqrt(np.mean(np.array(rays)**2))),rows=records)


def fit_image_only(data,training,output,quad=False):
    """Relative planar geometry, free camera pose per image, ID19 metric anchor.

    GT initializes poses only; it is absent from the fitting objective.
    """
    from scipy.sparse import lil_matrix
    nominal=sorted(copy.deepcopy(LAYOUT['markers']),key=lambda m:IDS.index(m['id']))
    variable=[mid for mid in IDS if mid!=19];frames=[];initial_poses=[]
    det=PhysicalPadDetector(LAYOUT,min_markers=1);K=np.array(data['K']);D=np.array(data['D'])
    for row in training:
        pairs=[]
        for key,pix in row['corners'].items():
            mid=int(key);c=np.array(pix,np.float64)
            if mid not in IDS or coarse(c) or np.linalg.norm(c-np.roll(c,-1,axis=0),axis=1).mean()<18:continue
            pairs.append((c,mid))
        if len(pairs)<2:continue
        res=det.estimate([c for c,_ in pairs],[mid for _,mid in pairs],K,D)
        if res is None:continue
        frames.append(pairs);Z=res['camera_from_pad'];initial_poses.append(np.r_[Rotation.from_matrix(Z[:3,:3]).as_rotvec(),Z[:3,3]])
    take=np.linspace(0,len(frames)-1,min(100,len(frames))).astype(int)
    frames=[frames[i] for i in take];initial_poses=[initial_poses[i] for i in take]
    fi=[];mi=[];ci=[];pix=[]
    for i,pairs in enumerate(frames):
        for c,mid in pairs:
            fi.extend([i]*4);mi.extend([IDS.index(mid)]*4);ci.extend(range(4));pix.extend(c)
    fi=np.array(fi);mi=np.array(mi);ci=np.array(ci);pix=np.array(pix);n=40 if quad else 20
    def geometry(x):
        markers=copy.deepcopy(nominal)
        for m in markers:
            if m['id']==19:continue
            j=variable.index(m['id'])
            if quad:
                points=marker_points([m])[0];points[:,:2]+=x[j*8:j*8+8].reshape(4,2)
                m['corners_m']=points.tolist();center=points[:,:2].mean(0)
                m['center_m']['x']=float(center[0]);m['center_m']['y']=float(center[1])
                m['side_m']=float(np.linalg.norm(points-np.roll(points,-1,axis=0),axis=1).mean())
                axis=(points[1]-points[0]+points[2]-points[3])/2
                m['yaw_deg']=float(np.degrees(np.arctan2(axis[1],axis[0])));continue
            dx,dy,logs,rz=x[j*4:j*4+4]
            m['center_m']['x']+=dx;m['center_m']['y']+=dy;m['side_m']*=np.exp(logs);m['yaw_deg']+=np.degrees(rz)
        return markers,marker_points(markers)
    def residual(x):
        markers,points=geometry(x[:n]);poses=x[n:].reshape(-1,6);R=Rotation.from_rotvec(poses[:,:3]).as_matrix()
        pc=np.einsum('nij,nj->ni',R[fi],points[mi,ci])+poses[fi,3:]
        pp=cv2.projectPoints(pc,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2)
        prior=np.full(n,.003) if quad else np.array([.04,.04,.15,np.radians(8.)]*5)
        return np.r_[(pp-pix).ravel(),x[:n]/prior]
    initial=np.r_[np.zeros(n),np.array(initial_poses).ravel()]
    bounds=np.full(n,.008) if quad else np.array([.04,.04,.2,np.radians(8.)]*5)
    low=np.r_[-bounds,np.full(6*len(frames),-np.inf)];high=np.r_[bounds,np.full(6*len(frames),np.inf)]
    sparse=lil_matrix((len(pix)*2+n,len(initial)),dtype=int)
    for i in range(len(frames)):
        obs=np.flatnonzero(fi==i);indices=np.ravel(np.c_[obs*2,obs*2+1]);sparse[indices,:n]=1;sparse[indices,n+i*6:n+(i+1)*6]=1
    sparse[-n:,:n]=np.eye(n)
    print('image-only bundle',len(frames),'images',len(pix),'corners',flush=True)
    fit=least_squares(residual,initial,jac_sparsity=sparse.tocsr(),bounds=(low,high),loss='soft_l1',f_scale=.4,x_scale='jac',max_nfev=250,ftol=1e-8,xtol=1e-8)
    markers,_=geometry(fit.x[:n]);anchor=next(m for m in markers if m['id']==43)
    shift=np.array([anchor['center_m']['x'],anchor['center_m']['y']]);yaw=anchor['yaw_deg']+90.;a=np.radians(yaw);R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    for m in markers:
        xy=(np.array([m['center_m']['x'],m['center_m']['y']])-shift)@R
        m['center_m']['x']=float(xy[0]);m['center_m']['y']=float(xy[1]);m['yaw_deg']-=yaw
        if 'corners_m' in m:
            pts=np.array(m['corners_m']);pts[:,:2]=(pts[:,:2]-shift)@R;m['corners_m']=pts.tolist()
    summary=dict(success=bool(fit.success),nfev=fit.nfev,train_images=len(frames),train_corners=len(pix),
        train_corner_rmse_per_coordinate_px=float(np.sqrt(np.mean(residual(fit.x)[:-n]**2))),
        near_bounds=np.flatnonzero(abs(fit.x[:n])>.97*bounds).tolist(),markers=markers,
        scope=('image-only free planar quads' if quad else 'image-only square markers')+'; ID19 metric side fixed; recentered to ID43; GT only for later global registration')
    (output/('image_only_quad_bundle_fit.json' if quad else 'image_only_bundle_fit.json')).write_text(json.dumps(summary,indent=2)+'\n');return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('--owner',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--cache',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(a.owner/'src'));from aruco_landing.physical_pad import PhysicalPadDetector,center_crop,inverse
    root=Path(__file__).resolve().parents[4];LAYOUT=yaml.safe_load((a.owner/'config/baseline_b2_700mm_runtime.yaml').read_text());X=np.array(yaml.safe_load((root/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml').read_text())['matrix_row_major']).reshape(4,4)
    observations=a.output/'observations.json'
    if observations.exists():
        data=json.loads(observations.read_text())
        if Path(data['bag']).resolve()!=a.bag.resolve():raise ValueError('cached observations belong to another bag')
    else:data=extract(a.bag,a.cache);observations.write_text(json.dumps(data,indent=2)+'\n')
    train=[r for r in data['rows'] if r['bag_s']<data['descend_s']-.05][::2]
    held=[r for r in data['rows'] if r['bag_s']>=data['descend_s']+.05]
    assert train and held and max(r['bag_s'] for r in train)<min(r['bag_s'] for r in held)
    report=dict(opencv=cv2.__version__,bag=str(a.bag),train='takeoff+initial alignment; alternate frames',heldout='all descent frames after DESCEND+0.05s',heldout_frames=len(held),models={},scope='No runtime changes; intrinsics, mount, timing fixed; global pad RP=0; ID43 fixes pad origin/yaw')
    for mode in ['nominal_registration','uniform_scale','per_marker']:
        model=fit_model(data,mode,train,a.output);print(mode,'fit',model['train_rmse_per_coordinate_px'],model.get('uniform_scale'),model['near_bounds'],flush=True)
        for label,rs in [('train',train),('heldout',held)]:
            for guarded in [False,True]:
                key=label+('_coarse_guard' if guarded else '_native')
                model[key]=evaluate(data,model,rs,guarded)
                print(mode,key,{k:v for k,v in model[key].items() if k!='rows'},flush=True)
        report['models'][mode]=model
    original_layout=copy.deepcopy(LAYOUT)
    bundle=fit_image_only(data,train,a.output)
    LAYOUT=copy.deepcopy(original_layout);LAYOUT['markers']=bundle['markers']
    model=fit_model(data,'uniform_scale',train,a.output,tag='image_only_then_gt_scale');model['image_only_bundle']=bundle
    for label,rs in [('train',train),('heldout',held)]:
        for guarded in [False,True]:
            key=label+('_coarse_guard' if guarded else '_native');model[key]=evaluate(data,model,rs,guarded)
            print('image_only_then_gt_scale',key,{k:v for k,v in model[key].items() if k!='rows'},flush=True)
    report['models']['image_only_then_gt_scale']=model
    LAYOUT=original_layout
    bundle=fit_image_only(data,train,a.output,quad=True)
    LAYOUT=copy.deepcopy(original_layout);LAYOUT['markers']=bundle['markers']
    model=fit_model(data,'uniform_scale',train,a.output,tag='image_only_quads_then_gt_scale');model['image_only_bundle']=bundle
    for label,rs in [('train',train),('heldout',held)]:
        for guarded in [False,True]:
            key=label+('_coarse_guard' if guarded else '_native');model[key]=evaluate(data,model,rs,guarded)
            print('image_only_quads_then_gt_scale',key,{k:v for k,v in model[key].items() if k!='rows'},flush=True)
    report['models']['image_only_quads_then_gt_scale']=model
    LAYOUT=original_layout
    (a.output/'geometry_report.json').write_text(json.dumps(report,indent=2)+'\n')
    for name in ['per_marker','image_only_then_gt_scale','image_only_quads_then_gt_scale']:
        candidate=copy.deepcopy(LAYOUT);candidate['name']+='__'+name+'_offline_candidate'
        candidate['markers']=report['models'][name]['markers']
        candidate['calibration']=dict(method=name,source_bag=a.bag.stem,status='offline candidate; not deployed',
            runtime_note='quads require explicit corners_m support; current deployed detector reads square center/side/yaw only')
        (a.output/('b2_'+name+'_candidate.yaml')).write_text(yaml.safe_dump(json.loads(json.dumps(candidate)),sort_keys=False))

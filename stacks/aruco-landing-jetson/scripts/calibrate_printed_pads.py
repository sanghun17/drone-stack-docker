#!/usr/bin/env python3
"""Calibrate separately mounted printed pads from a shared camera bag.

Keeps camera intrinsics fixed and reconstructs planar square marker centers,
yaws and relative sizes. Nominal marker size anchors metric scale, which is
not independently surveyed. Time-block held-out views are used for validation.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.sparse import lil_matrix

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT/'ws/aruco-landing/src/aruco_landing/config'
CORNERS = np.array([[-.5,.5],[.5,.5],[.5,-.5],[-.5,-.5]])
LAYOUTS = {'baseline':CONFIG/'paper_pad_layout.yaml', 'proposed1':CONFIG/'proposed_pad_1_layout.yaml'}
cv2.setNumThreads(2)


def save(path, value):
    Path(path).write_text(json.dumps(value,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else a.item())+'\n')


def nominal(path, size=.7):
    layout=yaml.safe_load(Path(path).read_text());scale=size/layout['canvas_units']
    pars={m['id']:np.array([(m['x']+m['size']/2-layout['canvas_units']/2)*scale,
                           (layout['canvas_units']/2-m['y']-m['size']/2)*scale,
                           np.log(m['size']*scale),np.radians(m.get('yaw_deg',0))]) for m in layout['markers']}
    return layout,pars


def squares(ps):
    ps=np.atleast_2d(ps);c=np.cos(ps[:,3]);s=np.sin(ps[:,3])
    rots=np.stack((c,-s,s,c),axis=1).reshape(-1,2,2)
    return np.einsum('mij,kj->mki',rots,CORNERS)*np.exp(ps[:,2,None,None])+ps[:,:2,None].transpose(0,2,1)


def homography(pars, detected, omit=()):
    ids=[i for i in pars if i in detected and len(detected[i])==1 and i not in omit]
    if not ids:return None
    obj=np.concatenate([squares(pars[i])[0] for i in ids])
    pix=np.concatenate([detected[i][0]['undistorted'] for i in ids])
    H,mask=cv2.findHomography(obj,pix,cv2.RANSAC,5.)
    if H is None or not np.isfinite(H).all():return None
    return H


def project_h(H, points):
    p=np.c_[points,np.ones(len(points))]@H.T
    return p[:,:2]/p[:,2:3]


def assign(detected, models):
    baseline={i:v[0] for i,v in detected.items() if i in models['baseline'] and i!=1 and len(v)==1}
    proposed={0:detected[0][0]} if 0 in detected and len(detected[0])==1 else {}
    hs={'baseline':homography(models['baseline'],detected,omit=(1,)),
        'proposed1':homography({0:models['proposed1'][0]},detected)}
    costs=[]
    for name,H in hs.items():
        if H is None:continue
        predicted=project_h(H,squares(models[name][1])[0])
        side=np.linalg.norm(predicted-np.roll(predicted,-1,axis=0),axis=1).mean()
        for j,obs in enumerate(detected.get(1,[])):
            error=np.sqrt(np.mean(np.sum((predicted-obs['undistorted'])**2,axis=1)))
            if error<max(12.,side*.65):costs.append((error/max(side,1),name,j))
    assigned=set();used=set()
    for _,name,j in sorted(costs):
        if name in assigned or j in used:continue
        (baseline if name=='baseline' else proposed)[1]=detected[1][j]
        assigned.add(name);used.add(j)
    return {'baseline':baseline,'proposed1':proposed}


def extract(args):
    import rosbag
    out=args.output;out.mkdir(parents=True,exist_ok=True);(out/'frames').mkdir(exist_ok=True)
    models={name:nominal(path)[1] for name,path in LAYOUTS.items()}
    params=cv2.aruco.DetectorParameters();params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize=3;params.cornerRefinementMaxIterations=50
    params.cornerRefinementMinAccuracy=.01;params.adaptiveThreshWinSizeMax=101
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100),params)
    records=[];last=-1e30
    with rosbag.Bag(str(args.bag)) as bag:
        info=next(bag.read_messages(topics=['/landing/camera/camera_info']))[1]
        K=np.array(info.K).reshape(3,3);D=np.array(info.D)
        if not K[0,0]>0:raise ValueError('Missing calibrated CameraInfo')
        camera={'K':K,'D':D,'width':info.width,'height':info.height,'distortion_model':info.distortion_model,
                'frame_id':info.header.frame_id}
        save(out/'camera.json',camera)
        topic='/landing/camera/image_raw/compressed';t0=bag.get_start_time()
        for _,msg,bt in bag.read_messages(topics=[topic]):
            stamp=msg.header.stamp.to_sec()
            if stamp-last<args.period-.002:continue
            last=stamp
            image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
            if image.shape[:2]!=(info.height,info.width):raise ValueError('CameraInfo/image dimensions differ')
            gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            cs,ids,_=detector.detectMarkers(gray)
            detected={}
            if ids is not None:
                for pts,mid in zip(cs,ids.ravel()):
                    pts=pts.reshape(4,2)
                    side=np.linalg.norm(pts-np.roll(pts,-1,axis=0),axis=1).mean()
                    if side<15:continue
                    undist=cv2.undistortPoints(pts[:,None],K,D,P=K).reshape(4,2)
                    detected.setdefault(int(mid),[]).append({'raw':pts.tolist(),'undistorted':undist.tolist(),'side_px':float(side)})
            groups=assign(detected,models)
            record={'index':len(records),'stamp':stamp,'relative_s':stamp-t0,
                    'sharpness':float(cv2.Laplacian(gray,cv2.CV_64F).var()),'pads':groups,
                    'detected_id_counts':{i:len(v) for i,v in detected.items()}}
            records.append(record)
            if len(records)%10==1:
                overlay=image.copy()
                if ids is not None:cv2.aruco.drawDetectedMarkers(overlay,cs,ids)
                cv2.imwrite(str(out/'frames'/f'{record["index"]:04d}.jpg'),overlay)
            if len(records)%40==0:print('extract',len(records),'t',round(stamp-t0,1),{k:len(v) for k,v in groups.items()},flush=True)
    save(out/'detections.json',{'bag':str(args.bag.resolve()),'topic':topic,'period_s':args.period,'records':records})
    print('observations', {name:dict(Counter(int(i) for r in records for i in r['pads'][name])) for name in models},flush=True)


def pose(pars, obs, K):
    ids=[i for i in obs if i in pars]
    obj=np.c_[np.concatenate([squares(pars[i])[0] for i in ids]),np.zeros(4*len(ids))]
    pix=np.concatenate([obs[i]['undistorted'] for i in ids]).astype(float)
    solutions=cv2.solvePnPGeneric(obj,pix,K,None,flags=cv2.SOLVEPNP_IPPE)
    choices=[]
    for rv,tv in zip(solutions[1],solutions[2]):
        if tv[2,0]<=0:continue
        rv,tv=cv2.solvePnPRefineLM(obj,pix,K,None,rv,tv)
        error=np.linalg.norm(cv2.projectPoints(obj,rv,tv,K,None)[0].reshape(-1,2)-pix,axis=1)
        choices.append((np.mean(np.minimum(error,15)**2),np.r_[rv.ravel(),tv.ravel()]))
    return min(choices,key=lambda v:v[0])[1] if choices else None


def fit_bundle(records, initial, K, anchor, max_views=140):
    eligible=[r for r in records if len(r['obs'])>=2]
    if len(eligible)>max_views:eligible=[eligible[i] for i in np.linspace(0,len(eligible)-1,max_views).astype(int)]
    pairs=[(r,pose(initial,r['obs'],K)) for r in eligible]
    pairs=[(r,p) for r,p in pairs if p is not None]
    if len(pairs)<8:raise ValueError('Insufficient connected multi-marker views')
    records=[r for r,p in pairs];poses=np.array([p for r,p in pairs]);n=len(records)
    ids=sorted({i for r in records for i in r['obs']});variables=[i for i in ids if i!=anchor]
    if anchor not in ids:raise ValueError('Anchor not seen')
    offsets={i:6*n+4*j for j,i in enumerate(variables)};lookup={i:j for j,i in enumerate(ids)}
    obs=[(j,i,np.array(o['undistorted'])) for j,r in enumerate(records) for i,o in r['obs'].items() if i in ids]
    oi=np.array([o[0] for o in obs]);mi=np.array([lookup[o[1]] for o in obs]);pixels=np.array([o[2] for o in obs])
    x=np.r_[poses.ravel(),np.concatenate([initial[i] for i in variables])]
    priors=np.concatenate([initial[i] for i in variables]);prior_sigma=np.tile([.05,.05,.08,np.radians(10)],len(variables))
    sparsity=lil_matrix((len(obs)*8+4*len(variables),len(x)),dtype=int)
    for j,(frame,i,_) in enumerate(obs):
        sparsity[j*8:(j+1)*8,frame*6:frame*6+6]=1
        if i!=anchor:sparsity[j*8:(j+1)*8,offsets[i]:offsets[i]+4]=1
    for k in range(4*len(variables)):sparsity[len(obs)*8+k,6*n+k]=1
    def unpack(v):
        ps=np.array([initial[i] if i==anchor else v[offsets[i]:offsets[i]+4] for i in ids])
        return v[:6*n].reshape(n,6),ps
    def residual(v,with_prior=True):
        pp,ps=unpack(v);xy=squares(ps);xyz=np.concatenate((xy,np.zeros((len(ids),4,1))),axis=2)
        rotations=Rotation.from_rotvec(pp[:,:3]).as_matrix()
        cam=np.einsum('nij,nkj->nki',rotations[oi],xyz[mi])+pp[oi,3:,None].transpose(0,2,1)
        pred=cam@K.T;pred=pred[:,:,:2]/pred[:,:,2:3]
        r=(pred-pixels).ravel()
        return np.r_[r,(v[6*n:]-priors)/prior_sigma] if with_prior else r
    started=time.monotonic();mask=np.ones(len(obs)*8+4*len(variables),bool)
    for pass_index in range(2):
        print('fit pass',pass_index,'views',n,'markers',len(ids),'obs',len(obs),flush=True)
        fit=least_squares(lambda v:residual(v)[mask],x,jac_sparsity=sparsity.tocsr()[mask],
                          loss='soft_l1',f_scale=1.5,x_scale='jac',max_nfev=180,
                          ftol=2e-7,xtol=2e-7,gtol=2e-7,
                          tr_options={'atol':1e-7,'btol':1e-7,'maxiter':500})
        x=fit.x;errors=np.sqrt(np.mean(np.sum(residual(x,False).reshape(-1,4,2)**2,axis=2),axis=1))
        print('fit',fit.message,'nfev',fit.nfev,'median',np.median(errors),'p95',np.percentile(errors,95),'wall',time.monotonic()-started,flush=True)
        if pass_index==0:mask[:len(obs)*8]=np.repeat(errors<max(4.,np.median(errors)*4),8)
    pp,ps=unpack(x)
    result={i:ps[j] for j,i in enumerate(ids)}
    return result,{'anchor_id':anchor,'training_views':[r['index'] for r in records],
                   'observation_count':len(obs),'inlier_observation_count':int(mask[:len(obs)*8].sum()/8),
                   'median_marker_rmse_px':float(np.median(errors)), 'p95_marker_rmse_px':float(np.percentile(errors,95)),
                   'optimizer_success':bool(fit.success),'optimizer_message':fit.message,'nfev':fit.nfev,
                   'marker_observations':dict(Counter(i for _,i,_ in obs))}


def align_to_nominal(pars, base):
    ids=sorted(pars);a=np.array([pars[i][:2] for i in ids]);b=np.array([base[i][:2] for i in ids])
    ac=a.mean(0);bc=b.mean(0);u,_,vt=np.linalg.svd((a-ac).T@(b-bc));rot=vt.T@u.T
    if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
    yaw=np.arctan2(rot[1,0],rot[0,0]);out={}
    for i,p in pars.items():
        q=p.copy();q[:2]=rot@(p[:2]-ac)+bc;q[3]+=yaw;out[i]=q
    return out


def evaluate(records, pars, K):
    frames=[];per_marker={}
    for record in records:
        obs={i:o for i,o in record['obs'].items() if i in pars}
        if len(obs)<2:continue
        p=pose(pars,obs,K)
        if p is None:continue
        ids=list(obs);obj=np.c_[np.concatenate([squares(pars[i])[0] for i in ids]),np.zeros(4*len(ids))]
        pred=cv2.projectPoints(obj,p[:3],p[3:],K,None)[0].reshape(-1,4,2)
        pixels=np.array([obs[i]['undistorted'] for i in ids]);err=np.linalg.norm(pred-pixels,axis=2)
        frames.append({'index':record['index'],'relative_s':record['relative_s'],'rmse_px':float(np.sqrt(np.mean(err**2)))})
        for i,e in zip(ids,err):per_marker.setdefault(i,[]).extend(e.tolist())
    all_errors=np.concatenate(list(per_marker.values())) if per_marker else np.array([np.nan])
    return {'frames':frames,'frame_count':len(frames),'corner_rmse_px':float(np.sqrt(np.mean(all_errors**2))),
            'median_corner_error_px':float(np.median(all_errors)),'p95_corner_error_px':float(np.percentile(all_errors,95)),
            'per_marker_rmse_px':{i:float(np.sqrt(np.mean(np.array(e)**2))) for i,e in per_marker.items()}}


def screen_records(records, base, name):
    """Fixed nominal-map gate, shared by before/after and train/held-out sets.

    Reject gross decoded-ID/corner outliers, not millimetre-level map errors.
    Baseline requires six markers; Proposed requires its two unique markers.
    """
    output=[];removed=[]
    for record in records:
        obs=record['obs']
        if name=='baseline':
            if len(obs)<6:
                removed.append({'index':record['index'],'reason':'fewer than six baseline markers'});continue
            ids=list(obs);obj=np.concatenate([squares(base[i])[0] for i in ids])
            pix=np.concatenate([obs[i]['undistorted'] for i in ids])
            H,mask=cv2.findHomography(obj,pix,cv2.RANSAC,5.,maxIters=3000,confidence=.999)
            if H is None:continue
            errors=np.sqrt(np.mean(np.sum((project_h(H,obj)-pix).reshape(-1,4,2)**2,axis=2),axis=1))
            keep={i:obs[i] for i,e in zip(ids,errors) if e<max(8.,.1*obs[i]['side_px'])}
            if len(keep)<6 or len(keep)<.6*len(obs):
                removed.append({'index':record['index'],'reason':'inconsistent baseline geometry'});continue
            for i,e in zip(ids,errors):
                if i not in keep:removed.append({'index':record['index'],'marker_id':i,'nominal_homography_rmse_px':float(e)})
            obs=keep
        elif len(obs)!=2:continue
        output.append(dict(record,obs=obs))
    return output,removed


def fit(args):
    out=args.output;data=json.loads((out/'detections.json').read_text());camera=json.loads((out/'camera.json').read_text());K=np.array(camera['K'])
    report={}
    for name,path in LAYOUTS.items():
        layout,base=nominal(path)
        records=[dict(r,obs={int(i):o for i,o in r['pads'][name].items()}) for r in data['records']]
        raw_records=records
        records,screening=screen_records(records,base,name)
        train=[r for r in records if int(r['relative_s']//2)%5!=4];test=[r for r in records if int(r['relative_s']//2)%5==4]
        counts=Counter(i for r in train for i in r['obs'])
        anchor=(max([i for i in counts if i>=91],key=lambda i:counts[i]) if name=='baseline' else 1)
        pars,stats=fit_bundle(train,base,K,anchor,args.max_views)
        pars=align_to_nominal(pars,base)
        metrics={'training':{'nominal':evaluate(train,base,K),'calibrated':evaluate(train,pars,K)},
                 'held_out':{'nominal':evaluate(test,base,K),'calibrated':evaluate(test,pars,K)}}
        missing=sorted(set(base)-set(pars))
        if missing:raise ValueError(f'{name}: unobserved markers {missing}; refusing partial YAML')
        result=copy.deepcopy(layout);result['name']=layout['name']+'_physical_20260920'
        result['required_pad_size_m']=.7
        result['calibration']={'source_bag':Path(data['bag']).name,'source_layout':path.name,
                               'source_layout_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                               'camera_info_topic':'/landing/camera/camera_info','image_topic':data['topic'],
                               'metric_scale':'nominal anchor marker size; not independently measured',
                               'anchor_marker_id':anchor,'anchor_nominal_side_m':float(np.exp(base[anchor][2])),
                               'frame':'same layout/image convention as source; rigid best alignment to nominal centers; not surveyed origin',
                               'yaw_zero_body_coordinates':'X_forward = layout_y_up; Y_left = -layout_x_right',
                               'method':'fixed-intrinsic planar square-marker bundle adjustment; robust loss; centers/yaw/relative size',
                               'held_out_rule':'2-second blocks with floor(relative_s/2) modulo 5 equal to 4',
                               'held_out_corner_rmse_px':{k:v['corner_rmse_px'] for k,v in metrics['held_out'].items()}}
        scale=.7/layout['canvas_units'];changes=[]
        for m in result['markers']:
            i=m['id'];p=pars[i];side=np.exp(p[2]);m.update(x=float((p[0]+.35-side/2)/scale),y=float((.35-p[1]-side/2)/scale),
                                                        size=float(side/scale),yaw_deg=float((np.degrees(p[3])+180)%360-180))
            changes.append({'id':i,'dx_mm':float((p[0]-base[i][0])*1000),'dy_mm':float((p[1]-base[i][1])*1000),
                            'yaw_deg':m['yaw_deg'],'nominal_side_mm':float(np.exp(base[i][2])*1000),'side_mm':float(side*1000)})
        target=path.with_name(path.stem+'_physical_20260920.yaml')
        target.write_text('# Calibrated physical pad; use pad_size_m=0.7. Original simulation layout unchanged.\n'+yaml.safe_dump(result,sort_keys=False))
        export_runtime(target)
        report[name]={'output_yaml':str(target),'fit':stats,'metrics':metrics,'changes':changes,'screening':screening,
                      'raw_held_out':{'nominal':evaluate([r for r in raw_records if int(r['relative_s']//2)%5==4],base,K),
                                      'calibrated':evaluate([r for r in raw_records if int(r['relative_s']//2)%5==4],pars,K)}}
        save(out/f'{name}_report.json',report[name]);print('SAVED',target,metrics['held_out']['nominal']['corner_rmse_px'],'->',metrics['held_out']['calibrated']['corner_rmse_px'],flush=True)
    save(out/'report.json',report)
    write_report(out,report)

def export_runtime(layout_path):
    """Convert source layout to the canonical hardware detector's metric schema."""
    layout,pars=nominal(layout_path)
    runtime={'name':layout['name']+'_landing', 'dictionary':layout['dictionary'],
             'frame_convention':'pad center origin; yaw 0: +X forward/image up, +Y image left, +Z out of paper',
             'source_layout':Path(layout_path).name,
             'calibration':copy.deepcopy(layout.get('calibration',{})), 'markers':[]}
    for i,p in sorted(pars.items()):
        runtime['markers'].append({'id':i,'center_m':{'x':float(p[1]),'y':float(-p[0])},
                                   'side_m':float(np.exp(p[2])),
                                   'yaw_deg':float((np.degrees(p[3])-90+180)%360-180)})
    target=Path(layout_path).with_name(Path(layout_path).stem+'_runtime.yaml')
    target.write_text('# Hardware physical_pad_estimator configuration. Metric coordinates; no pad_size_m parameter needed.\n'+yaml.safe_dump(runtime,sort_keys=False))
    return target


def write_report(out, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig,axes=plt.subplots(2,2,figsize=(12,11))
    rows=['# Printed pad calibration — 2026-09-20 23:45:36', '',
          'Both YAMLs sit beside the original simulation layouts. Originals were not changed.',
          'Use `pad_size_m=0.7`. These are camera-derived physical layouts, not independent ground truth.',
          'Image observations share fixed recorded CameraInfo intrinsics. Baseline/Proposed ID 1 detections are assigned by their respective board geometry.',
          'Scale is fixed by one nominal marker size per pad, as confirmed by the user; no independent ruler measurement was supplied.',
          'Optimized variables: planar marker center, yaw, relative square size, and per-view camera pose.',
          'After fitting, a rigid alignment to nominal marker centers preserves the design frame convention. The physical pad origin is not independently surveyed.',
          'YAML x/y remain the source top-left/image convention. In the centered frame, body X-forward = image-up, body Y-left = -image-right; yaw 0 means +X up, +Y left.',
          'Time-block holdout: floor(seconds/2) modulo 5 = 4. Camera pose is fitted independently in each validation frame; pixel reprojection is not metric trajectory RMSE.',
          'A fixed nominal-map homography gate rejects gross ID/corner outliers, using the same observations for nominal and calibrated scores. Baseline needs >=6 consistent markers; Proposed needs both markers.',
          'Unscreened held-out metrics are retained separately in JSON, including misdetections and degenerate partial views.', '',
          '| Pad | Markers | Train views | Held-out views | Held-out corner RMS: nominal → calibrated (px) | Median / max center correction (mm) |',
          '|---|---:|---:|---:|---|---|']
    for col,(name,result) in enumerate(report.items()):
        layout,base=nominal(LAYOUTS[name]);_,physical=nominal(Path(result['output_yaml']))
        ax=axes[0,col]
        for i in base:
            original=squares(base[i])[0]*1000;actual=squares(physical[i])[0]*1000
            ax.add_patch(Polygon(original,fill=False,edgecolor='.65',linestyle='--',linewidth=.7))
            ax.add_patch(Polygon(actual,fill=False,edgecolor='#0969da',linewidth=.9))
            ax.text(*physical[i][:2]*1000,str(i),fontsize=6 if name=='baseline' else 10,ha='center',va='center')
        ax.set_aspect('equal');ax.autoscale_view();ax.margins(.12)
        ax.set_xlabel('Right = -body Y (mm)');ax.set_ylabel('Up = +body X (mm)')
        ax.set_title(name+': gray = nominal; blue = calibrated');ax.grid(alpha=.2)
        ax=axes[1,col];scores=result['metrics']['held_out']
        for label,color in [('nominal','#777777'),('calibrated','#0969da')]:
            frames=scores[label]['frames'];ax.plot([v['relative_s'] for v in frames],[v['rmse_px'] for v in frames],'.-',label=label,color=color)
        ax.set_xlabel('Bag relative time (s)');ax.set_ylabel('Held-out per-frame corner RMS (px)');ax.legend();ax.grid(alpha=.2)
        changes=result['changes'];displacements=np.array([np.hypot(c['dx_mm'],c['dy_mm']) for c in changes])
        result['summary']={'median_center_correction_mm':float(np.median(displacements)),
                           'max_center_correction_mm':float(displacements.max()),
                           'max_abs_yaw_deg':float(max(abs(c['yaw_deg']) for c in changes))}
        rows.append(f"| {name} | {len(changes)} | {len(result['fit']['training_views'])} | {scores['calibrated']['frame_count']} | {scores['nominal']['corner_rmse_px']:.3f} → {scores['calibrated']['corner_rmse_px']:.3f} | {np.median(displacements):.2f} / {displacements.max():.2f} |")
        csv=['id,dx_image_right_mm,dy_image_up_mm,dx_body_forward_mm,dy_body_left_mm,yaw_deg,nominal_side_mm,calibrated_side_mm']
        for c in changes:csv.append(','.join(str(v) for v in [c['id'],c['dx_mm'],c['dy_mm'],c['dy_mm'],-c['dx_mm'],c['yaw_deg'],c['nominal_side_mm'],c['side_mm']]))
        (out/f'{name}_marker_changes.csv').write_text('\n'.join(csv)+'\n')
        save(out/f'{name}_report.json',result)
    fig.tight_layout();fig.savefig(out/'calibration_comparison.png',dpi=180);fig.savefig(out/'calibration_comparison.pdf');plt.close(fig)
    rows+=['', 'The square-marker model cannot correct internal grid deformation or board warping. Residuals include detection, lens calibration and nonplanarity errors.',
           'Proposed large-marker size is estimated relative to the fixed 57.4 mm small marker, not a direct physical measurement.',
           '', '## YAML paths', *['- '+v['output_yaml'] for v in report.values()], '',
           '## Reproduce', '```bash',
           '/usr/bin/python3 stacks/aruco-landing-jetson/scripts/calibrate_printed_pads.py extract --bag flight_logs/pad-calibration-20260920-234536/2026-09-20-23-45-36.bag --output flight_logs/pad-calibration-20260920-234536',
           '/usr/bin/python3 stacks/aruco-landing-jetson/scripts/calibrate_printed_pads.py fit --output flight_logs/pad-calibration-20260920-234536 --max-views 120',
           '```', '', 'No files were uploaded to Jetson and no runtime estimator configuration was switched.']
    (out/'README.md').write_text('\n'.join(rows)+'\n');save(out/'report.json',report)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('stage',choices=['extract','fit'])
    parser.add_argument('--bag',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--period',type=float,default=.2);parser.add_argument('--max-views',type=int,default=140)
    args=parser.parse_args();extract(args) if args.stage=='extract' else fit(args)

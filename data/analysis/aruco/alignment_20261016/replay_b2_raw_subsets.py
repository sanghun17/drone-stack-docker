#!/usr/bin/env python3
"""Exact native-OpenCV B2 raw replay and diagnostic marker subsets.

Use only original raw images. No fitted geometry and no runtime modifications.
Stage trace mirrors production IPPE -> all-point LM -> 4px subset LM.
"""
import argparse,json,sys
from pathlib import Path
import cv2,numpy as np,rosbag,yaml
from scipy.spatial.transform import Rotation
from audit import matrix,interpolate,quantile


def replay(bag,owner,root,out):
    sys.path.insert(0,str(owner/'src'))
    from aruco_landing.physical_pad import PhysicalPadDetector,center_crop,inverse
    layout=yaml.safe_load((owner/'config/baseline_b2_700mm_runtime.yaml').read_text())
    X=np.array(yaml.safe_load((root/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml').read_text())['matrix_row_major']).reshape(4,4)
    gt=[];live=[];pads=[];K=D=None;params=None
    with rosbag.Bag(str(bag)) as b:
        start=b.get_start_time()
        for topic,m,_ in b.read_messages(topics=['/vrpn_client_node/pure/pose','/landing/vision_pose_marker','/landing/pad_pose_global','/landing/camera/camera_info','/landing/estimator/status']):
            if topic=='/vrpn_client_node/pure/pose':gt.append((m.header.stamp.to_sec(),matrix(m.pose)))
            elif topic=='/landing/vision_pose_marker':live.append((m.header.stamp.to_sec(),matrix(m.pose)))
            elif topic=='/landing/pad_pose_global':pads.append(matrix(m.pose))
            elif topic=='/landing/camera/camera_info':K=np.array(m.K).reshape(3,3);D=np.array(m.D)
            else:params=json.loads(m.data)
    detector=PhysicalPadDetector(layout,min_markers=1)
    detector.params.adaptiveThreshWinSizeMin=3;detector.params.adaptiveThreshWinSizeMax=23;detector.params.adaptiveThreshWinSizeStep=10
    cv2.setNumThreads(2)
    Y=pads[0];offset=params['image_time_offset_s'];times=np.array([x[0] for x in live]);liveT=np.array([x[1] for x in live]);rows=[]
    def errors(C_P,G):
        M=Y@inverse(C_P)@inverse(X);delta=M[:3,3]-G[:3,3]
        return dict(position_m=float(np.linalg.norm(delta)),rotation_deg=float(np.degrees(Rotation.from_matrix(M[:3,:3]@G[:3,:3].T).magnitude())))
    def trace(corners,ids,G,K):
        obj=[];pix=[];used=[]
        for c,mid in zip(corners,ids):
            p=np.array(c).reshape(4,2)
            if mid not in detector.models or np.linalg.norm(p-np.roll(p,-1,axis=0),axis=1).mean()<12:continue
            obj.extend(detector.models[mid]);pix.extend(p);used.append(mid)
        if not used:return None
        obj=np.array(obj,np.float64);pix=np.array(pix,np.float64)
        sol=cv2.solvePnPGeneric(obj,pix,K,D,flags=cv2.SOLVEPNP_IPPE,reprojectionError=np.zeros((2,1),np.float64));choices=[]
        for rv,tv in zip(sol[1],sol[2]):
            rv,tv=cv2.solvePnPRefineLM(obj,pix,K,D,rv,tv)
            e=np.linalg.norm(cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)-pix,axis=1)
            if np.isfinite(e).all() and tv[2,0]>0:choices.append((float(np.median(e)+np.mean(np.minimum(e,10))),rv.copy(),tv.copy(),e))
        if not choices:return None
        score,rv,tv,e=min(choices,key=lambda c:c[0]);T=np.eye(4);T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv.ravel()
        before=errors(T,G);good=np.flatnonzero(e<4.)
        if len(good)<.75*len(obj):return dict(before=before,rejected=True)
        rv,tv=cv2.solvePnPRefineLM(obj[good],pix[good],K,D,rv,tv)
        T[:3,:3]=cv2.Rodrigues(rv)[0];T[:3,3]=tv.ravel();after=errors(T,G)
        C_P_gt=inverse(X)@inverse(G)@Y
        rvgt=cv2.Rodrigues(C_P_gt[:3,:3])[0];tvgt=C_P_gt[:3,3]
        eg=np.linalg.norm(cv2.projectPoints(obj,rvgt,tvgt,K,D)[0].reshape(-1,2)-pix,axis=1)
        return dict(before=before,after=after,good_points=len(good),total_points=len(obj),
                    per_marker_good_points={str(mid):int(np.isin(np.arange(4*i,4*i+4),good).sum()) for i,mid in enumerate(used)},
                    per_marker_rms_vs_gt_px={str(mid):float(np.sqrt(np.mean(eg[4*i:4*i+4]**2))) for i,mid in enumerate(used)})
    with rosbag.Bag(str(bag)) as b:
        for _,m,bt in b.read_messages(topics=['/landing/camera/image_raw']):
            stamp=m.header.stamp.to_sec()+offset;G,ok=interpolate(gt,np.array([stamp]));G=G[0]
            if not ok[0] or G[2,3]<1.5:continue
            channels=1 if m.encoding=='mono8' else 3
            image=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*channels].reshape(m.height,m.width,channels)
            gray=image[:,:,0] if channels==1 else cv2.cvtColor(image,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
            gray,ck=center_crop(gray,K,720,720)
            result,corners,ids=detector.detect(gray,ck,D);row=dict(bag_s=bt.to_sec()-start,stamp=stamp,gt_height_m=float(G[2,3]),ids=ids,corners={str(i):np.asarray(c).reshape(4,2).tolist() for c,i in zip(corners,ids)},poses={},trace=None)
            j=int(np.argmin(abs(times-stamp)))
            if result is not None:
                err=errors(result['camera_from_pad'],G);row['poses']['all']=dict(**err,rms_px=result['rms_px'],inliers=result['inlier_ids'])
                if abs(times[j]-stamp)<1e-5:
                    M=Y@inverse(result['camera_from_pad'])@inverse(X)
                    row['replay_vs_live_m']=float(np.linalg.norm(M[:3,3]-liveT[j,:3,3]))
                    row['live_error_m']=float(np.linalg.norm(liveT[j,:3,3]-G[:3,3]))
                row['trace']=trace(corners,ids,G,ck)
            subsets={str(mid):{mid} for mid in [1,2,3,4,19,43]}
            subsets.update(without19={1,2,3,4,43},without43={1,2,3,4,19},outers={1,2,3,4})
            for key,subset in subsets.items():
                pairs=[(c,i) for c,i in zip(corners,ids) if i in subset]
                if not pairs:continue
                res=detector.estimate([c for c,i in pairs],[i for c,i in pairs],ck,D)
                if res is not None:row['poses'][key]=dict(**errors(res['camera_from_pad'],G),rms_px=res['rms_px'],inliers=res['inlier_ids'])
            rows.append(row)
            if len(rows)%100==0:print('raw replay frames',len(rows),flush=True)
    summary=dict(bag=bag.stem,opencv=cv2.__version__,scope='original raw, GT z>=1.5m; original fixed registration; no fitting',frames=len(rows),subsets={})
    for key in ['all','without19','without43','outers','1','2','3','4','19','43']:
        poses=[r['poses'][key] for r in rows if key in r['poses']]
        summary['subsets'][key]=dict(n=len(poses),position_m_p50_p95_max=quantile([v['position_m'] for v in poses]),
            rotation_deg_p50_p95_max=quantile([v['rotation_deg'] for v in poses]),over20cm=sum(v['position_m']>.2 for v in poses))
    summary['replay_vs_live_m_p50_p95_max']=quantile([r['replay_vs_live_m'] for r in rows if 'replay_vs_live_m' in r])
    traces=[r['trace'] for r in rows if r['trace'] and 'after' in r['trace']]
    summary['refinement_trace']=dict(n=len(traces),before_good_after_bad=sum(t['before']['position_m']<=.2<t['after']['position_m'] for t in traces),
        before_bad_after_good=sum(t['after']['position_m']<=.2<t['before']['position_m'] for t in traces),
        before_position_m_p50_p95_max=quantile([t['before']['position_m'] for t in traces]),after_position_m_p50_p95_max=quantile([t['after']['position_m'] for t in traces]))
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(dict(summary=summary,rows=rows),indent=2)+'\n');print(json.dumps(summary,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag',type=Path);p.add_argument('--owner',type=Path,required=True);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[4]);p.add_argument('--output',type=Path,required=True);a=p.parse_args();replay(a.bag,a.owner,a.root,a.output)

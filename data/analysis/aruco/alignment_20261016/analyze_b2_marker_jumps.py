#!/usr/bin/env python3
"""Read-only per-frame B2 pose jumps and marker-set associations.

Inlier/ID messages lack a header: associate nearest bag receipt within 8 ms,
report the association timing, and keep unknown associations separate.
Membership is association, not a causal leave-one-marker-out experiment.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation
from audit import matrix,interpolate,quantile


def nearby(stream,t,limit=.008):
    if not stream:return None,None
    times=np.array([s[0] for s in stream]);j=int(np.argmin(abs(times-t)));dt=float(times[j]-t)
    return (stream[j][1] if abs(dt)<=limit else None),dt


def stats(rows):
    if not rows:return dict(n=0)
    return dict(n=len(rows),bad=sum(r['bad'] for r in rows),bad_pct=100*sum(r['bad'] for r in rows)/len(rows),
        position_m_p50_p95_max=quantile([r['error_m'] for r in rows]),
        rotation_deg_p50_p95_max=quantile([r['angle_deg'] for r in rows]))


def analyze(path):
    gt=[];marker=[];inliers=[];ids=[];trials=[];router=[];info=[];visible=[];est=[];camera=[]
    topics=['/vrpn_client_node/pure/pose','/landing/vision_pose_marker','/landing/estimator/inlier_ids',
            '/landing/markers/ids','/landing/trial/status','/landing/pose_transition/status',
            '/landing/camera/camera_info','/landing/target_visible','/landing/estimator/status']
    with rosbag.Bag(str(path)) as b:
        start=b.get_start_time();raw_count=b.get_type_and_topic_info().topics.get('/landing/camera/image_raw')
        for topic,m,stamp in b.read_messages(topics=topics):
            t=stamp.to_sec()-start
            if topic=='/vrpn_client_node/pure/pose':gt.append((m.header.stamp.to_sec(),matrix(m.pose)))
            elif topic=='/landing/vision_pose_marker':marker.append((t,m.header.stamp.to_sec(),matrix(m.pose)))
            elif topic=='/landing/estimator/inlier_ids':inliers.append((t,sorted(m.data)))
            elif topic=='/landing/markers/ids':ids.append((t,sorted(m.data)))
            elif topic=='/landing/trial/status':trials.append((t,json.loads(m.data)))
            elif topic=='/landing/pose_transition/status':router.append((t,json.loads(m.data)))
            elif topic=='/landing/camera/camera_info':info.append((t,m.header.stamp.to_sec()))
            elif topic=='/landing/target_visible':visible.append((t,m.data))
            elif topic=='/landing/estimator/status':est.append((t,json.loads(m.data)))
    lo=next(t for t,d in trials if d['phase']=='APPROACH')
    hi=next(t for t,d in trials if t>lo and d['phase']!='APPROACH')
    selected=[m for m in marker if lo<=m[0]<hi]
    G,valid=interpolate(gt,np.array([m[1] for m in selected]));rows=[]
    for m,g,ok in zip(selected,G,valid):
        if not ok:continue
        t,stamp,T=m;delta=T[:3,3]-g[:3,3];error=float(np.linalg.norm(delta))
        R=T[:3,:3]@g[:3,:3].T;angle=float(np.degrees(Rotation.from_matrix(R).magnitude()))
        ins,di=nearby(inliers,t);det,dd=nearby(ids,t)
        rows.append(dict(t_s=t,stamp=stamp,error_m=error,angle_deg=angle,delta_xyz_m=delta.tolist(),
            delta_rpy_deg=Rotation.from_matrix(R).as_euler('xyz',degrees=True).tolist(),
            bad=error>.2 or angle>12,trial_bad=error>.15 or angle>12,
            marker_rotation=T[:3,:3].tolist(),gt_rotation=g[:3,:3].tolist(),
            inliers=ins,detected_ids=det,inlier_association_dt_s=di,ids_association_dt_s=dd,
            marker_xyz=T[:3,3].tolist(),gt_xyz=g[:3,3].tolist(),gt_height_m=float(g[2,3])))
    episodes=[];episode=[]
    for row in rows:
        if row['bad']:
            if episode and row['t_s']-episode[-1]['t_s']>.06:episodes.append(episode);episode=[]
            episode.append(row)
        elif episode:episodes.append(episode);episode=[]
    if episode:episodes.append(episode)
    jumps=[]
    for a,b in zip(rows,rows[1:]):
        if b['stamp']-a['stamp']>.05:continue
        step=float(np.linalg.norm(np.array(b['marker_xyz'])-a['marker_xyz']));gtstep=float(np.linalg.norm(np.array(b['gt_xyz'])-a['gt_xyz']))
        if step>.15:jumps.append(dict(t_s=b['t_s'],pose_step_m=step,gt_step_m=gtstep,
            gt_rotation_step_deg=float(np.degrees(Rotation.from_matrix(np.array(b['gt_rotation'])@np.array(a['gt_rotation']).T).magnitude())),
            marker_rotation_step_deg=float(np.degrees(Rotation.from_matrix(np.array(b['marker_rotation'])@np.array(a['marker_rotation']).T).magnitude())),
            same_inliers=a['inliers']==b['inliers'],inliers_before=a['inliers'],inliers_after=b['inliers']))
    checks=[d['switch_check'] for t,d in router if lo<=t<hi];vv=[v for t,v in visible if lo<=t<hi]
    ci=[t for t,_ in info if lo<=t<hi];es=[d for t,d in est if lo<=t<hi]
    sets=sorted({tuple(r['inliers']) for r in rows if r['inliers'] is not None})
    per_set=[dict(ids=list(s),**stats([r for r in rows if r['inliers'] is not None and tuple(r['inliers'])==s])) for s in sets]
    per_marker={str(mid):dict(included=stats([r for r in rows if r['inliers'] is not None and mid in r['inliers']]),
        excluded=stats([r for r in rows if r['inliers'] is not None and mid not in r['inliers']])) for mid in [1,2,3,4,19,43]}
    gt_steps=[]
    for (ta,A),(tb,B) in zip(gt,gt[1:]):
        angle=float(np.degrees(Rotation.from_matrix(B[:3,:3]@A[:3,:3].T).magnitude()))
        if angle>20:
            gt_steps.append(dict(t_s=tb-start,dt_s=tb-ta,rotation_step_deg=angle,
                translation_step_m=float(np.linalg.norm(B[:3,3]-A[:3,3])),in_approach=lo<=tb-start<hi))
    report=dict(session=path.stem,gt_attitude_steps_over20deg=gt_steps,approach_s=[lo,hi],duration_s=hi-lo,pose_frames=stats(rows),
        trial_gate_bad_frames=sum(r['trial_bad'] for r in rows),
        detection=dict(messages=len(vv),true=sum(vv)),camera=dict(camera_info_samples=len(ci),
            camera_info_hz=(len(ci)-1)/(ci[-1]-ci[0]),max_info_gap_s=max(np.diff(ci)),
            estimate_hz_p50_p95_max=quantile([d['processing_hz'] for d in es]),
            raw_images_recorded=0 if raw_count is None else raw_count.message_count),
        marker_set_association_dt_abs_s_p50_p95_max=quantile([abs(r['inlier_association_dt_s']) for r in rows]),
        unknown_marker_associations=sum(r['inliers'] is None for r in rows),
        router=dict(checks=len(checks),bad_checks=sum(not c['consistent'] for c in checks),
            max_stable_s=max(c['stable_duration_s'] for c in checks),ready_checks=sum(c['ready'] for c in checks)),
        bad_episodes=dict(count=len(episodes),per_second=len(episodes)/(hi-lo),
            duration_s_p50_p95_max=quantile([e[-1]['t_s']-e[0]['t_s']+1/60 for e in episodes]),
            peak_error_m_p50_p95_max=quantile([max(r['error_m'] for r in e) for e in episodes])),
        interframe_jumps=dict(count=len(jumps),per_second=len(jumps)/(hi-lo),
            magnitude_m_p50_p95_max=quantile([j['pose_step_m'] for j in jumps]),
            gt_step_m_p50_p95_max=quantile([j['gt_step_m'] for j in jumps]),
            unchanged_inliers=sum(j['same_inliers'] for j in jumps)),
        by_inlier_set=sorted(per_set,key=lambda d:d['n'],reverse=True),by_inlier_marker=per_marker,
        error_angle_correlation=float(np.corrcoef([r['error_m'] for r in rows],[r['angle_deg'] for r in rows])[0,1]),
        bad_delta_xyz_m_median=np.median([r['delta_xyz_m'] for r in rows if r['bad']],axis=0).tolist(),
        bad_rpy_deg_median=np.median([r['delta_rpy_deg'] for r in rows if r['bad']],axis=0).tolist())
    return report,rows,jumps


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bags',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);reports=[]
    for bag in a.bags:
        report,rows,jumps=analyze(bag);reports.append(report)
        (a.output/(bag.stem+'.json')).write_text(json.dumps(dict(report=report,rows=rows,jumps=jumps),indent=2)+'\n')
        print(json.dumps({k:report[k] for k in ['session','pose_frames','bad_episodes','interframe_jumps','router']},indent=2),flush=True)
    (a.output/'summary.json').write_text(json.dumps(dict(reports=reports),indent=2)+'\n')

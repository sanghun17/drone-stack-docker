#!/usr/bin/env python3
"""Read-only, timestamp-interpolated audit of recorded hardware pose disagreement.

Run with /usr/bin/python3 (ROS Noetic, numpy, scipy, matplotlib, PyYAML).
Never publish ROS messages or modify source recordings. Quantiles use matched
measurement stamps, not bag receipt time. Constant-frame fits are diagnostic,
not an automatically deployable calibration.
"""
import argparse
from collections import Counter
import glob
import hashlib
import json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp
import yaml

POSES = {'gt': '/vrpn_client_node/pure/pose', 'marker': '/landing/vision_pose_marker',
         'local': '/mavros/local_position/pose', 'pad': '/landing/pad_pose_global',
         'relative': '/landing/vehicle_pose_pad', 'vision': '/mavros/vision_pose/pose',
         'camera_target': '/landing/target_pose_camera'}

def matrix(pose):
    if hasattr(pose, 'pose'): pose = pose.pose
    p, q = pose.position, pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T

def quantile(v):
    return np.percentile(v, [50, 95, 100]).tolist() if len(v) else None

def interpolate(samples, times):
    t, T = np.array([s[0] for s in samples]), np.array([s[1] for s in samples])
    unique = np.r_[True, np.diff(t) > 0]
    t, T = t[unique], T[unique]
    index = np.searchsorted(t, times, side='right').clip(1, len(t)-1)
    ok = (times >= t[0]) & (times <= t[-1]) & (t[index]-t[index-1] <= .04)
    f = ((times-t[index-1])/(t[index]-t[index-1])).clip(0, 1)
    out = np.tile(np.eye(4), (len(times), 1, 1))
    out[:, :3, 3] = T[index-1, :3, 3]*(1-f[:, None])+T[index, :3, 3]*f[:, None]
    out[:, :3, :3] = Slerp(t, Rotation.from_matrix(T[:, :3, :3]))(times.clip(t[0], t[-1])).as_matrix()
    return out, ok

def summary(delta, angles):
    return {'count': len(delta), 'position_m_p50_p95_max': quantile(np.linalg.norm(delta, axis=1)),
            'median_delta_xyz_m': np.median(delta, axis=0).tolist() if len(delta) else None,
            'angle_deg_p50_p95_max': quantile(angles)}

def audit(path, output):
    samples = {k: [] for k in POSES}
    reverse = {v: k for k, v in POSES.items()}
    events, ids, gyro, status = [], [], [], []
    last = {}
    with rosbag.Bag(str(path)) as bag:
        start = bag.get_start_time()
        topics = bag.get_type_and_topic_info().topics
        report = {'bag': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                  'duration_s': bag.get_end_time()-start,
                  'topics': {k: {'type': v.msg_type, 'count': v.message_count} for k, v in topics.items()}}
        selected = list(POSES.values()) + ['/landing/trial/status', '/flight_safety/state',
            '/landing/vision_pose_source', '/landing/estimator/inlier_ids', '/mavros/imu/data_raw',
            '/landing/estimator/status', '/landing/pose_transition/status']
        for topic, msg, stamp in bag.read_messages(topics=selected):
            now = stamp.to_sec()-start
            if topic in reverse:
                samples[reverse[topic]].append((msg.header.stamp.to_sec(), matrix(msg.pose), stamp.to_sec()))
            elif topic == '/landing/estimator/inlier_ids':
                ids.append((stamp.to_sec(), tuple(msg.data)))
            elif topic == '/mavros/imu/data_raw':
                w = msg.angular_velocity
                gyro.append((msg.header.stamp.to_sec(), [w.x,w.y,w.z]))
            elif topic == '/landing/estimator/status':
                status.append(json.loads(msg.data))
            elif topic == '/landing/trial/status':
                d = json.loads(msg.data)
                key = (d.get('phase'), d.get('reason'), d.get('estimation_source'))
                if last.get(topic) != key:
                    events.append({'s': now, 'kind': 'trial', 'phase': key[0], 'reason': key[1], 'source': key[2]})
                last[topic] = key
            elif topic == '/flight_safety/state':
                key = (msg.level, msg.control_lane, msg.active_response)
                if last.get(topic) != key:
                    events.append({'s': now, 'kind': 'safety', 'level': int(msg.level),
                        'lane': msg.control_lane, 'response': msg.active_response, 'summary': msg.summary})
                last[topic] = key
            elif topic == '/landing/vision_pose_source':
                events.append({'s': now, 'kind': 'source', 'source': msg.data})
    report['events'] = sorted(events, key=lambda d:d['s'])
    report['inlier_counts'] = {str(k):v for k,v in Counter(x[1] for x in ids).items()}
    report['estimator_first_status'] = status[0] if status else None
    report['estimator_last_status'] = status[-1] if status else None
    if samples['relative'] and samples['camera_target']:
        target={s[0]:s[1] for s in samples['camera_target']}
        mounts=[np.linalg.inv(s[1])@np.linalg.inv(target[s[0]])
                for s in samples['relative'] if s[0] in target]
        if mounts:
            mounts=np.array(mounts)
            calibration=Path(__file__).resolve().parents[4]/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml'
            expected=np.array(yaml.safe_load(calibration.read_text())['matrix_row_major']).reshape(4,4)
            report['runtime_mount_recovered_from_pose_topics']={
                'count':len(mounts),'first_matrix':mounts[0].tolist(),
                'max_abs_matrix_difference_to_checked_in_calibration':float(np.abs(mounts-expected).max()),
                'scope':'checks software transform actually applied; cannot verify physical mounting'}
    manifest = path.with_suffix('.runtime_manifest.yaml')
    if manifest.exists():
        snap = yaml.safe_load(manifest.read_text())['snapshots']['start']
        report['runtime'] = {'repositories': {k:{n:v.get(n) for n in ('head','dirty')} for k,v in snap['repositories'].items()},
            'parameters': {k:v for k,v in snap['ros']['parameters'].items() if k in (
                'physical_pad_estimator','landing_vision_pose_adapter','flight_safety_diagnosis')},
            'config_sha256': {k:v.get('sha256') for k,v in snap['config_files'].items()}}
    all_arrays = {}
    for name in ('marker','local','vision'):
        if not samples[name] or len(samples['gt']) < 2: continue
        times = np.array([s[0] for s in samples[name]])
        estimate = np.array([s[1] for s in samples[name]])
        gt, ok = interpolate(samples['gt'], times)
        times, estimate, gt = times[ok], estimate[ok], gt[ok]
        delta = estimate[:,:3,3]-gt[:,:3,3]
        angles = np.rad2deg(Rotation.from_matrix(np.swapaxes(gt[:,:3,:3],1,2)@estimate[:,:3,:3]).magnitude())
        report[name+'_vs_gt'] = summary(delta, angles)
        switches = [d['s'] for d in events if d['kind']=='source' and d['source']=='marker']
        if switches:
            switched = times-start >= min(switches)
            report[name+'_vs_gt']['after_first_marker_switch'] = summary(delta[switched],angles[switched])
        all_arrays[name] = (times-start, delta, angles, gt)
        if name == 'marker':
            report['marker_by_gt_height'] = []
            for low, high in [(-1,.4),(.4,1.),(1.,1.7),(1.7,2.4)]:
                mask = (gt[:,2,3]>=low)&(gt[:,2,3]<high)
                report['marker_by_gt_height'].append(dict(height_m=[low,high],**summary(delta[mask],angles[mask])))
            # A fixed global rotation/translation fit tests a geometric bias; held-out
            # temporal blocks prevent reporting the training residual as validation.
            train = (np.floor((times-start)/2).astype(int)%2)==0
            airborne = gt[:,2,3]>.5
            if train.sum()>10 and (~train).sum()>10:
                a,b = estimate[train,:3,3],gt[train,:3,3]
                R,_ = Rotation.align_vectors(b-b.mean(0),a-a.mean(0))
                shift = b.mean(0)-R.apply(a.mean(0))
                adjusted = R.apply(estimate[:,:3,3])+shift-gt[:,:3,3]
                report['diagnostic_position_rigid_fit'] = {'rotation_xyz_deg':R.as_euler('xyz',degrees=True).tolist(),
                    'rotation_angle_deg':float(np.rad2deg(R.magnitude())), 'translation_m':shift.tolist(),
                    'heldout_original_m':quantile(np.linalg.norm(delta[~train],axis=1)),
                    'heldout_adjusted_m':quantile(np.linalg.norm(adjusted[~train],axis=1)),
                    'position_excitation_singular_values':np.linalg.svd(a-a.mean(0),compute_uv=False).tolist(),
                    'scope':'diagnostic only: position fit confounds pad, camera and rigid-body frame errors'}
    if samples['relative'] and len(samples['gt'])>=2:
        times=np.array([s[0] for s in samples['relative']]); rel=np.array([s[1] for s in samples['relative']])
        gt,ok=interpolate(samples['gt'],times)
        implied=gt[ok]@np.linalg.inv(rel[ok])
        # Test a replacement pad registration with X and all recorded PnP poses
        # unchanged. Learn only from alternating airborne 2-second blocks.
        # This isolates registration error; it does not identify its physical cause.
        airborne=(gt[ok,2,3]>=1.7)&(gt[ok,2,3]<2.4)
        train=(np.floor((times[ok]-start)/2).astype(int)%2)==0
        learn=airborne&train
        test=airborne&~train
        if learn.sum()>20 and test.sum()>20:
            Y=np.eye(4)
            Y[:3,:3]=Rotation.from_matrix(implied[learn,:3,:3]).mean().as_matrix()
            Y[:3,3]=np.median(implied[learn,:3,3],axis=0)
            prediction=Y@rel[ok]
            delta=prediction[:,:3,3]-gt[ok,:3,3]
            angles=np.rad2deg(Rotation.from_matrix(np.swapaxes(gt[ok,:3,:3],1,2)@prediction[:,:3,:3]).magnitude())
            report['diagnostic_airborne_registration']={'train_count':int(learn.sum()),
                'heldout':summary(delta[test],angles[test]),'global_pad_matrix':Y.tolist(),
                'scope':'offline diagnostic; same mount and PnP; ground and airborne registration cannot be assumed interchangeable'}
            if samples['pad'] and all(np.allclose(s[1],samples['pad'][0][1],atol=1e-8) for s in samples['pad']):
                baseline=samples['pad'][0][1]@rel[ok]
                original_delta=baseline[:,:3,3]-gt[ok,:3,3]
                original_angles=np.rad2deg(Rotation.from_matrix(np.swapaxes(gt[ok,:3,:3],1,2)@baseline[:,:3,:3]).magnitude())
                report['diagnostic_airborne_registration']['same_heldout_original']=summary(original_delta[test],original_angles[test])
                # User-confirmed physical constraint: the pad is flat in the
                # OptiTrack world frame. Preserve deployed translation/yaw;
                # remove only its spurious roll/pitch, with no GT refit.
                level=samples['pad'][0][1].copy()
                yaw=np.arctan2(level[1,0],level[0,0])
                level[:3,:3]=Rotation.from_euler('z',yaw).as_matrix()
                projected=level@rel[ok]
                pd=projected[:,:3,3]-gt[ok,:3,3]
                pa=np.rad2deg(Rotation.from_matrix(np.swapaxes(gt[ok,:3,:3],1,2)@projected[:,:3,:3]).magnitude())
                report['diagnostic_level_pad_projection']={
                    'constraint':'user confirms pad roll=pitch=0 in OptiTrack world; yaw unrestricted',
                    'method':'retain deployed translation and yaw; set roll/pitch=0; no GT fitting',
                    'same_airborne_heldout':summary(pd[test],pa[test]),
                    'all_matched_samples':summary(pd,pa),
                    'global_pad_matrix':level.tolist(),
                    'scope':'offline only; does not correct or identify underlying camera/GT/PnP attitude bias'}
        report['implied_pad_by_height']=[]
        for low,high in [(-1,.4),(.4,1.),(1.,1.7),(1.7,2.4)]:
            mask=(gt[ok,2,3]>=low)&(gt[ok,2,3]<high)
            if not mask.any(): continue
            report['implied_pad_by_height'].append({'height_m':[low,high],'count':int(mask.sum()),
                'median_xyz_m':np.median(implied[mask,:3,3],axis=0).tolist(),
                'mean_rotation_xyz_deg':Rotation.from_matrix(implied[mask,:3,:3]).mean().as_euler('xyz',degrees=True).tolist()})
        if ids:
            receipt=np.array([s[2] for s in samples['relative']])[ok]
            id_times=np.array([s[0] for s in ids])
            ix=np.searchsorted(id_times,receipt,side='right')-1
            valid_id=(ix>=0)&(receipt-id_times[ix.clip(0)]<.05)
            group=np.array([str(ids[j][1]) if valid else 'unknown' for j,valid in zip(ix,valid_id)])
            report['implied_pad_by_height_and_inliers']=[]
            for key in sorted(set(group)-{'unknown'}):
                for low,high in [(-1,.4),(1.7,2.4)]:
                    mask=(group==key)&(gt[ok,2,3]>=low)&(gt[ok,2,3]<high)
                    if not mask.any():continue
                    report['implied_pad_by_height_and_inliers'].append({'inliers':key,'height_m':[low,high],
                        'count':int(mask.sum()),'median_xyz_m':np.median(implied[mask,:3,3],axis=0).tolist(),
                        'mean_rotation_xyz_deg':Rotation.from_matrix(implied[mask,:3,:3]).mean().as_euler('xyz',degrees=True).tolist(),
                        'association':'latest ID receipt within 50ms; ID message has no header'})
    if len(gyro)>10 and len(samples['gt'])>10:
        t=np.array([s[0] for s in samples['gt']]); mats=np.array([s[1] for s in samples['gt']]);dt=np.diff(t)
        good=(dt>.002)&(dt<.04)
        steps=Rotation.from_matrix(np.swapaxes(mats[:-1,:3,:3],1,2)@mats[1:,:3,:3]).as_rotvec()
        report['gt_attitude_jumps']={'count_over_2deg':int((np.linalg.norm(steps,axis=1)>np.deg2rad(2)).sum()),
            'count_over_5deg':int((np.linalg.norm(steps,axis=1)>np.deg2rad(5)).sum()),
            'max_step_deg':float(np.rad2deg(np.linalg.norm(steps,axis=1).max())),
            'stamp_interval_s_p0_p50_p99':np.percentile(dt,[0,50,99]).tolist()}
        # VRPN receipt stamps can arrive in bursts (<0.1 ms); dividing each
        # increment by those intervals creates spurious angular velocities.
        # Compare 100ms increments on a uniform grid, away from disarm impacts.
        grid=np.arange(t[0],t[-1]-.1,.02)
        early,ok1=interpolate(samples['gt'],grid)
        late,ok2=interpolate(samples['gt'],grid+.1)
        omega=Rotation.from_matrix(np.swapaxes(early[:,:3,:3],1,2)@late[:,:3,:3]).as_rotvec()/.1
        good=ok1&ok2&(early[:,2,3]>.5)&(late[:,2,3]>.5)&(np.linalg.norm(omega,axis=1)<1.)
        tm=grid+.05
        it=np.array([s[0] for s in gyro]);iw=np.array([s[1] for s in gyro]); good&=(tm>=it[0])&(tm<=it[-1])
        measured=np.column_stack([np.interp(tm,it,iw[:,i]) for i in range(3)])[good]; observed=omega[good]
        centered=observed-observed.mean(0); target=measured-measured.mean(0)
        R,_=Rotation.align_vectors(target,centered)
        report['diagnostic_gt_gyro_axes_fit']={'count':int(good.sum()),'rotation_gt_to_imu_xyz_deg':R.as_euler('xyz',degrees=True).tolist(),
            'rotation_angle_deg':float(np.rad2deg(R.magnitude())),
            'rms_before_rad_s':float(np.sqrt(np.mean(np.sum((target-centered)**2,axis=1)))),
            'rms_after_rad_s':float(np.sqrt(np.mean(np.sum((target-R.apply(centered))**2,axis=1)))),
            'excitation_singular_values':np.linalg.svd(centered,compute_uv=False).tolist(),
            'scope':'approximate 100ms increments, airborne only, no time-offset fit; not an extrinsic calibration'}
    (output/(path.stem+'.json')).write_text(json.dumps(report,indent=2)+'\n')
    return report, all_arrays

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags',default='/home/ml/drone-data/shared/results/webcam-recordings/flight_2026-10-15*/*.bag')
    parser.add_argument('--output',type=Path,default=Path('data/results/alignment_20261016'))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    reports=[]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    paths=sorted(glob.glob(args.bags));fig,axs=plt.subplots(len(paths),2,figsize=(13,2.4*len(paths)),squeeze=False)
    for row,path in enumerate(paths):
        report,arrays=audit(Path(path),args.output);reports.append(report)
        for name,(times,delta,angle,gt) in arrays.items():
            if name=='vision':continue
            axs[row,0].plot(times,np.linalg.norm(delta,axis=1),label=name,linewidth=.8)
            if name=='marker': axs[row,1].scatter(gt[:,2,3],delta[:,0],s=1,label='dx');axs[row,1].scatter(gt[:,2,3],delta[:,1],s=1,label='dy')
        warn=report.get('runtime',{}).get('parameters',{}).get('flight_safety_diagnosis',{}).get('consistency',{}).get('warn_m',.2)
        axs[row,0].axhline(warn,color='r',linestyle='--',linewidth=.6,label='safety WARN')
        for event in report['events']:
            if event['kind']=='source' and event['source']=='marker':axs[row,0].axvline(event['s'],color='green',linewidth=.6)
        axs[row,0].set(title=Path(path).stem,xlabel='bag time (s)',ylabel='position difference (m)');axs[row,0].legend(fontsize=6)
        axs[row,1].set(xlabel='GT body altitude (m)',ylabel='marker - GT (m)');axs[row,1].legend(fontsize=6)
        print(Path(path).stem, 'marker',report.get('marker_vs_gt',{}).get('position_m_p50_p95_max'), 'images',[(k,v['count']) for k,v in report['topics'].items() if v['type'] in ('sensor_msgs/Image','sensor_msgs/CompressedImage')],flush=True)
    fig.tight_layout();fig.savefig(args.output/'pose_disagreement.png',dpi=160);plt.close(fig)
    (args.output/'audit.json').write_text(json.dumps(reports,indent=2)+'\n')

if __name__=='__main__':main()

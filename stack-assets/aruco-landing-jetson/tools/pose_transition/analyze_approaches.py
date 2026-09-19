#!/usr/bin/env python3
"""Three real reacquisitions: audit ROS continuity and PX4 resets/innovations."""
import argparse
import json
from pathlib import Path
import re
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation,Slerp
from pyulog import ULog


def input_data(bag):
    mocap=[];marker=[]
    with rosbag.Bag(str(bag)) as b:
        t0=b.get_start_time()
        for topic,m,r in b.read_messages(topics=['/vrpn_client_node/pure/pose','/landing/vision_pose_marker']):
            p=m.pose;row=[m.header.stamp.to_sec()-t0,p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w,r.to_sec()-t0]
            (mocap if '/pure/' in topic else marker).append(row)
    mocap,marker=np.array(mocap),np.array(marker)
    gaps=np.where(np.diff(marker[:,8])>=1.)[0]
    entries=[float(marker[i+1,8]) for i in gaps]
    return mocap,marker,entries


def analyze(folder):
    data=json.loads((folder/'observations.json').read_text());start=data['start_wall'];duration=data['duration_s']
    mocap,marker,entries=input_data(data['bag'])
    def truth(ts):return np.stack([np.interp(ts,mocap[:,0],mocap[:,j])for j in (1,2,3)],axis=-1)
    events=[];seen=set()
    for _,status in data['status']:
        event=status.get('last_switch')
        if event and event['time'] not in seen:
            seen.add(event['time']);events.append(dict(event,elapsed_s=event['time']-start))
    local=np.array(data['local']);out=np.array(data['output'])
    local_rel=local[:,0]-start;out_rel=out[:,0]-start
    def metrics(a,lo,hi):
        k=np.flatnonzero((a[:,0]-start>=lo)&(a[:,0]-start<=hi))
        if len(k)<2:return {'samples':len(k)}
        b=a[k];delta=np.diff(b[:,2:5],axis=0)
        # Use original measurement stamps for motion compensation; no spatial fit.
        expected_delta=np.diff(truth(b[:,1]-start),axis=0)
        error=b[:,2:5]-truth(b[:,1]-start)
        return {'samples':len(b),'max_receipt_gap_s':float(np.diff(b[:,0]).max()),
                'max_position_step_m':float(np.linalg.norm(delta,axis=1).max()),
                'max_motion_compensated_step_m':float(np.linalg.norm(delta-expected_delta,axis=1).max()),
                'position_error_rms_m':float(np.sqrt(np.mean(np.sum(error**2,axis=1)))),
                'position_error_max_m':float(np.linalg.norm(error,axis=1).max())}
    result={'bag':data['bag'],'duration_s':duration,'marker_hold_s':data['marker_hold_s'],
            'fallback_timeout_s':data['fallback_timeout_s'],'recorded_imu':data['recorded_imu'],
            'imu_yaw_deg':data.get('imu_yaw_deg',0.),
            'optitrack_only':data['optitrack_only'],'warmup_s':data.get('warmup_s'),
            'entries_s':entries,'events':events,'approaches':[],
            'local_entire_replay':metrics(local,0,duration),
            'one_vision_publisher':dict(data['publishers']).get('/mavros/vision_pose/pose')==['/landing_vision_pose_adapter'],
            'output_stamps_strictly_increasing':bool((np.diff(out[:,1])>0).all())}
    for index,entry in enumerate(entries):
        end=entries[index+1] if index+1<len(entries) else duration
        candidates=[e for e in events if e['source']=='marker' and entry-.1<=e['elapsed_s']<end]
        event=candidates[0] if candidates else None
        switch=event['elapsed_s'] if event else entry+1.
        record={'index':index+1,'entry_s':entry,'switch_s':None if not event else switch,
                'qualification_delay_s':None if not event else switch-entry,
                'switch_check':None if not event else event['check'],
                'local_near_switch':metrics(local,switch-.5,switch+1.),
                'local_after_entry':metrics(local,entry,entry+3.),
                'vision_near_switch':metrics(out,switch-.5,switch+1.)}
        k=int(np.searchsorted(out[:,0],start+switch))
        if event and 0<k<len(out):
            record['vision_handover_step_m']=float(np.linalg.norm(out[k,2:5]-out[k-1,2:5]))
            record['vision_handover_measurement_gap_s']=float(out[k,1]-out[k-1,1])
        result['approaches'].append(record)
    result['fallbacks']=[]
    for event in events:
        if event['source']=='optitrack':
            t=event['elapsed_s'];result['fallbacks'].append({'elapsed_s':t,'check':event['check'],
                'local_near_fallback':metrics(local,t-.6,t+.5),'vision_near_fallback':metrics(out,t-.6,t+.5)})
    ul=ULog(str(next((folder/'px4/log').rglob('*.ulg'))),['estimator_status','vehicle_local_position','vehicle_status'])
    ekf=ul.get_dataset('estimator_status').data;lp=ul.get_dataset('vehicle_local_position').data
    # Probe calls associate PX4 boot time with bag elapsed time (millisecond precision).
    offsets=[]
    for probe in data['px4_probes']:
        match=re.search(r'\btimestamp: (\d+)',probe['estimator_status'])
        if match:offsets.append(probe['elapsed']-int(match.group(1))/1e6)
    boot_offset=float(np.median(offsets))
    et=ekf['timestamp']/1e6+boot_offset;lt=lp['timestamp']/1e6+boot_offset
    k=(et>=0)&(et<=duration);j=(lt>=0)&(lt<=duration)
    ev=(ekf['control_mode_flags']&(7<<12))==(7<<12)
    changes=[]
    for name in ('xy_reset_counter','z_reset_counter','vxy_reset_counter'):
        indices=np.where(np.diff(lp[name].astype(int))!=0)[0]+1
        changes += [{'kind':name,'elapsed_s':float(lt[i]),'counter':int(lp[name][i])}for i in indices if 0<=lt[i]<=duration]
    result['px4']={'ev_position_yaw_height_fraction':float(ev[k].mean()),
                   'filter_fault_flags_max':int(ekf['filter_fault_flags'][k].max()),
                   'innovation_rejection_flags_values':np.unique(ekf['innovation_check_flags'][k]).tolist(),
                   'xy_valid_fraction':float(lp['xy_valid'][j].mean()),'z_valid_fraction':float(lp['z_valid'][j].mean()),
                   'resets_during_replay':changes,
                   'ever_armed':bool((ul.get_dataset('vehicle_status').data['arming_state']==2).any())}
    result['review_thresholds']={'local_gap_s':.1,'motion_compensated_position_step_m':.05,
        'note':'Diagnostic review thresholds for this replay, not flight certification limits.'}
    windows=[a['local_near_switch'] for a in result['approaches']]+[f['local_near_fallback']for f in result['fallbacks']]
    result['checks']={'all_three_reacquisitions_switched':(len(entries)==3 and all(a['switch_s'] is not None for a in result['approaches'])) if not data['optitrack_only'] else None,
        'local_gap_under_100ms_near_transitions':all(w.get('max_receipt_gap_s',1)<.1 for w in windows),
        'extra_position_step_under_5cm_near_transitions':all(w.get('max_motion_compensated_step_m',1)<.05 for w in windows),
        'no_px4_position_resets':not any(c['kind'] in ('xy_reset_counter','z_reset_counter')for c in changes),
        'no_ekf_filter_fault':result['px4']['filter_fault_flags_max']==0,
        'no_ekf_innovation_rejection':result['px4']['innovation_rejection_flags_values']==[0],
        'local_position_always_valid':result['px4']['xy_valid_fraction']==1 and result['px4']['z_valid_fraction']==1}
    (folder/'approach-summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result,data,mocap,marker

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args();analyze(a.folder)

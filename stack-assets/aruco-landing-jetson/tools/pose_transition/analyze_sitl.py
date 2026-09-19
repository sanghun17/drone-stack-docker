#!/usr/bin/env python3
"""Audit ROS source continuity and PX4 external-vision fusion from one replay."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog


def analyze(directory):
    d=json.loads((directory/'observations.json').read_text())
    output=np.array(d['output']);start=d['start_wall'];duration=d['duration_s']
    switches=[s['last_switch'] for _,s in d['status'] if s['last_switch']]
    switch=switches[0] if switches else None
    counts={phase:int(np.sum((output[:,0]-start>lo)&(output[:,0]-start<hi))) for phase,lo,hi in [
        ('after_mocap_cutoff',duration*.65+.3,duration*.8-.1),
        ('inside_marker_dropout',duration*.8+.3,duration*.9-.1),
        ('after_marker_recovery',duration*.9+.3,duration) ]} if len(output) else {}
    publishers=dict(d['publishers']).get('/mavros/vision_pose/pose',[])
    ulg=next((directory/'px4/log').rglob('*.ulg'))
    u=ULog(str(ulg),['estimator_status','vehicle_status','vehicle_visual_odometry'])
    ekf=u.get_dataset('estimator_status').data
    flags=ekf['control_mode_flags'];active=(flags&(7<<12))==(7<<12)
    faults=ekf['filter_fault_flags'];innov=ekf['innovation_check_flags']
    status=u.get_dataset('vehicle_status').data
    fusion={'samples':len(flags),'position_yaw_height_active_samples':int(active.sum()),
            'fault_flags_max':int(faults.max()),'innovation_rejection_flags_max_during_fusion':int(innov[active].max()) if active.any() else None,
            'vision_odometry_logged_samples':len(u.get_dataset('vehicle_visual_odometry').data['timestamp']),
            'ever_armed':bool((status['arming_state']==2).any())}
    checks={'switched_to_marker':bool(switch and switch['source']=='marker'),
            'one_output_publisher':publishers==['/landing_vision_pose_adapter'],
            'strictly_increasing_output_stamps':bool(len(output)>1 and (np.diff(output[:,1])>0).all()),
            'marker_output_after_mocap_loss':counts.get('after_mocap_cutoff',0)>0,
            'no_output_during_marker_loss':counts.get('inside_marker_dropout',-1)==0,
            'output_recovers_with_marker':counts.get('after_marker_recovery',0)>0,
            'px4_fuses_external_position_yaw_height':fusion['position_yaw_height_active_samples']>5,
            'px4_no_filter_fault':fusion['fault_flags_max']==0,
            'stayed_disarmed':not fusion['ever_armed']}
    summary={'passed':all(checks.values()),'checks':checks,'bag':d['bag'],'px4_version':d['px4_version'],
             'switch':switch,'output_messages':len(output),'phase_counts':counts,'fusion':fusion,
             'scope':'Open-loop pose-source replay with independent jMAVSim IMU. Not a closed-loop flight/landing validation; dynamic recorded motion can conflict with simulated IMU.'}
    if switch and len(output):
        index=int(np.searchsorted(output[:,0],switch['time']))
        if 0<index<len(output):
            summary['output_step_at_switch_m']=float(np.linalg.norm(output[index,2:5]-output[index-1,2:5]))
            summary['output_gap_at_switch_s']=float(output[index,1]-output[index-1,1])
    (directory/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path)
    raise SystemExit(0 if analyze(p.parse_args().directory)['passed'] else 1)

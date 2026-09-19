#!/usr/bin/env python3
"""Offline audit of a manual-flight bag. No ROS master or vehicle connection."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def stats(values):
    a=np.asarray(values,float)
    if not len(a): return None
    return dict(n=len(a),mean=float(np.mean(a)),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),max=float(np.max(a)))


def analyze(path, out):
    arrays=defaultdict(list); messages=defaultdict(list)
    with rosbag.Bag(str(path)) as bag:
        t0=bag.get_start_time(); duration=bag.get_end_time()-t0
        for topic,m,t in bag.read_messages():
            rt=t.to_sec()-t0
            if m._type in ('geometry_msgs/PoseStamped','geometry_msgs/PoseWithCovarianceStamped'):
                p=m.pose if m._type.endswith('/PoseStamped') else m.pose.pose
                arrays[topic].append([rt,m.header.stamp.to_sec()-t0,p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
            elif m._type=='geometry_msgs/TwistStamped':
                v=m.twist.linear;arrays[topic].append([rt,m.header.stamp.to_sec()-t0,v.x,v.y,v.z])
            elif m._type=='std_msgs/String':
                try:d=json.loads(m.data)
                except ValueError:d=m.data
                messages[topic].append((rt,d))
            elif m._type=='std_msgs/Bool':messages[topic].append((rt,m.data))
            elif topic=='/mavros/state':messages[topic].append((rt,dict(armed=m.armed,mode=m.mode,connected=m.connected)))
            elif topic=='/mavros/estimator_status':messages[topic].append((rt,{k:getattr(m,k) for k in m.__slots__ if k!='header'}))
            elif topic=='/rosout_agg':messages[topic].append((rt,dict(node=m.name,level=m.level,text=m.msg)))
            elif topic=='/mavros/statustext/recv':messages[topic].append((rt,dict(severity=m.severity,text=m.text)))
    arrays={k:np.asarray(v) for k,v in arrays.items()}
    mocap=arrays['/vrpn_client_node/pure/pose']; local=arrays['/mavros/local_position/pose']; vision=arrays['/mavros/vision_pose/pose']
    states=messages['/mavros/state']; armed=[]; start=None; transitions=[];prev=None
    for t,m in states:
        if m!=prev:transitions.append(dict(time_s=t,**m));prev=m
        if m['armed'] and start is None:start=t
        if not m['armed'] and start is not None:armed.append([start,t]);start=None
    if start is not None:armed.append([start,duration])
    lo,hi=max(armed,key=lambda x:x[1]-x[0])
    def window(a):return a[(a[:,0]>=lo)&(a[:,0]<=hi)]
    def truth(times):return np.column_stack([np.interp(times,mocap[:,1],mocap[:,j]) for j in (2,3,4)])
    def continuity(a):
        gaps=np.diff(a[:,0]);steps=np.linalg.norm(np.diff(a[:,2:5],axis=0),axis=1)
        residual=np.diff(a[:,2:5],axis=0)-np.diff(truth(a[:,1]),axis=0)
        norms=np.linalg.norm(residual,axis=1)
        error=a[:,2:5]-truth(a[:,1]);center=np.median(error,axis=0)
        return dict(samples=len(a),rate_hz=(len(a)-1)/(a[-1,0]-a[0,0]),receipt_gap_s=stats(gaps),
            position_step_m=stats(steps),motion_compensated_step_m=stats(norms),
            median_offset_vs_optitrack_m=center.tolist(),position_error_m=stats(np.linalg.norm(error,axis=1)),
            centered_position_error_m=stats(np.linalg.norm(error-center,axis=1)),
            largest_compensated_steps=[dict(time_s=float(a[i+1,0]),m=float(norms[i]),receipt_gap_s=float(gaps[i])) for i in np.argsort(norms)[-5:][::-1]])
    pad=arrays['/landing/pad_pose_global'][0]
    R_GP=Rotation.from_quat(pad[5:9]).as_matrix();p_GP=pad[2:5]
    calibration=json.loads(path.with_suffix('.calibration.json').read_text())
    extrinsic=yaml.safe_load(calibration['calibration/20260919/base_link_to_see3cam_optical_frame.yaml'])
    lever=np.asarray(extrinsic['matrix_row_major']).reshape(4,4)[:3,3]
    unique=np.r_[True,np.diff(mocap[:,1])>0]
    orientation=Slerp(mocap[unique,1],Rotation.from_quat(mocap[unique,5:9]))
    def truth_camera_pad(times):
        ts=np.clip(times,mocap[0,1],mocap[-1,1])
        global_camera=truth(ts)+orientation(ts).apply(lever)
        return (global_camera-p_GP) @ R_GP
    source_events=[];seen=set()
    for t,s in messages['/landing/pose_transition/status']:
        e=s['last_switch']
        if e and e['time'] not in seen:
            seen.add(e['time']);source_events.append(dict(time_s=e['time']-t0,source=e['source'],check=e['check']))
    mocap_map={round(row[1],6):row[2:] for row in mocap}; matched=0;different=0;unmatched=0
    for row in vision:
        ref=mocap_map.get(round(row[1],6))
        if ref is None:unmatched+=1
        elif np.array_equal(ref,row[2:]):matched+=1
        else:different+=1
    flags={k:dict(Counter(str(d[k]) for t,d in messages['/mavros/estimator_status'])) for k in messages['/mavros/estimator_status'][0][1]}
    audit_errors=Counter(e for t,d in messages['/landing/manual_audit/status'] for e in d['errors'])
    def intervals(records,predicate):
        result=[];begin=None
        for t,m in records:
            if predicate(m) and begin is None:begin=t
            elif not predicate(m) and begin is not None:result.append([begin,t]);begin=None
        if begin is not None:result.append([begin,duration])
        return result
    ctrl={}; plot_data={}
    for source in ['optitrack','transition']:
        root='/landing/shadow/'+source; cmd=arrays[root+'/cmd_vel_pad']; camera=arrays[root+'/camera_pose_pad']
        selected=np.searchsorted(camera[:,0],cmd[:,0],side='right')-1
        valid=selected>=0; c=cmd[valid]; p=camera[selected[valid]]
        delta=c[:,0]-p[:,0];rad=np.linalg.norm(p[:,2:4],axis=1); speed=np.linalg.norm(c[:,2:4],axis=1)
        active=(c[:,4]<-.01)&(delta<.1); airborne=(c[:,0]>=lo)&(c[:,0]<=hi)
        measure=active&airborne&(rad>.05)&(speed>.01)
        dot=np.sum(c[:,2:4]*(-p[:,2:4]),axis=1)
        # Reproduce configured PD derivative and latency prediction from recorded
        # camera poses; transport callback ordering can differ by one sample.
        deriv=np.zeros((len(camera),2));previous=None;previous_stamp=None;d=np.zeros(2)
        for i,row in enumerate(camera):
            error=-row[2:4]; stamp=row[1]
            if previous is not None and stamp>previous_stamp:
                dt=stamp-previous_stamp
                if dt<=.25:
                    alpha=dt/(.10+dt);d=(1-alpha)*d+alpha*(error-previous)/dt
                    if np.linalg.norm(d)>2:d*=2/np.linalg.norm(d)
            deriv[i]=d;previous=error;previous_stamp=stamp
        dv=deriv[selected[valid]];age=np.clip(c[:,1]-p[:,1],0,.2)
        expected=.8*(-p[:,2:4]+dv*age[:,None])+.15*dv
        mag=np.linalg.norm(expected,axis=1);expected*=np.minimum(1,2/np.maximum(mag,1e-12))[:,None]
        model_error=np.linalg.norm(c[:,2:4]-expected,axis=1)
        away=measure&(dot<0)
        truth_p=truth_camera_pad(c[:,1]);true_rad=np.linalg.norm(truth_p[:,:2],axis=1)
        true_dot=np.sum(c[:,2:4]*(-truth_p[:,:2]),axis=1)
        true_measure=active&airborne&(true_rad>.05)&(speed>.01)
        source_times=[e['time_s'] for e in source_events]
        source_states=['optitrack']+[e['source'] for e in source_events]
        marker_source=np.array([source_states[i]=='marker' for i in np.searchsorted(source_times,c[:,0],side='right')])
        true_away=true_measure&(true_dot<0)
        ctrl[source]=dict(command_continuity=dict(rate_hz=(len(cmd)-1)/(cmd[-1,0]-cmd[0,0]),receipt_gap_s=stats(np.diff(cmd[:,0]))),armed_command_samples=int(airborne.sum()),
            independent_optitrack_direction=dict(samples=int(true_measure.sum()),toward_center_fraction=float(np.mean(true_dot[true_measure]>0)),
                away_samples=int(true_away.sum()),away_radius_m=stats(true_rad[true_away]),
                marker_selected_samples=int((true_measure&marker_source).sum()),
                marker_selected_toward_fraction=float(np.mean(true_dot[true_measure&marker_source]>0)) if (true_measure&marker_source).any() else None,
                camera_position_error_m=stats(np.linalg.norm(p[:,2:5]-truth_p,axis=1)[active&airborne]),
                away_time_s=[float(c[i,0]) for i in np.flatnonzero(true_away)[::max(1,int(true_away.sum())//6)][:7]]),
            horizontal_speed_mps=stats(speed[airborne]),vertical_command_values=np.unique(c[airborne,4]).tolist(),
            active_samples=int((active&airborne).sum()),direction_test_samples=int(measure.sum()),
            toward_center_fraction=float(np.mean(dot[measure]>0)) if measure.any() else None,
            away_from_center_samples=int(away.sum()),away_radius_m=stats(rad[away]),
            pd_reconstruction_error_mps=stats(model_error[active&airborne]),
            state_intervals={state:intervals(messages[root+'/controller/state'],lambda v:v==state) for state in sorted(set(v for _,v in messages[root+'/controller/state']))},
            invalid_pose_intervals=intervals(messages[root+'/valid'],lambda v:not v),
            away_examples=[dict(time_s=float(c[i,0]),xy_m=p[i,2:4].tolist(),cmd_xy_mps=c[i,2:4].tolist(),radius_m=float(rad[i]),model_error_mps=float(model_error[i])) for i in np.flatnonzero(away)[::max(1,int(away.sum())//5)][:6]])
        plot_data[source]=(c,p,rad,dot,measure)
    est=messages['/landing/estimator/status']; warninglogs=[dict(time_s=t,**m) for t,m in messages['/rosout_agg'] if m['level']>=4]
    marker=arrays['/landing/vision_pose_marker'];markererror=marker[:,2:5]-truth(marker[:,1])
    candidate=arrays['/landing/vision_pose_selected']
    source_states=['optitrack']+[e['source'] for e in source_events]
    marker_selected=np.array([source_states[i]=='marker' for i in np.searchsorted([e['time_s'] for e in source_events],candidate[:,0],side='right')])
    selected_error=np.linalg.norm(candidate[:,2:5]-truth(candidate[:,1]),axis=1)
    for event in source_events:
        i=np.searchsorted(candidate[:,0],event['time_s'])
        if 0<i<len(candidate):
            event['output_step_m']=float(np.linalg.norm(candidate[i,2:5]-candidate[i-1,2:5]))
            event['output_receipt_gap_s']=float(candidate[i,0]-candidate[i-1,0])
        near=candidate[(candidate[:,0]>event['time_s']-.1)&(candidate[:,0]<event['time_s']+.3)]
        event['max_step_in_nearby_window_m']=float(np.max(np.linalg.norm(np.diff(near[:,2:5],axis=0),axis=1))) if len(near)>1 else None
    result=dict(candidate_marker_selected_position_error_m=stats(selected_error[marker_selected]),bag=str(path),duration_s=duration,armed_intervals_s=armed,main_flight_interval_s=[lo,hi],state_changes=transitions,
        actual_vision_identity=dict(exact_matches=matched,different=different,unmatched_boundary=unmatched),
        local_entire=continuity(local),local_main_flight=continuity(window(local)),vision_main_flight=continuity(window(vision)),
        estimator_flags=flags,source_events=source_events,controller=ctrl,
        marker_position_error_m=stats(np.linalg.norm(markererror,axis=1)),
        audit_errors=dict(audit_errors),logs=warninglogs,
        camera_processing_hz=stats([d['processing_hz'] for t,d in est if lo<=t<=hi]),
        camera_zero_processing_intervals=intervals(est,lambda d:d['processing_hz']==0),
        limits=['Real EKF2 used OptiTrack only; candidate transitions did not drive PX4.',
                'MAVROS flags are sampled at ~1 Hz; ULog required to rule out every internal EKF reset/rejection.',
                'Command directions are pad-frame previews, not executed OFFBOARD setpoints.'])
    out.mkdir(parents=True,exist_ok=True)
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(5,1,figsize=(13,15),sharex=True)
    for ax in axes:
        ax.axvspan(lo,hi,color='green',alpha=.05)
        for e in source_events:ax.axvline(e['time_s'],color='orange' if e['source']=='marker' else 'gray',alpha=.5,linestyle='--')
        ax.grid(alpha=.25)
    for j,label in enumerate(['x','y','z']):
        axes[0].plot(mocap[:,0],mocap[:,j+2],label='OptiTrack '+label,alpha=.7)
        axes[0].plot(local[:,0],local[:,j+2],ls='--',label='PX4 '+label,alpha=.7)
    axes[0].set_ylabel('Global position (m)');axes[0].legend(ncol=3)
    axes[1].plot(local[1:,0],np.diff(local[:,0])*1000,label='local pose receipt gap')
    axes[1].set_ylabel('Gap (ms)');axes[1].legend()
    for source,(c,p,rad,dot,measure) in plot_data.items():
        axes[2].plot(c[:,0],rad,label=source+' camera XY distance')
        axes[3].plot(c[:,0],c[:,4],label=source+' vz',alpha=.7)
        independent=truth_camera_pad(c[:,1]);independent_rad=np.linalg.norm(independent[:,:2],axis=1)
        axes[4].plot(c[:,0],np.sum(c[:,2:4]*(-independent[:,:2]),axis=1)/np.maximum(independent_rad,1e-9),label=source+' inward speed (OptiTrack reference)',alpha=.7)
    axes[2].set_ylabel('Pad distance (m)');axes[3].set_ylabel('Vertical command (m/s)');axes[4].set_ylabel('Inward XY command (m/s)')
    for ax in axes[2:]:ax.legend()
    axes[4].axhline(0,color='black',lw=.7);axes[4].set_xlabel('Seconds from manual bag start (22:11:06 KST)')
    fig.suptitle('Manual flight: actual OptiTrack EKF input and observation-only landing commands')
    fig.tight_layout();fig.savefig(out/'overview.png',dpi=150);fig.savefig(out/'overview.pdf');plt.close(fig)
    fig,axs=plt.subplots(3,1,figsize=(13,10),sharex=True)
    axs[0].plot(candidate[:,0],selected_error,label='Selected candidate vs OptiTrack')
    axs[0].axhline(.15,ls='--',c='red',label='Entry gate 0.15 m (not enforced after entry)')
    axs[0].set_ylabel('Position disagreement (m)');axs[0].legend()
    for source,(c,p,rad,dot,measure) in plot_data.items():
        independent=truth_camera_pad(c[:,1]);r=np.linalg.norm(independent[:,:2],axis=1)
        inward=np.sum(c[:,2:4]*(-independent[:,:2]),axis=1)/np.maximum(r,1e-9)
        axs[1].plot(c[:,0],inward,label=source+' command',alpha=.8)
    axs[1].axhline(0,c='black');axs[1].set_ylabel('True inward XY speed (m/s)');axs[1].legend()
    axs[2].plot([t for t,d in est],[d['processing_hz'] for t,d in est],label='Camera processing Hz')
    axs[2].axhline(60,c='gray',ls='--',label='60 Hz target');axs[2].set_ylabel('Processing Hz');axs[2].legend()
    for ax in axs:
        ax.axvspan(lo,hi,color='green',alpha=.05);ax.grid(alpha=.25)
        for e in source_events:ax.axvline(e['time_s'],color='orange' if e['source']=='marker' else 'gray',ls='--',alpha=.5)
    axs[2].set_xlabel('Seconds from manual bag start');fig.suptitle('Marker candidate diverges after entry; actual PX4 continued on OptiTrack')
    fig.tight_layout();fig.savefig(out/'candidate_diagnostics.png',dpi=150);fig.savefig(out/'candidate_diagnostics.pdf');plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('out',type=Path);a=p.parse_args();analyze(a.bag,a.out)

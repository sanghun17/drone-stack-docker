#!/usr/bin/env python3
"""Deterministic offline execution of current ROS controller/preview/router code.

Only ROS transport, timers and parameters are replaced with in-memory bindings.
No ROS master, MAVROS connection, or vehicle output is created.
"""
import importlib.util
import sys
import json
from collections import Counter
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rosbag
import rospy as real_rospy
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32MultiArray
from scipy.spatial.transform import Rotation as R
import audit_coordinate_frames as audit
from aruco_landing.physical_pad import SessionAlignment

OUT=audit.ROOT/'stack-assets/aruco-landing-jetson/results/manual-flight-20260919/fixed-body-control'
SCRIPTS=audit.ROOT/'ws/aruco-landing/src/aruco_landing/scripts'
clock=SimpleNamespace(now=0.)


class Publisher:
    def __init__(self,kind):self.kind=kind;self.last=None;self.hooks=[]
    def publish(self,message):
        if not hasattr(message,'_type'):message=self.kind(message)
        self.last=message
        for callback in self.hooks:callback(message)


class OfflineROS:
    def __init__(self,params):self.params=params
    def init_node(self,*args,**kw):pass
    def get_param(self,name,default=None):return self.params.get(name.lstrip('~'),default)
    def get_time(self):return clock.now
    def resolve_name(self,name):return name
    def Publisher(self,topic,kind,**kw):return Publisher(kind)
    def Subscriber(self,*args,**kw):pass
    def Service(self,*args,**kw):pass
    def Timer(self,*args,**kw):pass
    def Duration(self,x):return x
    def loginfo(self,*args,**kw):pass
    def logwarn_throttle(self,*args,**kw):pass
    def get_master(self):return SimpleNamespace(getSystemState=lambda:(1,'',([],[],[])))


def load(name,filename,params):
    spec=importlib.util.spec_from_file_location(name,SCRIPTS/filename);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.rospy=OfflineROS(params)
    return module


def message(T,stamp,frame='odom'):
    m=PoseStamped();m.header.stamp=real_rospy.Time.from_sec(stamp);m.header.frame_id=frame
    m.pose.position.x,m.pose.position.y,m.pose.position.z=T[:3,3]
    q=R.from_matrix(T[:3,:3]).as_quat();m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
    return m


def intervals(times,mask,step=1/60):
    ix=np.flatnonzero(mask)
    if not len(ix):return []
    chunks=np.split(ix,np.flatnonzero(np.diff(ix)>1)+1)
    return [[float(times[c[0]]),float(times[c[-1]]+step)]for c in chunks]


def run():
    global OUT
    seed=None
    if '--prealigned' in sys.argv:
        seed=json.loads((OUT/'recomputed_commands.json').read_text())['learned_session_global_pad_matrix']
        OUT=OUT/'prealigned'
    OUT.mkdir(parents=True,exist_ok=True)
    config=audit.ROOT/'stack-assets/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml'
    X=np.array(yaml.safe_load(config.read_text())['matrix_row_major']).reshape(4,4)
    cfg=yaml.safe_load((SCRIPTS.parent/'config/common_experiment.yaml').read_text())
    controllers={}
    for source in ('optitrack','transition'):
        p={**cfg,'enabled':True,'preview_only':True,'command_topic':'/landing/shadow/'+source+'/cmd_vel_pad'}
        controllers[source]=load('controller_'+source,'landing_controller_node.py',p).LandingController()
    preview=load('preview','manual_flight_preview.py',{'extrinsic_file':str(config)}).Preview()
    router=load('router','landing_vision_pose_adapter.py',dict(auto_switch=True,allow_marker_switch=True,auto_fallback_to_optitrack=True,marker_loss_timeout_s=.5)).LandingVisionPoseAdapter()
    router.output.hooks.append(lambda msg:preview.pose(msg,'transition'))
    for source,c in controllers.items():
        preview.pub[source]['body'].hooks.append(c.pose_callback)
        preview.pub[source]['camera'].hooks.append(c.camera_pose_callback)
        preview.pub[source]['valid'].hooks.append(c.visible_callback)
    alignment=SessionAlignment(min_samples=60,min_duration_s=2.,window_size=240,max_translation_std_m=.03,max_rotation_std_deg=2.)
    if seed is not None:
        alignment.ready=True;alignment.transform=np.array(seed);alignment.reference_time=1.
    topics=['/vrpn_client_node/pure/pose','/landing/target_pose_camera','/landing/target_visible','/landing/estimator/inlier_ids','/mavros/state']
    events=[];mocap=[];states=[];recorded=[]
    with rosbag.Bag(str(audit.FLIGHT))as bag:
        epoch=bag.get_start_time();end=bag.get_end_time()
        for topic,m,t in bag.read_messages(topics=topics+['/landing/shadow/optitrack/cmd_vel_pad','/landing/shadow/transition/cmd_vel_pad']):
            if topic.endswith('cmd_vel_pad'):
                v=m.twist;recorded.append((topic,t.to_sec()-epoch,[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]));continue
            events.append((t.to_sec(),0,topic,m))
            if '/pure/' in topic:mocap.append((m.header.stamp.to_sec(),audit.pose(m.pose)))
            if topic=='/mavros/state':states.append((t.to_sec()-epoch,m.armed))
    poses=(np.array([v[0]for v in mocap]),np.array([v[1]for v in mocap]));unique=np.r_[True,np.diff(poses[0])>0];poses=(poses[0][unique],poses[1][unique])
    ticks=epoch+np.arange(int((end-epoch)*60)+1)/60
    truth=audit.interp(poses,np.clip(ticks,poses[0][0],poses[0][-1]))
    events.extend((t,2,'control_tick',i)for i,t in enumerate(ticks))
    events.extend((t,1,'router_tick',None)for t in epoch+np.arange(int((end-epoch)*50)+1)/50)
    events.sort(key=lambda e:(e[0],e[1]))
    rows={s:[] for s in controllers};transitions=[];last_source=router.router.source;last_pad=-np.inf;ready_at=(0. if seed is not None else None)
    for now,_,topic,m in events:
        now=float(now);clock.now=now
        if topic=='/vrpn_client_node/pure/pose':
            alignment.add_mocap(m.header.stamp.to_sec(),audit.pose(m.pose));preview.pose(m,'optitrack');router.pose(m,'optitrack')
        elif topic=='/landing/target_visible':router.quality(m,'visible')
        elif topic=='/landing/estimator/inlier_ids':router.quality(m,'inliers')
        elif topic=='/landing/target_pose_camera':
            stamp=m.header.stamp.to_sec();P_B=np.linalg.inv(audit.pose(m.pose))@np.linalg.inv(X);G_B=alignment.observe(stamp,P_B)
            if G_B is not None:
                if ready_at is None:ready_at=now-epoch
                router.pose(message(G_B,stamp),'marker')
                if now-last_pad>.2:
                    preview.pad_pose(message(alignment.transform,alignment.reference_time));last_pad=now
            preview.aligned(Bool(alignment.ready));router.quality(Bool(alignment.ready),'aligned')
        elif topic=='router_tick':
            router.status(None)
            if router.router.source!=last_source:
                transitions.append({'time_s':now-epoch,**router.last_switch});last_source=router.router.source
        elif topic=='control_tick':
            preview.tick(None)
            for source,c in controllers.items():
                c.update(SimpleNamespace(current_real=real_rospy.Time.from_sec(now)))
                cmd=c.command_publisher.last.twist
                values=[cmd.linear.x,cmd.linear.y,cmd.linear.z,cmd.angular.x,cmd.angular.y,cmd.angular.z]
                if alignment.ready:
                    P_B=np.linalg.inv(alignment.transform)@truth[m];P_C=P_B@X
                    yaw=R.from_matrix(P_B[:3,:3]).as_euler('xyz')[2];yaw_error=np.arctan2(np.sin(-yaw),np.cos(-yaw))
                    ref=[*P_C[:3,3],yaw_error]
                else:ref=[np.nan]*4
                rows[source].append([now-epoch,*values,*ref,int(c.state==c.DESCENDING),int(c.state==c.TOUCHDOWN),int(c.visible),
                    int(router.router.source=='marker'),c.yaw_error_publisher.last.data,
                    now-c.vehicle_pose.header.stamp.to_sec()if c.vehicle_pose else np.nan])
    arms=[];start=None
    for t,armed in states:
        if armed and start is None:start=t
        if not armed and start is not None:arms.append([start,t]);start=None
    lo,hi=max(arms,key=lambda x:x[1]-x[0]);summary={'main_flight_interval_s':[lo,hi],'alignment_ready_at_s':ready_at,'source_transitions':transitions,
        'learned_session_global_pad_matrix':alignment.transform.tolist(),
        'frame_start_assumption':('Retrospective fixed pad transform from cold-start replay, assumed available before recording. Matches prealigned original-session usage but uses later observations for this coordinate reference.' if seed is not None else 'No pre-bag alignment is available; learn from bag start.'),
        'parameters':{k:cfg[k]for k in ('kp_xy','kd_xy','horizontal_speed_limit_mps','descent_speed_mps','yaw_kp','yaw_rate_limit_rad_s','yaw_deadband_deg','horizontal_reference')},
        'method':'Current LandingController, Preview and LandingVisionPoseAdapter methods executed offline with ROS transport replaced; 60 Hz controller/preview and 50 Hz router ticks. Raw image detections are reused from bag. Original receipt order, unchanged configured timestamp correction. Alignment start assumption is specified separately; no changes to node logic.',
        'limits':['Recomputed hypothetical commands, not sent to PX4 and not a closed-loop flight replay.','Timer phase/callback transport jitter not reproduced; extrema are deterministic replay values.','All command axes are pad frame, not body/world/NED.','Full-bag diagnostic includes training data used for mount estimation.'], 'controllers':{}}
    for source,records in rows.items():
        ar=np.array(records);flight=(ar[:,0]>=lo)&(ar[:,0]<=hi);active=flight&(ar[:,11]>0);v=ar[:,1:7]
        axes={}
        for i,key in enumerate(('vx','vy','vz','roll_rate','pitch_rate','yaw_rate')):
            vals=v[flight,i];j=np.flatnonzero(flight)[np.argmax(abs(vals))]
            axes[key]={'min':float(vals.min()),'max':float(vals.max()),'max_abs':float(abs(vals).max()),'max_abs_at_s':float(ar[j,0])}
        radial=np.linalg.norm(ar[:,7:9],axis=1);dot=np.sum(v[:,:2]*-ar[:,7:9],axis=1)
        check=active&(radial>.05)&(np.linalg.norm(v[:,:2],axis=1)>.01)
        yawcheck=active&np.isfinite(ar[:,10])&(abs(ar[:,10])>np.radians(cfg['yaw_deadband_deg']))
        yawcommand=yawcheck&(abs(v[:,5])>1e-6);yawsame=yawcommand&(v[:,5]*ar[:,10]>0)
        off=flight&(ar[:,0]>=ready_at)&(ar[:,13]==0)
        summary['controllers'][source]={'axes':axes,'horizontal_max_mps':float(np.linalg.norm(v[flight,:2],axis=1).max()),'speed_3d_max_mps':float(np.linalg.norm(v[flight,:3],axis=1).max()),
            'active_samples':int(active.sum()),'evaluated_horizontal_samples':int(check.sum()),'inward_fraction':float(np.mean(dot[check]>0))if check.any()else None,
            'outward_intervals_s':intervals(ar[:,0],check&(dot<=0)),
            'yaw_error_abs_deg':audit.stats(np.degrees(abs(ar[yawcheck,10])))if yawcheck.any()else None,
            'yaw_nonzero_samples':int(yawcommand.sum()),'yaw_toward_target_fraction':float(yawsame.sum()/yawcommand.sum())if yawcommand.any()else None,
            'yaw_away_intervals_s':intervals(ar[:,0],yawcommand&~yawsame),
            'yaw_withheld_while_error_exceeds_deadband_samples':int((yawcheck&~yawcommand).sum()),
            'invalid_preview_intervals_s':intervals(ar[:,0],off),'touchdown_zero_samples':int((flight&(ar[:,12]>0)).sum()),
            'configured_xy_norm_limit_mps':cfg['horizontal_speed_limit_mps'],'configured_yaw_rate_limit_rad_s':cfg['yaw_rate_limit_rad_s']}
        np.savez_compressed(OUT/(source+'_commands.npz'),rows=ar,columns=np.array(['time_s','vx','vy','vz','roll_rate','pitch_rate','yaw_rate','true_camera_x','true_camera_y','true_camera_z','true_yaw_error','descending','touchdown','visible','marker_selected','controller_yaw_error','vehicle_pose_age']))
    (OUT/'calibration_used.yaml').write_text(config.read_text())
    (OUT/'controller_parameters_used.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    summary['recorded_original_yaw_rate_max_abs_rad_s']=max(abs(row[2][5])for row in recorded if lo<=row[1]<=hi)
    (OUT/'recomputed_commands.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2),flush=True)

if __name__=='__main__':run()

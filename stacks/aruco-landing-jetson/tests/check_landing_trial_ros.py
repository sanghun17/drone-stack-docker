#!/usr/bin/env python3
"""Real ROS nodes on localhost only; synthetic FCU, no hardware or PX4 transport."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[3]
REPEAT=os.environ.get('TEST_REPEAT_TRIAL')=='1'
CUT=os.environ.get('TEST_TERMINATION')=='1' or REPEAT
AUTO=os.environ.get('TEST_AUTO_PLANNER')=='1' or REPEAT
SPEEDS=os.environ.get('TEST_LANDING_SPEEDS')=='1' or REPEAT
TRANSITION=os.environ.get('TEST_TRANSITION')=='1'
PORT=11358 if TRANSITION else 11357
OUT=ROOT/('flight_logs/aruco-landing/trial-transition-integration' if TRANSITION else 'flight_logs/aruco-landing/trial-regression')


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    with socket.socket()as s:
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('127.0.0.1',PORT))
    env=dict(os.environ,ROS_LOG_DIR=str(OUT/'roslog'),ROS_MASTER_URI='http://127.0.0.1:%s'%PORT,ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME',None)
    env['ROS_PACKAGE_PATH']=':'.join(str(ROOT/p)for p in ['ws/aruco-landing/src/aruco_landing','ws/flight-safety/src/flight_safety'])+':'+env.get('ROS_PACKAGE_PATH','/opt/ros/noetic/share')
    env['PYTHONPATH']=':'.join(str(ROOT/p)for p in ['ws/flight-safety/devel/.private/flight_safety/lib/python3/dist-packages','ws/flight-safety/src/flight_safety/src','ws/aruco-landing/src/aruco_landing/src'])+':'+env.get('PYTHONPATH','')
    os.environ.update(env)
    import sys
    for p in env['PYTHONPATH'].split(':'):
        if p and p not in sys.path:sys.path.insert(0,p)
    import rospy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
    from mavros_msgs.msg import State, ExtendedState, PositionTarget
    from mavros_msgs.srv import SetMode, SetModeResponse, ParamGet, ParamGetResponse, CommandLong, CommandLongResponse, CommandBool, CommandBoolResponse
    from std_msgs.msg import Bool, Int32MultiArray, String
    from std_srvs.srv import Trigger
    from aruco_landing.pose_alignment import matrix_pose
    children=[];logs=[];stop=threading.Event();latest={};commands=[];mode_calls=[];kill_calls=[];arm_calls=[];phase_history=[];tagged_commands=[]
    sim=dict(mode='POSCTL',armed=not AUTO,visible=AUTO,z=1.2,x=4. if SPEEDS else 1.,landed=AUTO)
    if REPEAT:sim.update(x=0.,y=0.,z=.2,sync_ready=False)
    def spawn(name,args):
        f=(OUT/(name+'.log')).open('w');logs.append(f)
        p=subprocess.Popen(args,env=env,cwd=ROOT,stdout=f,stderr=f,start_new_session=True);children.append(p)
    def wait_for(predicate,timeout=6):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            if any(p.poll()is not None for p in children):raise RuntimeError('test child stopped')
            time.sleep(.03)
        raise AssertionError('timeout; latest='+str(latest.get('status')))
    def pose(T,kind=PoseStamped,frame='odom'):
        m=kind();m.header.stamp=rospy.Time.now();m.header.frame_id=frame;p=m.pose.pose if hasattr(m.pose,'pose')else m.pose
        xyz,q=matrix_pose(T);p.position.x,p.position.y,p.position.z=xyz;p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q;return m
    def status(m):
        latest['status']=json.loads(m.data);phase_history.append(latest['status']['phase'])
    def command(m):
        commands.append((time.monotonic(),m));tagged_commands.append((latest.get('status',{}).get('phase'),m))
    def arm(req):
        assert REPEAT and req.value and sim['mode']=='OFFBOARD' and not sim['armed'] and sim['landed'] and sim['sync_ready']
        arm_calls.append(time.monotonic());sim['armed']=True
        return CommandBoolResponse(success=True,result=0)
    def mode(req):mode_calls.append(req.custom_mode);sim['mode']=req.custom_mode;return SetModeResponse(True)
    def force(req):
        kill_calls.append(dict(command=req.command,param1=req.param1,param2=req.param2))
        assert req.command==400 and req.param1==0 and req.param2==21196
        assert sim['mode']=='OFFBOARD' and sim['armed']
        sim['armed']=False
        return CommandLongResponse(success=True,result=0)
    def param(_):
        r=ParamGetResponse();r.success=True;r.value.real=2.;return r
    try:
        spawn('roscore',['roscore','-p',str(PORT)])
        master=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        for _ in range(100):
            try:master.getPid('/trial_test');break
            except OSError:time.sleep(.05)
        rospy.init_node('trial_integration_test',disable_signals=True)
        rospy.Service('/mavros/cmd/command',CommandLong,force)
        rospy.Service('/mavros/cmd/arming',CommandBool,arm)
        rospy.Service('/mavros/set_mode',SetMode,mode);rospy.Service('/mavros/param/get',ParamGet,param)
        spawn('vision_mux',['rosrun','topic_tools','mux','/mavros/vision_pose/pose','/vrpn_client_node/pure/pose','/estimation/external_pose','__name:=vision_pose_mux','mux:=vision_pose_mux'] + (['_initial_topic:=/landing/vision_pose_selected','/landing/vision_pose_selected'] if TRANSITION else []) )
        rospy.set_param('/flight_safety_response',yaml.safe_load((ROOT/'ws/flight-safety/src/flight_safety/config/response.yaml').read_text()))
        rospy.set_param('/flight_safety_response/allow_external_termination',CUT)
        spawn('safety',['python3',str(ROOT/'ws/flight-safety/src/flight_safety/scripts/response_node.py')])
        repeat_args=[]
        if REPEAT:
            repeat_config=yaml.safe_load((ROOT/'stacks/aruco-landing-jetson/config/repeat_trial.yaml').read_text())
            repeat_config.update(sync_status_file=str(OUT/'.sync_gate.json'),settle_s=.2,random_seed=17)
            (OUT/'repeat.yaml').write_text(yaml.safe_dump(repeat_config))
            repeat_args=['repeat_test:=true','repeat_config:='+str(OUT/'repeat.yaml')]
        spawn('trial',['roslaunch',str(ROOT/'ws/aruco-landing/src/aruco_landing/launch/landing_trial.launch'),'dry_run:=false','landing_finish_mode:='+('force_disarm' if CUT else 'auto_land'),'auto_start_on_offboard:='+str(AUTO).lower(),'estimation_transition:='+str(TRANSITION).lower(),'config_root:='+str(ROOT/'stacks/aruco-landing-jetson/config')]+(['approach_speed_mps:=0.5','landing_horizontal_speed_mps:=2.0','descent_speed_mps:=0.5'] if SPEEDS else [])+repeat_args)
        pubs={}
        for key,topic,kind in [('marker','/landing/vision_pose_marker',PoseStamped),('velocity','/mavros/local_position/velocity_local',TwistStamped),('mocap','/vrpn_client_node/pure/pose',PoseStamped),('local','/mavros/local_position/pose',PoseStamped),('state','/mavros/state',State),('extended','/mavros/extended_state',ExtendedState),('pad','/landing/pad_pose_global',PoseStamped),('ready','/landing/alignment/ready',Bool),('visible','/landing/target_visible',Bool),('inliers','/landing/estimator/inlier_ids',Int32MultiArray),('body','/landing/vehicle_pose_pad',PoseWithCovarianceStamped),('camera','/landing/camera_pose_pad',PoseWithCovarianceStamped)]:pubs[key]=rospy.Publisher(topic,kind,queue_size=10)
        rospy.Subscriber('/landing/trial/status',String,status)
        rospy.Subscriber('/landing/pose_transition/status',String,lambda m:latest.update(router=json.loads(m.data)))
        rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,command)
        X=np.array(yaml.safe_load((ROOT/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml').read_text())['matrix_row_major']).reshape(4,4)
        def feed():
            while not stop.is_set():
                velocity=np.zeros(3)
                if REPEAT:
                    if commands and sim['mode']=='OFFBOARD' and sim['armed']:
                        c=commands[-1][1]
                        for i,flag in enumerate([PositionTarget.IGNORE_VX,PositionTarget.IGNORE_VY,PositionTarget.IGNORE_VZ]):
                            if not c.type_mask & flag:velocity[i]=[c.velocity.x,c.velocity.y,c.velocity.z][i]
                        for i,key in enumerate(['x','y','z']):sim[key]+=velocity[i]*.01
                        sim['landed']=sim['z']<.25
                    temp=OUT/'.sync_gate.json.tmp'
                    temp.write_text(json.dumps(dict(checked_at=time.time(),ready=sim['sync_ready'],latest_session=None,reason='mock_sync')))
                    temp.replace(OUT/'.sync_gate.json')
                vmsg=TwistStamped();vmsg.header.stamp=rospy.Time.now()
                vmsg.twist.linear.x,vmsg.twist.linear.y,vmsg.twist.linear.z=velocity;pubs['velocity'].publish(vmsg)
                G=np.eye(4);G[:3,3]=[sim['x'],sim.get('y',.2),sim['z']]
                mc=pose(G);pubs['mocap'].publish(mc);pubs['local'].publish(pose(G,frame='map'))
                st=State();st.header.stamp=rospy.Time.now();st.connected=True;st.armed=sim['armed'];st.mode=sim['mode'];pubs['state'].publish(st)
                ex=ExtendedState();ex.header.stamp=rospy.Time.now();ex.landed_state=ExtendedState.LANDED_STATE_ON_GROUND if sim['landed']else ExtendedState.LANDED_STATE_IN_AIR;pubs['extended'].publish(ex)
                pubs['visible'].publish(Bool(sim['visible']));pubs['inliers'].publish(Int32MultiArray(data=([1] if os.environ.get('TEST_SINGLE_MARKER')=='1' else [1,2,3])if sim['visible']else []))
                if sim['visible']:
                    pubs['pad'].publish(pose(np.eye(4)));pubs['ready'].publish(Bool(True))
                    b=pose(G,PoseWithCovarianceStamped,'physical_landing_pad');b.header.stamp=mc.header.stamp-rospy.Duration(.04)
                    c=pose(G@X,PoseWithCovarianceStamped,'physical_landing_pad');c.header.stamp=mc.header.stamp-rospy.Duration(.04)
                    pubs['body'].publish(b);pubs['camera'].publish(c)
                    marker=pose(G);marker.header.stamp=b.header.stamp
                    marker.pose.position.x+=sim.get('marker_dx',0.)
                    pubs['marker'].publish(marker)
                time.sleep(.01)
        thread=threading.Thread(target=feed,daemon=True);thread.start()
        rospy.wait_for_service('/landing_trial/start',timeout=10)
        start=rospy.ServiceProxy('/landing_trial/start',Trigger);reset=rospy.ServiceProxy('/landing_trial/reset',Trigger)
        wait_for(lambda:latest.get('status',{}).get('healthy'),10)
        if REPEAT:
            time.sleep(1.3);sim['mode']='OFFBOARD'
            wait_for(lambda:latest['status']['phase']=='FAILED_HOLD')
            assert not arm_calls
            sim['mode']='POSCTL';wait_for(lambda:latest['status']['phase']=='CANCELLED')
            sim['sync_ready']=True
            targets=[]
            for number in [1,2]:
                wait_for(lambda:latest['status']['offboard_entry_ready'],8)
                sim['mode']='OFFBOARD'
                wait_for(lambda:len(arm_calls)==number,5)
                wait_for(lambda:latest['status']['phase']=='RANDOM_POSITION',35)
                targets.append(latest['status']['repeat_test']['random_goal_global_m'])
                wait_for(lambda:latest['status']['phase']=='DESCEND',35)
                wait_for(lambda:latest['status']['phase']=='COMPLETE',15)
                assert len(kill_calls)==number and not sim['armed']
                time.sleep(.3);assert len(arm_calls)==number
                sim['mode']='POSCTL';sim['landed']=True
                time.sleep(.2)
            assert targets[0]!=targets[1]
            for phase,m in tagged_commands:
                if phase in ('TAKEOFF','CENTER','RANDOM_POSITION'):
                    assert np.hypot(m.velocity.x,m.velocity.y)<=.500001
                    assert abs(m.velocity.z)<=.500001
            assert all(p in phase_history for p in ['ARMING','TAKEOFF','CENTER','RANDOM_POSITION','APPROACH','DESCEND','CUT_WAIT','COMPLETE'])
            report=dict(result='PASS',transport='localhost mock FCU and velocity-integrated synthetic pose, not flight dynamics',auto_arm_calls=len(arm_calls),completed_trials=len(kill_calls),random_targets=targets,sync_blocks_arm=True,transfer_speed_cap=.5,height_m=2.)
            (OUT/'ros_repeat_trial.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return
        if AUTO:
            wait_for(lambda:latest['status']['offboard_entry_ready'])
            time.sleep(1.2);assert latest['status']['phase']=='IDLE';assert not mode_calls
            if TRANSITION:assert latest['router']['source']=='optitrack'
            sim.update(armed=True,landed=False);time.sleep(.2)
        else:assert start().success
        time.sleep(1.2);assert not mode_calls
        sim['visible']=False;sim['mode']='OFFBOARD';wait_for(lambda:latest['status']['phase']=='APPROACH')
        time.sleep(.15);sp=commands[-1][1];assert sp.velocity.x<0 and abs(sp.velocity.z)<1e-9 and abs(sp.position.z-1.2)<.01
        assert abs(np.hypot(sp.velocity.x,sp.velocity.y)-.5)<1e-6
        sim['visible']=True;dwell_start=time.monotonic()
        time.sleep(.25)
        assert latest['status']['phase']=='APPROACH'
        assert abs(commands[-1][1].velocity.x)<1e-9
        if TRANSITION:assert latest['router']['source']=='optitrack'
        wait_for(lambda:latest['status']['phase']=='DESCEND',5)
        if TRANSITION:assert latest['router']['source']=='marker' and latest['status']['estimation_source']=='marker'
        assert time.monotonic()-dwell_start>=.95
        time.sleep(.15)
        assert commands[-1][1].velocity.z<-.1
        if SPEEDS:
            sp=commands[-1][1]
            assert abs(np.hypot(sp.velocity.x,sp.velocity.y)-2.)<1e-6
            assert abs(sp.velocity.z+.5)<1e-6
        if TRANSITION:
            assert latest['router']['last_output_source']=='marker'
            # A still-visible but inconsistent pose must not keep descending.
            sim['marker_dx']=.3
            time.sleep(.12)
            assert latest['router']['switch_check']['consistent'] is False
            assert commands[-1][1].type_mask & PositionTarget.IGNORE_VZ
            sim['marker_dx']=0.
            wait_for(lambda:latest['router']['switch_check']['consistent'])
            wait_for(lambda:commands[-1][1].velocity.z<-.1)
        # A failed trial must latch a POSITION hover and never resume on rediscovery.
        sim['x']=.7;sim['visible']=False;lost_at=time.monotonic()
        wait_for(lambda:latest['status']['phase']=='FAILED_HOLD',2)
        loss_delay=time.monotonic()-lost_at;assert .45<loss_delay<.7
        wait_for(lambda:abs(commands[-1][1].position.x-.7)<.02,1.)
        if TRANSITION:wait_for(lambda:latest['router']['source']=='optitrack',1.)
        sp=commands[-1][1];assert not sp.type_mask&PositionTarget.IGNORE_PX and sp.type_mask&PositionTarget.IGNORE_VZ
        sim['visible']=True;time.sleep(1.3);assert latest['status']['phase']=='FAILED_HOLD';assert not mode_calls
        if TRANSITION:assert latest['router']['source']=='optitrack'
        sim['mode']='POSCTL';wait_for(lambda:latest['status']['phase']=='CANCELLED')
        if not AUTO:assert reset().success
        # Explicit second attempt reaches the height handoff and waits for real ground+disarm flags.

        if AUTO:wait_for(lambda:latest['status']['offboard_entry_ready'])
        else:assert start().success
        time.sleep(1.1);sim['mode']='OFFBOARD';wait_for(lambda:latest['status']['phase']=='DESCEND',5)
        sim['z']=.19-X[2,3]
        if CUT:
            wait_for(lambda:latest['status']['phase']=='COMPLETE',4)
            assert len(kill_calls)==1 and not mode_calls
            time.sleep(.3);assert len(kill_calls)==1
            producers=dict(master.getSystemState('/trial_test')[2][0])
            assert producers['/mavros/setpoint_raw/local']==['/flight_safety_response']
            report=dict(result='PASS',transition=TRANSITION,loss_to_failed_hold_s=loss_delay,router=latest.get('router'),transport='localhost mock FCU; real safety/planner/MUX',kill_calls=kill_calls,mode_calls=mode_calls,final=latest['status'])
            (OUT/'ros_force_disarm.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return
        wait_for(lambda:'AUTO.LAND'in mode_calls,3)
        assert latest['status']['phase']=='AUTO_LAND'
        sim['landed']=True;time.sleep(.15);assert latest['status']['phase']=='AUTO_LAND'
        sim['armed']=False;wait_for(lambda:latest['status']['phase']=='COMPLETE')
        pubs_state=master.getSystemState('/trial_test')[2][0]
        producers=dict(pubs_state)
        assert producers['/mavros/setpoint_raw/local']==['/flight_safety_response']
        assert producers['/mavros/vision_pose/pose']==['/vision_pose_mux']
        assert mode_calls==['AUTO.LAND']
        # Loss during AUTO.LAND must stop autonomous descent through POSCTL.
        sim.update(mode='POSCTL',armed=True,landed=False,z=1.2)
        time.sleep(.2)
        if not AUTO:assert reset().success
        if AUTO:wait_for(lambda:latest['status']['offboard_entry_ready'])
        else:assert start().success
        time.sleep(1.1);sim['mode']='OFFBOARD';wait_for(lambda:latest['status']['phase']=='DESCEND',5)
        sim['z']=.19-X[2,3];wait_for(lambda:len(mode_calls)==2,3)
        sim['visible']=False;wait_for(lambda:'POSCTL'in mode_calls,3)
        assert mode_calls==['AUTO.LAND','AUTO.LAND','POSCTL']
        assert max(np.hypot(m.velocity.x,m.velocity.y) for _,m in commands)<=(2.000001 if SPEEDS else .500001)
        report=dict(result='PASS',phase_speed_limits_tested=SPEEDS,transport='isolated localhost ROS; mock FCU, actual safety/vision mux/controller/trial nodes',loss_to_failed_hold_s=loss_delay,
            checked=(['pre-arm launch with visible markers stays IDLE','automatic preparation without start/reset services','phase-specific final XY command limits'] if AUTO else [])+['manual OFFBOARD admission','constant approach altitude','marker dwell then descent','0.5s loss -> position hold','no automatic retry','0.2m -> AUTO.LAND, no kill','ground+disarm -> complete','AUTO.LAND marker loss -> POSCTL','sole safety setpoint publisher','sole vision mux publisher'],mode_calls=mode_calls)
        (OUT/('ros_landing_speeds.json' if SPEEDS else 'ros_auto_planner.json' if AUTO else 'ros_integration.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    finally:
        stop.set()
        for p in reversed(children):
            if p.poll()is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
        for f in logs:f.close()

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Isolated PX4 v1.11 SITL pose-source replay. Does not arm or command a vehicle."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT=Path(__file__).resolve().parents[4]
PX4=Path(os.environ.get('PX4_ROOT','/home/ml/PX4/PX4-Autopilot'))
BIN=PX4/'build/px4_sitl_default/bin'
PKG=ROOT/'ws/aruco-landing/src/aruco_landing'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--instance',type=int,default=7,choices=range(10))
    parser.add_argument('--ros-port',type=int,default=11341)
    parser.add_argument('--recorded-imu',action='store_true')
    parser.add_argument('--imu-yaw-deg',type=float,default=0.)
    parser.add_argument('--natural-visibility',action='store_true')
    parser.add_argument('--marker-hold',type=float,default=1.)
    parser.add_argument('--fallback-timeout',type=float)
    parser.add_argument('--optitrack-only',action='store_true')
    args=parser.parse_args();args.bag=args.bag.resolve();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    sim_port=4560+args.instance; mavros_port=14540+args.instance; px4_port=14580+args.instance
    # Refuse occupied test endpoints; never attach to an existing master/FCU.
    for port,kind in [(args.ros_port,socket.SOCK_STREAM),(sim_port,socket.SOCK_STREAM),(mavros_port,socket.SOCK_DGRAM),(px4_port,socket.SOCK_DGRAM)]:
        with socket.socket(socket.AF_INET,kind) as s:
            if kind==socket.SOCK_STREAM:s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind(('127.0.0.1',port))
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(args.ros_port),ROS_IP='127.0.0.1',ROS_LOG_DIR=str(out/'roslog'),PX4_SIM_MODEL='iris_vision')
    env.pop('ROS_HOSTNAME',None)
    env['PATH']=str(BIN)+':'+env['PATH']
    env['PYTHONPATH']=str(PKG/'src')+':'+env.get('PYTHONPATH','')
    processes=[]
    def start(name,cmd,cwd=ROOT):
        log=open(out/(name+'.log'),'wb')
        p=subprocess.Popen(cmd,cwd=cwd,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
        processes.append((name,p,log));return p
    def px4(command):
        result=subprocess.run([str(BIN/('px4-'+command[0])),'--instance',str(args.instance)]+command[1:],env=env,capture_output=True,text=True,timeout=8)
        return result.stdout+result.stderr
    try:
        start('roscore',['roscore','-p',str(args.ros_port)])
        work=out/'px4';work.mkdir()
        startup=(PX4/'ROMFS/px4fmu_common/init.d-posix/rcS').read_text()
        startup=startup.replace('SCRIPT_DIR="$(CDPATH=\'\' cd -- "$(dirname -- "$0")" && pwd)"','SCRIPT_DIR="'+str(PX4/'ROMFS/px4fmu_common/init.d-posix')+'"')
        startup=startup.replace('mavlink start -x -u $udp_gcs_port_local -r 4000000','mavlink start -x -u $udp_gcs_port_local -r 4000000 -o 14657 -t 127.0.0.1')
        # Log even while disarmed so fusion flags/innovations can be audited.
        startup=startup.replace('sh etc/init.d/rc.logging','param set SDLOG_MODE 1\nsh etc/init.d/rc.logging')
        (out/'rcS').write_text(startup)
        start('px4',[str(BIN/'px4'),'-d','-i',str(args.instance),str(PX4/'ROMFS/px4fmu_common'),'-s',str(out/'rcS')],work)
        if args.recorded_imu:
            import rosbag, numpy as np
            pose=[];imu=[]
            with rosbag.Bag(str(args.bag)) as bag:
                bag_start=bag.get_start_time()
                for topic,m,_ in bag.read_messages(topics=['/vrpn_client_node/pure/pose','/mavros/imu/data_raw']):
                    t=m.header.stamp.to_sec()-bag_start
                    if topic.endswith('/pose'):
                        p=m.pose;pose.append([t,p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
                    else:
                        v=m.linear_acceleration;w=m.angular_velocity;imu.append([t,v.x,v.y,v.z,w.x,w.y,w.z])
            if len(imu)<100:raise RuntimeError('Insufficient recorded IMU')
            np.savez(out/'sensors.npz',imu=np.array(imu),pose=np.array(pose))
            start('sensor_replay',['python3',str(Path(__file__).with_name('replay_imu.py')),str(out/'sensors.npz'),str(out/'schedule.json'),'--imu-yaw-deg',str(args.imu_yaw_deg),'--port',str(sim_port)])
        else:
            classes=ROOT/'.build/aruco-transition-jmavsim/classes'
            start('jmavsim',['java','-cp',str(classes)+':'+str(PX4/'Tools/jMAVSim/lib/*'),'me.drton.jmavsim.Simulator','-tcp','127.0.0.1:'+str(sim_port),'-r','250','-lockstep','-no-gui'],PX4/'Tools/jMAVSim')
        time.sleep(3)
        start('mavros',['roslaunch','mavros','px4.launch','fcu_url:=udp://127.0.0.1:'+str(mavros_port)+'@127.0.0.1:'+str(px4_port),'gcs_url:=','tgt_system:='+str(args.instance+1)])
        os.environ.update({k:env[k] for k in ('ROS_MASTER_URI','ROS_IP','ROS_LOG_DIR')});os.environ.pop('ROS_HOSTNAME',None)
        import rospy, rosbag
        from mavros_msgs.msg import State
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        rospy.init_node('transition_sitl_probe',anonymous=True,disable_signals=True)
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            try:
                state=rospy.wait_for_message('/mavros/state',State,timeout=2)
                if state.connected:break
            except rospy.ROSException:pass
        else:raise RuntimeError('SITL MAVROS did not connect; inspect logs')
        assert not state.armed
        settings={}
        for name,value in [('EKF2_AID_MASK','24'),('EKF2_HGT_MODE','3'),('EKF2_EV_DELAY','0')]:
            settings[name]=px4(['param','set',name,value])
        (out/'parameters.json').write_text(json.dumps(settings,indent=2))
        start('router',['python3',str(PKG/'scripts/landing_vision_pose_adapter.py'),
            '_allow_marker_switch:=true','_auto_switch:='+str(not args.optitrack_only).lower(),
            '_stable_duration_s:='+str(args.marker_hold),'_stable_min_samples:='+('1' if args.marker_hold==0 else '30'),
            '_auto_fallback_to_optitrack:='+str(args.fallback_timeout is not None).lower(),
            '_marker_loss_timeout_s:='+str(args.fallback_timeout or .5)])
        observations={'sources':[],'status':[],'output':[],'local':[]}
        def pose_list(m):return [m.header.stamp.to_sec(),m.pose.position.x,m.pose.position.y,m.pose.position.z,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]
        def obsserv(m,key):observations[key].append([time.time()]+pose_list(m))
        subscribers=[rospy.Subscriber('/landing/vision_pose_source',String,lambda m:observations['sources'].append([time.time(),m.data])),
                     rospy.Subscriber('/landing/pose_transition/status',String,lambda m:observations['status'].append([time.time(),json.loads(m.data)])),
                     rospy.Subscriber('/mavros/vision_pose/pose',PoseStamped,lambda m:obsserv(m,'output')),
                     rospy.Subscriber('/mavros/local_position/pose',PoseStamped,lambda m:obsserv(m,'local'))]
        selected={'/vrpn_client_node/pure/pose','/landing/vision_pose_marker','/landing/alignment/ready','/landing/target_visible','/landing/estimator/inlier_ids'}
        with rosbag.Bag(str(args.bag)) as bag:
            messages=list(bag.read_messages(topics=list(selected)))
        if not messages:raise RuntimeError('Bag has no transition input topics')
        publishers={}
        for topic,msg,_ in messages:
            if topic not in publishers:publishers[topic]=rospy.Publisher(topic,type(msg),queue_size=100,latch=topic=='/landing/alignment/ready')
        time.sleep(2)
        warmup_s=0.
        if args.recorded_imu:
            import copy
            initial=next(m for t,m,_ in messages if t=='/vrpn_client_node/pure/pose')
            target=np.array([initial.pose.position.x,initial.pose.position.y,initial.pose.position.z])
            warmup_start=time.monotonic();stable_since=None
            while time.monotonic()-warmup_start<45:
                sample=copy.deepcopy(initial);sample.header.stamp=rospy.Time.now()
                publishers['/vrpn_client_node/pure/pose'].publish(sample)
                recent=observations['local'][-1] if observations['local'] else None
                stable=(recent is not None and time.time()-recent[0]<.2 and np.linalg.norm(np.array(recent[2:5])-target)<.15)
                stable_since=(stable_since or time.monotonic()) if stable else None
                if stable_since is not None and time.monotonic()-stable_since>5:break
                time.sleep(.02)
            else:raise RuntimeError('EKF did not settle on initial stationary pose before replay')
            warmup_s=time.monotonic()-warmup_start
        t0=bag_start if args.recorded_imu else messages[0][2].to_sec();duration=messages[-1][2].to_sec()-t0
        start_wall=time.time()+.5; offset=start_wall-t0
        observations.update(bag=str(args.bag),start_wall=start_wall,duration_s=duration,mocap_cutoff_s=duration*.65,marker_dropout_s=[duration*.8,duration*.9],px4_version=subprocess.check_output(['git','-C',str(PX4),'describe','--tags','--always'],text=True).strip(),ros_master=env['ROS_MASTER_URI'])
        observations.update(imu_yaw_deg=args.imu_yaw_deg,warmup_s=warmup_s,marker_hold_s=args.marker_hold,fallback_timeout_s=args.fallback_timeout,
            optitrack_only=args.optitrack_only,natural_visibility=args.natural_visibility,recorded_imu=args.recorded_imu)
        (out/'schedule.json').write_text(json.dumps({'start_wall':start_wall}))
        next_probe=0;probes=[]
        for topic,msg,stamp in messages:
            rel=stamp.to_sec()-t0
            delay=start_wall+rel-time.time()
            if delay>0:time.sleep(delay)
            if not args.natural_visibility and topic=='/vrpn_client_node/pure/pose' and rel>duration*.65:continue
            if not args.natural_visibility and topic=='/landing/vision_pose_marker' and duration*.8<rel<duration*.9:continue
            if hasattr(msg,'header'):msg.header.stamp=rospy.Time.from_sec(msg.header.stamp.to_sec()+offset)
            publishers[topic].publish(msg)
            if rel>=next_probe:
                probes.append({'elapsed':rel,'estimator_status':px4(['listener','estimator_status','1']),'visual_odometry':px4(['listener','vehicle_visual_odometry','1'])})
                next_probe=rel+5
        time.sleep(.5)
        observations['publishers']=rospy.get_master().getSystemState()[2][0]
        observations['px4_probes']=probes
        (out/'observations.json').write_text(json.dumps(observations,indent=2))
        rospy.signal_shutdown('replay complete')
        print('Replay complete:',out,flush=True)
        print('Sources:',observations['sources'],flush=True)
    finally:
        for _,p,_ in reversed(processes):
            if p.poll() is None:
                try:os.killpg(p.pid,signal.SIGINT)
                except ProcessLookupError:pass
        deadline=time.monotonic()+8
        for _,p,log in reversed(processes):
            try:p.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:os.killpg(p.pid,signal.SIGTERM)
                except ProcessLookupError:pass
            log.close()

if __name__=='__main__':main()

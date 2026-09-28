#!/usr/bin/env python3
"""Replay identical camera/IMU inputs into isolated ROVIO configurations.

Run inside the simulation container with both ROS workspaces sourced. GT is
read for scoring only; it is never published to any filter input.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
import xmlrpc.client

import numpy as np
import rosbag


def section(text, path):
    start, end = 0, len(text)
    for name in path:
        match = re.search(r'(?m)^\s*' + re.escape(name) + r'\s*\{', text[start:end])
        if not match:
            raise ValueError(path)
        start += match.end()
        depth, end = 1, start
        while depth:
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        end -= 1
    return start, end


def main():
    p = argparse.ArgumentParser()
    p.add_argument('trial', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--port', type=int, default=11411)
    p.add_argument('--rate', type=float, default=1)
    p.add_argument('--configs', type=Path, help='Directory of named .info variants')
    p.add_argument('--imu-warmup', type=float, default=0,
                   help='Initialize from a stationary IMU mean over this duration; no GT used')
    args = p.parse_args()
    with socket.socket() as probe:
        try:
            probe.bind(('127.0.0.1', args.port))
        except OSError as error:
            raise SystemExit(f'Replay port {args.port} is already occupied; choose a free --port') from error
    args.output.mkdir(parents=True, exist_ok=False)
    base = (args.trial/'rhem_runtime_config/rovio.info').read_text()
    if args.configs:
        configs = {f.stem:f.read_text() for f in args.configs.glob('*.info')}
    else:
        historical = Path('/work/ws/rhem/src/rhem_planner/rovio_bsp/cfg/rovio.info').read_text()
        upstream = subprocess.check_output(['git', '-C', '/work/ws/rhem/src/rhem_planner/rovio_bsp',
                                           'show', 'HEAD:cfg/rovio.info'], text=True)
        # The recorded trial may already use the fixed profile. Reconstruct the
        # historical covariance blocks explicitly while preserving its geometry.
        for path in [('Init', 'Covariance'), ('Prediction', 'PredictionNoise')]:
            a,b = section(base, path); c,d = section(historical, path)
            base = base[:a] + historical[c:d] + base[b:]
        configs = {}
        for name, paths in [('historical', []), ('initial_cov', [('Init', 'Covariance')]),
                            ('process_noise', [('Prediction', 'PredictionNoise')]),
                            ('both', [('Init', 'Covariance'), ('Prediction', 'PredictionNoise')])]:
            config = base
            for path in paths:
                a,b = section(config, path); c,d = section(upstream, path)
                config = config[:a] + upstream[c:d] + config[b:]
            configs[name] = config
    assert configs
    for name, config in configs.items():
        (args.output/(name+'.info')).write_text(config)
    image_topic = '/camera/left/image_raw'; imu_topic = '/airsim_node/hmcl/imu/imu'
    gt = []
    warmup = None
    if args.imu_warmup:
        samples=[]; first=None
        with rosbag.Bag(str(args.trial/'flight.bag')) as src:
            for _,m,t in src.read_messages(topics=[imu_topic]):
                if first is None: first=m.header.stamp.to_sec()
                samples.append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,
                                m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
                if m.header.stamp.to_sec()-first >= args.imu_warmup:
                    initial=copy.deepcopy(m);initial_receive=t;break
            else: raise ValueError('Insufficient warmup IMU')
        samples=np.array(samples)
        if np.linalg.norm(samples[:,3:],axis=1).max()>.05 or samples[:,:3].std(axis=0).max()>.12:
            raise ValueError('Warmup interval was not stationary')
        for j,k in enumerate('xyz'):setattr(initial.linear_acceleration,k,float(samples[:,j].mean()))
        warmup={'duration_s':initial.header.stamp.to_sec()-first,'samples':len(samples),
                'mean':samples.mean(axis=0).tolist(),'std':samples.std(axis=0).tolist()}
    with rosbag.Bag(str(args.trial/'flight.bag')) as src, rosbag.Bag(str(args.output/'inputs.bag'),'w',compression='lz4') as dst:
        if warmup:dst.write(imu_topic,initial,initial_receive)
        for topic,msg,t in src.read_messages(topics=[image_topic,imu_topic,'/gt_odom']):
            if topic == '/gt_odom':
                v,q = msg.pose.pose.position,msg.pose.pose.orientation
                gt.append([msg.header.stamp.to_sec(),v.x,v.y,v.z,q.x,q.y,q.z,q.w])
            else:
                if warmup and msg.header.stamp<=initial.header.stamp:continue
                dst.write(topic,msg,t)
    with rosbag.Bag(str(args.output/'inputs.bag')) as replay_input:
        input_duration = replay_input.get_end_time()-replay_input.get_start_time()
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{args.port}',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',
                      ROS_HOME=str(args.output/'ros'), ROS_LOG_DIR=str(args.output/'ros/log'))
    procs, logs = [], []
    def launch(cmd, name):
        log = open(args.output/(name+'.log'),'w'); logs.append(log)
        proc = subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        procs.append(proc); return proc
    data = {'gt':gt}; subs=[]
    try:
        launch(['roscore','-p',str(args.port)],'master')
        for _ in range(100):
            try:
                xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/replay'); break
            except OSError: time.sleep(.1)
        else: raise RuntimeError('Master did not start')
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        from sensor_msgs import point_cloud2
        rospy.init_node('replay_audit',disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        def odom_cb(msg, key):
            v,q,w=msg.pose.pose.position,msg.pose.pose.orientation,msg.twist.twist.linear
            data[key].append([msg.header.stamp.to_sec(),v.x,v.y,v.z,q.x,q.y,q.z,q.w,w.x,w.y,w.z])
        def bias_cb(msg,key):
            a,g=msg.linear_acceleration,msg.angular_velocity
            data[key].append([msg.header.stamp.to_sec(),a.x,a.y,a.z,g.x,g.y,g.z])
        def feature_cb(msg,key):
            points=list(point_cloud2.read_points(msg,field_names=['id','status'],skip_nans=False))
            states=[int(status) for idx,status in points if idx>=0]
            data[key].append([msg.header.stamp.to_sec()]+[states.count(i) for i in range(5)])
        for name in configs:
            data[name]=[]; data[name+'_bias']=[]; data[name+'_features']=[]
            subs.append(rospy.Subscriber('/'+name+'/rovio/odometry',Odometry,odom_cb,name,queue_size=10000))
            subs.append(rospy.Subscriber('/'+name+'/rovio/imu_biases',Imu,bias_cb,name+'_bias',queue_size=10000))
            subs.append(rospy.Subscriber('/'+name+'/rovio/pcl',PointCloud2,feature_cb,name+'_features',queue_size=10000))
            launch(['rosrun','rovio','rovio_node','__ns:=/'+name,'__name:=filter',
                    '_filter_config:='+str(args.output/(name+'.info')),
                    '_camera0_config:='+str(args.trial/'rhem_runtime_config/camera.yaml'),
                    '_world_frame:='+name+'_world', '_map_frame:='+name+'_map',
                    '_imu_frame:='+name+'_imu', '_camera_frame:='+name+'_camera',
                    'cam0/image_raw:='+image_topic,'imu0:='+imu_topic],name)
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(200):
            state=master.getSystemState('/replay')[2]
            connections=dict(state[1]).get(image_topic,[])
            if len(connections)==len(configs): break
            if any(proc.poll() is not None for proc in procs): raise RuntimeError('Child exited early')
            time.sleep(.1)
        else: raise RuntimeError('Filters did not subscribe')
        time.sleep(1)
        player=launch(['rosbag','play','--quiet','--clock','--rate',str(args.rate),'--delay','1',str(args.output/'inputs.bag')],'play')
        deadline = time.monotonic()+input_duration/args.rate+60
        while player.poll() is None:
            if time.monotonic()>deadline:
                raise RuntimeError('Replay exceeded input duration and flush allowance')
            if any(proc.poll() is not None for proc in procs[:-1]):
                raise RuntimeError('Filter or master exited during replay')
            (args.output/'progress.json').write_text(json.dumps({k:{'samples':len(v),'latest':v[-1][:4] if len(v) else None} for k,v in data.items()}))
            time.sleep(5)
        if player.returncode!=0: raise RuntimeError('Replay failed')
        time.sleep(3)
    finally:
        for proc in reversed(procs):
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        for proc in reversed(procs):
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGTERM)
        for log in logs: log.close()
        np.savez_compressed(args.output/'states.npz',**{k:np.array(v) for k,v in data.items()})
        diagnostic = args.trial/'diagnostic.json'
        (args.output/'manifest.json').write_text(json.dumps({'trial':str(args.trial),'rate':args.rate,'variants':list(configs),
            'gt_pose_fused':False,'simulator_assisted_input':diagnostic.exists(),
            'imu_warmup':warmup,
            'input_diagnostic':json.loads(diagnostic.read_text()) if diagnostic.exists() else None,
            'counts':{k:len(v) for k,v in data.items()}},indent=2)+'\n')
    print(json.dumps({k:{'samples':len(v),'final':v[-1][:4] if len(v) else None} for k,v in data.items()},indent=2))


if __name__=='__main__':
    main()

#!/usr/bin/env python3
"""Isolated real-camera recording benchmark. Never starts FCU/control/webcam triggers."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
import yaml

ROOT=Path('/work')
PORT=11349


def main():
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('Private test master is occupied')
    # Refuse to contend with a camera already owned by another live stack.
    check=subprocess.run(['pgrep','-x','see3cam_node'],capture_output=True)
    if check.returncode==0:raise RuntimeError('Camera node already running; benchmark did not start')
    os.environ.update(ROS_MASTER_HOST='127.0.0.1',ROS_MASTER_PORT=str(PORT),ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,
                      ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1')
    children=[];lock=threading.Lock();events=[];statuses=[]
    out=ROOT/'stack-assets/aruco-landing-jetson/results/manual-flight-20260919/performance'
    out.mkdir(parents=True,exist_ok=True)
    log=(out/'benchmark.log').open('w')
    def spawn(args):
        proc=subprocess.Popen(args,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
        children.append(proc);return proc
    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
    try:
        spawn(['roscore','-p',str(PORT)])
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(50):
            try:master.getPid('/recording_benchmark');break
            except OSError:time.sleep(.1)
        camera=spawn(['bash',str(ROOT/'modules/sensor/see3cam-24cug/run.sh')])
        estimator=spawn(['bash',str(ROOT/'modules/perception/aruco-landing/run.sh')])
        import rospy
        from sensor_msgs.msg import CameraInfo
        from std_msgs.msg import String
        rospy.init_node('recording_profile_benchmark',disable_signals=True)
        def image(msg):
            with lock:events.append((time.monotonic(),msg.header.stamp.to_sec()))
        def status(msg):
            with lock:statuses.append((time.monotonic(),json.loads(msg.data)))
        subs=[rospy.Subscriber('/landing/camera/camera_info',CameraInfo,image,queue_size=100),
              rospy.Subscriber('/landing/estimator/status',String,status,queue_size=20)]
        time.sleep(5)
        if camera.poll() is not None or estimator.poll() is not None:raise RuntimeError('Camera/estimator failed; see benchmark.log')
        profiles={
            'baseline_no_recorder':None,
            'old_record_all':yaml.safe_load((ROOT/'ws/flight-safety/src/flight_safety/config/recorder.yaml').read_text()),
            'aruco_allowlist':yaml.safe_load((ROOT/'stack-assets/aruco-landing-jetson/config/flight_safety_recorder.yaml').read_text()),
        }
        results=[]
        with tempfile.TemporaryDirectory(prefix='aruco-camera-benchmark-') as temp:
            for name,config in profiles.items():
                record=None
                if config is not None:
                    args=['taskset','-c',os.environ['CPUS_RECORDER'],'rosbag','record','--lz4','-O',temp+'/'+name+'.bag']
                    if config['record_all']:
                        args+=['-a','-x',config['exclude']]
                    else:args+=config['topics']
                    record=spawn(args)
                time.sleep(3)
                begin=time.monotonic();time.sleep(12);end=time.monotonic()
                with lock:
                    samples=[e for e in events if begin<=e[0]<=end]
                    state=[s for t,s in statuses if begin<=t<=end]
                gaps=[b[0]-a[0] for a,b in zip(samples,samples[1:])]
                result=dict(profile=name,observed_frames=len(samples),interval_s=end-begin,
                    camera_hz=(len(samples)-1)/(samples[-1][0]-samples[0][0]) if len(samples)>1 else 0,
                    max_camera_gap_s=max(gaps) if gaps else None,
                    processing_hz=[s['processing_hz']for s in state],
                    processing_ms_mean=[s['processing_ms_mean']for s in state],
                    recorder_alive=None if record is None else record.poll() is None)
                results.append(result)
                print(json.dumps(result),flush=True)
                if record:stop(record)
                time.sleep(2)
            affinity={}
            for name in ['/landing/camera','/physical_pad_estimator']:
                uri=master.lookupNode('/recording_benchmark',name)[2]
                pid=xmlrpc.client.ServerProxy(uri).getPid('/recording_benchmark')[2]
                affinity[name]=dict(pid=pid,cpus=sorted(os.sched_getaffinity(pid)))
            report=dict(results=results,affinity=affinity,policy={k:v for k,v in os.environ.items()if k.startswith('CPUS_')},
                scope='Private ROS master, live camera and physical estimator; no MAVROS/PX4/control nodes or webcam triggers. Synthetic benchmark bags deleted.')
            (out/'recording_profiles.json').write_text(json.dumps(report,indent=2)+'\n')
        rospy.signal_shutdown('benchmark complete')
    finally:
        for child in reversed(children):stop(child)
        log.close()


if __name__=='__main__':main()

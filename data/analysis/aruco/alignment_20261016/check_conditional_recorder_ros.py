#!/usr/bin/env python3
"""Exercise the real recorder on a private master, with synthetic ARM messages.
No FCU, flight controller or drone network topics are connected.
"""
import argparse,json,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import rosbag
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    port=11419
    with socket.socket() as s:
        assert s.connect_ex(('127.0.0.1',port))!=0
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:%s'%port,
             ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',
             ROS_HOME=str(a.output.resolve()/'ros_home'),
             PYTHONPATH=str(a.source.resolve()/'src')+':'+os.environ.get('PYTHONPATH',''))
    os.environ.update({k:env[k] for k in ['ROS_MASTER_URI','ROS_IP','ROS_HOSTNAME','ROS_HOME']})
    procs=[];logs=[]
    def start(cmd,label):
        f=(a.output/(label+'.log')).open('w');logs.append(f)
        proc=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        procs.append(proc);return proc
    try:
        start(['roscore','-p',str(port)],'master')
        api=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        for _ in range(100):
            try:api.getPid('/conditional_check');break
            except OSError:time.sleep(.05)
        else:raise RuntimeError('private master unavailable')
        rospy.init_node('conditional_recorder_check',disable_signals=True)
        cfg=dict(bag_dir=str(a.output.resolve()/'bags'),prefix='synthetic',topics=['/test/base'],
                 record_all=False,lz4=True,webcam_enable=False,odom_history_enable=False,
                 runtime_manifest_enable=False,conditional_topic_groups=[dict(topics=['/test/raw'],
                 status_topic='/test/status',field='queue.next_index',equals=0,max_age_s=2,on_unavailable='include')])
        rospy.set_param('/test_recorder',cfg)
        node=start(['/usr/bin/python3',str(a.source.resolve()/'scripts/recorder_node.py'),
                    '__name:=test_recorder'],'recorder')
        state=rospy.Publisher('/mavros/state',State,queue_size=5)
        status=rospy.Publisher('/test/status',String,queue_size=5)
        base=rospy.Publisher('/test/base',String,queue_size=5)
        raw=rospy.Publisher('/test/raw',String,queue_size=5)
        def send(armed,index,pad,duration):
            end=time.monotonic()+duration
            while time.monotonic()<end:
                assert node.poll() is None
                status.publish(json.dumps(dict(pad=pad,queue=dict(next_index=index))))
                state.publish(State(connected=True,armed=armed,mode='SYNTHETIC'))
                base.publish('base evidence');raw.publish('raw evidence')
                time.sleep(.02)
        result=[]
        for index,pad,expected in [(0,'pad_a',True),(1,'pad_a',False),(0,'pad_b',True)]:
            send(False,index,pad,1.0)
            send(True,index,pad,1.5)
            send(False,index,pad,1.0)
            bags=sorted((a.output/'bags').glob('*.bag'))
            assert len(bags)==len(result)+1, bags
            with rosbag.Bag(str(bags[-1])) as b:
                topics=b.get_type_and_topic_info().topics
                assert '/test/base' in topics
                assert ('/test/raw' in topics)==expected, topics
                result.append(dict(pad=pad,index=index,raw_recorded='/test/raw' in topics,
                                   bag=str(bags[-1]),topics=list(topics)))
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))
    finally:
        for proc in reversed(procs):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3)
        for f in logs:f.close()


if __name__=='__main__':main()

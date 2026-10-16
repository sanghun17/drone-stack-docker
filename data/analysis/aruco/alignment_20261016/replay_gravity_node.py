#!/usr/bin/env python3
"""Run the actual ROS estimator on a private master, ground then flight.
Only camera/GT/IMU sensor topics are replayed; no control or FCU is launched.
"""
import argparse,json,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation
from audit import matrix,interpolate,summary

ROOT=Path(__file__).resolve().parents[4]

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--ground',type=Path,required=True);p.add_argument('--flight',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=11416)
 p.add_argument('--gravity-constraint',choices=['true','false'],default='true')
 p.add_argument('--level-pad',choices=['true','false'],default='true')
 p.add_argument('--max-error-m',type=float,default=.10,help='Experiment acceptance bound; recorded independently of metrics')
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 with socket.socket() as s:
  if s.connect_ex(('127.0.0.1',a.port))==0:raise RuntimeError('Private master port is occupied')
 env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:%d'%a.port,ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_HOME=str(a.output.resolve()/'ros_home'),PYTHONNOUSERSITE='1')
 env['PYTHONPATH']=str(a.source.resolve()/'src')+':'+env.get('PYTHONPATH','')
 os.environ.update({k:env[k] for k in ['ROS_MASTER_URI','ROS_IP','ROS_HOSTNAME','ROS_HOME']})
 processes=[];logs=[]
 def start(args,label):
  f=(a.output/(label+'.log')).open('w');logs.append(f);proc=subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
 def stop(proc):
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=8)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=3)
 try:
  master=start(['roscore','-p',str(a.port)],'master');api=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
  for _ in range(100):
   try:api.getPid('/gravity_replay');break
   except OSError:time.sleep(.05)
  else:raise RuntimeError('Master failed')
  subprocess.run(['rosparam','set','use_sim_time','true'],env=env,check=True)
  rospy.init_node('gravity_replay_collector',disable_signals=True)
  output=[];statuses=[];gt=[]
  subscriptions=[rospy.Subscriber('/landing/vision_pose_marker',PoseStamped,lambda m:output.append((m.header.stamp.to_sec(),matrix(m.pose))),queue_size=1000),rospy.Subscriber('/landing/estimator/status',String,lambda m:statuses.append(json.loads(m.data)),queue_size=100),rospy.Subscriber('/vrpn_client_node/pure/pose',PoseStamped,lambda m:gt.append((m.header.stamp.to_sec(),matrix(m.pose))),queue_size=1000)]
  config=ROOT/'stacks/aruco-landing-jetson/config'
  estimator=start(['/usr/bin/python3',str(a.source.resolve()/'scripts/physical_pad_estimator.py'),'_pad_configuration:='+str(a.source.resolve()/'config/proposed_pad_1_layout_physical_20260920_runtime.yaml'),'_body_camera_calibration:='+str(config/'calibration/20260919/base_link_to_see3cam_optical_frame.yaml'),'_time_alignment:='+str(config/'calibration/20260919/time_alignment.yaml'),'_gravity_constraint:='+a.gravity_constraint,'_level_pad:='+a.level_pad],'estimator')
  # Allow ROS publishers/subscribers to register before any bag measurements.
  time.sleep(1.)
  for label,path in [('ground',a.ground),('flight',a.flight)]:
   play=start(['rosbag','play','--clock','--hz','200','--delay','1',str(path.resolve()),'--topics','/landing/camera/image_raw','/landing/camera/camera_info','/vrpn_client_node/pure/pose','/mavros/imu/data'],label+'_play')
   while play.poll() is None:
    if estimator.poll() is not None:raise RuntimeError('Estimator exited; see estimator.log')
    time.sleep(.25)
   if play.returncode:raise RuntimeError('Playback failed')
   time.sleep(.4)
   print(label,'outputs',len(output),'status',statuses[-1] if statuses else None,flush=True)
  with rosbag.Bag(str(a.flight)) as b:begin,end=b.get_start_time(),b.get_end_time()
  chosen=[v for v in output if begin-.1<=v[0]<=end];times=np.array([v[0] for v in chosen]);est=np.array([v[1] for v in chosen]);G,ok=interpolate(gt,times);delta=est[ok,:3,3]-G[ok,:3,3];angles=np.rad2deg(Rotation.from_matrix(np.swapaxes(G[ok,:3,:3],1,2)@est[ok,:3,:3]).magnitude());metrics=summary(delta,angles)
  result={'source':str(a.source.resolve()),'ground':str(a.ground),'flight':str(a.flight),'gravity_constraint':a.gravity_constraint=='true','level_pad':a.level_pad=='true','acceptance_max_error_m':a.max_error_m,'scope':'Actual ROS node on isolated localhost master; enabled reference and pad placement learned in independent earlier ground bag, then frozen before flight replay; no flight GT used for updating either reference','metrics':metrics,'final_status':statuses[-1] if statuses else None,'statuses':statuses,'flight_samples':[{'stamp':float(t),'delta_xyz_m':v.tolist(),'angle_error_deg':float(angle)} for t,v,angle in zip(times[ok],delta,angles)]}
  (a.output/'node_replay.json').write_text(json.dumps(result,indent=2)+'\n');print('FLIGHT_METRICS',json.dumps(metrics),flush=True)
  if metrics['count']<500 or metrics['position_m_p50_p95_max'][2]>a.max_error_m:raise RuntimeError('Node replay accuracy/coverage validation failed')
 finally:
  for proc in reversed(processes):stop(proc)
  for f in logs:f.close()

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Isolated matched-input FAST-LIVO timing comparison; no GT filter inputs."""
import argparse,copy,hashlib,json,os,signal,socket,subprocess,sys,time,xmlrpc.client
from pathlib import Path
import numpy as np
import rosbag,yaml
p=argparse.ArgumentParser();p.add_argument('inputs',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--port',type=int,default=11421);p.add_argument('--rate',type=float,default=.75)
p.add_argument('--camera-calibration',choices=['recorded','shared-profile'],default='recorded',help='Calibration is identical within each timing pair')
a=p.parse_args()
with socket.socket() as probe:probe.bind(('127.0.0.1',a.port))
a.output.mkdir(parents=True,exist_ok=False)
manifest=json.loads((a.inputs/'manifest.json').read_text());trial=Path(manifest['trial']);params=yaml.safe_load((trial/'parameters.yaml').read_text())
os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{a.port}',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_HOME=str(a.output/'ros'),ROS_LOG_DIR=str(a.output/'ros/log'))
procs=[];logs=[];subs=[];data=dict(np.load(a.inputs/'reference.npz'))
def launch(command,name):
 f=(a.output/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p
try:
 launch(['roscore','-p',str(a.port)],'master')
 for _ in range(100):
  try:xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/timing_replay');break
  except OSError:time.sleep(.1)
 else:raise RuntimeError('Master did not start')
 import rospy
 from nav_msgs.msg import Odometry
 rospy.init_node('timing_replay',disable_signals=True);rospy.set_param('/use_sim_time',True)
 corrected_extrinsics=None
 if a.camera_calibration=='shared-profile':
  sys.path.insert(0,'/work/modules/simulation/airsim/scripts')
  from camera_geometry import make_static_transforms
  from tf.transformations import quaternion_matrix
  profile=yaml.safe_load((trial.parent/'provenance/airsim_cameras.yaml').read_text())
  mounts={}
  for stream in profile['streams']:
   matrix=np.eye(4)
   for transform in make_static_transforms(stream['camera'],stream['ros']):
    q,t=transform.transform.rotation,transform.transform.translation
    element=quaternion_matrix([q.x,q.y,q.z,q.w]);element[:3,3]=[t.x,t.y,t.z]
    matrix=matrix@element
   mounts[stream['image_type']]=matrix
  camera_from_depth=np.linalg.inv(mounts[0])@mounts[1]
  corrected_extrinsics={'extrinsic_T':mounts[1][:3,3].tolist(),'extrinsic_R':mounts[1][:3,:3].ravel().tolist(),
                       'Pcl':camera_from_depth[:3,3].tolist(),'Rcl':camera_from_depth[:3,:3].ravel().tolist()}

 def receive(m,key):
  p,q=m.pose.pose.position,m.pose.pose.orientation
  data[key].append([m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
 groups=['common','extrin_calib','time_offset','preprocess','vio','imu','lio','local_map','uav','publish','evo','pcd_save']
 variants=['legacy','fixed']
 for name in variants:
  cfg={key:copy.deepcopy(params[key]) for key in groups}
  cfg['common'].update(img_topic='/timing/'+name+'/image',lid_topic='/timing/'+name+'/cloud',imu_topic='/timing/imu')
  cfg['debug']={'verbose':False}
  if corrected_extrinsics is not None:cfg['extrin_calib'].update(corrected_extrinsics)
  # Keep estimator tuning and calibration identical to the recorded run.
  rospy.set_param('/'+name,cfg);camera=copy.deepcopy(params['laserMapping']);camera.pop('reinitialize_with_gt_odom',None)
  rospy.set_param('/'+name+'/laserMapping',camera)
  (a.output/(name+'.yaml')).write_text(yaml.safe_dump({'parameters':cfg,'camera':camera}))
  for suffix,topic in [('', '/odom'),('_imu','/imu_odom')]:
   key=name+suffix;data[key]=[];subs.append(rospy.Subscriber('/'+name+topic,Odometry,receive,key,queue_size=20000))
  launch(['rosrun','fast_livo','fastlivo_mapping','__ns:=/'+name,'__name:=laserMapping',
          '/aft_mapped_to_init:=/'+name+'/odom','/LIVO2/imu_propagate:=/'+name+'/imu_odom',
          '/tf:=/'+name+'/tf','/tf_static:=/'+name+'/tf_static','/gt_odom:=/'+name+'/unused_gt'],name)
 master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
 for _ in range(150):
  subscriptions=dict(master.getSystemState('/timing_replay')[2][1])
  if all('/timing/'+name+'/image' in subscriptions for name in variants):break
  if any(p.poll() is not None for p in procs):raise RuntimeError('A child exited before replay')
  time.sleep(.1)
 else:raise RuntimeError('Filters did not subscribe')
 time.sleep(1)
 player=launch(['rosbag','play','--quiet','--clock','--rate',str(a.rate),'--delay','1',str(a.inputs/'inputs.bag')],'play')
 deadline=time.monotonic()+manifest['duration_s']/a.rate+90
 while player.poll() is None:
  if time.monotonic()>deadline:raise RuntimeError('Replay timeout')
  if any(p.poll() is not None for p in procs[:-1]):raise RuntimeError('Estimator or master exited during replay')
  (a.output/'progress.json').write_text(json.dumps({k:{'samples':len(v),'latest':v[-1][:4] if len(v) else None} for k,v in data.items()}))
  time.sleep(5)
 if player.returncode:raise RuntimeError('Player failed')
 time.sleep(3)
finally:
 for p in reversed(procs):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in reversed(procs):
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
 for f in logs:f.close()
 np.savez_compressed(a.output/'states.npz',**{k:np.asarray(v) for k,v in data.items()})
 binary=Path('/work/ws/fast-livo-sim/devel/.private/fast_livo/lib/fast_livo/fastlivo_mapping')
 manifest.update(rate=a.rate,binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),gt_pose_fused=False,counts_output={k:len(v) for k,v in data.items()},calibration=a.camera_calibration,calibration_identical_within_pair=True)
 (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest,indent=2))

import rosbag,json,numpy as np,cv2
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path('/home/ml/drone-stack-docker');D=Path('/home/ml/webcam_recorder/recordings/flight_2026-09-20_00-21-17');O=ROOT/'stack-assets/aruco-landing-jetson/results/landing-flight-20260920-002117'
a={};msgs={};states=[]
with rosbag.Bag(str(next(D.glob('*.bag')))) as b:
 epoch=b.get_start_time()
 for t,m,ts in b.read_messages():
  tm=ts.to_sec()-epoch;msgs.setdefault(t,[]).append((tm,m))
  row=None
  if t in ['/vrpn_client_node/pure/pose','/mavros/local_position/pose']:p=m.pose.position;row=[tm,p.x,p.y,p.z]
  elif t in ['/landing/camera_pose_pad','/landing/trial/camera_pose_pad']:p=m.pose.pose.position;row=[tm,p.x,p.y,p.z]
  elif t=='/mavros/local_position/velocity_local':v=m.twist.linear;row=[tm,v.x,v.y,v.z]
  elif t in ['/mavros/setpoint_raw/local','/mavros/setpoint_raw/target_local','/local_controller/setpoint_raw/local']:row=[tm,m.position.x,m.position.y,m.position.z,m.velocity.x,m.velocity.y,m.velocity.z,m.yaw,m.yaw_rate,m.type_mask]
  elif t=='/mavros/imu/data':v=m.linear_acceleration;w=m.angular_velocity;row=[tm,v.x,v.y,v.z,w.x,w.y,w.z]
  if row:a.setdefault(t,[]).append(row)
a={k:np.array(v) for k,v in a.items()};np.savez(O/'samples.npz',**a)
fig,axs=plt.subplots(5,1,figsize=(13,13),sharex=True)
for t in ['/vrpn_client_node/pure/pose','/mavros/local_position/pose','/landing/camera_pose_pad','/landing/trial/camera_pose_pad']:
 r=a[t];axs[0].plot(r[:,0],r[:,3],label=t)
for t in ['/vrpn_client_node/pure/pose','/mavros/local_position/pose']:
 r=a[t];axs[1].plot(r[:,0],r[:,1],label=t+' x');axs[1].plot(r[:,0],r[:,2],label=t+' y')
r=a['/mavros/local_position/velocity_local'];axs[2].plot(r[:,0],r[:,3],label='measured local vz')
for t in ['/local_controller/setpoint_raw/local','/mavros/setpoint_raw/local','/mavros/setpoint_raw/target_local']:
 r=a[t];axs[2].plot(r[:,0],r[:,6],label=t+' vz (check mask)')
r=a['/mavros/imu/data'];axs[3].plot(r[:,0],np.linalg.norm(r[:,1:4],axis=1),label='accel norm');axs[4].plot(r[:,0],np.degrees(np.linalg.norm(r[:,4:7],axis=1)),label='angular speed deg/s')
for ax in axs:
 ax.grid();ax.legend(fontsize=7);ax.set_xlim(20,28.7)
 for v in [24.048,24.202,25.798,28.203]:ax.axvline(v,c='gray',alpha=.5)
axs[-1].set_xlabel('bag receipt seconds');fig.tight_layout();fig.savefig(O/'touchdown.png',dpi=150)
print('time mocap_z marker_z local_vz actual_sp_vz sp_mask target_vz target_mask')
for tm in np.arange(23.5,28.51,.25):
 vals=[]
 for t,cols in [('/vrpn_client_node/pure/pose',[3]),('/landing/camera_pose_pad',[3]),('/mavros/local_position/velocity_local',[3]),('/mavros/setpoint_raw/local',[6,9]),('/mavros/setpoint_raw/target_local',[6,9])]:
  r=a[t];v=r[np.argmin(abs(r[:,0]-tm))];vals.extend(v[cols])
 print(round(tm,2),*[round(v,4) for v in vals])
for cam in ['cam1','cam2']:
 cap=cv2.VideoCapture(str(next((D/cam).glob('*.mp4'))));tiles=[]
 for tm in np.arange(23.,27.51,.5):
  cap.set(cv2.CAP_PROP_POS_MSEC,float(tm*1000));ok,im=cap.read()
  if not ok:continue
  im=cv2.resize(im,(648,486));cv2.putText(im,f'{cam} video {tm:.2f}s',(15,32),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2);tiles.append(im)
 cap.release()
 cv2.imwrite(str(O/(cam+'_landing_sheet.jpg')),np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)]))
# summarize telemetry evidence, not internal EKF assertion
mc={m.header.stamp.to_nsec():m for _,m in msgs['/vrpn_client_node/pure/pose']};same=diff=0
for _,m in msgs['/mavros/vision_pose/pose']:
 ref=mc.get(m.header.stamp.to_nsec())
 if ref is not None:
  if ref==m:same+=1
  else:diff+=1
print('vision same/diff',same,diff)
for t in ['/mavros/estimator_status','/rosout_agg']:
 print(t)
 if t.endswith('status'):
  print(str(msgs[t][0][1]));print('unique flags',set(tuple(getattr(m,k) for k in m.__slots__ if k!='header') for _,m in msgs[t]))
 else:
  for tm,m in msgs[t]:print(round(tm,3),m.name,m.msg)

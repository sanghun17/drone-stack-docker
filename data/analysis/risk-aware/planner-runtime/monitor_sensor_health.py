#!/usr/bin/env python3
"""Lightweight independent sensor timing log; no map evaluation or payload copy."""
import argparse,csv,json,threading,time
from pathlib import Path
import rospy
from sensor_msgs.msg import Image,Imu
from nav_msgs.msg import Odometry
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=1800);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
rospy.init_node('independent_sensor_timing',anonymous=True,disable_signals=True)
lock=threading.Lock();files=[];subs=[];counts={}
def receive(m,entry):
 name,writer=entry
 # wall_monotonic records reception independently of potentially delayed /clock.
 row=[time.monotonic(),rospy.Time.now().to_sec(),m.header.stamp.to_nsec(),m.header.seq]
 if hasattr(m,'angular_velocity'):
  row += [m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]
 if hasattr(m,'pose'):
  v=m.pose.pose.position;q=m.pose.pose.orientation;row += [v.x,v.y,v.z,q.x,q.y,q.z,q.w]
 with lock:writer.writerow(row);counts[name]+=1
try:
 for name,topic,cls in [('imu','/airsim_node/hmcl/imu/imu',Imu),('image','/camera/left/image_raw',Image),('gt','/gt_odom',Odometry),('rovio','/rhem/rovio/odometry',Odometry)]:
  f=(a.output/(name+'.csv')).open('w');files.append(f);w=csv.writer(f)
  w.writerow(['wall_monotonic','clock_ros','header_ns','seq']+(['wx','wy','wz','ax','ay','az'] if name=='imu' else ['x','y','z','qx','qy','qz','qw'] if name in ['gt','rovio'] else []))
  counts[name]=0;subs.append(rospy.Subscriber(topic,cls,receive,(name,w),queue_size=10000,buff_size=2**24,tcp_nodelay=True))
 start=time.monotonic()
 while not rospy.is_shutdown() and time.monotonic()-start<a.duration:
  time.sleep(2)
  with lock:
   for f in files:f.flush()
   (a.output/'progress.json').write_text(json.dumps(counts))
except KeyboardInterrupt:pass
finally:
 for s in subs:s.unregister()
 for f in files:f.close()

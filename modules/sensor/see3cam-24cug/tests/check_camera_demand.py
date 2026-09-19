import os,subprocess,time,signal,json
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11359',ROS_MASTER_HOST='127.0.0.1',ROS_MASTER_PORT='11359',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1')
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
logs=[];children=[]
def start(name,argv):
 f=open('/tmp/seecam-'+name+'.log','w');logs.append(f);p=subprocess.Popen(argv,stdout=f,stderr=f,start_new_session=True);children.append(p);return p
try:
 start('master',['roscore','-p','11359']);time.sleep(2)
 camera=start('camera',['bash','/work/modules/sensor/see3cam-24cug/run.sh'])
 rospy.init_node('camera_demand_test',anonymous=True,disable_signals=True)
 status=[];rospy.Subscriber('/landing/camera/stream_status',String,lambda m:status.append(m.data))
 time.sleep(3)
 for i in range(5):
  m=rospy.wait_for_message('/landing/camera/image_raw',Image,timeout=12)
  print('one-shot',i,m.width,m.height,m.header.stamp.to_sec(),flush=True)
  time.sleep(2)
  print('idle',camera.poll(),status[-1:] ,flush=True)
 counts=[];sub=rospy.Subscriber('/landing/camera/image_raw',Image,lambda m:counts.append(time.monotonic()),queue_size=1,buff_size=4000000)
 time.sleep(35);sub.unregister();time.sleep(2)
 print(json.dumps(dict(frames=len(counts),hz=(len(counts)-1)/(counts[-1]-counts[0]) if len(counts)>1 else 0,exit=camera.poll(),status=status[-3:])),flush=True)
finally:
 for p in reversed(children):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in children:
  try:p.wait(timeout=7)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
 for f in logs:f.close()

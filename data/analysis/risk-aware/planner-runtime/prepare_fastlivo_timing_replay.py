#!/usr/bin/env python3
"""Prepare matched pixel/point/IMU inputs with measured exposure/readback times.
GT is saved for scoring and never published to a filter. Native Unreal CRC32
identifies each returned pixel payload when concurrent captures share a stamp.
"""
import argparse,copy,hashlib,json,re,zlib
from collections import defaultdict
from pathlib import Path
import numpy as np
import rosbag,rospy,yaml
p=argparse.ArgumentParser();p.add_argument('trial',type=Path);p.add_argument('--trace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--duration',type=float,default=300);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
pattern=r'AIRSIM_CAPTURE_TIMING camera=(\S+) type=(\d+) capture_ns=(\d+) readback_ns=(\d+) payload_crc=(\d+)'
pairs=defaultdict(set)
for camera,kind,capture,readback,crc in re.findall(pattern,a.trace.read_text(errors='replace')):
 pairs[(camera,int(capture),int(crc))].add(int(readback))
if not pairs:raise SystemExit('No payload-associated timing evidence in trace')
events=[json.loads(s) for s in (a.trial/'events.jsonl').read_text().splitlines()]
stop=next(e['ros_time'] for e in events if e['phase']=='E.stop')
rgb='/camera/left/image_raw';depth='/camera/depth/image_raw';cloud='/voxel_grid/output';imu='/airsim_node/hmcl/imu/imu'
timing={rgb:{},depth:{}};gt=[];selected=[];missing=[]
with rosbag.Bag(str(a.trial/'flight.bag')) as bag:
 start=bag.get_start_time();end=min(stop,start+a.duration)
 for topic,m,t in bag.read_messages(topics=[rgb,depth,'/gt_odom'],end_time=rospy.Time.from_sec(end)):
  if topic=='/gt_odom':
   v,q=m.pose.pose.position,m.pose.pose.orientation;gt.append([m.header.stamp.to_sec(),v.x,v.y,v.z,q.x,q.y,q.z,q.w]);continue
  stamp=m.header.stamp.to_nsec();crc=zlib.crc32(m.data)&0xffffffff
  key=('scene_cam1' if topic==rgb else 'depth_cam',stamp,crc)
  matches=sorted(pairs.get(key,[]))
  if not matches:missing.append({'topic':topic,'stamp_ns':stamp,'crc':crc});continue
  # Multiple identical pixels from the same exposure are interchangeable. Use
  # their first measured readback, and disclose the timing spread explicitly.
  timing[topic][stamp]=matches[0]
  selected.append({'topic':topic,'capture_ns':stamp,'readback_ns':matches[0],
                   'identical_response_count':len(matches),'readback_spread_ns':matches[-1]-matches[0]})
if missing:raise SystemExit(f'{len(missing)} image payloads lack exact timestamp/CRC evidence; refusing an approximate comparison')
counts=defaultdict(int);hashes={k:hashlib.sha256() for k in [rgb,cloud,imu]}
with rosbag.Bag(str(a.trial/'flight.bag')) as src,rosbag.Bag(str(a.output/'inputs.bag'),'w',compression='lz4') as dst:
 for topic,m,t in src.read_messages(topics=[rgb,cloud,imu],end_time=rospy.Time.from_sec(end)):
  counts[topic]+=1
  if topic==imu:
   dst.write('/timing/imu',m,t);continue
  stamp=m.header.stamp.to_nsec();old=timing[rgb if topic==rgb else depth].get(stamp)
  if old is None:raise RuntimeError(f'No matching depth exposure for cloud {stamp}')
  label='image' if topic==rgb else 'cloud'
  dst.write('/timing/fixed/'+label,m,t)
  hashes[topic].update(m.data)
  m=copy.deepcopy(m);m.header.stamp=rospy.Time(old//10**9,old%10**9)
  dst.write('/timing/legacy/'+label,m,t)
np.savez_compressed(a.output/'reference.npz',gt=np.array(gt))
(a.output/'timing_pairs.json').write_text(json.dumps(selected,indent=2)+'\n')
manifest={'trial':str(a.trial),'trace':str(a.trace),'duration_s':end-start,'end_ros':end,
 'counts':dict(counts),'payload_sha256':{k:v.hexdigest() for k,v in hashes.items() if k!=imu},
 'gt_published':False,'same_pixels_points_imu':True,'scope':'Timestamp-only matched-input replay; frame-deduplication and transport changes are not separately evaluated',
 'timing_choice':'First readback among responses with identical camera, exposure stamp and CRC32',
 'readback_minus_capture_ms_percentiles':np.percentile([(v['readback_ns']-v['capture_ns'])/1e6 for v in selected],[0,50,95,99,100]).tolist(),
 'ambiguous_identical_payloads':sum(v['identical_response_count']>1 for v in selected),
 'maximum_identical_readback_spread_ms':max(v['readback_spread_ns'] for v in selected)/1e6}
(a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))

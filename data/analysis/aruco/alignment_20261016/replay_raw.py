#!/usr/bin/env python3
"""Replay original hardware images with the committed production estimator.
Read-only with respect to source bags and deployment; all outputs are diagnostic.
Uses native OpenCV 4.2 on ML to match the recorded Jetson Python runtime.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
NATIVE=Path('/usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so')
spec=importlib.util.spec_from_file_location('cv2',str(NATIVE))
cv2=importlib.util.module_from_spec(spec);sys.modules['cv2']=cv2;spec.loader.exec_module(cv2)
import rosbag
from scipy.spatial.transform import Rotation
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from audit import matrix,interpolate,quantile


def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('bag',type=Path);parser.add_argument('--output',type=Path,required=True)
 parser.add_argument('--revision',default='c0a3548');parser.add_argument('--period',type=float,default=.033)
 args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
 snapshot=ROOT/'.build'/('alignment-'+args.revision)/'aruco_landing';snapshot.mkdir(parents=True,exist_ok=True)
 for name in ['__init__.py','geometry.py','physical_pad.py','pose_alignment.py']:
  (snapshot/name).write_bytes(subprocess.check_output(['git','-C',str(ROOT/'ws/aruco-landing/src/aruco_landing'),'show',args.revision+':src/aruco_landing/'+name]))
 sys.path.insert(0,str(snapshot.parent))
 from aruco_landing.physical_pad import PhysicalPadDetector,center_crop,inverse
 layout=yaml.safe_load(subprocess.check_output(['git','-C',str(ROOT/'ws/aruco-landing/src/aruco_landing'),'show',args.revision+':config/proposed_pad_1_layout_physical_20260920_runtime.yaml']))
 X=np.array(yaml.safe_load((ROOT/'stacks/aruco-landing-jetson/config/calibration/20260919/base_link_to_see3cam_optical_frame.yaml').read_text())['matrix_row_major']).reshape(4,4)
 offset=float(yaml.safe_load((ROOT/'stacks/aruco-landing-jetson/config/calibration/20260919/time_alignment.yaml').read_text())['image_to_body_offset_s'])
 detector=PhysicalPadDetector(layout,min_markers=1)
 detector.params.adaptiveThreshWinSizeMin=7;detector.params.adaptiveThreshWinSizeMax=27;detector.params.adaptiveThreshWinSizeStep=20
 cv2.setNumThreads(2)
 if detector.detector:detector.detector.setDetectorParameters(detector.params)
 gt=[];live=[];pads=[];rows=[];images=[];K=D=None;last=-1e30
 topics=['/landing/camera/camera_info','/landing/camera/image_raw','/vrpn_client_node/pure/pose','/landing/target_pose_camera','/landing/pad_pose_global']
 with rosbag.Bag(str(args.bag)) as bag:
  start=bag.get_start_time();begin=time.monotonic()
  for topic,m,bt in bag.read_messages(topics=topics):
   if topic=='/vrpn_client_node/pure/pose':gt.append((m.header.stamp.to_sec(),matrix(m.pose)))
   elif topic=='/landing/target_pose_camera':live.append((m.header.stamp.to_sec(),matrix(m.pose)))
   elif topic=='/landing/pad_pose_global':pads.append(matrix(m.pose))
   elif topic=='/landing/camera/camera_info':K=np.array(m.K).reshape(3,3);D=np.array(m.D)
   elif K is not None:
    stamp=m.header.stamp.to_sec()
    if stamp-last<args.period-.002:continue
    last=stamp
    image=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
    gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
    gray,cropK=center_crop(gray,K,720,720)
    result,corners,ids=detector.detect(gray,cropK,D)
    row={'stamp':stamp+offset,'image_stamp':stamp,'bag_s':bt.to_sec()-start,'ids':ids,
      'corners':{str(mid):np.asarray(c).reshape(4,2).tolist() for mid,c in zip(ids,corners)},'poses':{}}
    if result:row['poses']['combined']={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in result.items()}
    for mid in (0,1):
     selected=[(c,i) for c,i in zip(corners,ids) if i==mid]
     if not selected:continue
     r=detector.estimate([x[0] for x in selected],[x[1] for x in selected],cropK,D)
     if r:row['poses'][str(mid)]={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in r.items()}
    if len(images)<6 and row['bag_s']>=len(images)*2:
     color=cv2.cvtColor(image,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else image.copy()
     color,_=center_crop(color,K,720,720)
     images.append(color)
     cv2.imwrite(str(args.output/('raw_%02d.png'%(len(images)-1))),color)
    rows.append(row)
    if len(rows)%100==0:print('raw frames',len(rows),'bag_s',round(row['bag_s'],2),'wall_s',round(time.monotonic()-begin,1),flush=True)
 times=np.array([r['stamp'] for r in rows]);A,valid=interpolate(gt,times)
 live_times=np.array([r[0] for r in live]);live_mats=np.array([r[1] for r in live]);Y=pads[0]
 for row,a,ok in zip(rows,A,valid):
  if not ok:continue
  row['gt']=a.tolist();row['gt_body_height_m']=float(a[2,3])
  for key,r in row['poses'].items():
   Z=np.array(r['camera_from_pad']);relative=inverse(Z)@inverse(X);implied=a@X@Z;body=Y@relative
   r['implied_global_pad']=implied.tolist();r['body_global']=body.tolist()
   r['position_error_m']=float(np.linalg.norm(body[:3,3]-a[:3,3]));r['delta_xyz_m']=(body[:3,3]-a[:3,3]).tolist()
   r['implied_pad_euler_xyz_deg']=Rotation.from_matrix(implied[:3,:3]).as_euler('xyz',degrees=True).tolist()
  if row['poses'].get('combined') and len(live):
   j=int(np.argmin(abs(live_times-row['stamp'])))
   if abs(live_times[j]-row['stamp'])<1e-5:
    dz=inverse(live_mats[j])@np.array(row['poses']['combined']['camera_from_pad'])
    row['replay_vs_live']={'translation_m':float(np.linalg.norm(dz[:3,3])),'rotation_deg':float(np.rad2deg(Rotation.from_matrix(dz[:3,:3]).magnitude()))}
  if all(k in row['poses'] for k in ('0','1')):
   z0=np.array(row['poses']['0']['camera_from_pad']);z1=np.array(row['poses']['1']['camera_from_pad'])
   row['single_marker_pad_normal_disagreement_deg']=float(np.rad2deg(np.arccos(np.clip(z0[:3,2]@z1[:3,2],-1,1))))
 summary={'bag':str(args.bag),'opencv':cv2.__version__,'source_revision':args.revision,'sampled_frames':len(rows),'GT_paired_frames':int(valid.sum()),'frozen_pad_euler_xyz_deg':Rotation.from_matrix(Y[:3,:3]).as_euler('xyz',degrees=True).tolist(),'by_height':[]}
 for low,high in [(-1,.4),(.4,1.),(1.,1.7),(1.7,2.4)]:
  selected=[r for r in rows if low<=r.get('gt_body_height_m',-10)<high]
  group={'height_m':[low,high],'frames':len(selected),'poses':{}}
  for key in ['combined','0','1']:
   found=[r['poses'][key] for r in selected if key in r['poses']]
   if not found:continue
   group['poses'][key]={'n':len(found),'position_error_m_p50_p95_max':quantile([r['position_error_m'] for r in found]),'rms_px_p50_p95_max':quantile([r['rms_px'] for r in found]),'median_inferred_pad_euler_xyz_deg':np.median([r['implied_pad_euler_xyz_deg'] for r in found],axis=0).tolist()}
  group['single_marker_normal_difference_deg_p50_p95_max']=quantile([r['single_marker_pad_normal_disagreement_deg'] for r in selected if 'single_marker_pad_normal_disagreement_deg' in r])
  summary['by_height'].append(group)
 replay=[r['replay_vs_live'] for r in rows if 'replay_vs_live' in r]
 summary['replay_vs_live']={'n':len(replay),'position_m_p50_p95_max':quantile([r['translation_m'] for r in replay]),'angle_deg_p50_p95_max':quantile([r['rotation_deg'] for r in replay])}
 (args.output/'raw_replay.json').write_text(json.dumps({'summary':summary,'layout':layout,'body_from_camera':X.tolist(),'frozen_global_pad':Y.tolist(),'crop_K':cropK.tolist(),'D':D.tolist(),'rows':rows},indent=2)+'\n')
 (args.output/'raw_replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 print(json.dumps(summary,indent=2))

if __name__=='__main__':main()

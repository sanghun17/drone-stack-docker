#!/usr/bin/env python3
"""Recover recorded annotated PNGs and matched raw images for marker error review."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import rosbag
from scipy.spatial.transform import Rotation, Slerp
import yaml


def main(bag_path, full_bag, out):
    out.mkdir(parents=True,exist_ok=True)
    topics=['/vrpn_client_node/pure/pose','/landing/vision_pose_marker','/landing/vehicle_pose_pad',
            '/landing/pad_pose_global','/landing/estimator/inlier_ids','/landing/camera/camera_info']
    rows={t:[] for t in topics};camera_info=None
    with rosbag.Bag(str(bag_path)) as b:
        t0=b.get_start_time()
        for topic,m,t in b.read_messages(topics=topics):
            if topic.endswith('camera_info'):
                if camera_info is None:camera_info=dict(K=list(m.K),D=list(m.D),width=m.width,height=m.height)
                continue
            if topic.endswith('inlier_ids'):rows[topic].append((t.to_sec()-t0,list(m.data)));continue
            p=m.pose.pose if hasattr(m.pose,'pose') else m.pose
            rows[topic].append([t.to_sec()-t0,m.header.stamp.to_sec()-t0,p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    opt=np.array(rows[topics[0]]);mark=np.array(rows[topics[1]]);body=np.array(rows[topics[2]]);pad=np.array(rows[topics[3]])[0]
    interpolate=lambda ts:np.column_stack([np.interp(ts,opt[:,1],opt[:,k])for k in [2,3,4]])
    errors=np.linalg.norm(mark[:,2:5]-interpolate(mark[:,1]),axis=1)
    (out/'camera_info.json').write_text(json.dumps(camera_info,indent=2))
    snapshot=json.loads(bag_path.with_suffix('.calibration.json').read_text())
    mount=np.array(yaml.safe_load(snapshot['calibration/20260919/base_link_to_see3cam_optical_frame.yaml'])['matrix_row_major']).reshape(4,4)
    offset=yaml.safe_load(snapshot['calibration/20260919/time_alignment.yaml'])
    # The saved calibration explicitly contains the same correction used by the estimator.
    dt=float(yaml.safe_load(snapshot['calibration/20260919/base_link_to_see3cam_optical_frame.yaml'])['image_to_body_time_offset_s'])
    unique=np.r_[True,np.diff(opt[:,1])>0]
    orientations=Slerp(opt[unique,1],Rotation.from_quat(opt[unique,5:9]))
    ts=np.clip(mark[:,1],opt[0,1],opt[-1,1]);true_body=interpolate(ts)
    true_camera=true_body+orientations(ts).apply(mount[:3,3])
    relative=(true_camera-pad[2:5])@Rotation.from_quat(pad[5:9]).as_matrix()
    rotation_error=(orientations(ts).inv()*Rotation.from_quat(mark[:,5:9])).magnitude()*180/np.pi
    targets=[38.5,39.3,40,41,42,43,44,44.9,45.05,46.3,48,55,65,70,72,73,74,75,76,77,78,80,82,84,85,86,87,88]
    chosen=sorted(set(int(np.argmin(abs(mark[:,0]-t)))for t in targets))
    frames=[]
    for idx in chosen:
        frames.append(dict(pose_receipt_s=float(mark[idx,0]),pose_stamp_s=float(mark[idx,1]),raw_stamp_s=float(mark[idx,1]-dt),
            position_error_m=float(errors[idx]),rotation_error_deg=float(rotation_error[idx]),
            true_camera_pad_xyz_m=relative[idx].tolist(),true_camera_distance_m=float(np.linalg.norm(relative[idx])),
            inlier_ids=min(rows[topics[4]],key=lambda x:abs(x[0]-mark[idx,0]))[1]))
    best=[None]*len(frames);best_delta=np.full(len(frames),np.inf)
    desired=np.array([r['raw_stamp_s']for r in frames])
    with rosbag.Bag(str(bag_path)) as b:
        for _,m,t in b.read_messages(topics=['/landing/debug/image/compressed']):
            stamp=m.header.stamp.to_sec()-t0
            for i in np.flatnonzero(abs(desired-stamp)<best_delta):
                best_delta[i]=abs(desired[i]-stamp);best[i]=(stamp,bytes(m.data))
    for i,(entry,image) in enumerate(zip(frames,best)):
        name='t%06.2f_error%04.0fcm'%(entry['pose_receipt_s'],100*entry['position_error_m'])
        entry['name']=name;entry['detected_stamp_s']=image[0];entry['detected_match_dt_s']=float(best_delta[i])
        pixels=cv2.imdecode(np.frombuffer(image[1],np.uint8),cv2.IMREAD_COLOR)
        entry['detected_png']=str(out/(name+'_detected.png'));cv2.imwrite(entry['detected_png'],pixels)
    (out/'frames.json').write_text(json.dumps(frames,indent=2)+'\n')
    # Recover exact capture images from the arm-triggered full bag. Annotated
    # frames alone are unsuitable for a precise corner/reprojection analysis.
    raw=[None]*len(frames);distance=np.full(len(frames),np.inf)
    with rosbag.Bag(str(full_bag)) as b:
        for _,m,t in b.read_messages(topics=['/landing/camera/image_raw']):
            stamp=m.header.stamp.to_sec()-t0
            candidates=np.flatnonzero((abs(desired-stamp)<distance)&(abs(desired-stamp)<.08))
            if not len(candidates):continue
            channels=1 if m.encoding=='mono8' else 3
            a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*channels].reshape(m.height,m.width,channels)
            a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else (cv2.cvtColor(a,cv2.COLOR_GRAY2BGR) if channels==1 else a.copy())
            for i in candidates:distance[i]=abs(desired[i]-stamp);raw[i]=(stamp,a)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
    detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters())
    for i,(entry,sample) in enumerate(zip(frames,raw)):
        if sample is None:continue
        stamp,pixels=sample;entry['raw_match_dt_s']=float(distance[i]);entry['raw_png']=str(out/(entry['name']+'_raw.png'))
        cv2.imwrite(entry['raw_png'],pixels)
        gray=cv2.cvtColor(pixels,cv2.COLOR_BGR2GRAY);corners,ids,_=detector.detectMarkers(gray)
        metrics=[]
        for pts,mid in zip(corners,[] if ids is None else ids.ravel()):
            xy=pts.reshape(4,2);edge=np.linalg.norm(xy-np.roll(xy,-1,axis=0),axis=1)
            mask=np.zeros(gray.shape,np.uint8);cv2.fillConvexPoly(mask,np.round(xy).astype(np.int32),255)
            mask=cv2.erode(mask,np.ones((5,5),np.uint8))
            lap=cv2.Laplacian(gray,cv2.CV_64F);values=lap[mask>0]
            metrics.append(dict(id=int(mid),mean_edge_px=float(edge.mean()),min_edge_px=float(edge.min()),
                laplacian_variance=float(values.var()) if len(values) else None))
        entry['offline_detected_marker_metrics']=metrics
    (out/'frames.json').write_text(json.dumps(frames,indent=2)+'\n')
    tiles=[]
    for entry in frames:
        img=cv2.imread(entry['detected_png']);img=cv2.resize(img,(480,270));tile=np.full((322,480,3),245,np.uint8);tile[:270]=img
        cv2.putText(tile,'t=%.2fs err=%.2fm rot=%.1fdeg'%(entry['pose_receipt_s'],entry['position_error_m'],entry['rotation_error_deg']),(8,290),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,0),1)
        cv2.putText(tile,'distance=%.2fm inliers=%s'%(entry['true_camera_distance_m'],entry['inlier_ids']),(8,312),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,0,0),1)
        tiles.append(tile)
    while len(tiles)%3:tiles.append(np.full_like(tiles[0],255))
    sheet=np.vstack([np.hstack(tiles[i:i+3])for i in range(0,len(tiles),3)])
    cv2.imwrite(str(out/'contact_sheet.png'),sheet)
    # Error history allows recovery analysis without assuming the new frame is
    # trustworthy simply because its own reprojection fit is small.
    np.savez_compressed(out/'error_history.npz',receipt_s=mark[:,0],measurement_s=mark[:,1],position_error_m=errors,
        rotation_error_deg=rotation_error,true_camera_pad_xyz_m=relative)
    print(json.dumps(frames,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('bag',type=Path);p.add_argument('full_bag',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args();main(a.bag,a.full_bag,a.out)

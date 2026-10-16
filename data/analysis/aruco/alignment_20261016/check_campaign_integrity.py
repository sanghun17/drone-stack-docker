#!/usr/bin/env python3
"""Read-only bag/manifest/dual-webcam audit for a selected hardware campaign."""
import argparse,concurrent.futures,hashlib,json,subprocess,sys
from pathlib import Path
from collections import Counter
import numpy as np
import rosbag
import yaml
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[4]
CRITICAL=['/vrpn_client_node/pure/pose','/mavros/imu/data','/mavros/state',
          '/mavros/extended_state','/landing/trial/status','/landing/vision_pose_marker',
          '/landing/target_pose_camera','/landing/pad_pose_global','/landing/camera/camera_info',
          '/landing/debug/image/compressed','/landing/estimator/status','/flight_safety/state']


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(8*1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def video_check(path):
    probe=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries',
        'stream=codec_name,width,height,nb_frames,avg_frame_rate:format=duration,size',
        '-of','json',str(path)],capture_output=True,text=True,timeout=30,check=True)
    info=json.loads(probe.stdout);stream=info['streams'][0]
    decode=subprocess.run(['ffmpeg','-nostdin','-v','error','-xerror','-err_detect','explode',
        '-threads','1','-progress','pipe:1','-i',str(path),'-map','0:v:0','-an','-f','null','-'],
        capture_output=True,text=True,timeout=180)
    frames=[int(l.split('=',1)[1]) for l in decode.stdout.splitlines() if l.startswith('frame=')]
    result=dict(path=str(path),bytes=path.stat().st_size,duration_s=float(info['format']['duration']),
                stream=stream,decode_exit=decode.returncode,decode_stderr=decode.stderr.strip(),
                decoded_frames=frames[-1] if frames else 0)
    result['valid']=bool(result['duration_s']>0 and result['decoded_frames']>0 and decode.returncode==0
                         and result['decoded_frames']==int(stream['nb_frames']))
    return result


def comparison_notes(report):
    groups={}
    for r in report['successful_flights']:
        p=r['perception_parameters']
        windows=tuple(p.get(k,default) for k,default in [
            ('adaptive_threshold_min',7),('adaptive_threshold_max',27),('adaptive_threshold_step',20)])
        r['effective_threshold_min_max_step']=list(windows)
        groups.setdefault(windows,[]).append(r['selection']['index'])
    report['detector_configuration_groups']=[dict(min_max_step=list(k),route_indices=v) for k,v in groups.items()]
    report['standard_detector_threshold_min_max_step']=[3,23,10]
    report['comparison_note']='Current and subsequent pad experiments use the restored 3/13/23 windows. All 20 successful flights are retained in this campaign; recorded per-flight parameters remain available as provenance.'
    report['raw_route_indices']=[r['selection']['index'] for r in report['successful_flights'] if r['raw_image_frames']]
    report['capture_manifests_complete']=all(r['capture_status']=='complete' and not r['capture_errors'] for r in report['successful_flights'])
    report['topic_mean_hz_ranges']={t:np.percentile([r['topic_counts'][t]/r['duration_s'] for r in report['successful_flights']],[0,50,100]).tolist()
                                    for t in ['/vrpn_client_node/pure/pose','/mavros/imu/data','/landing/debug/image/compressed']}
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner',type=Path,required=True)
    p.add_argument('--set-file',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--recordings',type=Path,default=Path('/home/ml/drone-data/shared/results/webcam-recordings'))
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(a.owner/'src'))
    from aruco_landing.campaign import flight_record
    evidence_path=a.output/'jetson_evidence.json'
    evidence=json.loads(evidence_path.read_text()) if evidence_path.exists() and evidence_path.stat().st_size else dict(receipts={},progress={})
    assert sha(a.set_file)=='54f13331d55d4d822300cc617be7c925e3a068e7d200efee77fee10bb2bb39ff'
    expected=json.loads(a.set_file.read_text())['selections']
    records=[];errors=[]
    for bag in sorted(a.recordings.glob('flight_2026-10-16_1[67]*/*.bag')):
        if bag.parent.name<'flight_2026-10-16_16-47-11':continue
        try:
            r=flight_record(bag,a.owner)
            if r['set_sha256']!='54f13331d55d4d822300cc617be7c925e3a068e7d200efee77fee10bb2bb39ff' or r['pad']['key']!='proposed1':continue
            manifest=bag.with_suffix('.runtime_manifest.yaml');doc=yaml.safe_load(manifest.read_text())
            receipt=evidence['receipts'].get(bag.stem+'.verified',{})
            r['receipt_bag_matches']=(receipt.get('sha256',{}).get(bag.name)==r['bag_sha256']) if receipt else None
            r['manifest_sha256']=sha(manifest)
            r['receipt_manifest_matches']=(receipt.get('sha256',{}).get(manifest.name)==r['manifest_sha256']) if receipt else None
            r['recorded_bag_sha_matches']=doc['recorder']['bag_artifact'].get('sha256')==r['bag_sha256']
            r['rosbag_returncode']=doc['recorder'].get('rosbag_returncode')
            r['capture_status']=doc.get('capture_status');r['capture_errors']=doc.get('capture_errors')
            params=doc['snapshots']['start']['ros']['parameters']
            r['perception_parameters']=params.get('physical_pad_estimator',{})
            with rosbag.Bag(str(bag)) as b:
                r['duration_s']=b.get_end_time()-b.get_start_time()
                r['topic_counts']={k:v.message_count for k,v in b.get_type_and_topic_info().topics.items()}
                r['missing_critical_topics']=[t for t in CRITICAL if not r['topic_counts'].get(t)]
                # Read every serialized payload to exercise all compressed chunks.
                observed=Counter()
                for topic,msg,t in b.read_messages(raw=True):observed[topic]+=1
                r['all_messages_read']=dict(observed)==r['topic_counts']
                r['arm_span_s']=None
                armed=[]
                for _,m,t in b.read_messages(topics=['/mavros/state']):
                    if m.armed:armed.append(t.to_sec())
                if armed:r['arm_span_s']=max(armed)-min(armed)
            r['raw_image_frames']=r['topic_counts'].get('/landing/camera/image_raw',0)
            r['bag_integrity_ok']=bool(r['all_messages_read'] and not r['missing_critical_topics']
                and r['recorded_bag_sha_matches'] and r['receipt_bag_matches'] is not False
                and r['receipt_manifest_matches'] is not False and r['rosbag_returncode']==0)
            records.append(r)
            print(bag.stem,r['selection']['index'],r['outcome'],'bag_ok',r['bag_integrity_ok'],flush=True)
        except Exception as e:errors.append(dict(bag=str(bag),error=str(e)));print('ERROR',bag,e,flush=True)
    successes={}
    for r in records:
        if r['outcome']=='success':successes.setdefault(r['selection']['index'],[]).append(r)
    selected=[]
    for i in range(20):
        if len(successes.get(i,[]))!=1:
            errors.append(dict(index=i,error='expected exactly one successful recording',count=len(successes.get(i,[]))));continue
        r=successes[i][0]
        if any(r['selection'][k]!=expected[i][k] for k in ('type','reference_id')):
            errors.append(dict(index=i,error='reference identity mismatch'))
        selected.append(r)
    jobs=[]
    for r in selected:
        root=Path(r['bag_path']).parent
        for camera in ('cam1','cam2'):
            path=root/camera/(r['session']+'_'+camera+'.mp4')
            jobs.append((r,camera,path))
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        pending={pool.submit(video_check,path):(r,camera,path) for r,camera,path in jobs}
        for future in concurrent.futures.as_completed(pending):
            r,camera,path=pending[future]
            try:result=future.result()
            except Exception as e:result=dict(path=str(path),valid=False,error=str(e))
            r.setdefault('webcams',{})[camera]=result
            print('VIDEO',r['selection']['index'],camera,result.get('valid'),flush=True)
    for page in range(2):
        fig,axes=plt.subplots(5,4,figsize=(12,8))
        for j in range(10):
            index=page*10+j
            for c,camera in enumerate(('cam1','cam2')):
                ax=axes[j//2,(j%2)*2+c];ax.axis('off')
                if index>=len(selected):continue
                r=selected[index];v=r['webcams'][camera]
                if not v.get('valid'):continue
                capture=cv2.VideoCapture(v['path']);capture.set(cv2.CAP_PROP_POS_MSEC,v['duration_s']*500)
                ok,frame=capture.read();capture.release()
                if ok:
                    ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
                    v['midframe_mean_std']=[float(frame.mean()),float(frame.std())]
                else:errors.append(dict(index=index,camera=camera,error='midframe unavailable'))
                ax.set_title('route %02d / %s'%(index+1,camera),fontsize=9)
        fig.tight_layout();fig.savefig(a.output/('webcams_%02d.png'%(page+1)),dpi=160);plt.close(fig)
    videos=[v for r in selected for v in r.get('webcams',{}).values()]
    report=dict(project='aruco-landing',pad='proposed1',date='2026-10-16',successful_routes=len(selected),
                all_20_present=len(selected)==20,bag_integrity_ok=all(r['bag_integrity_ok'] for r in selected),
                webcam_files=len(videos),all_webcams_decode=bool(len(videos)==40 and all(v.get('valid') for v in videos)),
                first_route_raw_present=bool(successes.get(0) and successes[0][0]['raw_image_frames']>0),
                errors=errors,successful_flights=selected,other_attempts=[r for r in records if r['outcome']!='success'],
                jetson_progress=evidence['progress'],
                checksum_scope='ML bag compared with Jetson-generated finalized manifest SHA256. Receiver receipts additionally compared when fetched; Jetson can be offline during this audit.')
    report['passed']=bool(report['all_20_present'] and report['bag_integrity_ok'] and report['all_webcams_decode']
                           and report['first_route_raw_present'] and not errors)
    comparison_notes(report)
    (ROOT/'data/manifests/aruco-campaign-integrity-20261016-proposed1.json').write_text(json.dumps(report,indent=2)+'\n')
    print('SUMMARY',json.dumps({k:v for k,v in report.items() if k not in ('successful_flights','other_attempts','jetson_progress')}),flush=True)
    if not report['passed']:raise RuntimeError('Campaign integrity audit failed; see manifest')


if __name__=='__main__':main()

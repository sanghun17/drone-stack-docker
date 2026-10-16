#!/usr/bin/env python3
"""Read-only B2 bag scan and full decode of both external camera recordings."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import rosbag
import yaml


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def audit(path):
    row = dict(session=path.name, errors=[], videos=[])
    bag = path/(path.name+'.bag')
    manifest = path/(path.name+'.runtime_manifest.yaml')
    try:
        m = yaml.safe_load(manifest.read_text())
        row['manifest_capture_status'] = m.get('capture_status')
        row['manifest_capture_errors'] = m.get('capture_errors')
        row['manifest_sha256'] = sha(manifest)
        with rosbag.Bag(str(bag)) as b:
            info = b.get_type_and_topic_info().topics
            row['bag_duration_s'] = b.get_end_time()-b.get_start_time()
            row['bag_messages'] = b.get_message_count()
            row['raw_camera_frames'] = info['/landing/camera/image_raw'].message_count if '/landing/camera/image_raw' in info else 0
            row['debug_camera_frames'] = info['/landing/debug/image/compressed'].message_count if '/landing/debug/image/compressed' in info else 0
            pad = None; indices=set(); phases=[]; last=None
            for _,msg,_ in b.read_messages(topics=['/landing/trial/status']):
                d=json.loads(msg.data); last=d
                trajectory=d.get('trajectory_set') or {}
                pad=trajectory.get('pad') or pad
                selection=trajectory.get('current') or {}
                if selection.get('index') is not None:
                    indices.add(selection['index'])
                if not phases or phases[-1] != d['phase']:
                    phases.append(d['phase'])
            row.update(pad=pad, reference_indices=sorted(indices), phases=phases,
                       last_reason=None if last is None else last.get('reason'))
            # Read every payload to detect incomplete chunks, not just a valid index.
            row['bag_scanned_messages'] = sum(1 for _ in b.read_messages(raw=True))
            if row['bag_scanned_messages'] != row['bag_messages']:
                row['errors'].append('bag message count mismatch')
        row['bag_bytes']=bag.stat().st_size
        row['bag_sha256']=sha(bag)
    except Exception as e:
        row['errors'].append('bag/manifest: '+repr(e))
    for cam in ['cam1','cam2']:
        video=path/cam/(path.name+'_'+cam+'.mp4')
        v=dict(camera=cam,path=str(video))
        try:
            probe=subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_streams','-show_format','-of','json',str(video)],capture_output=True,text=True,check=True)
            meta=json.loads(probe.stdout);stream=meta['streams'][0]
            v.update(bytes=video.stat().st_size,duration_s=float(meta['format']['duration']),
                frames=int(stream.get('nb_frames',0)),width=stream['width'],height=stream['height'])
            decode=subprocess.run(['ffmpeg','-nostdin','-hide_banner','-v','error','-xerror','-threads','1','-i',str(video),'-map','0:v:0','-an','-filter_threads','1','-threads','1','-f','null','-'],capture_output=True,text=True)
            v['full_decode_ok']=decode.returncode==0
            v['decode_errors']=decode.stderr[-2000:]
            v['sha256']=sha(video)
            if decode.returncode:
                row['errors'].append(cam+' full decode failed')
            if v['duration_s']+1 < row.get('bag_duration_s',0):
                row['errors'].append(cam+' shorter than bag by >1s')
        except Exception as e:
            row['errors'].append(cam+': '+repr(e))
        row['videos'].append(v)
    return row


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    paths=[x for x in sorted(a.base.glob('flight_2026-10-16_*')) if x.name >= 'flight_2026-10-16_19-58-08']
    rows=[]
    with ThreadPoolExecutor(max_workers=4) as pool:
        for f in as_completed([pool.submit(audit,x) for x in paths]):
            row=f.result();rows.append(row)
            print(row['session'],row.get('reference_indices'),row['errors'],flush=True)
    rows.sort(key=lambda r:r['session'])
    report=dict(scope='Read-only bag full-payload scan, SHA256, camera MP4 full video decode; no recording changes',
        sessions=len(rows),b2_sessions=sum((r.get('pad') or {}).get('key')=='b2' for r in rows),
        video_files=sum(v.get('full_decode_ok',False) for r in rows for v in r['videos']),
        errors=sum(bool(r['errors']) for r in rows),
        covered_indices=sorted({i for r in rows for i in r.get('reference_indices',[])}),rows=rows)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))


if __name__=='__main__':
    main()

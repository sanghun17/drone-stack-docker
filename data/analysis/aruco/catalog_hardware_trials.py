#!/usr/bin/env python3
"""Catalog finalized flights by pad/reference; preserve original bags and videos."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[3]


def atomic_json(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.tmp')
    with temp.open('w') as stream:
        json.dump(data,stream,indent=2,ensure_ascii=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    os.replace(temp,path)


def selected_successes(records):
    selected={}
    for record in sorted(records,key=lambda r:r['session']):
        if record['outcome']!='success':continue
        p=record['pad'];s=record['selection']
        key=(p['key'],p['layout_sha256'],record['set_sha256'],s['index'])
        selected[key]=record
    return list(selected.values())


def catalog_once(source_root,output_root,owner_source,since):
    sys.path.insert(0,str(owner_source/'src'))
    from aruco_landing.campaign import flight_record
    cache_path=output_root/'scan-cache.json'
    try:cache=json.loads(cache_path.read_text())
    except (OSError,ValueError):cache={}
    updated={};records=[];skipped=[]
    for bag in sorted(source_root.glob('flight_*/*.bag')):
        if bag.stem<'flight_'+since:continue
        manifest=bag.with_suffix('.runtime_manifest.yaml')
        if not manifest.exists():continue
        key=str(bag.resolve());stat=bag.stat();ms=manifest.stat()
        fingerprint=[stat.st_size,stat.st_mtime_ns,ms.st_size,ms.st_mtime_ns]
        previous=cache.get(key,{})
        if previous.get('fingerprint')==fingerprint:
            item=previous
        else:
            try:item=dict(fingerprint=fingerprint,record=flight_record(bag,owner_source))
            except Exception as error:item=dict(fingerprint=fingerprint,error=str(error))
        updated[key]=item
        if 'record' in item:records.append(item['record'])
        else:skipped.append(dict(bag=key,reason=item['error']))
    selected=selected_successes(records);selected_sessions={r['session'] for r in selected}
    wanted=set();groups={}
    for record in selected:
        p=record['pad'];s=record['selection']
        destination=output_root/p['key']/('layout_'+p['layout_sha256'][:12])/('set_'+record['set_sha256'][:12])/s['type']/('ref_%06d'%s['reference_id'])/record['session']
        destination.mkdir(parents=True,exist_ok=True);wanted.add(destination)
        original=Path(record['bag_path']).parent
        links={'flight.bag':Path(record['bag_path']),'runtime_manifest.yaml':Path(record['manifest_path'])}
        for camera in ['cam1','cam2']:
            if (original/camera).is_dir():links[camera]=original/camera
        for name,target in links.items():
            link=destination/name
            if link.is_symlink():
                if link.resolve()!=target.resolve():raise ValueError('unexpected catalog link: '+str(link))
            elif link.exists():raise ValueError('preserving non-link catalog entry: '+str(link))
            else:link.symlink_to(target,target_is_directory=target.is_dir())
        atomic_json(destination/'attempt.json',dict(catalog_owner='aruco_hardware_trials_v1',**record))
        key=p['key']+'/'+p['layout_sha256']+'/'+record['set_sha256']
        groups.setdefault(key,dict(pad=p,set_sha256=record['set_sha256'],completed_indices=[]))['completed_indices'].append(s['index'])
    # Remove only old generated views, never targets or unknown user files.
    for metadata in output_root.glob('*/layout_*/set_*/*/ref_*/*/attempt.json'):
        folder=metadata.parent
        if folder in wanted:continue
        data=json.loads(metadata.read_text())
        if data.get('catalog_owner')!='aruco_hardware_trials_v1':continue
        entries=list(folder.iterdir())
        if any(p!=metadata and not p.is_symlink() for p in entries):continue
        for path in entries:path.unlink()
        folder.rmdir()
    summaries=[]
    for group in groups.values():
        group['completed_indices']=sorted(group['completed_indices']);group['completed']=len(group['completed_indices'])
        summaries.append(group)
    index=dict(schema_version=1,selection_policy='latest confirmed success per pad layout / trajectory set / reference',campaigns=summaries,flights=selected)
    atomic_json(output_root/'index.json',index)
    atomic_json(output_root/'history.json',dict(flights=records,excluded_from_active=[dict(session=r['session'],outcome=r['outcome']) for r in records if r['session'] not in selected_sessions],skipped=skipped))
    atomic_json(cache_path,updated)
    return dict(selected_successes=len(selected),excluded_attempts=len(records)-len(selected),campaigns=summaries,skipped=len(skipped))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--owner-source',type=Path,default=ROOT/'ws/aruco-landing/src/aruco_landing')
    parser.add_argument('--since',default='2026-10-15')
    parser.add_argument('--watch',action='store_true')
    parser.add_argument('--interval',type=float,default=10.)
    args=parser.parse_args();last=None
    while True:
        try:
            result=catalog_once(args.source_root,args.output_root,args.owner_source,args.since)
            if result!=last:print(json.dumps(result,ensure_ascii=False),flush=True);last=result
        except Exception as error:
            if not args.watch:raise
            print('catalog scan failed: '+str(error),file=sys.stderr,flush=True)
        if not args.watch:return
        time.sleep(max(1.,args.interval))

if __name__=='__main__':main()

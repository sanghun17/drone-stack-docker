#!/usr/bin/env python3
"""Archive an explicit payload plan; remove sources only after all copies verify."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile
import time
from urllib.parse import quote


def digest(path):
    h=hashlib.sha256();size=path.stat().st_size;offset=0;retries=0
    # GVFS can return EINVAL on oversized/EOF reads. Bound reads to the exact
    # remaining length and reopen at the verified offset on transient errors.
    while offset<size:
        try:
            with path.open('rb',buffering=0) as f:
                if offset:f.seek(offset)
                while offset<size:
                    block=f.read(min(65536,size-offset))
                    if not block:raise OSError('Premature EOF: '+str(path))
                    h.update(block);offset+=len(block)
        except OSError:
            retries+=1
            if retries>8:raise
            time.sleep(1)
    if path.stat().st_size!=size:raise RuntimeError('File size changed while hashing')
    return h.hexdigest()


def unchanged(entry, checksum=None):
    p=Path(entry['source']);s=p.stat()
    if p.is_symlink() or not p.is_file() or (s.st_size,s.st_mtime_ns)!=(entry['size'],entry['mtime_ns']):
        raise RuntimeError('Source changed: '+str(p))
    if checksum is not None and digest(p)!=checksum:
        raise RuntimeError('Source checksum changed: '+str(p))


def write_json(path,value):
    payload=(json.dumps(value,indent=2)+'\n').encode()
    with path.open('xb') as f:f.write(payload)
    if path.read_bytes()!=payload:raise RuntimeError('Metadata readback failed: '+str(path))


def transfer(src,dst):
    """Exclusive destination creation; return source hash after NAS readback."""
    if dst.exists():raise FileExistsError(dst)
    dst.parent.mkdir(parents=True,exist_ok=True)
    part=dst.with_name(dst.name+'.partial')
    if part.exists():
        checksum=digest(src)
        if part.stat().st_size!=src.stat().st_size or digest(part)!=checksum:
            raise RuntimeError('Incomplete partial copy needs inspection: '+str(part))
        part.rename(dst)
        return checksum
    share=Path('/run/user/1000/gvfs/smb-share:server=10.74.22.95,share=research')
    uri='smb://10.74.22.95/research/'+quote(part.relative_to(share).as_posix(),safe='/')
    h=hashlib.sha256()
    process=subprocess.Popen(['gio','save','--create',uri],stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        with src.open('rb') as f:
            for block in iter(lambda:f.read(4*1024*1024),b''):
                h.update(block);process.stdin.write(block)
        process.stdin.close();process.stdin=None
        _,error=process.communicate()
        if process.returncode:raise RuntimeError(error.decode(errors='replace'))
    except BaseException:
        if process.poll() is None:process.kill()
        process.wait();raise
    if part.stat().st_size!=src.stat().st_size or digest(part)!=h.hexdigest():
        raise RuntimeError('NAS content mismatch; source retained: '+str(src))
    part.rename(dst)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan',type=Path)
    parser.add_argument('--resume',action='store_true',help='Reverify an existing archive of exactly this plan')
    a=parser.parse_args();plan=json.loads(a.plan.read_text());audit=a.plan.parent
    destination=Path(plan['destination']);root=Path(plan['local_root'])
    destination.mkdir(parents=True,exist_ok=a.resume)
    receipts=destination/'verification';receipts.mkdir(exist_ok=a.resume)
    if a.resume:
        if json.loads((destination/'plan.json').read_text())!=plan:raise RuntimeError('Archive plan differs')
    else:write_json(destination/'plan.json',plan)
    # A portable metadata snapshot includes relative iteration symlinks, configs,
    # results and plots. Large payloads retain the same root-relative paths.
    metadata=audit/'metadata.tar.gz'
    if not a.resume:
        with tarfile.open(metadata,'x:gz',dereference=False) as archive:
            for name in plan['batches']:
                for parent in ('flight_logs','data/results'):
                    p=root/parent/name
                    if p.exists():archive.add(p,arcname=str(p.relative_to(root)),
                        filter=lambda info:None if info.name.endswith('.bag') else info)
        metadata_hash=transfer(metadata,destination/metadata.name)
        write_json(audit/'metadata-verified.json',dict(sha256=metadata_hash,bytes=metadata.stat().st_size))
    else:
        metadata_hash=digest(metadata)
        if digest(destination/metadata.name)!=metadata_hash:raise RuntimeError('Metadata archive differs')
    completed=[]
    for i,entry in enumerate(plan['entries'],1):
        unchanged(entry)
        print(json.dumps(dict(state='copying',index=i,total=len(plan['entries']),source=entry['relative'],bytes=entry['size'])),flush=True)
        receipt_path=audit/('%03d.verified.json'%i)
        if a.resume and receipt_path.exists():
            receipt=json.loads(receipt_path.read_text())
            if receipt['entry']!=entry or digest(Path(entry['destination']))!=receipt['sha256']:
                raise RuntimeError('Existing receipt/copy differs')
            unchanged(entry,receipt['sha256'])
        else:
            checksum=transfer(Path(entry['source']),Path(entry['destination']))
            unchanged(entry)
            receipt=dict(entry=entry,sha256=checksum,verified_at=time.time())
            write_json(receipt_path,receipt)
            write_json(receipts/('%03d.json'%i),receipt)
        completed.append(receipt)
    write_json(destination/'VERIFIED.json',dict(files=len(completed),bytes=plan['total_bytes'],metadata_sha256=metadata_hash))
    # All source records and all remote receipts are intact before removal starts.
    for receipt in completed:
        unchanged(receipt['entry'],receipt['sha256'])
        if Path(receipt['entry']['destination']).stat().st_size!=receipt['entry']['size']:
            raise RuntimeError('NAS copy changed before source removal')
    for i,receipt in enumerate(completed,1):
        entry=receipt['entry'];src=Path(entry['source'])
        unchanged(entry)
        # Retain a discoverable location/byte-identity receipt beside local metrics.
        write_json(src.with_suffix(src.suffix+'.nas.json'),receipt)
        src.unlink()
        write_json(audit/('%03d.moved.json'%i),receipt)
    result=dict(state='complete',files=len(completed),bytes=plan['total_bytes'],destination=str(destination),completed_at=time.time())
    write_json(destination/'COMPLETE.json',result);write_json(audit/'COMPLETE.json',result)
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()

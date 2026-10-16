#!/usr/bin/env python3
"""Compare deployed PnP implementations without ROS publishers or flight control.

Default: real bad-quad fixtures and synthetic normal multi/single-marker frames.
With --observations: evaluate cached raw corners against frozen recorded GT.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import yaml


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--owner', type=Path, required=True)
    p.add_argument('--baseline-owner', type=Path, required=True)
    p.add_argument('--observations', type=Path)
    p.add_argument('--extrinsic', type=Path)
    p.add_argument('--loops', type=int, default=30)
    a = p.parse_args()
    sys.path.insert(0, str(a.owner/'src'))
    from aruco_landing.physical_pad import PhysicalPadDetector, inverse
    spec = importlib.util.spec_from_file_location('baseline_physical_pad', a.baseline_owner/'src/aruco_landing/physical_pad.py')
    old = importlib.util.module_from_spec(spec); spec.loader.exec_module(old)
    layout = yaml.safe_load((a.owner/'config/baseline_b2_700mm_runtime.yaml').read_text())
    detectors = {'baseline':old.PhysicalPadDetector(layout), 'ransac':PhysicalPadDetector(layout)}
    fixture = json.loads((a.owner/'test/fixtures/b2_bad_quads_20261016.json').read_text())
    K, D = np.array(fixture['K']), np.array(fixture['D'])
    frames = fixture['frames']
    for ids in [[1,2,3,4,19,43], [1,2,3,4,19], [3,4,19], [43]]:
        c = {str(mid):cv2.projectPoints(detectors['ransac'].models[mid], np.array([2.8,.08,-.1]), np.array([.02,-.01,1.5]), K, D)[0].reshape(4,2).tolist() for mid in ids}
        frames.append(dict(ids=ids,corners=c))
    Y = X = None
    if a.observations:
        if not a.extrinsic:
            p.error('--observations requires --extrinsic')
        data = json.loads(a.observations.read_text())
        K, D, Y = np.array(data['K']), np.array(data['D']), np.array(data['Y0'])
        X = np.array(yaml.safe_load(a.extrinsic.read_text())['matrix_row_major']).reshape(4,4)
        frames = [dict(r, ids=list(map(int,r['corners']))) for r in data['rows']]
    cv2.setNumThreads(1)
    prepared = [(r,[np.array(r['corners'][str(mid)],np.float32).reshape(1,4,2) for mid in r['ids']]) for r in frames]
    output = dict(opencv=cv2.__version__,frames=len(frames),loops=a.loops,
                  scope='PnP only; no image detection, ROS, calibration fitting, or control',methods={})
    per_frame = {}
    for name, det in detectors.items():
        for r, corners in prepared[:4]:
            det.estimate(corners,r['ids'],K,D)
        durations=[]; errors=[]; accepted=0; records=[]
        for _ in range(a.loops):
            for r,corners in prepared:
                start=time.perf_counter();res=det.estimate(corners,r['ids'],K,D)
                durations.append((time.perf_counter()-start)*1000)
                accepted+=res is not None
                record=dict(bag_s=r.get('bag_s'),valid=res is not None)
                if res is not None:
                    record['consensus']=res.get('consensus')
                    if Y is not None:
                        T=Y@inverse(res['camera_from_pad'])@inverse(X)
                        e=float(np.linalg.norm(T[:3,3]-np.array(r['gt'])[:3,3]))
                        errors.append(e);record['error_m']=e
                records.append(record)
        output['methods'][name]=dict(valid=accepted,calls=len(durations),
            elapsed_ms_p50_p95_p99_max=np.quantile(durations,[.5,.95,.99,1.]).tolist(),
            over20cm=sum(e>.2 for e in errors),
            error_m_p50_p95_max=np.quantile(errors,[.5,.95,1.]).tolist() if errors else None)
        per_frame[name]=records
    if Y is not None:
        output['rows']=per_frame
    print(json.dumps(output,indent=2))


if __name__ == '__main__':
    main()

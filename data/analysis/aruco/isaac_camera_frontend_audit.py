#!/usr/bin/env python3
"""Replay saved landing images through CPU and CUDA detectors and shared PnP."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import sys
import cv2
import numpy as np
import torch
import yaml

ROOT=Path(__file__).resolve().parents[3]
ARUCO=ROOT/'ws/aruco-landing/src/aruco_landing'
sys.path.insert(0,str(ARUCO/'src'))
sys.path.insert(0,str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from aruco_landing.physical_pad import PhysicalPadDetector, inverse
from aruco_landing.gpu_opencv import GpuOpenCVDetector
from pad_scene import metric_pad_manifest


def audit(inputs, output, allow_frontend_change=False):
    cv2.setNumThreads(1)
    library=Path(os.environ['ARUCO_OPENCV_CUDA_LIBRARY'])
    cases=[]
    for directory in inputs:
        manifest=json.loads((directory/'manifest.json').read_text())
        cfg=manifest['config']
        if cfg['detector_backend']!='gpu-opencv-compat': raise ValueError('GPU capture required')
        if not allow_frontend_change and hashlib.sha256(library.read_bytes()).hexdigest()!=manifest['gpu_detector_library_sha256']:
            raise ValueError('replay library differs from capture')
        if not allow_frontend_change and hashlib.sha256((ARUCO/'src/aruco_landing/gpu_opencv.py').read_bytes()).hexdigest()!=manifest['gpu_compatibility_sources_sha256']['gpu_opencv.py']:
            raise ValueError('replay source differs from capture; use --allow-frontend-change for correction audits')
        base=ROOT/'stacks/aruco-landing-isaac-x86' if cfg.get('pad_manifest_root')=='stack' else ARUCO
        pad_path=base/cfg['pad_manifest']
        if hashlib.sha256(pad_path.read_bytes()).hexdigest()!=manifest['pad_sha256']:
            raise ValueError('pad model differs from capture')
        model=metric_pad_manifest(yaml.safe_load(pad_path.read_text()),cfg['initial_protocol']['pad_side_m'])
        pose=PhysicalPadDetector(model)
        gpu=GpuOpenCVDetector(pose.dictionary,pose.params,str(library))
        cam=cfg['camera']
        K=np.array([[cam['fx'],0.,cam['cx']],[0.,cam['fy'],cam['cy']],[0.,0.,1.]])
        entries=json.loads((directory/'camera-corpus.json').read_text())
        def cpu_image(image):
            detector=cv2.aruco.ArucoDetector(pose.dictionary,pose.params)
            corners,ids,_=detector.detectMarkers(image)
            return corners,[] if ids is None else ids.flatten().tolist()
        with ThreadPoolExecutor(max_workers=8) as pool:
            for start in range(0,len(entries),16):
                part=entries[start:start+16]
                images=[]
                for entry in part:
                    path=directory/'camera-samples'/Path(entry['path']).name
                    if hashlib.sha256(path.read_bytes()).hexdigest()!=entry['file_sha256']:
                        raise ValueError('saved PNG checksum mismatch')
                    image=cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
                    if image is None or hashlib.sha256(image.tobytes()).hexdigest()!=entry['gray_sha256']:
                        raise ValueError('grayscale pixels differ from capture')
                    images.append(image)
                resident=torch.from_numpy(np.stack(images)).cuda()
                reference=list(pool.map(cpu_image,images))
                actual,_=gpu.detect_gray(resident)
                for entry,(q,ids),(rq,rids) in zip(part,actual,reference):
                    equal=ids==rids
                    corner=None if not ids or not equal else float(max(np.max(np.abs(a-b)) for a,b in zip(q,rq)))
                    def estimate(corners,decoded):
                        try: result=pose.estimate(corners,decoded,K,np.zeros(5))
                        except cv2.error: result=None
                        return None if result is None else inverse(result['camera_from_pad'])
                    a,b=estimate(rq,rids),estimate(q,ids)
                    truth=np.array(entry['gt_pad_from_camera'])
                    delta=None if a is None or b is None else float(np.linalg.norm(a[:3,3]-b[:3,3]))
                    angle=None if a is None or b is None else float(np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2,-1.,1.))))
                    validity_equal=(a is None)==(b is None)
                    captured=entry['main_estimated_pad_from_camera']
                    replay=None if captured is None or b is None else float(np.linalg.norm(np.array(captured)[:3,3]-b[:3,3]))
                    reference_passed=(equal and (corner is None or corner<=1e-3) and validity_equal
                            and (delta is None or delta<=.03) and (angle is None or angle<=5.))
                    capture_passed=(entry['main_gpu_ids']==ids and (captured is None)==(b is None)
                            and (replay is None or replay<=1e-6))
                    cases.append(dict(configuration=cfg['configuration_label'],image=Path(entry['path']).name,
                        gray_sha256=entry['gray_sha256'],camera_batch=entry['camera_batch'],environment=entry['environment'],
                        cpu_ids=rids,gpu_ids=ids,ordered_ids_equal=equal,corner_max_delta_px=corner,
                        cpu_pose_valid=a is not None,gpu_pose_valid=b is not None,
                        pose_position_delta_m=delta,pose_rotation_delta_deg=angle,
                        cpu_gt_position_error_m=None if a is None else float(np.linalg.norm(a[:3,3]-truth[:3,3])),
                        gpu_gt_position_error_m=None if b is None else float(np.linalg.norm(b[:3,3]-truth[:3,3])),
                        main_ids_replayed=entry['main_gpu_ids']==ids,main_position_replay_delta_m=replay,
                        main_pose_validity_equal=(captured is None)==(b is None),
                        reference_passed=reference_passed,capture_replay_passed=capture_passed,
                        passed=reference_passed and capture_passed))
        gpu.close()
        print(cfg['configuration_label'],len(entries),'images audited',flush=True)
    result=dict(images=len(cases),opencv_version=cv2.__version__,torch_version=torch.__version__,
        library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
        inputs=[str(p) for p in inputs],cases=cases,
        passed=sum(c['passed'] for c in cases),failed=sum(not c['passed'] for c in cases),
        reference_failed=sum(not c['reference_passed'] for c in cases),
        reference_gate_passed=all(c['reference_passed'] for c in cases),
        capture_replay_failed=sum(not c['capture_replay_passed'] for c in cases),
        pose_validity_mismatches=sum(c['cpu_pose_valid']!=c['gpu_pose_valid'] for c in cases),
        main_pose_validity_mismatches=sum(not c['main_pose_validity_equal'] for c in cases),
        decoder_mode='cell ratios' if hasattr(cv2.aruco.DetectorParameters(),'validBitIdThreshold') else 'binary majority',
        replay_source_sha256=hashlib.sha256((ARUCO/'src/aruco_landing/gpu_opencv.py').read_bytes()).hexdigest(),
        capture_source_sha256=[json.loads((p/'manifest.json').read_text())['gpu_compatibility_sources_sha256']['gpu_opencv.py'] for p in inputs],
        frontend_change_allowed=allow_frontend_change,
        capture_library_sha256=[json.loads((p/'manifest.json').read_text())['gpu_detector_library_sha256'] for p in inputs],
        ordered_ids_mismatches=sum(not c['ordered_ids_equal'] for c in cases),
        max_corner_delta_px=max((c['corner_max_delta_px'] or 0 for c in cases),default=0),
        max_pose_position_delta_m=max((c['pose_position_delta_m'] or 0 for c in cases),default=0),
        main_replay_ids_mismatches=sum(not c['main_ids_replayed'] for c in cases),
        max_main_position_replay_delta_m=max((c['main_position_replay_delta_m'] or 0 for c in cases),default=0),
        max_cpu_gt_position_error_m=max((c['cpu_gt_position_error_m'] or 0 for c in cases),default=0),
        max_gpu_gt_position_error_m=max((c['gpu_gt_position_error_m'] or 0 for c in cases),default=0),
        scope='Identical saved landing grayscale images; ordered IDs, corners and shared PnP; sampled captures, not exhaustive universal equivalence.')
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='cases'},indent=2))
    return result['failed']==0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',nargs='+',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--allow-frontend-change',action='store_true',
        help='audit a corrected frontend; captured-output reproduction stays a separate strict gate')
    args=parser.parse_args()
    if not audit(args.inputs,args.output,args.allow_frontend_change): raise SystemExit(1)


if __name__=='__main__': main()

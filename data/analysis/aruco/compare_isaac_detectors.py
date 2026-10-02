#!/usr/bin/env python3
"""Compare CPU/CUDA ArUco on saved static Isaac optical-smoke images.

Uses the smoke's recorded seed and camera mount to reconstruct its static
ground truth. Timings exclude rendering, disk reads and host-to-device upload.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import torch
import yaml

from aruco_landing.batch_landing import initial_condition
from aruco_landing.gpu_aruco import GpuArucoDetector
from aruco_landing.physical_pad import PhysicalPadDetector, inverse
from aruco_landing.pose_alignment import pose_matrix


def pose_error(estimate, truth):
    if estimate is None:
        return None
    actual = inverse(estimate['camera_from_pad'])
    rotation = truth[:3,:3].T @ actual[:3,:3]
    return dict(translation_m=float(np.linalg.norm(actual[:3,3]-truth[:3,3])),
                rotation_deg=math.degrees(math.acos(float(np.clip((np.trace(rotation)-1)/2,-1.,1.)))))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,nargs='+',required=True)
    parser.add_argument('--aruco-root',type=Path,required=True)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    rows, corner_errors = [], []
    for directory in args.inputs:
        cfg = json.loads((directory/'manifest.json').read_text())['config']
        manifest = yaml.safe_load((args.aruco_root/cfg['pad_manifest']).read_text())
        reference = PhysicalPadDetector(manifest)
        gpu = GpuArucoDetector(reference.dictionary,str(args.library))
        camera = cfg['camera']
        K = np.array([[camera['fx'],0.,camera['cx']],[0.,camera['fy'],camera['cy']],[0.,0.,1.]])
        mount = pose_matrix(camera['body_position_m'],camera['body_quaternion_xyzw'])
        for path in sorted(directory.glob('camera-*.png')):
            env = int(path.stem.split('-')[1])
            initial = initial_condition(cfg['seed'],env,cfg['initial_bounds'])
            yaw = math.radians(initial['yaw_deg'])/2
            truth = pose_matrix([initial[k] for k in ('x','y','z')],
                                [0.,0.,math.sin(yaw),math.cos(yaw)]) @ mount
            bgr = cv2.imread(str(path))
            gray = cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
            rgb = torch.from_numpy(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)[None]).cuda()
            gpu.detect(rgb)  # Allocate/warm up outside timing.
            begin = time.perf_counter()
            corners, ids, _ = reference.detector.detectMarkers(gray)
            ids = [] if ids is None else ids.flatten().tolist()
            cpu_detect_s = time.perf_counter()-begin
            begin = time.perf_counter()
            (gpu_corners,gpu_ids), = gpu.detect(rgb)[0]
            gpu_detect_s = time.perf_counter()-begin
            cpu_pose = reference.estimate(corners,ids,K,np.zeros(5))
            gpu_pose = reference.estimate(gpu_corners,gpu_ids,K,np.zeros(5))
            def eligible(cs,ms):
                return {mid:np.asarray(c).reshape(4,2) for c,mid in zip(cs,ms)
                        if mid in reference.models and np.linalg.norm(
                            np.asarray(c).reshape(4,2)-np.roll(np.asarray(c).reshape(4,2),-1,axis=0),axis=1).mean()>=12}
            a,b = eligible(corners,ids),eligible(gpu_corners,gpu_ids)
            for mid in a.keys() & b.keys():
                corner_errors.extend(np.linalg.norm(a[mid]-b[mid],axis=1).tolist())
            delta = None if cpu_pose is None or gpu_pose is None else float(np.linalg.norm(
                inverse(cpu_pose['camera_from_pad'])[:3,3]-inverse(gpu_pose['camera_from_pad'])[:3,3]))
            rows.append(dict(image=str(path),cpu_detect_s=cpu_detect_s,gpu_detect_s=gpu_detect_s,
                cpu_eligible_ids=sorted(a),gpu_eligible_ids=sorted(b),
                missed_vs_cpu=sorted(a.keys()-b.keys()),extra_vs_cpu=sorted(b.keys()-a.keys()),
                cpu_ground_truth_error=pose_error(cpu_pose,truth),
                gpu_ground_truth_error=pose_error(gpu_pose,truth),pose_delta_m=delta))
        gpu.close()
    if not rows: raise ValueError('no smoke images found')
    report = dict(scope='actual static Isaac renders; detector timing excludes rendering/upload/CPU PnP',
        images=len(rows),corner_cpu_agreement_rmse_px=float(np.sqrt(np.mean(np.square(corner_errors)))),
        cpu_detect_s=sum(r['cpu_detect_s'] for r in rows),
        gpu_detect_s=sum(r['gpu_detect_s'] for r in rows),rows=rows)
    report['strict_agreement_passed'] = (report['corner_cpu_agreement_rmse_px']<.75
        and all(not r['missed_vs_cpu'] and not r['extra_vs_cpu'] and r['pose_delta_m'] is not None
                and r['pose_delta_m']<.02 for r in rows))
    report['static_pose_gate_passed'] = all(r['gpu_ground_truth_error'] is not None
        and r['gpu_ground_truth_error']['translation_m']<=.03
        and r['gpu_ground_truth_error']['rotation_deg']<=5. for r in rows)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))


if __name__=='__main__': main()

#!/usr/bin/env python3
"""CPU/GPU pose comparison on saved Isaac optical-smoke images at grid reset."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import torch
import yaml

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'ws/aruco-landing/src/aruco_landing/src'))
sys.path.insert(0,str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from pad_scene import metric_pad_manifest
from trial_inputs import trial_initial_condition
from aruco_landing.physical_pad import PhysicalPadDetector,inverse
from aruco_landing.pose_alignment import pose_matrix
from aruco_landing.gpu_aruco import GpuArucoDetector


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--library',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    metadata=json.loads((args.input/'manifest.json').read_text())
    smoke=json.loads((args.input/'smoke.json').read_text())
    config=metadata['config']
    if config['initial_protocol']['kind']!='funnel_grid' or smoke['statistics']['physics_steps']!=0:
        raise ValueError('this check requires optical smoke at the prescribed grid reset')
    pad_root=ROOT/'stacks/aruco-landing-isaac-x86' if config.get('pad_manifest_root')=='stack' else ROOT/'ws/aruco-landing/src/aruco_landing'
    pad=metric_pad_manifest(yaml.safe_load((pad_root/config['pad_manifest']).read_text()),config['initial_protocol']['pad_side_m'])
    estimator=PhysicalPadDetector(pad)
    detector=GpuArucoDetector(estimator.dictionary,str(args.library))
    camera=config['camera']
    K=np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1.]])
    rows=[]
    for path in sorted(args.input.glob('camera-*.png')):
        env=int(path.stem.split('-')[-1])
        image=cv2.imread(str(path))
        rgb=torch.from_numpy(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)[None].copy()).cuda()
        found,transferred=detector.detect(rgb)
        gpu=estimator.estimate(*found[0],K,np.zeros(5))
        cpu,_,_=estimator.detect(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),K,np.zeros(5))
        initial=trial_initial_condition(config,env,.002,None)
        truth=pose_matrix(initial['camera_initial_position_pad_m'],camera['body_quaternion_xyzw'])
        record=dict(camera=env,image_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    gpu_ids=found[0][1],cpu_inlier_ids=cpu['inlier_ids'] if cpu else [],transferred_bytes=transferred)
        for name,observation in [('gpu',gpu),('cpu',cpu)]:
            estimate=inverse(observation['camera_from_pad']) if observation else None
            record[name+'_position_error_m']=float(np.linalg.norm(estimate[:3,3]-truth[:3,3])) if estimate is not None else None
        record['cpu_gpu_position_delta_m']=float(np.linalg.norm(inverse(cpu['camera_from_pad'])[:3,3]-inverse(gpu['camera_from_pad'])[:3,3])) if cpu and gpu else None
        rows.append(record)
    detector.close()
    passed=len(rows)==smoke['expected'] and all(r['gpu_position_error_m'] is not None and
        r['cpu_position_error_m'] is not None and r['gpu_position_error_m']<=.03 and
        r['cpu_position_error_m']<=.03 and r['cpu_gpu_position_delta_m']<=.03 for r in rows)
    result=dict(scope='Same rendered reset images; expected camera reset GT (ignores <= tens-of-micrometers world-origin rounding).',
        opencv_version=cv2.__version__,torch_version=torch.__version__,cuda_device=torch.cuda.get_device_name(),
        library_sha256=hashlib.sha256(args.library.read_bytes()).hexdigest(),
        recorded_gpu_smoke=smoke['pose_quality'],passed=passed,images=len(rows),rows=rows)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Passed' if passed else 'Failed',len(rows),'rendered images;',
          'max CPU/GPU pose delta',max((r['cpu_gpu_position_delta_m'] for r in rows if r['cpu_gpu_position_delta_m'] is not None),default=None))
    if not passed: raise SystemExit(1)


if __name__=='__main__': main()

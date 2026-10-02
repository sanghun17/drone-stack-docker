#!/usr/bin/env python3
"""Read-only host check for the pinned Isaac RTX runtime."""
import argparse
import json
import subprocess
import sys


def version(value):
    return tuple(int(part) for part in value.split('.'))


def check(uuid):
    result=subprocess.run(['nvidia-smi','--id='+uuid,
        '--query-gpu=uuid,name,driver_version,memory.total','--format=csv,noheader,nounits'],
        capture_output=True,text=True)
    if result.returncode:
        raise RuntimeError(result.stderr.strip())
    actual,name,driver,memory=[item.strip() for item in result.stdout.strip().split(',')]
    compatible=version(driver)>=version('550.90.07')
    return dict(gpu_uuid=actual,gpu_name=name,driver=driver,memory_mib=int(memory),
        rtx_driver_floor_passed=compatible, image='Isaac Lab 3.0.0-rc1 / Isaac Sim 6.1',
        renderer_log_minimum='550.90.07',renderer_log_recommended='580.95.05',
        caveat='Driver floor is necessary, not a full compatibility or GPU health verdict.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu-uuid',required=True)
    parser.add_argument('--report',action='store_true',help='report incompatibility without failing')
    args=parser.parse_args()
    report=check(args.gpu_uuid)
    print(json.dumps(report,indent=2))
    return 0 if args.report or report['rtx_driver_floor_passed'] else 2


if __name__=='__main__': sys.exit(main())

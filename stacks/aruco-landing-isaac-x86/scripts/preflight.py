#!/usr/bin/env python3
"""Read-only host check for the pinned Isaac RTX runtime."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def version(value):
    return tuple(int(part) for part in value.split('.'))


def check(uuid):
    denied = os.environ.get('DSD_DENIED_GPU_UUIDS','').split(',')
    if uuid.lower() in {value.strip().lower() for value in denied if value.strip()}:
        raise RuntimeError('GPU UUID is explicitly denied by this stack: '+uuid)
    for pci in filter(None,os.environ.get('ISAAC_REQUIRED_ISOLATION_PCI','').split(',')):
        if not re.fullmatch(r'[0-9a-f]{4}:[0-9a-f]{2}:[0-9a-f]{2}\.[0-7]',pci):
            raise RuntimeError('invalid required GPU isolation PCI address')
        driver=Path('/sys/bus/pci/devices')/pci/'driver'
        if driver.is_symlink() and driver.resolve().name in ('nvidia','nouveau'):
            raise RuntimeError('Failed GPU '+pci+' is still attached to '+driver.resolve().name+
                               '; install host PCI isolation and reboot before evaluation')
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

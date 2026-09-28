#!/usr/bin/env python3
"""Build corrected AirSim exposure timestamps in an isolated Unreal project."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--engine', type=Path, default=Path('/home/ml/UnrealEngine'))
    p.add_argument('--jobs', type=int, default=4)
    p.add_argument('--cache-dir', type=Path, default=ROOT/'.build/sim-x86/capture-time-project')
    p.add_argument('--timing-diagnostics', action='store_true',
                   help='Build optional paired-clock logging in an isolated project')
    p.add_argument('--async-readback', action='store_true',
                   help='Build optional RGB/depth readback with per-slot exposure metadata')
    args = p.parse_args()
    if args.jobs < 1:
        p.error('--jobs must be positive')
    source = ROOT/'data/assets/simulation/unreal/editor/MyFirstUE4'
    cache = args.cache_dir.resolve()
    project = cache/'MyFirstUE4'
    if not project.exists():
        project.mkdir(parents=True)
        for name in ['Config','Content','Source','Plugins','MyFirstUE4.uproject']:
            subprocess.run(['cp','-a','--reflink=auto',str(source/name),str(project/name)],check=True)
        for name in ['Binaries','Intermediate']:
            artifact = project/'Plugins/AirSim'/name
            if artifact.exists():
                shutil.rmtree(artifact)
    module = ROOT/'modules/simulation/airsim'
    prepare = ['python3',str(module/'scripts/prepare_capture_timestamps.py'),
               '--plugin-dir',str(project/'Plugins/AirSim')]
    if args.timing_diagnostics:
        prepare.append('--diagnostics')
    if args.async_readback:
        prepare.append('--async-readback')
    subprocess.run(prepare,check=True)
    subprocess.run([str(args.engine/'Engine/Build/BatchFiles/Linux/Build.sh'),
                    'MyFirstUE4Editor','Linux','Development',str(project/'MyFirstUE4.uproject'),
                    '-WaitMutex',f'-MaxParallelActions={args.jobs}'],check=True)
    manifest = {'backend':'capture-time-editor','project':str(project/'MyFirstUE4.uproject'),
                'engine':str(args.engine),'source_project':str(source),
                'module_revision':json.loads((ROOT/'config/modules.lock.json').read_text())['modules']['simulation/airsim']['revision'],
                'patch_sha256':sha(module/'patches/rendered_pose_timestamp.patch'),
                'plugin_binary_sha256':sha(project/'Plugins/AirSim/Binaries/Linux/libUE4Editor-AirSim.so'),
                'map':'/Game/ModernLivingRoom/Maps/Main_long',
                'project_config_sha256':{f.name:sha(f) for f in (project/'Config').glob('*.ini')},
                'map_asset_sha256':sha(project/'Content/ModernLivingRoom/Maps/Main_long.umap'),
                'timestamp_semantics':'rendered physics pose, not GPU readback completion'}
    if args.timing_diagnostics:
        manifest['timing_diagnostic_patch_sha256'] = sha(module/'patches/capture_timing_diagnostics.patch')
        manifest['timing_diagnostic_flag'] = '-AirSimCaptureTiming'
    if args.async_readback:
        manifest['async_readback_patch_sha256'] = sha(module/'patches/capture_slot_readback.patch')
        manifest['async_readback_flag'] = '-AirSimAsyncCapture'
    (cache/'build.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Built simulation: '+str(cache/'build.json'))


if __name__ == '__main__':
    main()

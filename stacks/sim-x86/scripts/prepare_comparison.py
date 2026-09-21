#!/usr/bin/env python3
"""Prepare the isolated July 2026 runtime without changing the active checkout."""
import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[3]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT / 'ws/risk-aware/src/risk_aware_planning')
    parser.add_argument('--legacy-checkpoints', type=Path,
                        default=ROOT / 'data/archive/asset-backups/checkpoints_legacy')
    parser.add_argument('--assets', type=Path,
                        default=ROOT / 'data/assets/checkpoints/comparison-20260706')
    args = parser.parse_args()
    profile = yaml.safe_load((ROOT / 'stacks/sim-x86/config/comparison-20260706.yml').read_text())
    dst = ROOT / profile['source_workspace'] / 'src/risk_aware_planning'
    revision = profile['source_revision']
    if not dst.exists():
        dst.parent.mkdir(parents=True, exist_ok=True)
        source = str(args.source) if args.source.is_dir() else profile['source_repository']
        subprocess.run(['git', 'clone', '--no-checkout', source, str(dst)], check=True)
        subprocess.run(['git', '-C', str(dst), 'checkout', '--detach', revision], check=True)
    actual = subprocess.check_output(['git', '-C', str(dst), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != revision:
        raise SystemExit(f'Preserving existing checkout {dst}: expected {revision}, found {actual}')
    dirty = subprocess.check_output(['git', '-C', str(dst), 'status', '--porcelain'], text=True)
    if dirty:
        raise SystemExit(f'Preserving modified comparison checkout: {dst}\n{dirty}')
    for relative in (profile['checkpoint'], profile['kinetic_statistics']):
        src, target = args.legacy_checkpoints / relative, args.assets / relative
        if not src.is_file():
            raise SystemExit(f'Missing original comparison asset: {src}')
        if digest(src) != profile['asset_sha256'][relative]:
            raise SystemExit(f'Historical asset hash does not match the profile: {src}')
        if target.exists() and digest(target) != digest(src):
            raise SystemExit(f'Refusing to overwrite a different asset: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists():
            shutil.copy2(src, target)
        print(digest(target), target)
    print(f'Prepared {dst} at {revision}')
    print('Next: bash stacks/sim-x86/scripts/build_comparison.sh')


if __name__ == '__main__':
    main()

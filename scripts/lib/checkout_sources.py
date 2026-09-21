#!/usr/bin/env python3
"""Materialize pinned component repositories and apply versioned compatibility patches."""
import argparse
from pathlib import Path
import subprocess

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--local-source', type=Path)
    args = parser.parse_args()
    manifest = args.manifest.resolve()
    for entry in yaml.safe_load(manifest.read_text())['sources']:
        target = args.destination.resolve() / entry['path']
        revision = entry['revision']
        if not (target / '.git').exists():
            if target.exists() and any(target.iterdir()):
                raise SystemExit(f'Preserving existing non-Git directory: {target}')
            target.parent.mkdir(parents=True, exist_ok=True)
            source = str(args.local_source / entry['path']) if args.local_source else entry['repository']
            subprocess.run(['git', 'clone', '--no-checkout', source, str(target)], check=True)
            subprocess.run(['git', '-C', str(target), 'checkout', '--detach', revision], check=True)
        git = ['git', '-C', str(target)]
        actual = subprocess.check_output(git + ['rev-parse', 'HEAD'], text=True).strip()
        if actual != revision:
            raise SystemExit(f'Preserving {target}: HEAD is {actual}, expected {revision}')
        for patch in entry.get('patches', []):
            path = str(manifest.parent / patch)
            applied = subprocess.run(git + ['apply', '--reverse', '--check', path], capture_output=True)
            if applied.returncode == 0:
                continue
            subprocess.run(git + ['apply', '--check', path], check=True)
            subprocess.run(git + ['apply', path], check=True)
        print(f'{entry["path"] or "."}: {revision}')


if __name__ == '__main__':
    main()

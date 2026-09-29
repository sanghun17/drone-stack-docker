#!/usr/bin/env python3
"""Materialize pinned component repositories and apply versioned compatibility patches."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

import yaml


def applied_patch_prefix(target, patches):
    """Check stacked patches in a disposable copy, newest to oldest.

    A later patch may change an earlier patch's context. Checking that earlier
    patch against the final tree in isolation incorrectly treats a correctly
    deployed checkout as unpatched. Never unwind patches in the real checkout.
    """
    files = set()
    for patch in patches:
        stats = subprocess.check_output(['git', 'apply', '--numstat', '-z', str(patch)])
        for record in stats.split(b'\0'):
            if not record:
                continue
            name = Path(record.split(b'\t', 2)[2].decode())
            if name.is_absolute() or '..' in name.parts:
                raise ValueError('Patch path escapes checkout: ' + str(name))
            files.add(name)
    for count in range(len(patches), 0, -1):
        with tempfile.TemporaryDirectory(prefix='source-patch-check-') as directory:
            scratch = Path(directory)
            subprocess.run(['git', 'init', '-q', str(scratch)], check=True)
            for name in files:
                source = target / name
                if source.exists() or source.is_symlink():
                    destination = scratch / name
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, destination, follow_symlinks=False)
            for patch in reversed(patches[:count]):
                command = ['git', '-C', str(scratch), 'apply', '--reverse', str(patch)]
                if subprocess.run(command, capture_output=True).returncode:
                    break
            else:
                return count
    return 0


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
        patches = [manifest.parent / patch for patch in entry.get('patches', [])]
        prefix = applied_patch_prefix(target, patches)
        for patch in patches[prefix:]:
            path = str(patch)
            subprocess.run(git + ['apply', '--check', path], check=True)
            subprocess.run(git + ['apply', path], check=True)
        print(f'{entry["path"] or "."}: {revision}')


if __name__ == '__main__':
    main()

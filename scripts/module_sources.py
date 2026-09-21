#!/usr/bin/env python3
"""Resolve pinned remote deployment packages into the local modules directory."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import tarfile
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
LOCK = 'config/modules.lock.json'
STAMP = '.module-origin.json'


def git(directory, *args):
    return subprocess.check_output(['git', '-C', str(directory), *args])


def relative(value):
    path = PurePosixPath(value)
    if not value or path.is_absolute() or any(p in ('.', '..') for p in value.split('/')):
        raise ValueError('expected a safe relative path: ' + repr(value))
    return path


def load_lock(root=ROOT):
    document = json.loads((root / LOCK).read_text())
    if document.get('version') != 1:
        raise ValueError('unsupported module lock version')
    for name, entry in document['modules'].items():
        relative(name)
        relative(entry['subdir'])
        if not re.fullmatch(r'[0-9a-f]{40}', entry['revision']):
            raise ValueError('module revision must be a full Git commit: ' + name)
        for path, checksum in entry['files'].items():
            relative(path)
            if not re.fullmatch(r'[0-9a-f]{64}', checksum):
                raise ValueError('invalid locked file checksum: ' + path)
    return document['modules']


def order(selected, entries):
    result, done, visiting = [], set(), set()
    def visit(name):
        if name in visiting:
            raise ValueError('module dependency cycle at ' + name)
        if name in done:
            return
        if name not in entries:
            raise ValueError('module is not locked: ' + name)
        visiting.add(name)
        for dependency in entries[name]['manifest'].get('needs', []):
            visit(dependency)
        visiting.remove(name)
        done.add(name)
        result.append(name)
    for name in ['base'] + list(selected):
        visit(name)
    return result


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(directory, exclude=()):
    result = {}
    for path in directory.rglob('*'):
        if path.is_symlink():
            raise ValueError('module package cannot contain symlinks: ' + str(path))
        if path.is_file():
            name = path.relative_to(directory).as_posix()
            if name not in exclude and '__pycache__' not in path.parts:
                result[name] = sha(path)
    return result


def verify(directory, entry, artifacts=False):
    for name, checksum in entry['files'].items():
        path = directory / name
        if path.is_symlink() or not path.is_file() or sha(path) != checksum:
            raise ValueError('module missing/modified; sync or edit its owning repository: ' + str(path))
    manifest = yaml.safe_load((directory / 'module.yml').read_text())
    if manifest != entry['manifest']:
        raise ValueError('module manifest differs from lock: ' + str(directory))
    if artifacts:
        for artifact in manifest.get('artifacts', []):
            path = directory / str(relative(artifact['path']))
            if path.exists() and (path.stat().st_size != artifact['size'] or sha(path) != artifact['sha256']):
                raise ValueError('module artifact checksum mismatch: ' + str(path))


def repository_cache(root, repository, revision):
    key = hashlib.sha256(repository.encode()).hexdigest()[:20]
    cache = root / '.build/module-repositories' / key
    if not cache.exists():
        cache.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'init', '--bare', str(cache)], check=True, stdout=subprocess.DEVNULL)
        git(cache, 'remote', 'add', 'origin', repository)
    if git(cache, 'remote', 'get-url', 'origin').decode().strip() != repository:
        raise ValueError('module cache repository mismatch')
    present = subprocess.run(['git', '-C', str(cache), 'cat-file', '-e', revision + '^{commit}'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if present.returncode:
        subprocess.run(['git', '-C', str(cache), 'fetch', '--depth', '1', 'origin', revision], check=True)
    return cache


def export(cache, revision, subdir, destination):
    payload = git(cache, 'archive', revision + ':' + str(relative(subdir)))
    with tarfile.open(fileobj=io.BytesIO(payload), mode='r:') as archive:
        for member in archive:
            name = str(relative(member.name.rstrip('/')))
            path = destination / name
            if member.isdir():
                path.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                path.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(member) as source, path.open('xb') as target:
                    shutil.copyfileobj(source, target)
                path.chmod(0o755 if member.mode & 0o111 else 0o644)
            else:
                raise ValueError('unsupported package entry: ' + member.name)


def artifact_sync(directory, artifact):
    path = directory / str(relative(artifact['path']))
    if path.exists():
        if path.stat().st_size != artifact['size'] or sha(path) != artifact['sha256']:
            raise ValueError('preserving modified artifact: ' + str(path))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=path.parent, prefix='.download-') as temp:
        temporary = Path(temp) / path.name
        release = artifact['github_release']
        subprocess.run(['gh', 'release', 'download', release['tag'], '--repo', release['repository'],
                        '--pattern', release['asset'], '--output', str(temporary)], check=True)
        if temporary.stat().st_size != artifact['size'] or sha(temporary) != artifact['sha256']:
            raise ValueError('downloaded artifact failed SHA-256: ' + path.name)
        temporary.rename(path)


def sync_module(root, name, entry, arch, download=True):
    directory = root / 'modules' / name
    if directory.exists():
        stamp = directory / STAMP
        if not stamp.exists():
            raise ValueError('preserving unmanaged module directory: ' + str(directory))
        previous = json.loads(stamp.read_text())
        artifacts = {item['path'] for item in previous['manifest'].get('artifacts', [])}
        current = fingerprint(directory, exclude={STAMP} | artifacts)
        if current != previous['files']:
            raise ValueError('preserving edited module; change its source repository: ' + name)
        if previous == entry:
            verify(directory, entry, artifacts=True)
        else:
            # A verified old snapshot can be replaced; edits and unknown files cannot.
            cache = repository_cache(root, entry['repository'], entry['revision'])
            with tempfile.TemporaryDirectory(dir=root / '.build', prefix='module-') as tmp:
                staged = Path(tmp) / 'package'
                staged.mkdir()
                export(cache, entry['revision'], entry['subdir'], staged)
                verify(staged, entry)
                (staged / STAMP).write_text(json.dumps(entry, indent=2) + '\n')
                backup = Path(tmp) / 'previous'
                directory.rename(backup)
                try:
                    staged.rename(directory)
                except BaseException:
                    backup.rename(directory)
                    raise
    else:
        cache = repository_cache(root, entry['repository'], entry['revision'])
        directory.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root / '.build', prefix='module-') as tmp:
            staged = Path(tmp) / 'package'
            staged.mkdir()
            export(cache, entry['revision'], entry['subdir'], staged)
            verify(staged, entry)
            (staged / STAMP).write_text(json.dumps(entry, indent=2) + '\n')
            staged.rename(directory)
    if download:
        for artifact in entry['manifest'].get('artifacts', []):
            if not artifact.get('arch') or arch in artifact['arch']:
                artifact_sync(directory, artifact)


def package_entry(repository, revision, subdir, checkout):
    actual = git(checkout, 'rev-parse', revision + '^{commit}').decode().strip()
    with tempfile.TemporaryDirectory() as tmp:
        directory = Path(tmp)
        export(checkout, actual, subdir, directory)
        manifest = yaml.safe_load((directory / 'module.yml').read_text())
        return {'repository': repository, 'revision': actual, 'subdir': subdir,
                'manifest': manifest, 'files': fingerprint(directory)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['sync', 'verify', 'lock'])
    parser.add_argument('stack', nargs='?')
    parser.add_argument('--arch', choices=['amd64', 'arm64'], default='amd64')
    parser.add_argument('--no-artifacts', action='store_true')
    parser.add_argument('--module')
    parser.add_argument('--repository')
    parser.add_argument('--checkout', type=Path)
    parser.add_argument('--revision')
    parser.add_argument('--subdir')
    args = parser.parse_args()
    if args.command == 'lock':
        if not all([args.module, args.repository, args.checkout, args.revision, args.subdir]):
            parser.error('lock requires --module --repository --checkout --revision --subdir')
        relative(args.module)
        path = ROOT / LOCK
        doc = json.loads(path.read_text()) if path.exists() else {'version': 1, 'modules': {}}
        doc['modules'][args.module] = package_entry(args.repository, args.revision, args.subdir, args.checkout)
        path.write_text(json.dumps(doc, indent=2, sort_keys=True) + '\n')
        return
    if not args.stack:
        parser.error('sync/verify requires a stack')
    relative(args.stack)
    stack = yaml.safe_load((ROOT / 'stacks' / args.stack / 'stack.yml').read_text())
    entries = load_lock()
    for name in order(stack['modules'], entries):
        if args.command == 'sync':
            sync_module(ROOT, name, entries[name], args.arch, not args.no_artifacts)
        else:
            verify(ROOT / 'modules' / name, entries[name], artifacts=True)
        print(name + ': ' + entries[name]['revision'][:12])


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        raise SystemExit('Module packages: ' + str(error))

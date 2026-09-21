#!/usr/bin/env python3
"""Enforce repository layout on staged files, committed trees and the checkout.

Uses only the Python standard library. Git checks read blobs from Git, so an
unstaged edit, ignored path or staged deletion cannot disguise the proposed tree.
"""
import argparse
import json
import os
from pathlib import Path, PurePosixPath
import re
import subprocess
import sys

POLICY = 'config/repository-layout.json'


def git(root, *args, data=None):
    return subprocess.check_output(['git', '-C', str(root), *args], input=data)


def policy_for(root, revision=None):
    if revision is None:
        return json.loads((root / POLICY).read_text())
    return json.loads(git(root, 'show', revision + ':' + POLICY))


def path_errors(path, policy, directory=False):
    parts = PurePosixPath(path).parts
    if not parts or any(p in {'.', '..'} for p in parts):
        return ['invalid path: ' + repr(path)]
    for prefix in policy.get('forbidden_path_prefixes', []):
        if path == prefix or path.startswith(prefix + '/'):
            return [path + ': retired/external path; use its current owner or home storage']
    first = parts[0]
    if first in policy['untracked_roots']:
        return [path + ': local workspaces, build products and logs must not be tracked']
    if len(parts) == 1 and not directory:
        return [] if path in policy['root_files'] else [path + ': root file is not allowed']
    allowed = policy['root_directories'] + policy['tracked_hidden_directories']
    if first not in allowed:
        return [path + ': root directory is not allowed']
    if first == 'data' and len(parts) > 1:
        if len(parts) == 2 and parts[1] == 'README.md' and not directory:
            return []
        if parts[1] in policy.get('data_payload_directories', []):
            return [] if directory else [path + ': local data payload must not be tracked']
        if parts[1] not in ('analysis', 'manifests'):
            return [path + ': use data/analysis, manifests, assets, results or archive']
        if parts[1] == 'manifests' and not directory:
            return [] if PurePosixPath(path).suffix.lower() in ('.json', '.yml', '.yaml', '.csv', '.md', '.txt') else [path + ': manifests contains metadata only']
    directories = parts if directory else parts[:-1]
    for part in directories:
        if part in policy['forbidden_directories'] or part.startswith(('_campaign_', '.tmp_')):
            return [path + ': generated/research directory is not allowed: ' + part]
    if first == 'modules' and len(parts) > 1:
        if len(parts) == 2 and not directory and parts[1] in policy['module_documents']:
            return []
        if parts[1] not in policy['module_groups']:
            return [path + ': unknown module group; shared helpers belong in scripts/lib']
        if not directory:
            return [path + ': remote module implementation must not be tracked; update its owner and modules.lock.json']
    if first == 'stacks' and len(parts) > 1:
        if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]*', parts[1]):
            return [path + ': use stacks/<name>/stack.yml']
        if len(parts) == 2 and not directory:
            return [path + ': loose stack files are not allowed; use stacks/<name>/stack.yml']
        if len(parts) > 2 and parts[2] not in policy['stack_entries']:
            return [path + ': unexpected stack entry: ' + parts[2]]
    if not directory and path not in policy['runtime_artifacts']:
        if PurePosixPath(path).suffix.lower() in policy['artifact_suffixes']:
            return [path + ': output/binary artifact; use home storage or declare a runtime exception']
    return []


def check_tree(root, revision):
    policy = policy_for(root, revision)
    entries = []
    errors = []
    if revision == '':
        for record in git(root, 'ls-files', '--stage', '-z').split(b'\0'):
            if not record:
                continue
            header, path = record.split(b'\t', 1)
            mode, oid, stage = header.decode().split()
            if stage != '0':
                errors.append('unresolved merge: ' + os.fsdecode(path))
            entries.append((mode, oid, os.fsdecode(path)))
    else:
        for record in git(root, 'ls-tree', '-r', '-z', revision).split(b'\0'):
            if record:
                header, path = record.split(b'\t', 1)
                mode, kind, oid = header.decode().split()
                entries.append((mode, oid, os.fsdecode(path)))
    paths = {path for _, _, path in entries}
    oids = sorted({oid for mode, oid, _ in entries if mode in {'100644', '100755'}})
    sizes = {}
    if oids:
        data = ('\n'.join(oids) + '\n').encode()
        for row in git(root, 'cat-file', '--batch-check=%(objectname) %(objectsize)', data=data).splitlines():
            oid, size = row.decode().split()
            sizes[oid] = int(size)
    for mode, oid, path in entries:
        errors.extend(path_errors(path, policy))
        if mode not in {'100644', '100755'}:
            errors.append(path + ': symlinks/submodules cannot bypass layout ownership')
            continue
        limit = policy['runtime_artifacts'].get(path, policy['max_file_bytes'])
        if sizes[oid] > limit:
            errors.append('%s: %d bytes exceeds allowed %d bytes' % (path, sizes[oid], limit))
        parts = PurePosixPath(path).parts
        if parts[0] == 'stacks' and len(parts) >= 3:
            manifest = 'stacks/' + parts[1] + '/stack.yml'
            if manifest not in paths:
                errors.append(path + ': missing ' + manifest)
        if path.startswith('modules/') and len(parts) >= 3 and parts[1] in policy['module_groups']:
            module = '/'.join(parts[:2] if parts[1] == 'base' else parts[:3])
            if module + '/module.yml' not in paths:
                errors.append(path + ': missing ' + module + '/module.yml')
    return sorted(set(errors))


def check_worktree(root, policy):
    errors = []
    local = set(policy['root_directories'] + policy['tracked_hidden_directories'] + policy['local_hidden_directories'])
    for item in root.iterdir():
        if item.name == '.git':  # also a file in linked worktrees
            continue
        if item.is_dir() and not item.is_symlink():
            if item.name not in local:
                errors.append(item.name + '/: unexpected root directory (including ignored files)')
        elif item.name not in policy['root_files']:
            errors.append(item.name + ': unexpected root file or symlink')
    # Inspect directories even if ignored. Skip only known runtime build trees
    # and Python's automatic bytecode caches; never descend into component repos.
    locked_modules = set()
    lockfile = root / 'config/modules.lock.json'
    if lockfile.exists():
        locked_modules = {'modules/' + name for name in json.loads(lockfile.read_text())['modules']}
    for name in ('config', 'modules', 'scripts', 'stacks', 'data'):
        for base, dirs, files in os.walk(root / name, followlinks=False):
            kept = []
            for directory in dirs:
                path = (Path(base) / directory).relative_to(root).as_posix()
                if directory == '__pycache__' or path in policy['local_generated_directories']:
                    continue
                if path in locked_modules:
                    if (Path(base) / directory).is_symlink():
                        errors.append(path + ': module package must be a real directory')
                    continue
                if path in {'data/' + n for n in policy.get('data_payload_directories', [])}:
                    if (Path(base) / directory).is_symlink():
                        errors.append(path + ': data roots must be real directories')
                    continue
                errors.extend(path_errors(path, policy, directory=True))
                kept.append(directory)
            dirs[:] = kept
    # Catch untracked output files as well as new misplaced source files.
    for raw in git(root, 'ls-files', '--others', '--exclude-standard', '-z').split(b'\0'):
        if raw:
            path = os.fsdecode(raw)
            errors.extend(path_errors(path, policy))
    return sorted(set(errors))


def outgoing_commits(root, remote, lines):
    commits = set()
    for line in lines:
        local_ref, local_oid, remote_ref, remote_oid = line.split()
        if set(local_oid) == {'0'}:
            continue  # deleting a remote ref introduces no files
        if set(remote_oid) != {'0'} and subprocess.run(
                ['git', '-C', str(root), 'cat-file', '-e', remote_oid],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
            args = [remote_oid + '..' + local_oid]
        else:
            args = [local_oid, '--not', '--remotes=' + remote]
        commits.update(git(root, 'rev-list', *args).decode().splitlines())
    return sorted(commits)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--staged', action='store_true')
    group.add_argument('--revision', help='validate the full tree at this Git revision')
    group.add_argument('--pre-push', metavar='REMOTE', help='read Git pre-push input from stdin')
    parser.add_argument('--worktree', action='store_true', help='also check ignored/untracked checkout layout')
    args = parser.parse_args()
    try:
        root = Path(git(Path.cwd(), 'rev-parse', '--show-toplevel').decode().strip())
        revisions = outgoing_commits(root, args.pre_push, sys.stdin) if args.pre_push else (
            [''] if args.staged else ([args.revision] if args.revision else []))
        errors = []
        for revision in revisions:
            errors.extend((revision[:12] or 'index') + ': ' + error for error in check_tree(root, revision))
        if args.worktree or not (args.staged or args.revision or args.pre_push):
            policy = policy_for(root, '' if args.staged else None)
            errors.extend(check_worktree(root, policy))
        if errors:
            print('Repository layout rejected:', file=sys.stderr)
            for error in errors:
                print('  - ' + error, file=sys.stderr)
            print('Use data/analysis for research code, data/results for outputs, '
                  'remote repositories for modules, and .build for build products. Rules: ' + POLICY, file=sys.stderr)
            return 1
        print('Repository layout OK')
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Repository layout check failed: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

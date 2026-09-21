#!/usr/bin/env python3
"""Protect the checkout's top-level directory entries using Unix permissions.

Existing child directories stay writable. This is an accidental-write barrier,
not isolation from a malicious process with the same UID or root privileges.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys

from check_layout import check_worktree, git, policy_for


def state_path(root):
    path = Path(os.fsdecode(git(root, 'rev-parse', '--git-path', 'layout-root-lock.json')).strip())
    return path if path.is_absolute() else root / path


def load_state(root, path):
    if not path.exists():
        return None
    state = json.loads(path.read_text())
    info = root.stat()
    if (state['device'], state['inode']) != (info.st_dev, info.st_ino):
        raise ValueError('lock metadata belongs to a different checkout; do not reuse copied .git lock state')
    return state


def require_owner(root):
    if root.stat().st_uid != os.geteuid():
        raise ValueError('run as the checkout owner; this command never changes ownership')


def check(root):
    errors = check_worktree(root, policy_for(root))
    if errors:
        raise ValueError('fix the layout before locking:\n  ' + '\n  '.join(errors))


def lock(root, path):
    require_owner(root)
    check(root)
    saved = load_state(root, path)
    if saved is None:
        info = root.stat()
        # Create the permitted local output/workspace roots while still writable.
        for name in policy_for(root)['root_directories'] + ['.build']:
            (root / name).mkdir(exist_ok=True)
        saved = {'version': 1, 'mode': stat.S_IMODE(info.st_mode),
                 'device': info.st_dev, 'inode': info.st_ino}
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(saved, indent=2) + '\n')
        temporary.chmod(0o600)
        temporary.replace(path)
    # Only the root inode is changed: never recurse into files, .git or modules.
    root.chmod(stat.S_IMODE(root.stat().st_mode) & ~0o222)
    return saved


def unlock(root, path):
    require_owner(root)
    saved = load_state(root, path)
    if saved is None:
        raise ValueError('no managed root lock exists; refusing to guess original permissions')
    root.chmod(saved['mode'])
    path.unlink()


def maintain(root, path, command):
    require_owner(root)
    saved = load_state(root, path)
    if saved is None or stat.S_IMODE(root.stat().st_mode) & 0o222:
        raise ValueError('maintenance requires an already locked checkout')
    if not command:
        raise ValueError('maintenance requires a command after --')
    # SIGINT normally raises KeyboardInterrupt; make SIGTERM unwind the same
    # finally block. SIGKILL/power failure cannot be handled by a user process.
    def terminate(signum, frame):
        raise SystemExit(128 + signum)
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        root.chmod(saved['mode'])
        result = subprocess.run(command, cwd=root).returncode
    finally:
        root.chmod(saved['mode'] & ~0o222)
        signal.signal(signal.SIGTERM, previous)
    check(root)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['lock', 'unlock', 'status', 'maintain'])
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        root = Path(os.fsdecode(git(Path.cwd(), 'rev-parse', '--show-toplevel')).strip())
        path = state_path(root)
        # Serialize managed lock/unlock windows. Git metadata remains writable.
        with path.with_suffix('.guard').open('a') as guard:
            fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if args.action == 'lock':
                lock(root, path)
            elif args.action == 'unlock':
                unlock(root, path)
            elif args.action == 'maintain':
                command = args.command[1:] if args.command[:1] == ['--'] else args.command
                return maintain(root, path, command)
            saved = load_state(root, path)
            mode = stat.S_IMODE(root.stat().st_mode)
            locked = not mode & 0o222
            print('Root creation %s: %s (mode %04o, managed=%s)' %
                  ('blocked' if locked else 'allowed', root, mode, saved is not None))
            if locked:
                print('Child directories remain writable. Root-file replacement/pull needs user-authorized maintenance.')
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print('Root guard: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())

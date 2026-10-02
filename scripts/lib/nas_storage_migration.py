#!/usr/bin/env python3
"""Move explicit local payloads to verified, resumable TAR archives on SMB."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import tarfile
import tempfile
import time
from urllib.parse import quote

def digest(path, report=None):
    expected = path.stat().st_size
    offset, retries = 0, 0
    checksum = hashlib.sha256()
    while offset < expected:
        resumed_at = offset
        try:
            with path.open('rb', buffering=0) as stream:
                if offset:
                    stream.seek(offset)
                while offset < expected:
                    block = stream.read(min(65536, expected - offset))
                    if not block:
                        raise OSError('Premature EOF: ' + str(path))
                    checksum.update(block)
                    offset += len(block)
                    if report:
                        report(part_bytes=offset)
        except OSError:
            # A long read may make substantial progress between disconnects.
            # Limit consecutive failures at one offset, rather than counting
            # unrelated recoverable disconnects across a multi-GiB archive.
            retries = 0 if offset > resumed_at else retries + 1
            if retries > 8:
                raise
            time.sleep(1)
    if path.stat().st_size != expected:
        raise RuntimeError('File size changed while hashing: ' + str(path))
    return checksum.hexdigest()


def matching_prefix(local, partial, report=None):
    """Recognize an interrupted upload without discarding unknown NAS bytes."""
    expected = partial.stat().st_size
    if expected > local.stat().st_size:
        return False
    offset, retries = 0, 0
    while offset < expected:
        resumed_at = offset
        try:
            with local.open('rb', buffering=0) as source, partial.open('rb', buffering=0) as remote:
                if offset:
                    source.seek(offset)
                    remote.seek(offset)
                while offset < expected:
                    block = remote.read(min(65536, expected - offset))
                    if not block:
                        raise OSError('Premature EOF in interrupted upload')
                    if source.read(len(block)) != block:
                        return False
                    offset += len(block)
                    if report:
                        report(part_bytes=offset)
        except OSError:
            retries = 0 if offset > resumed_at else retries + 1
            if retries > 8:
                raise
            if report:
                report(True, prefix_read_retry=retries, part_bytes=offset)
            time.sleep(1)
    if partial.stat().st_size != expected:
        raise RuntimeError('NAS partial changed during recognition')
    return True


def transfer_archive(source, destination, report):
    checksum = digest(source)
    partial = destination.with_name(destination.name + '.partial')
    share = Path('/run/user/1000/gvfs/smb-share:server=10.74.22.95,share=research')
    uri = 'smb://10.74.22.95/research/' + quote(partial.relative_to(share).as_posix(), safe='/')
    for attempt in range(1, 6):
        if partial.exists():
            report(True, phase='checking_interrupted_upload')
            if partial.stat().st_size == source.stat().st_size and digest(partial) == checksum:
                partial.rename(destination)
                return checksum
            if not matching_prefix(source, partial, report):
                raise RuntimeError('Unknown or corrupt NAS partial; source retained: ' + str(partial))
            partial.unlink()
        report(True, phase='uploading', upload_attempt=attempt, part_bytes=0,
               archive_bytes=source.stat().st_size)
        # Feeding GIO from a staged file avoids Python pipe producer failures.
        with source.open('rb') as stream, tempfile.TemporaryFile() as errors:
            process = subprocess.Popen(['gio', 'save', '--create', uri], stdin=stream,
                                       stdout=subprocess.DEVNULL, stderr=errors)
            progress, changed_at = -1, time.monotonic()
            try:
                while process.poll() is None:
                    try:
                        size = partial.stat().st_size
                        report(part_bytes=size)
                        if size != progress:
                            progress, changed_at = size, time.monotonic()
                    except OSError:
                        pass
                    if time.monotonic() - changed_at > 300:
                        process.kill()
                        raise OSError('NAS upload made no progress for five minutes')
                    time.sleep(2)
                errors.seek(0)
                detail = errors.read().decode(errors='replace')
            finally:
                if process.poll() is None:
                    process.kill()
                process.wait()
        if process.returncode:
            report(True, upload_error=detail)
            if attempt == 5:
                raise OSError('NAS upload failed after five attempts: ' + detail)
            time.sleep(min(2**attempt, 30))
            continue
        report(True, phase='verifying_nas_archive', part_bytes=0)
        if partial.stat().st_size != source.stat().st_size or digest(partial, report) != checksum:
            raise RuntimeError('NAS archive readback mismatch; source retained')
        partial.rename(destination)
        return checksum
    raise RuntimeError('NAS upload did not complete')


def save(path, value):
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def remote_bytes(path, payload):
    """Publish immutable metadata, retrying only a recognized partial prefix."""
    checksum = hashlib.sha256(payload).hexdigest()
    for attempt in range(5):
        try:
            if path.exists():
                size = path.stat().st_size
                if size == len(payload) and digest(path) == checksum:
                    return
                if size >= len(payload):
                    raise RuntimeError('Existing NAS metadata differs: ' + str(path))
                offset = 0
                with path.open('rb', buffering=0) as existing:
                    while offset < size:
                        block = existing.read(min(65536, size - offset))
                        if not block:
                            raise OSError('Premature EOF in NAS metadata')
                        if block != payload[offset:offset + len(block)]:
                            raise RuntimeError('Existing NAS metadata differs: ' + str(path))
                        offset += len(block)
                if path.stat().st_size != size:
                    raise RuntimeError('NAS metadata changed during recognition')
                path.unlink()
            with path.open('xb') as stream:
                for offset in range(0, len(payload), 65536):
                    block = payload[offset:offset + 65536]
                    if stream.write(block) != len(block):
                        raise OSError('Short NAS metadata write')
            if path.stat().st_size != len(payload) or digest(path) != checksum:
                raise RuntimeError('NAS metadata readback differs: ' + str(path))
            return
        except OSError:
            if attempt == 4:
                raise
            time.sleep(attempt + 1)


def remote_json(path, value):
    remote_bytes(path, (json.dumps(value, ensure_ascii=True, indent=2) + '\n').encode())


def beneath(path, root):
    return path == root or root in path.parents


def opened_paths():
    opened = set()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or proc.name == str(os.getpid()):
            continue
        try:
            descriptors = list((proc / 'fd').iterdir())
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                name = os.readlink(descriptor)
                if name.startswith('/'):
                    opened.add(name)
            except OSError:
                pass
    return opened


def snapshot(path, root, base, remove):
    info = path.lstat()
    root_info = root.lstat()
    if stat.S_ISREG(info.st_mode):
        kind = 'file'
    elif stat.S_ISLNK(info.st_mode):
        kind = 'symlink'
    elif stat.S_ISDIR(info.st_mode):
        kind = 'directory'
    else:
        raise RuntimeError('Unsupported payload type: ' + str(path))
    return dict(source=str(path), root=str(root), name=str(path.relative_to(base)),
                root_inode=root_info.st_ino, root_device=root_info.st_dev,
                kind=kind, size=info.st_size, inode=info.st_ino, device=info.st_dev,
                mode=info.st_mode, mtime_ns=info.st_mtime_ns, remove=remove,
                target=os.readlink(path) if kind == 'symlink' else None)


def unchanged(entry):
    path, root = Path(entry['source']), Path(entry['root'])
    root_info = root.lstat()
    if (not stat.S_ISDIR(root_info.st_mode) or
            (root_info.st_ino, root_info.st_dev) != (entry['root_inode'], entry['root_device'])):
        raise RuntimeError('Source root identity changed: ' + str(root))
    if not beneath(path, root) or (path != root and not beneath(path.parent.resolve(), root.resolve())):
        raise RuntimeError('Source escaped its planned root: ' + str(path))
    actual = path.lstat()
    if (actual.st_ino, actual.st_dev, actual.st_mode) != (
            entry['inode'], entry['device'], entry['mode']):
        raise RuntimeError('Source identity changed: ' + str(path))
    if entry['kind'] != 'directory' and actual.st_mtime_ns != entry['mtime_ns']:
        raise RuntimeError('Source changed: ' + str(path))
    if entry['kind'] == 'file' and actual.st_size != entry['size']:
        raise RuntimeError('Source size changed: ' + str(path))
    if entry['kind'] == 'symlink' and os.readlink(path) != entry['target']:
        raise RuntimeError('Source link changed: ' + str(path))


def prepare(config, audit):
    audit.mkdir(parents=True, exist_ok=True)
    plan_path = audit / 'plan.json'
    if plan_path.exists():
        raise FileExistsError(plan_path)
    protected = [Path(p).absolute() for p in config['protected']]
    local_root = Path(config.get('local_root', str(Path.home()))).absolute()
    opened = opened_paths()
    parts, entries, size, retained = [], [], 0, []
    limit = config.get('part_bytes', 8 * 1024**3)
    for selection in config['roots']:
        root = Path(selection['path']).absolute()
        if not root.is_dir() or root.is_symlink() or not beneath(root, local_root):
            raise RuntimeError('Expected a local home payload directory: ' + str(root))
        if any(beneath(root, p) or beneath(p, root) for p in protected):
            raise RuntimeError('Selection overlaps a protected tree: ' + str(root))
        excluded = [root / p for p in selection.get('exclude', [])]
        selected = [root]
        for base, directories, files in os.walk(root, followlinks=False):
            base = Path(base)
            for name in sorted(directories + files):
                path = base / name
                if any(beneath(path, p) for p in excluded):
                    if name in directories:
                        directories.remove(name)
                    continue
                selected.append(path)
        for path in selected:
            if str(path) in opened:
                retained.append(dict(path=str(path), reason='open file'))
                continue
            # Metadata is archived too, but can remain beside local results.
            remove = selection.get('remove', 'all') == 'all' or path.name.endswith(
                ('.bag', '.bag.active', '.bag.orig.active'))
            if selection.get('include') == 'bags' and not path.is_dir() and not path.name.endswith(
                    ('.bag', '.bag.active', '.bag.orig.active')):
                continue
            entry = snapshot(path, root, local_root, remove)
            if entry['kind'] == 'directory':
                entry['remove'] = False
            payload = entry['size'] if entry['kind'] == 'file' else 0
            if entries and size + payload > limit:
                parts.append(dict(name='part-%04d.tar' % (len(parts) + 1),
                                  source_bytes=size, entries=entries))
                entries, size = [], 0
            entries.append(entry)
            size += payload
    if entries:
        parts.append(dict(name='part-%04d.tar' % (len(parts) + 1),
                          source_bytes=size, entries=entries))
    plan = dict(version=1, created_at=time.time(), destination=config['destination'],
                local_root=str(local_root),
                share=config['share'], protected=config['protected'], roots=config['roots'],
                parts=parts, retained=retained,
                source_bytes=sum(p['source_bytes'] for p in parts))
    save(plan_path, plan)
    return plan


class HashedReader:
    def __init__(self, stream):
        self.stream = stream
        self.hash = hashlib.sha256()

    def read(self, size):
        block = self.stream.read(size)
        self.hash.update(block)
        return block


def stage(part, spool, report):
    hashes = {}
    with tarfile.open(spool, 'w', format=tarfile.PAX_FORMAT, dereference=False) as archive:
        archive.errorlevel = 2
        for entry in part['entries']:
            unchanged(entry)
            path = Path(entry['source'])
            info = archive.gettarinfo(str(path), arcname=entry['name'])
            if entry['kind'] == 'file':
                # Independent parts must not contain cross-part hardlink references.
                info.type, info.linkname, info.size = tarfile.REGTYPE, '', entry['size']
                fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
                with os.fdopen(fd, 'rb') as stream:
                    hashed = HashedReader(stream)
                    archive.addfile(info, hashed)
                    hashes[entry['source']] = hashed.hash.hexdigest()
            else:
                archive.addfile(info)
            unchanged(entry)
            report(current_file=entry['source'])
    return hashes


def remove_verified(part, receipt, report):
    # Validate every remaining member before unlinking any member of this part.
    opened = opened_paths()
    for entry in part['entries']:
        if not entry['remove']:
            continue
        path = Path(entry['source'])
        if not path.exists() and not path.is_symlink():
            continue  # An interrupted deletion can resume from its verified receipt.
        if str(path) in opened:
            raise RuntimeError('Source became open: ' + str(path))
        unchanged(entry)
        if entry['kind'] == 'file' and digest(path) != receipt['files'][entry['source']]:
            raise RuntimeError('Source checksum changed before removal: ' + str(path))
    for entry in part['entries']:
        if not entry['remove']:
            continue
        path = Path(entry['source'])
        if path.exists() or path.is_symlink():
            unchanged(entry)
            path.unlink()
            report(removed_file=entry['source'])


def privileged_operation(audit, part, operation, report):
    """Grant root access only to planned payload roots and this audit directory."""
    plan = json.loads((audit / 'plan.json').read_text())
    roots = [Path(s['path']) for s in plan['roots']]
    protected = [Path(p) for p in plan['protected']]
    opened = opened_paths()
    for entry in part['entries']:
        path = Path(entry['source'])
        if not any(beneath(path, root) for root in roots) or any(beneath(path, p) for p in protected):
            raise RuntimeError('Privileged operation escaped its planned selection')
        if entry['remove'] and str(path) in opened:
            raise RuntimeError('Planned source is open: ' + str(path))
    command = ['docker', 'run', '--rm', '--network', 'none', '--read-only',
               '--cap-drop', 'ALL', '--cap-add', 'DAC_OVERRIDE', '--cap-add', 'CHOWN',
               '--mount', 'type=bind,src=' + str(audit) + ',dst=/audit',
               '--mount', 'type=bind,src=' + str(Path(__file__).resolve()) + ',dst=/migration.py,readonly']
    for root in roots:
        command += ['--mount', 'type=bind,src=' + str(root) + ',dst=' + str(root) +
                    (',readonly' if operation == 'stage' else '')]
    command += ['--entrypoint', '/usr/bin/python3', 'drone-stack:sim-x86',
                '/migration.py', '/audit', '--root-' + operation, part['name']]
    report(True, phase=operation + '_with_payload_permissions')
    subprocess.run(command, check=True)
    if operation == 'stage':
        result = json.loads((audit / (part['name'] + '.root-stage.json')).read_text())
        if result['name'] != part['name']:
            raise RuntimeError('Privileged staging result differs')
        return result['files']


def root_operation(audit, name, operation):
    if os.geteuid() != 0 or audit != Path('/audit'):
        raise RuntimeError('Root operation is restricted to the bounded container audit mount')
    plan = json.loads((audit / 'plan.json').read_text())
    part = next(p for p in plan['parts'] if p['name'] == name)
    roots = [Path(s['path']) for s in plan['roots']]
    protected = [Path(p) for p in plan['protected']]
    for entry in part['entries']:
        path = Path(entry['source'])
        if not any(beneath(path, r) for r in roots) or any(beneath(path, p) for p in protected):
            raise RuntimeError('Root operation includes an unselected source')
    owner = audit.stat()
    last = 0

    def report(force=False, **values):
        nonlocal last
        if force or time.time() - last > 10:
            save(audit / 'root-operation-status.json', dict(operation=operation, name=name,
                                                          updated_at=time.time(), **values))
            os.chown(audit / 'root-operation-status.json', owner.st_uid, owner.st_gid)
            last = time.time()

    if operation == 'stage':
        spool = audit / (name + '.staging')
        hashes = stage(part, spool, report)
        os.chown(spool, owner.st_uid, owner.st_gid)
        result = audit / (name + '.root-stage.json')
        save(result, dict(name=name, files=hashes))
        os.chown(result, owner.st_uid, owner.st_gid)
    else:
        receipt = json.loads((audit / (name + '.verified.json')).read_text())
        if receipt['name'] != name or not receipt.get('verified_at'):
            raise RuntimeError('NAS verification receipt is missing')
        remove_verified(part, receipt, report)


def run(audit):
    with (audit / 'worker.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run_locked(audit)


def run_locked(audit):
    plan = json.loads((audit / 'plan.json').read_text())
    destination, share = Path(plan['destination']), Path(plan['share'])
    if not share.is_dir() or not beneath(destination, share) or destination == share:
        raise RuntimeError('Authenticated NAS share is unavailable or destination is unsafe')
    if (audit / 'COMPLETE.json').exists():
        completed = json.loads((audit / 'COMPLETE.json').read_text())
        expected = dict(state='complete', parts=len(plan['parts']),
                        source_bytes=plan['source_bytes'], destination=str(destination))
        if any(completed.get(key) != value for key, value in expected.items()):
            raise RuntimeError('Local completion receipt does not match this plan')
        for name, value in [('plan.json', plan), ('COMPLETE.json', completed)]:
            if not (destination / name).exists():
                raise RuntimeError('Completed NAS metadata is missing: ' + name)
            remote_json(destination / name, value)
        sums = []
        for part in plan['parts']:
            name = part['name']
            receipt = json.loads((audit / (name + '.verified.json')).read_text())
            removed = json.loads((audit / (name + '.removed.json')).read_text())
            if (removed['sha256'] != receipt['sha256'] or receipt['name'] != name or
                    (destination / name).stat().st_size != receipt['archive_bytes']):
                raise RuntimeError('Completed archive receipt or size changed: ' + name)
            if not (destination / (name + '.json')).exists():
                raise RuntimeError('Completed NAS verification receipt is missing: ' + name)
            remote_json(destination / (name + '.json'), receipt)
            sums.append(receipt['sha256'] + '  ' + name + '\n')
        if (destination / 'SHA256SUMS').read_bytes() != ''.join(sums).encode():
            raise RuntimeError('Completed checksum manifest changed')
        save(audit / 'status.json', dict(phase='complete', completed_parts=len(plan['parts']),
                                       total_parts=len(plan['parts']), source_bytes=plan['source_bytes'],
                                       updated_at=time.time(), completed_at=completed['completed_at']))
        return
    protected = [Path(p) for p in plan['protected']]
    for part in plan['parts']:
        for entry in part['entries']:
            if any(beneath(Path(entry['source']), p) for p in protected):
                raise RuntimeError('Plan includes a protected source')
    destination.mkdir(parents=True, exist_ok=True)
    remote_json(destination / 'plan.json', plan)
    state = dict(phase='starting', updated_at=time.time(), completed_parts=0,
                 total_parts=len(plan['parts']), source_bytes=plan['source_bytes'])
    last = 0

    def report(force=False, **values):
        nonlocal last
        state.update(values)
        if force or time.time() - last > 10:
            state['updated_at'] = time.time()
            save(audit / 'status.json', state)
            print(json.dumps(state), flush=True)
            last = time.time()

    checksums = []
    try:
        for index, part in enumerate(plan['parts'], 1):
            name = part['name']
            done = audit / (name + '.removed.json')
            verified = audit / (name + '.verified.json')
            final = destination / name
            report(True, part=index, name=name, phase='checking')
            if done.exists():
                receipt = json.loads(verified.read_text())
                if final.stat().st_size != receipt['archive_bytes']:
                    raise RuntimeError('Completed NAS archive size changed')
            else:
                spool = audit / (name + '.staging')
                pending = audit / (name + '.staged.json')
                if verified.exists():
                    receipt = json.loads(verified.read_text())
                    if digest(final) != receipt['sha256']:
                        raise RuntimeError('Existing NAS archive checksum differs')
                else:
                    if pending.exists() and spool.exists():
                        receipt = json.loads(pending.read_text())
                        if digest(spool) != receipt['sha256']:
                            raise RuntimeError('Local staged archive checksum differs')
                    else:
                        report(True, phase='staging')
                        if any(e['kind'] == 'file' and not os.access(e['source'], os.R_OK)
                               for e in part['entries']):
                            hashes = privileged_operation(audit, part, 'stage', report)
                        else:
                            hashes = stage(part, spool, report)
                        receipt = dict(name=name, archive_bytes=spool.stat().st_size,
                                       sha256=digest(spool), files=hashes)
                        save(pending, receipt)
                    if final.exists():
                        if digest(final) != receipt['sha256']:
                            raise RuntimeError('Existing destination differs from staged archive')
                    else:
                        report(True, phase='copying_and_verifying')
                        checksum = transfer_archive(spool, final, report)
                        if checksum != receipt['sha256']:
                            raise RuntimeError('Uploaded archive differs from staging receipt')
                    receipt['verified_at'] = time.time()
                    remote_json(destination / (name + '.json'), receipt)
                    save(verified, receipt)
                report(True, phase='removing_verified_sources')
                remaining = [e for e in part['entries'] if e['remove'] and
                             (Path(e['source']).exists() or Path(e['source']).is_symlink())]
                if any(not os.access(Path(e['source']).parent, os.W_OK) or
                       (e['kind'] == 'file' and not os.access(e['source'], os.R_OK))
                       for e in remaining):
                    privileged_operation(audit, part, 'remove', report)
                else:
                    remove_verified(part, receipt, report)
                save(done, dict(name=name, sha256=receipt['sha256'], removed_at=time.time()))
                if spool.exists():
                    spool.unlink()
            checksums.append(receipt['sha256'] + '  ' + name + '\n')
            report(True, phase='between_parts', completed_parts=index)
        payload = ''.join(checksums).encode()
        checksum_path = destination / 'SHA256SUMS'
        if checksum_path.exists():
            if checksum_path.read_bytes() != payload:
                raise RuntimeError('Existing checksum manifest differs')
        else:
            with checksum_path.open('xb') as stream:
                stream.write(payload)
            if checksum_path.read_bytes() != payload:
                raise RuntimeError('Checksum manifest readback differs')
        restore = dict(extract_into=plan['local_root'], verification='sha256sum -c SHA256SUMS',
                       instructions='Verify and extract every independent TAR into extract_into. '
                                    'Original modes, links, timestamps and paths are retained.',
                       retained_sources=plan['retained'])
        remote_json(destination / 'RESTORE.json', restore)
        result = dict(state='complete', completed_at=time.time(), parts=len(plan['parts']),
                      source_bytes=plan['source_bytes'], destination=str(destination))
        remote_json(destination / 'COMPLETE.json', result)
        save(audit / 'COMPLETE.json', result)
        report(True, phase='complete')
    except BaseException as error:
        report(True, phase='error', error=str(error))
        raise


def run_queue(audit, queue_path):
    queue = json.loads(queue_path.read_text())
    completed = []
    with (audit / 'queue.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        state = dict(phase='starting', total_groups=len(queue), completed_groups=completed)
        try:
            for group in queue:
                state.update(phase='archiving', current_group=group['name'], updated_at=time.time())
                save(audit / 'archive-queue-status.json', state)
                run(Path(group['audit']))
                completed.append(group['name'])
            state.update(phase='complete', completed_at=time.time(),
                         source_bytes=sum(g['source_bytes'] for g in queue))
            save(audit / 'archive-queue-COMPLETE.json', state)
            save(audit / 'archive-queue-status.json', state)
        except BaseException as error:
            state.update(phase='error', error=str(error), updated_at=time.time())
            save(audit / 'archive-queue-status.json', state)
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('audit', type=Path)
    parser.add_argument('--prepare', type=Path, help='Freeze an explicit selection; perform no transfer')
    parser.add_argument('--queue', type=Path, help='Complete each already prepared group in order')
    root_args = parser.add_mutually_exclusive_group()
    root_args.add_argument('--root-stage', help=argparse.SUPPRESS)
    root_args.add_argument('--root-remove', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.prepare and args.queue:
        parser.error('--prepare and --queue are mutually exclusive')
    if args.root_stage or args.root_remove:
        if args.prepare or args.queue:
            parser.error('Root operations cannot prepare or run queues')
        root_operation(args.audit, args.root_stage or args.root_remove,
                       'stage' if args.root_stage else 'remove')
    elif args.prepare:
        plan = prepare(json.loads(args.prepare.read_text()), args.audit)
        print(json.dumps(dict(parts=len(plan['parts']), source_bytes=plan['source_bytes'],
                              retained=plan['retained'])))
    elif args.queue:
        run_queue(args.audit, args.queue)
    else:
        run(args.audit)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Move explicit local payloads to verified, resumable TAR archives on SMB."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import tarfile
import time

from verified_archive import digest, transfer


def save(path, value):
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, ensure_ascii=True, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def remote_json(path, value):
    payload = (json.dumps(value, ensure_ascii=True, indent=2) + '\n').encode()
    if path.exists():
        if path.stat().st_size != len(payload) or digest(path) != hashlib.sha256(payload).hexdigest():
            raise RuntimeError('Existing NAS metadata differs: ' + str(path))
        return
    with path.open('xb') as stream:
        for offset in range(0, len(payload), 65536):
            stream.write(payload[offset:offset + 65536])
    if path.stat().st_size != len(payload) or digest(path) != hashlib.sha256(payload).hexdigest():
        raise RuntimeError('NAS metadata readback differs: ' + str(path))


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


def run(audit):
    with (audit / 'worker.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run_locked(audit)


def run_locked(audit):
    plan = json.loads((audit / 'plan.json').read_text())
    destination, share = Path(plan['destination']), Path(plan['share'])
    if not share.is_dir() or not beneath(destination, share) or destination == share:
        raise RuntimeError('Authenticated NAS share is unavailable or destination is unsafe')
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
                        hashes = stage(part, spool, report)
                        receipt = dict(name=name, archive_bytes=spool.stat().st_size,
                                       sha256=digest(spool), files=hashes)
                        save(pending, receipt)
                    if final.exists():
                        if digest(final) != receipt['sha256']:
                            raise RuntimeError('Existing destination differs from staged archive')
                    else:
                        report(True, phase='copying_and_verifying')
                        checksum = transfer(spool, final)
                        if checksum != receipt['sha256']:
                            raise RuntimeError('Uploaded archive differs from staging receipt')
                    receipt['verified_at'] = time.time()
                    remote_json(destination / (name + '.json'), receipt)
                    save(verified, receipt)
                report(True, phase='removing_verified_sources')
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
    args = parser.parse_args()
    if args.prepare and args.queue:
        parser.error('--prepare and --queue are mutually exclusive')
    if args.prepare:
        plan = prepare(json.loads(args.prepare.read_text()), args.audit)
        print(json.dumps(dict(parts=len(plan['parts']), source_bytes=plan['source_bytes'],
                              retained=plan['retained'])))
    elif args.queue:
        run_queue(args.audit, args.queue)
    else:
        run(args.audit)


if __name__ == '__main__':
    main()

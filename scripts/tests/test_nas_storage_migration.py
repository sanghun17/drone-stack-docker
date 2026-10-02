import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

LIB = Path(__file__).resolve().parents[1] / 'lib'
sys.path.insert(0, str(LIB))
spec = importlib.util.spec_from_file_location('nas_storage_migration', LIB / 'nas_storage_migration.py')
migration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(migration)


class MigrationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.source = self.base / 'source'
        self.source.mkdir()
        self.share = self.base / 'nas'
        self.share.mkdir()
        self.destination = self.share / 'archive'
        self.audit = self.base / 'audit'
        self.config = dict(local_root=str(self.base), share=str(self.share),
                           destination=str(self.destination), protected=[],
                           roots=[dict(path=str(self.source), remove='all')], part_bytes=4)

    def copy(self, src, dst, report=None):
        shutil.copyfile(src, dst)
        return migration.digest(dst)

    def run_migration(self, copy=None):
        with patch.object(migration, 'transfer_archive', side_effect=copy or self.copy), contextlib.redirect_stdout(io.StringIO()):
            migration.run(self.audit)

    def test_round_trip_preserves_links_and_independent_hardlinked_parts(self):
        a = self.source / 'a.bag'
        a.write_bytes(b'12345678')
        os.link(a, self.source / 'b.bag')
        (self.source / 'link').symlink_to('a.bag')
        (self.source / 'directory-link').symlink_to('.')
        (self.source / 'empty').mkdir()
        migration.prepare(self.config, self.audit)
        self.run_migration()
        self.assertFalse(a.exists())
        self.assertFalse((self.source / 'directory-link').is_symlink())
        restored = self.base / 'restored'
        restored.mkdir()
        for archive in sorted(self.destination.glob('*.tar')):
            with tarfile.open(archive) as stream:
                stream.extractall(restored)
        self.assertEqual((restored / 'source/a.bag').read_bytes(), b'12345678')
        self.assertEqual((restored / 'source/b.bag').read_bytes(), b'12345678')
        self.assertEqual(os.readlink(restored / 'source/link'), 'a.bag')
        self.assertTrue((restored / 'source/empty').is_dir())
        self.assertTrue((self.destination / 'COMPLETE.json').exists())

    def test_failed_upload_and_source_mutation_do_not_remove_sources(self):
        source = self.source / 'a.bag'
        source.write_bytes(b'original')
        migration.prepare(self.config, self.audit)
        with self.assertRaisesRegex(OSError, 'NAS failed'):
            self.run_migration(lambda src, dst, report: (_ for _ in ()).throw(OSError('NAS failed')))
        self.assertEqual(source.read_bytes(), b'original')

        def changed(src, dst, report):
            checksum = self.copy(src, dst)
            source.write_bytes(b'CHANGED!')
            return checksum

        with self.assertRaisesRegex(RuntimeError, 'Source changed'):
            self.run_migration(changed)
        self.assertEqual(source.read_bytes(), b'CHANGED!')
        self.assertFalse((self.audit / 'COMPLETE.json').exists())

    def test_interrupted_removal_resumes_and_keeps_metadata(self):
        source = self.source / 'a.bag'
        source.write_bytes(b'payload')
        metadata = self.source / 'metrics.json'
        metadata.write_text('{}')
        self.config['roots'][0]['remove'] = 'bags'
        migration.prepare(self.config, self.audit)
        original = migration.remove_verified
        interrupted = False

        def interruption(part, receipt, report):
            nonlocal interrupted
            original(part, receipt, report)
            if not interrupted:
                interrupted = True
                raise OSError('interrupted after unlink')

        with patch.object(migration, 'remove_verified', side_effect=interruption):
            with self.assertRaisesRegex(OSError, 'interrupted after unlink'):
                self.run_migration()
        self.run_migration()
        self.assertFalse(source.exists())
        self.assertTrue(metadata.exists())
        self.assertTrue((self.audit / 'COMPLETE.json').exists())

    def test_only_identical_prefix_is_recognized_as_interrupted_upload(self):
        source = self.source / 'a.tar'
        source.write_bytes(b'123456789')
        partial = self.share / 'a.tar.partial'
        partial.write_bytes(b'12345')
        self.assertTrue(migration.matching_prefix(source, partial))
        partial.write_bytes(b'12346')
        self.assertFalse(migration.matching_prefix(source, partial))

    def test_prefix_read_retry_resumes_both_streams_at_the_same_offset(self):
        source = self.source / 'a.tar'
        payload = bytes(range(256)) * 1024
        source.write_bytes(payload)
        partial = self.share / 'a.tar.partial'
        partial.write_bytes(payload[:180000])
        real_open, failed = Path.open, False

        class InterruptedRead:
            def __init__(self, stream):
                self.stream = stream

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.stream.close()

            def seek(self, offset):
                self.stream.seek(offset)

            def read(self, amount):
                nonlocal failed
                if not failed and self.stream.tell() >= 65536:
                    failed = True
                    raise OSError('transient SMB read error')
                return self.stream.read(amount)

        def open_with_failure(path, *args, **kwargs):
            stream = real_open(path, *args, **kwargs)
            return InterruptedRead(stream) if path == partial else stream

        with patch.object(Path, 'open', open_with_failure), patch.object(migration.time, 'sleep'):
            self.assertTrue(migration.matching_prefix(source, partial))
        self.assertTrue(failed)
        partial.write_bytes(b'1234567890')
        self.assertFalse(migration.matching_prefix(source, partial))

    def test_protected_selection_and_replaced_parent_are_rejected(self):
        source = self.source / 'a.bag'
        source.write_bytes(b'payload')
        self.config['protected'] = [str(self.source)]
        with self.assertRaisesRegex(RuntimeError, 'protected'):
            migration.prepare(self.config, self.audit)
        self.config['protected'] = []
        plan = migration.prepare(self.config, self.audit)
        entry = next(e for p in plan['parts'] for e in p['entries'] if e['kind'] == 'file')
        self.source.rename(self.base / 'moved')
        self.source.symlink_to(self.base / 'moved')
        with self.assertRaisesRegex(RuntimeError, 'identity changed'):
            migration.unchanged(plan['parts'][0]['entries'][0])

    def test_completed_group_can_resume_without_replacing_completion(self):
        (self.source / 'a.bag').write_bytes(b'payload')
        migration.prepare(self.config, self.audit)
        self.run_migration()
        completion = (self.destination / 'COMPLETE.json').read_bytes()
        self.run_migration()
        self.assertEqual((self.destination / 'COMPLETE.json').read_bytes(), completion)
        (self.destination / 'SHA256SUMS').write_text('changed')
        with self.assertRaisesRegex(RuntimeError, 'checksum manifest changed'):
            self.run_migration()

    def test_metadata_retry_recognizes_truncated_write_and_rejects_other_data(self):
        path = self.destination / 'receipt.json'
        self.destination.mkdir()
        payload = b'{"files":"' + b'x' * 150000 + b'"}\n'
        real_open, interrupted = Path.open, False

        class InterruptedWrite:
            def __init__(self, stream):
                self.stream = stream

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.stream.close()

            def write(self, block):
                nonlocal interrupted
                if not interrupted and self.stream.tell() >= 65536:
                    interrupted = True
                    raise OSError('transient SMB write error')
                return self.stream.write(block)

        def open_with_failure(target, *args, **kwargs):
            stream = real_open(target, *args, **kwargs)
            return InterruptedWrite(stream) if target == path and args == ('xb',) else stream

        with patch.object(Path, 'open', open_with_failure), patch.object(migration.time, 'sleep'):
            migration.remote_bytes(path, payload)
        self.assertTrue(interrupted)
        self.assertEqual(path.read_bytes(), payload)
        migration.remote_bytes(path, payload)
        path.write_bytes(b'{"unrecognized":true}\n')
        with self.assertRaisesRegex(RuntimeError, 'metadata differs'):
            migration.remote_bytes(path, payload)
        self.assertEqual(path.read_bytes(), b'{"unrecognized":true}\n')

    def test_multiple_read_disconnects_can_resume_while_bytes_keep_advancing(self):
        source = self.source / 'a.tar'
        partial = self.share / 'a.tar.partial'
        payload = bytes(range(256)) * 4096
        source.write_bytes(payload)
        partial.write_bytes(payload)
        real_open = Path.open

        class OneBlockRead:
            def __init__(self, stream):
                self.stream, self.read_once = stream, False

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.stream.close()

            def seek(self, offset):
                self.stream.seek(offset)

            def read(self, amount):
                if self.read_once:
                    raise OSError('SMB disconnected after making progress')
                self.read_once = True
                return self.stream.read(amount)

        def periodic_disconnect(path, *args, **kwargs):
            stream = real_open(path, *args, **kwargs)
            return OneBlockRead(stream) if path == partial else stream

        with patch.object(Path, 'open', periodic_disconnect), patch.object(migration.time, 'sleep'):
            self.assertTrue(migration.matching_prefix(source, partial))
            self.assertEqual(migration.digest(partial), migration.digest(source))


if __name__ == '__main__':
    unittest.main()

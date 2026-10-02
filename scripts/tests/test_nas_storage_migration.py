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

    def copy(self, src, dst):
        shutil.copyfile(src, dst)
        return migration.digest(dst)

    def run_migration(self, copy=None):
        with patch.object(migration, 'transfer', side_effect=copy or self.copy), contextlib.redirect_stdout(io.StringIO()):
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
            self.run_migration(lambda src, dst: (_ for _ in ()).throw(OSError('NAS failed')))
        self.assertEqual(source.read_bytes(), b'original')

        def changed(src, dst):
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


if __name__ == '__main__':
    unittest.main()

"""Real Git package resolution without network, Docker or private repositories."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('module_sources', ROOT/'scripts/module_sources.py')
modules = importlib.util.module_from_spec(spec)
spec.loader.exec_module(modules)


class ModuleSourceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'workspace'; self.root.mkdir()
        self.repo = Path(self.temp.name)/'remote'; self.repo.mkdir()
        def git(*args):
            return subprocess.check_output(['git', '-C', str(self.repo), *args], stderr=subprocess.DEVNULL)
        self.git = git
        git('init'); git('config', 'user.name', 'test'); git('config', 'user.email', 'test@example.invalid')
        package = self.repo/'deployment/demo'; package.mkdir(parents=True)
        (package/'module.yml').write_text('name: demo\nneeds: []\nrun: [run.sh]\n')
        (package/'run.sh').write_text('#!/bin/sh\necho locked\n')
        (package/'run.sh').chmod(0o755)
        git('add', '.'); git('commit', '-qm', 'package')
        self.entry = modules.package_entry(str(self.repo), 'HEAD', 'deployment/demo', self.repo)

    def test_pinned_package_is_materialized_with_executable_mode(self):
        modules.sync_module(self.root, 'utility/demo', self.entry, 'amd64', download=False)
        target = self.root/'modules/utility/demo'
        self.assertEqual((target/'run.sh').read_text(), '#!/bin/sh\necho locked\n')
        self.assertTrue((target/'run.sh').stat().st_mode & 0o111)
        (self.repo/'deployment/demo/run.sh').write_text('upstream changed\n')
        self.git('add', '.'); self.git('commit', '-qm', 'newer upstream')
        modules.sync_module(self.root, 'utility/demo', self.entry, 'amd64', download=False)
        self.assertIn('echo locked', (target/'run.sh').read_text())

    def test_dirty_or_unmanaged_module_is_preserved(self):
        modules.sync_module(self.root, 'utility/demo', self.entry, 'amd64', download=False)
        target = self.root/'modules/utility/demo/run.sh'; target.write_text('user edit')
        with self.assertRaisesRegex(ValueError, 'preserving edited'):
            modules.sync_module(self.root, 'utility/demo', self.entry, 'amd64', download=False)
        self.assertEqual(target.read_text(), 'user edit')
        unmanaged = self.root/'modules/utility/local'; unmanaged.mkdir()
        with self.assertRaisesRegex(ValueError, 'unmanaged'):
            modules.sync_module(self.root, 'utility/local', self.entry, 'amd64', download=False)

    def test_dependency_cycle_and_unknown_module_fail(self):
        entries = {'base': {'manifest': {}}, 'a': {'manifest': {'needs': ['b']}},
                   'b': {'manifest': {'needs': ['a']}}}
        with self.assertRaisesRegex(ValueError, 'cycle'): modules.order(['a'], entries)
        with self.assertRaisesRegex(ValueError, 'not locked'): modules.order(['missing'], entries)

    def test_symlink_and_traversal_packages_fail(self):
        for path in ('../bad', '/tmp/bad', 'a/../../bad'):
            with self.assertRaises(ValueError): modules.relative(path)
        (self.repo/'deployment/demo/escape').symlink_to('/etc/passwd')
        self.git('add', '.'); self.git('commit', '-qm', 'unsafe entry')
        with self.assertRaisesRegex(ValueError, 'unsupported package entry'):
            modules.package_entry(str(self.repo), 'HEAD', 'deployment/demo', self.repo)

    def test_mismatched_locked_content_is_rejected(self):
        self.entry['files']['run.sh'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'missing/modified'):
            modules.sync_module(self.root, 'utility/demo', self.entry, 'amd64', download=False)
        self.assertFalse((self.root/'modules/utility/demo').exists())

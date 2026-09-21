"""Exercise kernel permission failures as an ordinary Unix user, not mocked calls."""
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


@unittest.skipIf(os.geteuid() == 0, 'root bypasses discretionary filesystem permissions')
class RootGuardTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'checkout'
        self.root.mkdir(mode=0o775)
        self.root.chmod(0o775)
        self.original_mode = stat.S_IMODE(self.root.stat().st_mode)
        self.addCleanup(self.root.chmod, self.original_mode)
        self.run_cmd(['git', 'init', '-q'])
        for name in ['.gitignore', 'scripts/root_guard.py', 'scripts/check_layout.py', 'config/repository-layout.json']:
            dst = self.root / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, dst)
        (self.root / 'README.md').write_text('existing file\n')

    def run_cmd(self, args, ok=True):
        result = subprocess.run(args, cwd=self.root, text=True, capture_output=True)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def guard(self, *args, ok=True):
        return self.run_cmd(['python3', 'scripts/root_guard.py', *args], ok=ok)

    def test_kernel_blocks_shell_python_file_and_symlink_creation(self):
        self.guard('lock')
        result = self.run_cmd(['mkdir', 'experiments'], ok=False)
        self.assertNotEqual(result.returncode, 0)
        with self.assertRaises(PermissionError):
            (self.root / 'scratchpad').mkdir()
        with self.assertRaises(PermissionError):
            (self.root / 'result.txt').write_text('blocked')
        with self.assertRaises(PermissionError):
            (self.root / 'paper_assets').symlink_to(self.temp.name)
        self.assertFalse((self.root / 'experiments').exists())

    def test_child_work_and_git_index_and_existing_file_writes_still_work(self):
        self.guard('lock')
        for name in ['modules', 'scripts', 'stacks', 'config', 'ws', 'flight_logs', '.build']:
            child = self.root / name / 'test-child'
            child.mkdir()
            (child / 'test.txt').write_text('allowed')
        (self.root / 'README.md').write_text('in-place editing is allowed')
        self.run_cmd(['git', 'add', 'README.md'])
        self.assertIn('README.md', self.run_cmd(['git', 'diff', '--cached', '--name-only']).stdout)

    def test_root_atomic_replacement_and_rename_are_blocked(self):
        self.guard('lock')
        tmp = self.root / 'scripts' / 'replacement.md'
        tmp.write_text('replacement')
        with self.assertRaises(PermissionError):
            tmp.replace(self.root / 'README.md')
        with self.assertRaises(PermissionError):
            (self.root / 'modules').rename(self.root / 'experiments')

    def test_lock_is_idempotent_and_unlock_restores_original_permissions(self):
        self.guard('lock')
        self.guard('lock')
        self.assertEqual(stat.S_IMODE(self.root.stat().st_mode), self.original_mode & ~0o222)
        self.guard('unlock')
        self.assertEqual(stat.S_IMODE(self.root.stat().st_mode), self.original_mode)
        (self.root / 'experiments').mkdir()

    def test_maintenance_relocks_after_success_and_failure(self):
        self.guard('lock')
        code = "from pathlib import Path; p=Path('scripts/replacement'); p.write_text('new'); p.replace('README.md')"
        self.guard('maintain', '--', 'python3', '-c', code)
        self.assertEqual((self.root / 'README.md').read_text(), 'new')
        result = self.guard('maintain', '--', 'python3', '-c', 'raise SystemExit(7)', ok=False)
        self.assertEqual(result.returncode, 7)
        with self.assertRaises(PermissionError):
            (self.root / 'experiments').mkdir()

    def test_maintenance_relocks_when_command_is_missing(self):
        self.guard('lock')
        result = self.guard('maintain', '--', '/not-a-real-layout-command', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(stat.S_IMODE(self.root.stat().st_mode), self.original_mode & ~0o222)

    def test_maintenance_reports_violations_but_relocks_without_deleting_data(self):
        self.guard('lock')
        result = self.guard('maintain', '--', 'mkdir', 'experiments', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('experiments', result.stderr)
        self.assertTrue((self.root / 'experiments').is_dir())
        self.assertEqual(stat.S_IMODE(self.root.stat().st_mode), self.original_mode & ~0o222)

    def test_unknown_original_permissions_are_not_guessed(self):
        result = self.guard('unlock', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('no managed root lock', result.stderr)

    def test_invalid_checkout_is_not_locked(self):
        (self.root / 'experiments').mkdir()
        result = self.guard('lock', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(stat.S_IMODE(self.root.stat().st_mode), self.original_mode)

    def test_metadata_cannot_be_reused_for_another_checkout(self):
        self.guard('lock')
        state = self.root / '.git/layout-root-lock.json'
        value = json.loads(state.read_text())
        value['inode'] += 1
        state.write_text(json.dumps(value))
        result = self.guard('unlock', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('different checkout', result.stderr)


if __name__ == '__main__':
    unittest.main()

"""Integration tests against real Git indexes, commits and installed hooks."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class LayoutGuardTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'checkout'
        self.root.mkdir()
        self.git('init', '-q')
        self.git('symbolic-ref', 'HEAD', 'refs/heads/main')
        self.git('config', 'user.name', 'Layout test')
        self.git('config', 'user.email', 'layout-test@example.invalid')
        self.git('config', 'commit.gpgsign', 'false')
        self.git('config', 'core.hooksPath', str(self.root / '.git/hooks'))
        self.git('config', '--unset', 'core.hooksPath')
        for name in ['config/repository-layout.json', 'scripts/check_layout.py',
                     'scripts/install_git_hooks.sh', 'scripts/git-hooks/pre-commit',
                     'scripts/git-hooks/pre-push']:
            p = self.root / name
            p.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / name, p)
        self.write('README.md', 'Runtime checkout\n')
        self.git('add', '.')
        self.git('commit', '-qm', 'initial compliant tree')
        self.run_cmd(['bash', 'scripts/install_git_hooks.sh'])

    def run_cmd(self, args, ok=True, data=None):
        result = subprocess.run(args, cwd=self.root, input=data, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def git(self, *args, **kw):
        return self.run_cmd(['git', *args], **kw)

    def write(self, path, text):
        p = self.root / path
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
        return p

    def check(self, *args, ok=True):
        return self.run_cmd(['python3', 'scripts/check_layout.py', *args], ok=ok)

    def test_clean_tree_and_project_root_contract(self):
        policy = json.loads((self.root / 'config/repository-layout.json').read_text())
        self.assertEqual(set(policy['root_directories']),
                         {'ws', 'modules', 'scripts', 'stacks', 'config', 'flight_logs', 'data'})
        self.check('--staged', '--worktree')
        self.check('--revision', 'HEAD')

    def test_commit_hook_rejects_staged_file_even_when_removed_from_worktree(self):
        p = self.write('tools/analysis.py', 'print(1)\n')
        self.git('add', 'tools/analysis.py')
        p.unlink()
        p.parent.rmdir()
        result = self.git('commit', '-qm', 'bad layout', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('tools/analysis.py', result.stderr)

    def test_unstaged_policy_edit_does_not_weaken_index_check(self):
        self.write('paper_assets/figure.json', '{}')
        self.git('add', 'paper_assets')
        policy = self.root / 'config/repository-layout.json'
        value = json.loads(policy.read_text())
        value['root_directories'].append('paper_assets')
        value['forbidden_directories'].remove('paper_assets')
        policy.write_text(json.dumps(value))
        result = self.check('--staged', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('paper_assets', result.stderr)

    def test_ignored_empty_root_directory_is_rejected(self):
        self.write('.gitignore', 'experiments/\n')
        (self.root / 'experiments').mkdir()
        result = self.check('--worktree', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('experiments/', result.stderr)

    def test_local_workspaces_and_ignored_runtime_builds_are_allowed(self):
        self.write('.gitignore', '/ws/\n/.build/\n/flight_logs/\n__pycache__/\n')
        for path in ['ws/component/results/a.csv', '.build/cache/data.bin',
                     'flight_logs/run.bag', 'scripts/__pycache__/test.pyc']:
            self.write(path, 'local only')
        self.check('--worktree')
        self.git('add', '-f', 'flight_logs/run.bag')
        result = self.check('--staged', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('must not be tracked', result.stderr)

    def test_nested_results_and_unapproved_artifacts_are_rejected(self):
        self.write('stacks/demo/stack.yml', 'modules: []\n')
        self.write('stacks/demo/scripts/results/summary.json', '{}')
        self.write('stacks/demo/config/figure.png', 'not a runtime asset')
        self.git('add', 'stacks')
        result = self.check('--staged', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('results', result.stderr)
        self.assertIn('figure.png', result.stderr)

    def test_stack_manifest_required_and_remote_module_cannot_be_committed(self):
        self.write('stacks/demo/scripts/run.sh', '#!/bin/sh\n')
        self.write('modules/sensor/demo/run.sh', '#!/bin/sh\n')
        self.git('add', '.')
        result = self.check('--staged', ok=False)
        self.assertIn('missing stacks/demo/stack.yml', result.stderr)
        self.assertIn('remote module implementation', result.stderr)
        self.write('stacks/demo/stack.yml', 'modules: []\n')
        self.write('modules/sensor/demo/module.yml', 'name: demo\n')
        self.git('add', '.')
        self.assertNotEqual(self.check('--staged', ok=False).returncode, 0)
        self.git('rm', '-r', '--cached', 'modules')
        self.check('--staged')

    def test_size_limits_are_applied_to_metadata_git_blobs(self):
        path = 'data/manifests/dataset.json'
        self.write(path, '{"fixture": true}')
        self.git('add', '.')
        self.check('--staged')
        policy = self.root / 'config/repository-layout.json'
        value = json.loads(policy.read_text())
        value['max_file_bytes'] = 4
        policy.write_text(json.dumps(value))
        self.git('add', 'config/repository-layout.json')
        (self.root / path).write_text('')  # smaller unstaged file must not hide large staged blob
        result = self.check('--staged', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('exceeds allowed', result.stderr)

    def test_analysis_is_tracked_but_forced_data_payloads_are_rejected(self):
        self.write('data/analysis/risk-aware/verify.py', 'print("verify")\n')
        self.write('data/manifests/schema.json', '{}')
        self.git('add', 'data')
        self.check('--staged', '--worktree')
        self.write('.gitignore', '/data/results/\n/data/assets/\n/data/archive/\n')
        self.write('data/results/experiment/raw.bag', 'local recording')
        self.check('--worktree')
        self.git('add', '-f', 'data/results/experiment/raw.bag')
        result = self.git('commit', '-qm', 'accidental payload', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('local data payload', result.stderr)

    def test_analysis_output_must_not_be_committed(self):
        self.write('data/analysis/risk-aware/figure.png', 'output')
        self.git('add', 'data')
        self.assertIn('output/binary artifact', self.check('--staged', ok=False).stderr)

    def test_external_and_retired_paths_cannot_return(self):
        paths = [
            'modules/training/demo/module.yml',
            'modules/control/aruco-landing/module.yml',
            'modules/odometry/landing-vision-pose/module.yml',
            'modules/sensor/realsense-d435i/calibration/923322070596/imu_raw_accel.txt',
            'scripts/_vnc_gui.sh',
            'stacks/ete-train-5090/stack.yml',
        ]
        for path in paths:
            self.write(path, 'retired fixture\n')
        self.git('add', '.')
        result = self.check('--staged', ok=False)
        self.assertNotEqual(result.returncode, 0)
        for path in paths:
            self.assertIn(path, result.stderr)
        self.write('.gitignore', 'modules/\nstacks/\n')
        result = self.check('--worktree', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('modules/training', result.stderr)
        self.assertIn('stacks/ete-train-5090', result.stderr)

    def test_symlink_cannot_hide_an_archive_in_scripts(self):
        (self.root / 'scripts/archive_link').symlink_to('/tmp')
        self.git('add', '.')
        result = self.check('--staged', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('symlinks/submodules', result.stderr)

    def test_committed_tree_check_ignores_clean_worktree_substitute(self):
        self.write('scratchpad/note.md', 'research')
        self.git('add', '.')
        self.git('-c', 'core.hooksPath=/dev/null', 'commit', '-qm', 'bypass local commit hook')
        shutil.rmtree(self.root / 'scratchpad')
        result = self.check('--revision', 'HEAD', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('scratchpad', result.stderr)

    def test_pre_push_blocks_bad_intermediate_commit_even_after_cleanup(self):
        remote = Path(self.temp.name) / 'remote.git'
        self.run_cmd(['git', 'init', '-q', '--bare', str(remote)])
        self.git('remote', 'add', 'origin', str(remote))
        self.git('push', '-q', 'origin', 'main')
        old = self.git('rev-parse', 'origin/main').stdout.strip()
        self.write('experiments/run.json', '{}')
        self.git('add', '.')
        self.git('-c', 'core.hooksPath=/dev/null', 'commit', '-qm', 'bad intermediate tree')
        self.git('rm', '-qr', 'experiments')
        self.git('commit', '-qm', 'clean tip')
        result = self.git('push', 'origin', 'main', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('experiments/run.json', result.stderr)
        remote_head = self.run_cmd(['git', '--git-dir', str(remote), 'rev-parse', 'refs/heads/main']).stdout.strip()
        self.assertEqual(old, remote_head)

    def test_installer_is_idempotent_and_preserves_custom_hooks(self):
        self.run_cmd(['bash', 'scripts/install_git_hooks.sh'])
        self.assertEqual(self.git('config', '--get', 'core.hooksPath').stdout.strip(), 'scripts/git-hooks')
        self.git('config', 'core.hooksPath', '/custom/hooks')
        result = self.run_cmd(['bash', 'scripts/install_git_hooks.sh'], ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.git('config', '--get', 'core.hooksPath').stdout.strip(), '/custom/hooks')


if __name__ == '__main__':
    unittest.main()

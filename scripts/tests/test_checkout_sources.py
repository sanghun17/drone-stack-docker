"""A deployed patch stack stays reusable without overwriting local edits."""
import difflib
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'checkout_sources', Path(__file__).resolve().parents[1] / 'lib/checkout_sources.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PatchPrefixTests(unittest.TestCase):
    def test_overlapping_stack_and_partial_upgrade_preserve_worktree(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            checkout = root / 'checkout'
            checkout.mkdir()
            subprocess.run(['git', 'init', '-q', str(checkout)], check=True)
            original = 'first\nmiddle\nlast\n'
            first = 'first\nversion one\nlast\n'
            second = 'first\nversion two\nlast\n'
            patches = []
            for index, (before, after) in enumerate(((original, first), (first, second))):
                patch = root / f'{index}.patch'
                patch.write_text(''.join(difflib.unified_diff(
                    before.splitlines(True), after.splitlines(True),
                    fromfile='a/source.txt', tofile='b/source.txt')))
                patches.append(patch)
            source = checkout / 'source.txt'
            for text, expected in ((original, 0), (first, 1), (second, 2),
                                   ('first\nuser edit\nlast\n', 0)):
                source.write_text(text)
                self.assertEqual(module.applied_patch_prefix(checkout, patches), expected)
                self.assertEqual(source.read_text(), text)

    def test_added_file_and_empty_stack(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            checkout = root / 'checkout'
            checkout.mkdir()
            patch = root / 'add.patch'
            patch.write_text('diff --git a/new.txt b/new.txt\nnew file mode 100644\n'
                             '--- /dev/null\n+++ b/new.txt\n@@ -0,0 +1 @@\n+new file\n')
            self.assertEqual(module.applied_patch_prefix(checkout, [patch]), 0)
            (checkout / 'new.txt').write_text('new file\n')
            self.assertEqual(module.applied_patch_prefix(checkout, [patch]), 1)
            self.assertTrue((checkout / 'new.txt').exists())
            self.assertEqual(module.applied_patch_prefix(checkout, []), 0)


if __name__ == '__main__':
    unittest.main()

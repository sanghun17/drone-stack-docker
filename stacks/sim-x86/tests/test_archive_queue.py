"""A partial or failed NAS queue must never authorize collection."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('archive',ROOT/'scripts/lib/verified_archive.py')
archive=importlib.util.module_from_spec(spec);spec.loader.exec_module(archive)


class ArchiveQueueTests(unittest.TestCase):
    def test_failure_blocks_completion_and_later_archives(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);plans=[]
            for i in range(2):
                folder=root/str(i);folder.mkdir();p=folder/'plan.json'
                p.write_text('{}');plans.append(str(p))
            queue=root/'queue.json';queue.write_text(json.dumps(dict(plans=plans)))
            with patch.object(archive.subprocess,'run',side_effect=subprocess.CalledProcessError(1,'archive')) as run:
                with self.assertRaises(subprocess.CalledProcessError):archive.archive_queue(queue)
            self.assertEqual(run.call_count,1)
            self.assertFalse((root/'COMPLETE.json').exists())
            self.assertEqual(json.loads((root/'status.json').read_text())['state'],'review_required')

    def test_existing_completion_requires_matching_local_and_remote_receipts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'audit';folder.mkdir();nas=root/'nas';nas.mkdir()
            source=root/'old.bag';dest=nas/'old.bag';dest.write_bytes(b'evidence')
            entry=dict(source=str(source),destination=str(dest),size=8)
            plan=dict(destination=str(nas),total_files=1,total_bytes=8,entries=[entry])
            p=folder/'plan.json';p.write_text(json.dumps(plan));(nas/'plan.json').write_text(json.dumps(plan))
            receipt=dict(state='complete',files=1,bytes=8,destination=str(nas))
            for parent in [folder,nas]:(parent/'COMPLETE.json').write_text(json.dumps(receipt))
            source.with_suffix('.bag.nas.json').write_text(json.dumps(dict(entry=entry,sha256='previously_verified')))
            queue=root/'queue.json';queue.write_text(json.dumps(dict(plans=[str(p)])))
            dest.write_bytes(b'changed')
            with self.assertRaises(RuntimeError):archive.archive_queue(queue)
            self.assertFalse((root/'COMPLETE.json').exists())
            dest.write_bytes(b'evidence')
            with patch.object(archive.subprocess,'run') as run:archive.archive_queue(queue)
            run.assert_not_called()
            self.assertEqual(json.loads((root/'COMPLETE.json').read_text())['plans'],[str(p)])


if __name__=='__main__':unittest.main()

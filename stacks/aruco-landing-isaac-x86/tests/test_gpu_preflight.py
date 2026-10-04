import importlib.util
from pathlib import Path
import subprocess
import unittest
from unittest import mock

path=Path(__file__).parents[1]/'scripts/preflight.py'
spec=importlib.util.spec_from_file_location('gpu_preflight',path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
GOOD='GPU-5f5e6979-51fc-3166-7a87-4e264c81e4dc'
BAD='GPU-8149eb0b-023a-17dd-b942-c622f0b8c4ca'


class PreflightTest(unittest.TestCase):
    def test_denied_gpu_is_rejected_without_querying_it(self):
        with mock.patch.dict(module.os.environ,{'DSD_DENIED_GPU_UUIDS':BAD},clear=True), \
                mock.patch.object(module.subprocess,'run') as query:
            with self.assertRaisesRegex(RuntimeError,'explicitly denied'): module.check(BAD)
            query.assert_not_called()

    def test_failed_slot_still_bound_to_nvidia_stops_evaluation(self):
        with mock.patch.dict(module.os.environ,{'ISAAC_REQUIRED_ISOLATION_PCI':'0000:1a:00.0'},clear=True), \
                mock.patch.object(module,'Path') as path, mock.patch.object(module.subprocess,'run') as query:
            driver=path.return_value.__truediv__.return_value.__truediv__.return_value
            driver.is_symlink.return_value=True
            driver.resolve.return_value.name='nvidia'
            with self.assertRaisesRegex(RuntimeError,'still attached'): module.check(GOOD)
            query.assert_not_called()

    def test_detached_slot_allows_only_selected_gpu_query(self):
        output=GOOD+', NVIDIA GeForce RTX 2080 Ti, 570.211.01, 11264\n'
        with mock.patch.dict(module.os.environ,{'DSD_DENIED_GPU_UUIDS':BAD,
                'ISAAC_REQUIRED_ISOLATION_PCI':'0000:1a:00.0'},clear=True), \
                mock.patch.object(module,'Path') as path, \
                mock.patch.object(module.subprocess,'run',return_value=subprocess.CompletedProcess([],0,output,'')) as query:
            path.return_value.__truediv__.return_value.__truediv__.return_value.is_symlink.return_value=False
            self.assertTrue(module.check(GOOD)['rtx_driver_floor_passed'])
            self.assertIn('--id='+GOOD,query.call_args.args[0])
            self.assertNotIn('--id='+BAD,query.call_args.args[0])


if __name__=='__main__': unittest.main()

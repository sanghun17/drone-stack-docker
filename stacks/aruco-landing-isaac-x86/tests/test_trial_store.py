import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

path=Path(__file__).parents[1]/'scripts/trial_store.py'
spec=importlib.util.spec_from_file_location('trial_store',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class TrialStoreTest(unittest.TestCase):
    def test_resume_checks_settings_and_preserves_completed_trials(self):
        with tempfile.TemporaryDirectory() as temp:
            store=module.TrialStore(temp,{'seed':1701})
            store.write(dict(trial_id=7,outcome='touchdown'))
            # A leftover interrupted temporary write is never counted as complete.
            (Path(temp)/'trial-0000008.tmp').write_text('{}')
            resumed=module.TrialStore(temp,{'seed':1701},resume=True)
            self.assertEqual(list(resumed.completed),[7])
            with self.assertRaises(ValueError): resumed.write(dict(trial_id=7))
            with self.assertRaises(ValueError): module.TrialStore(temp,{'seed':42},resume=True)
            with self.assertRaises(ValueError): module.TrialStore(temp,{'seed':1701})

    def test_completed_trace_is_durable_and_corruption_blocks_resume(self):
        with tempfile.TemporaryDirectory() as temp:
            store=module.TrialStore(temp,{'seed':1})
            store.write(dict(trial_id=1,outcome='touchdown'),trace={'capture_time_s':np.array([0.,.1])})
            row=json.loads((Path(temp)/'trial-0000001.json').read_text())
            trace=Path(temp)/row['trace']['path']
            with np.load(trace,allow_pickle=False) as data:
                np.testing.assert_array_equal(data['capture_time_s'],[0.,.1])
            self.assertEqual(list(module.TrialStore(temp,{'seed':1},resume=True).completed),[1])
            trace.write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError,'missing or corrupt'):
                module.TrialStore(temp,{'seed':1},resume=True)


if __name__=='__main__': unittest.main()

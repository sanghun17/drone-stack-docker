import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

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


if __name__=='__main__': unittest.main()

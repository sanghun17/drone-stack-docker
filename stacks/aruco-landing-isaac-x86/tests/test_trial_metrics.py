import importlib.util
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np


root=Path(__file__).parents[3]
sys.path.insert(0,str(root/'stacks/aruco-landing-isaac-x86/scripts'))
from trial_trace import TrialTrace
from trial_store import TrialStore
path=root/'data/analysis/aruco/isaac_trial_metrics.py'
spec=importlib.util.spec_from_file_location('isaac_trial_metrics',path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TrialMetricsTest(unittest.TestCase):
    def test_raw_recompute_and_pooled_rmse_weight_valid_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            store=TrialStore(temp,{'termination':'vision_height_threshold'})
            policy=SimpleNamespace(state='descending',stamp=0.,visible=True)
            for trial, errors in enumerate(([1.,1.,None],[3.])):
                trace=TrialTrace(trial,np.eye(4),[7],.1)
                for frame, error in enumerate(errors):
                    estimated=np.eye(4)
                    estimated[0,3]=error or 0.
                    observation=None if error is None else dict(camera_from_pad=np.linalg.inv(estimated),
                        inlier_ids=[7],rms_px=0.)
                    trace.append(frame*.1,np.eye(4),np.zeros(6),observation,
                                 [] if error is None else [7],np.zeros(4),policy)
                store.write(dict(trial_id=trial,outcome='touchdown',success=True,lateral_error_m=0.),trace.arrays())
            _, rows=module.recompute(Path(temp))
            result=module.summarize(rows)
            self.assertAlmostEqual(result['localization_camera']['position_rmse_m'],np.sqrt(11/3))
            self.assertEqual(result['capture_frames'],4)
            self.assertEqual(result['marker_detection_availability'],.75)
            self.assertAlmostEqual(result['macro_mean_marker_availability'],5/6)
            (Path(temp)/'trace-0000000.npz').write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError,'checksum mismatch'): module.recompute(Path(temp))

    def test_legacy_results_do_not_invent_missing_frame_data(self):
        with tempfile.TemporaryDirectory() as temp:
            store=TrialStore(temp,{'termination':'vision_height_threshold'})
            store.write(dict(trial_id=0,outcome='touchdown'))
            with self.assertRaisesRegex(ValueError,'no frame trace'): module.recompute(Path(temp))


if __name__=='__main__': unittest.main()

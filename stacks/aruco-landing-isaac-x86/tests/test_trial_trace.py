import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np


path=Path(__file__).parents[1]/'scripts/trial_trace.py'
spec=importlib.util.spec_from_file_location('trial_trace', path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class TrialTraceTest(unittest.TestCase):
    def test_rmse_uses_matching_capture_truth_and_availability_counts_rejected_poses(self):
        mount=np.eye(4); mount[2,3]=-.1
        trace=module.TrialTrace(7, mount, [7], .1)
        policy=SimpleNamespace(state='descending', stamp=0., visible=True)
        truth=np.eye(4); truth[:3,3]=[1.,2.,3.]
        estimate=truth.copy(); estimate[:3,3]+=[.03,.04,0.]
        observation=dict(camera_from_pad=np.linalg.inv(estimate),inlier_ids=[7],rms_px=.5)
        trace.append(0., truth, np.zeros(6), observation, [7], np.zeros(4), policy)
        # The recorder must retain capture values if the caller later changes buffers.
        truth[0,3]=10.
        trace.append(.1, truth, np.zeros(6), None, [7], np.zeros(4), policy)
        trace.append(.2, truth, np.zeros(6), None, [99], np.zeros(4), policy)
        metrics=module.trace_metrics(trace.arrays())
        self.assertEqual(metrics['capture_frames'], 3)
        self.assertAlmostEqual(metrics['marker_detection_availability'], 2/3)
        self.assertAlmostEqual(metrics['pose_availability'], 1/3)
        self.assertAlmostEqual(metrics['localization_camera']['position_rmse_m'], .05)
        np.testing.assert_allclose(metrics['localization_camera']['position_axis_rmse_m'], [.03,.04,0.])
        self.assertAlmostEqual(metrics['localization_body']['position_rmse_m'], .05)
        self.assertEqual(metrics['longest_pose_dropout_frames'], 2)
        self.assertEqual(metrics['longest_marker_dropout_frames'], 1)

    def test_all_missing_poses_have_null_rmse_and_full_dropout(self):
        trace=module.TrialTrace(0,np.eye(4),[7],.05)
        policy=SimpleNamespace(state='waiting',stamp=None,visible=False)
        for now in (0.,.05): trace.append(now,np.eye(4),np.zeros(6),None,[],np.zeros(4),policy)
        metrics=module.trace_metrics(trace.arrays())
        self.assertIsNone(metrics['localization_camera']['position_rmse_m'])
        self.assertEqual(metrics['pose_availability'],0.)
        self.assertEqual(metrics['longest_marker_dropout_s'],.1)

    def test_moving_truth_and_camera_mount_rotation_affect_body_position_error(self):
        mount=np.eye(4);mount[0,3]=1.
        trace=module.TrialTrace(1,mount,[7],.1)
        policy=SimpleNamespace(state='descending',stamp=0.,visible=True)
        for i in range(2):
            truth=np.eye(4);truth[0,3]=i*10.
            estimate=truth.copy();estimate[:3,:3]=[[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]
            trace.append(i*.1,truth,np.zeros(6),dict(camera_from_pad=np.linalg.inv(estimate),
                inlier_ids=[7],rms_px=0.),[7],np.zeros(4),policy)
        metrics=module.trace_metrics(trace.arrays())
        self.assertEqual(metrics['localization_camera']['position_rmse_m'],0.)
        self.assertAlmostEqual(metrics['localization_camera']['rotation_rmse_deg'],90.)
        self.assertAlmostEqual(metrics['localization_body']['position_rmse_m'],np.sqrt(2))

    def test_reject_duplicate_capture_timestamps(self):
        trace=module.TrialTrace(0,np.eye(4),[],.1)
        policy=SimpleNamespace(state='waiting',stamp=None,visible=False)
        for _ in range(2): trace.append(0.,np.eye(4),np.zeros(6),None,[],np.zeros(4),policy)
        with self.assertRaisesRegex(ValueError,'increasing'): module.trace_metrics(trace.arrays())


if __name__=='__main__': unittest.main()

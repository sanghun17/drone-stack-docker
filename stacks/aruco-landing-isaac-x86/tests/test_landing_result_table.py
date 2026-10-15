from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'data/analysis/aruco'))
sys.path.insert(0,str(ROOT/'stacks/aruco-landing-isaac-x86/scripts'))
from isaac_landing_result_table import frame_metrics, aggregate
from trial_trace import TrialTrace, trace_metrics


class LandingTableTest(unittest.TestCase):
    def trace(self):
        truth=np.repeat(np.eye(4)[None],3,axis=0)
        truth[:,:3,:3]=np.diag([1,-1,-1])
        truth[:,2,3]=[1,2,1]
        truth[2,0,3]=.45
        return dict(capture_time_s=np.array([0.,.1,.2]),sample_period_s=np.array(.1),body_from_camera=np.eye(4),
            gt_pad_from_camera=truth,estimated_pad_from_camera=np.full((3,4,4),np.nan),
            pad_marker_ids=np.array([7]),decoded_marker_ids=np.array([7]),
            decoded_marker_offsets=np.array([0,1,1,1]))

    def stats(self,trace,**kw):
        model=np.array([[[-.1,.1,0],[.1,.1,0],[.1,-.1,0],[-.1,-.1,0]]])
        camera=dict(width=100,height=100,fx=200,fy=200,cx=50,cy=50)
        return frame_metrics(trace,model,camera,40,**kw)

    def test_geometry_zero_small_partial_corner_is_independent_of_failed_estimator(self):
        stats,frames=self.stats(self.trace())
        self.assertEqual(frames['valid_marker_count'].tolist(),[1,0,0])
        self.assertEqual(stats['global_minimum_markers'],0)
        self.assertFalse(stats['fully_visible'])
        self.assertAlmostEqual(stats['mean_valid_markers'],1/3)
        self.assertAlmostEqual(stats['std_valid_markers'],np.sqrt(2/9))
        self.assertEqual(stats['valid_pose_frames'],0)
        self.assertIsNone(stats['localization_camera']['position_rmse_m'])
        self.assertIsNone(stats['evaluation_complete'])

    def test_upright_marker_uses_full_image_width_and_correct_two_h_denominator(self):
        from isaac_paper_grid import geometric_availability
        side,height,fx,resolution=.2,1.,200.,100
        tangent_half_fov=resolution/(2*fx)
        expected=side*resolution/(2*height*tangent_half_fov)
        self.assertEqual(expected,40.)
        trace=self.trace();truth=trace['gt_pad_from_camera'][:1]
        model=np.array([[[-.1,.1,0],[.1,.1,0],[.1,-.1,0],[-.1,-.1,0]]])
        camera=dict(width=resolution,height=resolution,fx=fx,fy=fx,cx=50,cy=50)
        visible,_=geometric_availability(truth,model,camera,expected)
        too_small,_=geometric_availability(truth,model,camera,expected+1e-6)
        self.assertTrue(visible[0]);self.assertFalse(too_small[0])

    def test_explicit_phase_and_truncation_prevent_false_fully_visible(self):
        trace=self.trace()
        trace['evaluation_phase']=np.array([1,0,0])
        trace['reference_time_s']=np.array([0.,.1,.2])
        stats,_=self.stats(trace,scope='landing',interval=[0.,2.])
        self.assertTrue(stats['fully_visible_observed'])
        self.assertFalse(stats['evaluation_complete'])
        self.assertFalse(stats['fully_visible'])
        with self.assertRaisesRegex(ValueError,'explicit evaluation_phase'):
            self.stats(self.trace(),scope='landing')

    def test_no_finite_ground_truth_is_not_invented_zero_visibility(self):
        trace=self.trace();trace['gt_pad_from_camera'][0,0,3]=np.nan
        with self.assertRaisesRegex(ValueError,'nonfinite ground truth'): self.stats(trace)

    def test_table_body_rmse_includes_rotated_camera_mount_lever_arm(self):
        trace=self.trace()
        trace['estimated_pad_from_camera']=trace['gt_pad_from_camera'].copy()
        angle=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        trace['estimated_pad_from_camera'][:,:3,:3]=trace['gt_pad_from_camera'][:,:3,:3] @ angle
        trace['body_from_camera'][0,3]=1.
        stats,_=self.stats(trace)
        self.assertEqual(stats['localization_camera']['position_rmse_m'],0.)
        self.assertAlmostEqual(stats['localization_body']['position_rmse_m'],np.sqrt(2))
        row=aggregate([dict(configuration='Pad1',trajectory_type='Mixed',td_error_m=.03,**stats)])[0]
        self.assertAlmostEqual(row['LocalizationRMSE_m'],np.sqrt(2))
        self.assertEqual(row['CameraLocalizationRMSE_m'],0.)
        self.assertEqual(row['TDError_m'],.03)

    def test_equal_trial_population_moments_differ_from_pooled_frames(self):
        stats,_=self.stats(self.trace())
        common=dict(configuration='Pad1',trajectory_type='Mixed',td_error_m=None,**stats)
        a=dict(common,capture_frames=1,mean_valid_markers=0.,second_moment_valid_markers=0.)
        b=dict(common,capture_frames=3,mean_valid_markers=2.,second_moment_valid_markers=4.)
        row=aggregate([a,b])[0]
        self.assertEqual(row['MeanValidMarkers'],1.)
        self.assertEqual(row['StdValidMarkers'],1.)
        self.assertEqual(row['PooledMeanValidMarkers'],1.5)
        self.assertIsNone(row['LocalizationRMSE_m'])
        self.assertEqual(row['EvaluationCompletenessUnknownTrials'],2)

    def test_rmse_and_td_table_use_trial_arithmetic_mean_and_sample_sd(self):
        stats,_=self.stats(self.trace())
        base=dict(configuration='Pad1',trajectory_type='Mixed',**stats)
        rows=[]
        for value in (1.,3.):
            metric=dict(valid_frames=1,position_rmse_m=value)
            rows.append(dict(base,valid_pose_frames=1,localization_camera=metric,
                             localization_body=metric,td_error_m=value/100))
        table=aggregate(rows)[0]
        self.assertEqual(table['LocalizationRMSE_m'],2.)
        self.assertAlmostEqual(table['BodyLocalizationRMSEStd_m'],np.sqrt(2))
        self.assertAlmostEqual(table['BodyTrialRMSOfRMSE_m'],np.sqrt(5))
        self.assertEqual(table['TDError_m'],.02)
        self.assertAlmostEqual(table['TDStd_m'],np.sqrt(2)/100)

    def test_optional_reference_trace_fields_preserve_legacy_metrics_and_copy_vectors(self):
        trace=TrialTrace(0,np.eye(4),[7],.1)
        policy=SimpleNamespace(state='descending',stamp=0.,visible=False)
        sample={key:np.array([0.,0.,1.]) for key in
                ('position_pad_m','velocity_pad_m','acceleration_pad_m','jerk_pad_m')}
        trace.append(0.,np.eye(4),np.zeros(6),None,[],np.zeros(4),policy,
                     reference_time_s=0.,evaluation_phase=1,reference_sample=sample)
        sample['position_pad_m'][0]=99
        arrays=trace.arrays()
        self.assertEqual(int(arrays['schema_version']),1)
        self.assertEqual(arrays['reference_camera_position_pad_m'][0,0],0.)
        self.assertEqual(arrays['evaluation_phase'].tolist(),[1])
        self.assertEqual(trace_metrics(arrays)['capture_frames'],1)
        with self.assertRaisesRegex(ValueError,'supplied together'):
            trace.append(.1,np.eye(4),np.zeros(6),None,[],np.zeros(4),policy,reference_time_s=.1)
        trace.append(.1,np.eye(4),np.zeros(6),None,[],np.zeros(4),policy)
        with self.assertRaisesRegex(ValueError,'every logged capture'):trace.arrays()


if __name__=='__main__': unittest.main()

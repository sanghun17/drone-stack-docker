"""Focused adapter tests, using downloaded unchanged legacy policy fixtures."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parent/'scripts'))
if (HERE/'fixtures').exists():
    sys.path.insert(0, str(HERE/'fixtures'))  # Offline source snapshots only.
else:
    sys.path.insert(0, str(HERE.parents[2]/'ws/aruco-landing/src/aruco_landing/src'))
from trajectory_reference import (CameraReferenceBank, CameraReferencePolicy, EXPECTED_PROFILE,
    body_state_from_camera_reference, camera_command_to_body_velocity)
from aruco_landing.batch_landing import LandingPolicy
from reference_evaluation import load_reference, controller_contract

POLICY = dict(horizontal_reference='camera', height_reference='camera', kp_xy=1., kd_xy=.15,
    speed_limit_m_s=2., descent_speed_m_s=.55, h_min_m=.2, derivative_filter_tau_s=.1,
    derivative_speed_limit_m_s=2., acquisition_timeout_s=2., loss_timeout_s=.5,
    latency_compensation_enabled=True, max_pose_prediction_s=.2, touchdown_stop_lead_s=0.,
    yaw_kp=1., yaw_rate_limit_rad_s=.35, yaw_deadband_deg=1.)


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'synthetic.npz'
        # Synthetic two-piece polynomial with exact shifted coefficients.
        poly = np.array([.2, -.03, .02, -.01, .002, -.0001])
        self.c = np.zeros((1, 3, 3, 6)); self.c[:, 2] = np.nan
        self.c[0, 0, 0] = poly
        shifted = np.polynomial.Polynomial(poly)(np.polynomial.Polynomial([1., 1.])).coef
        self.c[0, 1, 0] = shifted
        self.c[0, 0, 2] = [2., -.55, 0, 0, 0, 0]
        self.c[0, 1, 2] = [1.45, -.55, 0, 0, 0, 0]
        self.write()
        self.bank = CameraReferenceBank(self.path, self.spec())

    def tearDown(self):
        self.temp.cleanup()

    def write(self):
        np.savez(self.path, coefficients=self.c, durations=[[1., (1.8/.55-1), 0.]],
                 piece_counts=[2], candidate_ids=[19], **self.embedded())

    def embedded(self):
        return dict(EXPECTED_PROFILE,profile_sha256='fixture',trajectory_type='Mixed')

    def spec(self):
        return dict(self.embedded(),seed=20261013,sha256=hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_variable_knots_padding_endpoint_and_raw_descent(self):
        times = np.array([0., .999999999, 1., 2., (1.8/.55)])
        r = self.bank.sample(0, times)
        self.assertTrue(np.isfinite(r['position']).all())
        np.testing.assert_array_equal(r['position'][:, 2], 2.-.55*times)
        np.testing.assert_array_equal(r['velocity'][:, 2], np.full(5, -.55))
        np.testing.assert_allclose(r['quaternion_xyzw']**2 @ np.ones(4), 1., atol=1e-14)
        self.assertEqual(self.bank.trial_metadata(0)['raw_pair_id'], 19)
        self.assertEqual(set(self.bank.trace_sample(0, 0)),
                         {'position_pad_m', 'velocity_pad_m', 'acceleration_pad_m', 'jerk_pad_m'})
        for t in (.3, 1., 2.7):
            eps = 1e-5
            before, after, at = self.bank.sample(0, t-eps), self.bank.sample(0, t+eps), self.bank.sample(0, t)
            for primitive, derivative in [('position','velocity'),('velocity','acceleration'),('acceleration','jerk')]:
                np.testing.assert_allclose((after[primitive]-before[primitive])/(2*eps), at[derivative], atol=2e-9)
        for t in (-.001, np.nan, (1.8/.55+.001)):
            with self.assertRaises(ValueError): self.bank.sample(0, t)
        with self.assertRaises(ValueError): self.bank.sample(.2, 0)

    def test_shared_piece_durations_and_invalid_provenance(self):
        np.savez(self.path, coefficients=self.c[:, :2], durations=[1., (1.8/.55-1)], candidate_ids=[19], **self.embedded())
        bank = CameraReferenceBank(self.path, self.spec())
        np.testing.assert_allclose(bank.sample(0, 2.)['position'], self.bank.sample(0, 2.)['position'])
        wrong = self.spec(); wrong['funnel_halfwidth_top_m'] = .35
        with self.assertRaises(ValueError): CameraReferenceBank(self.path, wrong)
        wrong = self.spec(); wrong['sha256'] = 'bad'
        with self.assertRaises(ValueError): CameraReferenceBank(self.path, wrong)
        np.savez(self.path, coefficients=self.c[:, :2], durations=[1., (1.8/.55-1)], candidate_ids=[19])
        with self.assertRaises(ValueError): CameraReferenceBank(self.path, self.spec())
        self.c[0, 0, 2, 2] = .01; self.write()
        with self.assertRaises(ValueError): CameraReferenceBank(self.path, self.spec())

    def test_nonhover_mount_state_roundtrip_and_command(self):
        reference = self.bank.sample(0, 0)
        offset = np.array([0, 0, -.1])
        body = body_state_from_camera_reference(reference, offset, .002)
        rotation = reference['rotation']
        np.testing.assert_allclose(body['position'] + rotation@offset - [0, 0, .002], reference['position'])
        mount_velocity = rotation@np.cross(reference['body_rates'], offset)
        np.testing.assert_allclose(body['velocity']+mount_velocity, reference['velocity'])
        self.assertNotEqual(body['velocity'][2], 0)
        np.testing.assert_allclose(body['angular_velocity_world'], rotation@reference['body_rates'])
        np.testing.assert_allclose(camera_command_to_body_velocity(reference['velocity'], rotation,
            reference['body_rates'], offset), body['velocity'])
        # Independent rotation trajectory: mount position derivative equals correction.
        def rotation_y(t):
            return np.array([[np.cos(t),0,np.sin(t)],[0,1,0],[-np.sin(t),0,np.cos(t)]])
        t, eps = .3, 1e-6
        finite = (rotation_y(t+eps)@offset-rotation_y(t-eps)@offset)/(2*eps)
        np.testing.assert_allclose(finite, rotation_y(t)@np.cross([0,1,0], offset), atol=1e-11)

    def test_optical_only_tracking_missing_pose_and_original_events(self):
        legacy = LandingPolicy(POLICY.copy())
        tracker = CameraReferencePolicy(legacy, self.bank, 0)
        p = np.eye(4); p[:3, 3] = self.bank.sample(0, 0)['position']
        tracker.submit(0, p, 0)
        command = tracker.command(0)
        np.testing.assert_allclose(command[:2], self.bank.sample(0, 0)['velocity'][:2], atol=1e-8)
        self.assertAlmostEqual(command[2], -.55,places=7)
        # No GT argument exists: shifted optical measurement alone changes feedback.
        second = CameraReferencePolicy(LandingPolicy(POLICY.copy()), self.bank, 0)
        p2 = p.copy(); p2[0, 3] += .1
        second.submit(0, p2, 0)
        self.assertAlmostEqual(second.command(0)[0]-command[0], -.1, places=7)
        tracker.submit(.1, None, 0)
        np.testing.assert_array_equal(tracker.command(.1), np.zeros(4))
        tracker.command(.6)
        self.assertEqual(tracker.state, 'aborted')
        self.assertTrue(tracker.phase_result(.6)['truncated_reference_phase'])
        endpoint = CameraReferencePolicy(LandingPolicy(POLICY.copy()), self.bank, 0)
        end_pose = np.eye(4); end_pose[:3, 3] = self.bank.sample(0, (1.8/.55))['position']
        end_pose[2,3]=.2
        endpoint.submit((1.8/.55), end_pose, 0)
        np.testing.assert_array_equal(endpoint.command((1.8/.55)), np.zeros(4))
        result = endpoint.phase_result((1.8/.55))
        self.assertFalse(result['truncated_reference_phase'])
        self.assertEqual(result['vision_height_event_time_s'], (1.8/.55))
        self.assertEqual(result['reference_complete_time_s'], (1.8/.55))

    def test_frozen_ready_bootstrap_and_subsequent_delivery_latency(self):
        tracker=CameraReferencePolicy(LandingPolicy(POLICY.copy()),self.bank,0)
        pose=np.eye(4);pose[:3,3]=self.bank.sample(0,0.)['position']
        self.assertTrue(tracker.bootstrap_ready_pose(pose))
        self.assertEqual(tracker.visible_count,0)
        self.assertEqual(tracker.state,'descending')
        self.assertAlmostEqual(tracker.command(0.)[2],-.55,places=7)
        tracker.submit(.1,pose,.1)
        tracker.command(.15);self.assertEqual(tracker.stamp,0.)
        tracker.command(.2);self.assertEqual(tracker.stamp,.1)
        failed=CameraReferencePolicy(LandingPolicy(POLICY.copy()),self.bank,0)
        self.assertFalse(failed.bootstrap_ready_pose(None))
        np.testing.assert_array_equal(failed.command(0.),np.zeros(4))
        self.assertEqual(failed.phase_result(0.)['acquisition_failure_time_s'],0.)

    def test_reference_contract_provenance_policy_and_endpoint_schedule(self):
        specification=Path(self.temp.name)/'spec.json';specification.write_text(json.dumps(self.spec()))
        config=dict(initial_protocol=dict(epsilon_m=.05),policy=POLICY.copy(),physics_dt_s=1/120,control_decimation=2,
            reference_bank=dict(path=str(self.path),specification_path=str(specification),
                reference_ids=[0],classification_interval_s=[0,1/.55],evaluation_interval_s=[0,(1.8/.55)],reference_point='camera_center'))
        bank,ids=load_reference(config)
        self.assertEqual(len(bank),1);np.testing.assert_array_equal(ids,[0])
        self.assertIn('ready optical frame',controller_contract(bank)['startup'])
        config['policy']['height_reference']='vehicle'
        with self.assertRaises(ValueError):load_reference(config)
        config['policy']=POLICY.copy();config['physics_dt_s']=.017
        bank,ids=load_reference(config)
        self.assertEqual(bank.capture_clocks(.017,2)[-1],bank.duration)
        self.assertEqual(load_reference({}),(None,None))


if __name__ == '__main__': unittest.main()

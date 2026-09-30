"""Check optical/body conventions against physical camera locations."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'stacks/sim-x86/scripts'))
from prepare_fast_livo_runtime import calibration


class CameraCalibrationTests(unittest.TestCase):
    def test_depth_point_projects_into_offset_rgb_camera(self):
        profile = yaml.safe_load((ROOT/'stacks/sim-x86/config/airsim_cameras.yaml').read_text())
        with patch('camera_geometry.rospy.Time.now'):
            cfg = calibration(profile)
        ext = cfg['extrin_calib']
        point = np.array([0., 0., 2.])
        body = np.array(ext['extrinsic_R']).reshape(3, 3) @ point + ext['extrinsic_T']
        np.testing.assert_allclose(body, [2.25, 0., .25], atol=1e-12)
        rgb = np.array(ext['Rcl']).reshape(3, 3) @ point + ext['Pcl']
        np.testing.assert_allclose(rgb, [-.15, 0., 2.], atol=1e-12)
        self.assertTrue(cfg['common']['online_intrinsics_en'])
        self.assertFalse(cfg['laserMapping']['reinitialize_with_gt_odom'])

    def test_geometry_is_derived_from_profile_and_ambiguous_inputs_rejected(self):
        profile = yaml.safe_load((ROOT/'stacks/sim-x86/config/airsim_cameras.yaml').read_text())
        profile['streams'][0]['camera']['mount_frd']['y'] = .3
        with patch('camera_geometry.rospy.Time.now'):
            ext = calibration(profile)['extrin_calib']
            np.testing.assert_allclose(ext['Pcl'], [-.3, 0., 0.], atol=1e-12)
            profile['streams'].append(profile['streams'][0])
            with self.assertRaises(ValueError):
                calibration(profile)


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Compose simulator-specific FAST-LIVO calibration from the shared cameras.

The estimator's own launch supplies its tuning. This overlay changes only
camera calibration, using the same geometry implementation as the publisher.
"""
from pathlib import Path
import sys

import numpy as np
import rospy
from tf.transformations import quaternion_matrix
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'modules/simulation/airsim/scripts'))
from camera_geometry import make_static_transforms


def calibration(profile):
    mounts = {}
    for kind in (0, 1):
        streams = [s for s in profile['streams'] if s['image_type'] == kind]
        if len(streams) != 1 or streams[0]['ros']['base_frame'] != 'base_link':
            raise ValueError('Expected one RGB and one depth camera on base_link')
        stream = streams[0]
        matrix = np.eye(4)
        for transform in make_static_transforms(stream['camera'], stream['ros']):
            q, t = transform.transform.rotation, transform.transform.translation
            element = quaternion_matrix([q.x, q.y, q.z, q.w])
            element[:3, 3] = [t.x, t.y, t.z]
            matrix = matrix @ element
        mounts[kind] = matrix
    camera_from_depth = np.linalg.inv(mounts[0]) @ mounts[1]
    return {
        'extrin_calib': {
            'extrinsic_T': mounts[1][:3, 3].tolist(),
            'extrinsic_R': mounts[1][:3, :3].ravel().tolist(),
            'Pcl': camera_from_depth[:3, 3].tolist(),
            'Rcl': camera_from_depth[:3, :3].ravel().tolist(),
        },
        'common': {
            'online_intrinsics_en': True,
            'cam_info_topic': next(s['ros']['camera_info_topic'] for s in profile['streams']
                                   if s['image_type'] == 0),
            # The stack supervises startup. An initial absolute /clock jump
            # must not expire the native one-shot CameraInfo wait.
            'cam_info_timeout': 0.0,
        },
        'laserMapping': {'reinitialize_with_gt_odom': False},
    }


def main():
    rospy.init_node('prepare_fast_livo_runtime', anonymous=True)
    profile = Path(rospy.get_param('/comparison/airsim_camera_profile'))
    output = ROOT / '.build/sim-x86/fast_livo_calibration.yaml'
    output.parent.mkdir(parents=True, exist_ok=True)
    overlay = calibration(yaml.safe_load(profile.read_text()))
    output.write_text(yaml.safe_dump(overlay))
    rospy.set_param('/comparison/fast_livo_calibration', {
        'profile': str(profile), 'overlay': overlay,
        'initial_alignment': 'native first GT pose only; no in-flight reinitialization',
    })
    print('FAST-LIVO common camera calibration: ' + str(output))


if __name__ == '__main__':
    main()

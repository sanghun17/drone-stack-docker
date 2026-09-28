#!/usr/bin/env python3
"""Derive RHEM's sensor and planner inputs from the active common sim profile."""
import math
from pathlib import Path
import re

import rospy
from sensor_msgs.msg import CameraInfo
import tf2_ros
import yaml
from rhem_filter_config import apply_covariance_profile, apply_image_gate, info_section

ROOT = Path(__file__).resolve().parents[3]


def main():
    rospy.init_node('prepare_rhem_runtime', anonymous=True)
    if rospy.get_param('/system/platform') != 'sim':
        raise SystemExit('The sim-x86 RHEM adapter requires platform=sim')
    info = rospy.wait_for_message(rospy.get_param('/system/camera_info_topic'), CameraInfo, timeout=15)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    body = rospy.get_param('/system/body_frame')
    transform = buffer.lookup_transform(body, info.header.frame_id, rospy.Time(0), rospy.Duration(15)).transform
    output = ROOT / '.build/sim-x86/rhem'
    output.mkdir(parents=True, exist_ok=True)
    camera = {
        'image_width': info.width, 'image_height': info.height, 'camera_name': 'shared_airsim_camera',
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': list(info.K)},
        'distortion_model': info.distortion_model,
        'distortion_coefficients': {'rows': 1, 'cols': len(info.D), 'data': list(info.D)},
    }
    (output / 'camera.yaml').write_text(yaml.safe_dump(camera))
    src = ROOT / 'ws/rhem/src/rhem_planner'
    config = (src / 'rovio_bsp/cfg/rovio.info').read_text()
    filter_profile = rospy.get_param('/comparison/rhem_filter_profile', 'historical')
    if filter_profile in ('upstream', 'gated', 'gated-fine', 'gated-bounded-bias'):
        config = apply_covariance_profile(config,
            (ROOT / 'stacks/sim-x86/config/rhem_rovio_covariance.info').read_text())
        if filter_profile in ('gated', 'gated-fine', 'gated-bounded-bias'):
            config = apply_image_gate(config, 5.99)
        if filter_profile == 'gated-bounded-bias':
            # Explicit simulator diagnostic: limit the covariance allowing
            # erroneous visual matches to move the gyro-bias estimate.
            for section, value in [(('Init', 'Covariance'), '1e-7'),
                                   (('Prediction', 'PredictionNoise'), '1e-11')]:
                first, last = info_section(config, section)
                block, count = re.subn(r'(?m)^(\s*gyb_[012]\s+)[^;\n]+;',
                                       lambda m: m[1] + value + ';', config[first:last])
                if count != 3:
                    raise ValueError('Expected three gyro-bias covariance entries')
                config = config[:first] + block + config[last:]
        if filter_profile == 'gated-fine':
            # Include native-resolution texture in the photometric update.
            config, count = re.subn(r'(?m)^(\s*endLevel\s+)[^;\n]+;', r'\g<1>0;', config)
            if count != 1:
                raise ValueError('Expected one ROVIO endLevel')
    elif filter_profile != 'historical':
        raise ValueError(f'Unknown ROVIO filter profile: {filter_profile}')
    # Tracker windows cannot connect to the host display from the runtime container.
    config = re.sub(r'(?m)^(\s*doFrameVisualisation\s+)true;', r'\g<1>false;', config)
    config = re.sub(r'(?m)^(\s*visualizePatches\s+)true;', r'\g<1>false;', config)
    fov_x = math.degrees(2 * math.atan(info.width / (2 * info.K[0])))
    fov_y = math.degrees(2 * math.atan(info.height / (2 * info.K[4])))
    settings = {**{f'qCM_{k}': getattr(transform.rotation, k) for k in 'xyzw'},
                **{f'MrMC_{k}': getattr(transform.translation, k) for k in 'xyz'},
                'cam_FoVx': fov_x, 'cam_FoVy': fov_y,
                'cam_FoVz': rospy.get_param('/planning/shared/sensor_max_range')}
    for key, value in settings.items():
        config, count = re.subn(r'(?m)^(\s*' + re.escape(key) + r'\s+)[^;\n]+;',
                                lambda m: m[1] + str(value) + ';', config)
        if count != 1:
            raise ValueError(f'Expected one {key} in pinned ROVIO configuration, found {count}')
    (output / 'rovio.info').write_text(config)
    params = yaml.safe_load((src / 'bsp_planner/cfg/bsp_settings.yaml').read_text())
    belief_mode = rospy.get_param('/comparison/rhem_belief_mode', 'rovio')
    if belief_mode not in ('rovio', 'disabled'):
        raise ValueError('Unknown RHEM belief mode')
    if belief_mode == 'disabled' and any(rospy.get_param('/comparison/sources/'+k) != 'gt'
            for k in ('planning_source', 'control_source')):
        raise ValueError('Belief isolation requires GT planning and control')
    limits = rospy.get_param('/planning/shared')
    bounds = rospy.get_param('/target_bounding_volume')
    params.update({
        'system/v_max': limits['max_vel_xy'], 'system/dyaw_max': limits['max_yaw_rate'],
        'system/camera/pitch': [0.0], 'system/camera/horizontal': [fov_x],
        'system/camera/vertical': [fov_y], 'bsp/enable': belief_mode == 'rovio',
        'nbvp/gain/range': limits['sensor_max_range'],
        'bbx/explorationExtensionX': 0.0, 'bbx/explorationExtensionY': 0.0,
        'bbx/explorationMinZ': bounds['z_min'], 'bbx/explorationMaxZ': 0.0,
        'resolution': rospy.get_param('/system/voxel_size'),
        'sensor_max_range': limits['sensor_max_range'],
        'treat_unknown_as_occupied': False, 'change_detection_enabled': False,
        'probability_hit': 0.65, 'probability_miss': 0.4,
        'threshold_min': 0.12, 'threshold_max': 0.97, 'threshold_occupancy': 0.7,
        'map_publish_frequency': 2.0,
    })
    for axis in 'xyz':
        params[f'bbx/min{axis.upper()}'] = bounds[f'{axis}_min']
        params[f'bbx/max{axis.upper()}'] = bounds[f'{axis}_max']
    progress_profile = rospy.get_param('/comparison/rhem_progress_profile', 'historical')
    if progress_profile in ('persistent', 'exploratory'):
        # Keep the existing distance penalty through the full mission instead
        # of rewarding arbitrarily long branches after five planning cycles.
        params['nbvp/gain/degressive_switchoffLoops'] = 2147483647
        if progress_profile == 'exploratory':
            # Retain distance attenuation while allowing a branch through the
            # current room to compete with nearby, repeatedly viewed frontiers.
            params['nbvp/gain/degressive_coeff'] = 0.15
    elif progress_profile != 'historical':
        raise ValueError(f'Unknown RHEM progress profile: {progress_profile}')
    if rospy.get_param('/comparison/rhem_gt_conservative', False):
        if any(rospy.get_param('/comparison/sources/'+k) != 'gt'
               for k in ('planning_source', 'control_source')):
            raise ValueError('Conservative diagnostic requires GT sources')
        params.update({'bbx/minZ': .8, 'bbx/maxZ': 1.8,
                       'system/bbx/z': .5, 'system/bbx/z_offset': 0.})
    (output / 'planner.yaml').write_text(yaml.safe_dump(params))
    print(f'RHEM runtime config: {output}; camera {info.width}x{info.height}, body={body}')


if __name__ == '__main__':
    main()

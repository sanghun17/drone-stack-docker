#!/usr/bin/env python3
"""Export stack trials to LA/PURE analysis CSVs without changing raw evidence.

Run in the comparison ROS environment. RosTime uses the first relevant bag
message, matching ExperimentPlotter; flight takeover is recorded separately.
GT coverage uses the historical sensor-cloud/latest-GT-pose accumulation.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import rosbag
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'stacks/sim-x86/scripts'))
from experiment_metrics import MapMetrics, LEGACY
from eval_core import pointcloud2_to_xyz, voxel_hash
sys.path.insert(0, str(LEGACY.parent / 'src/experiments'))
from experiment_plotter import ExperimentPlotter

MAP_TOPICS = {
    'rhem': {'/rhem/bsp_planner/octomap_occupied': ('occupied', 'markers'),
             '/rhem/bsp_planner/octomap_free': ('free', 'markers')},
    'pure': {'/planner/voxblox_node/tsdf_pointcloud': ('both', 'tsdf')},
    'la': {'/sdf_map/occupancy_all': ('occupied', 'points'),
           '/sdf_map/free': ('free', 'points')},
}
HEADER = ['MapName', 'RosTime', 'WallTime', 'GTOdomX', 'GTOdomY', 'GTOdomZ',
          'VIOX', 'VIOY', 'VIOZ', 'SurfaceRateVio', 'VolumeRateVio', 'VolumeM3', 'SurfaceRateGT']
UNITS = ['Unit', 'seconds', 'seconds'] + ['meters'] * 6 + ['ratio_0_1', 'ratio_0_1', 'm3', 'ratio_0_1']


def planner_endpoint_type(reason):
    """Preserve the historical L endpoint while retaining estimator-specific reasons."""
    failure=reason.split(':',1)[1]
    if failure=='LOCALIZATION' or failure.startswith('LOCALIZATION_'):
        return 'L'
    return {'NOVIEWPOINT':'N','COLLISION':'C','SAFE_ZONE_VIOLATION':'V'}.get(failure,'U')


def recorded_camera_transform(bag, frame, base='base_link'):
    """Resolve the actual static optical mount recorded by the common bridge."""
    edges={}
    for _,msg,_ in bag.read_messages(topics=['/tf_static']):
        for edge in msg.transforms:
            q=edge.transform.rotation;t=edge.transform.translation
            value=(edge.header.frame_id,ExperimentPlotter._quat_to_R([q.x,q.y,q.z,q.w]),
                   np.array([t.x,t.y,t.z]))
            old=edges.get(edge.child_frame_id)
            if old is not None and (old[0]!=value[0] or not np.allclose(old[1],value[1])
                                    or not np.allclose(old[2],value[2])):
                raise ValueError('Conflicting static transform: '+edge.child_frame_id)
            edges[edge.child_frame_id]=value
    rotation=np.eye(3);translation=np.zeros(3);visited=set()
    while frame!=base:
        if frame in visited or frame not in edges:
            raise ValueError('Missing/cyclic recorded camera transform: '+frame)
        visited.add(frame)
        parent,r,t=edges[frame]
        rotation=r@rotation;translation=r@translation+t;frame=parent
    return rotation,translation


def export(trial, output, gt_path, cadence=5.):
    result = json.loads((trial / 'result.json').read_text())
    if hashlib.sha256(gt_path.read_bytes()).hexdigest() != result['gt_sha256']:
        raise ValueError('GT PLY hash differs from recorded trial')
    params = yaml.safe_load((trial / 'parameters.yaml').read_text())
    def get_param(name):
        value = params
        for part in name.strip('/').split('/'):
            value = value[part]
        return value
    metrics = MapMetrics(result['planner'], get_param, gt_path, subscribe=False)
    maps = MAP_TOPICS[result['planner']]
    topics = list(maps) + ['/gt_odom', '/robot/odom', '/voxel_grid/output',
                           '/collision', '/unreal_ros_client/collision']
    output.mkdir(parents=True, exist_ok=False)
    rot = ExperimentPlotter._quat_to_R
    cam_rotation = rot(ExperimentPlotter._CAM_IN_BASE_QUAT)
    cam_translation = ExperimentPlotter._CAM_IN_BASE_TRANS
    camera_frame='camera_left_optical_frame'
    calibration=params.get('comparison',{}).get('sensor_calibration','historical')
    camera_policy='historical fixed ExperimentPlotter mount'
    gt_rotation = gt_translation = None
    gt_pos = vio_pos = ['', '', '']
    gt_dense, vio_dense, collisions = [], [], []
    gt_surface = set()
    t0 = next_sample = last_stamp = None
    last_gt_stamp = last_vio_stamp = None
    rows = cloud_count = 0
    with rosbag.Bag(str(trial / 'flight.bag')) as bag, (output / 'experiment_metrics.csv').open('x') as f:
        if calibration=='airsim':
            camera_frame='camera_depth_optical_frame'
            cam_rotation,cam_translation=recorded_camera_transform(bag,camera_frame)
            camera_policy='common AirSim depth optical mount from recorded /tf_static'
        counts = {k: v.message_count for k, v in bag.get_type_and_topic_info().topics.items()}
        required = ['/gt_odom', '/robot/odom', '/voxel_grid/output'] + list(maps)
        missing = [t for t in required if not counts.get(t)]
        if missing:
            raise ValueError('Missing analysis inputs: ' + ', '.join(missing))
        writer = csv.writer(f)
        writer.writerow(HEADER); writer.writerow(UNITS)
        for topic, msg, stamp in bag.read_messages(topics=topics):
            now = stamp.to_sec(); last_stamp = now
            if t0 is None:
                t0 = next_sample = now
            if topic in maps:
                metrics.receive(msg, maps[topic])
            elif topic in ('/gt_odom', '/robot/odom'):
                p = msg.pose.pose.position
                pos = [p.x, p.y, p.z]
                if topic == '/gt_odom':
                    gt_pos = pos; last_gt_stamp = now
                    q = msg.pose.pose.orientation
                    gt_rotation = rot([q.x, q.y, q.z, q.w])
                    gt_translation = np.asarray(pos)
                    gt_dense.append((now - t0, *pos))
                else:
                    vio_pos = pos; last_vio_stamp = now
                    vio_dense.append((now - t0, *pos))
            elif topic in ('/collision', '/unreal_ros_client/collision'):
                collisions.append((now, str(msg.data)))
            elif topic == '/voxel_grid/output' and gt_rotation is not None:
                if msg.header.frame_id != camera_frame:
                    raise ValueError('Unexpected sensor-cloud frame: ' + msg.header.frame_id)
                points = pointcloud2_to_xyz(msg).astype(np.float64)
                world = (points @ cam_rotation.T + cam_translation) @ gt_rotation.T + gt_translation
                gt_surface.update(voxel_hash(world[metrics.bbox.contains(world)], metrics.voxel))
                cloud_count += 1
            while now >= next_sample:
                m = metrics.sample()
                # The historical exporter emits zero map rates before the first map.
                values = [m[k] if m else 0. for k in ('surface_rate_vio', 'volume_rate_vio', 'volume_m3')]
                elapsed = next_sample - t0
                writer.writerow([f'{rows:05d}', f'{elapsed:.3f}', f'{elapsed:.3f}',
                                 *gt_pos, *vio_pos, *values, len(metrics.gt & gt_surface) / len(metrics.gt)])
                rows += 1; next_sample += cadence
    if not gt_dense or not vio_dense:
        raise ValueError('No usable GT/VIO position samples in the recording')
    ExperimentPlotter._write_gt_vs_vio_csv(str(output / 'gt_vs_vio.csv'), gt_dense, vio_dense)
    reason = result['termination']
    end_time = next((json.loads(line)['ros_time'] for line in (trial / 'events.jsonl').read_text().splitlines()
                    if json.loads(line)['phase'] == 'E.stop'), last_stamp)
    end_type = {'time_limit': 'T', 'coverage': 'S', 'collision': 'C'}.get(reason, 'U')
    if collisions:
        end_time, collision_reason = collisions[0]
        end_type = 'C'
        reason = 'bag collision event: ' + collision_reason
    elif reason.startswith('planner_failure:'):
        end_type = planner_endpoint_type(reason)
    with (output / 'endpoint.csv').open('x') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'type', 'reason']); writer.writerow(['seconds', 'C|L|N|T|S|V|U', ''])
        writer.writerow([f'{end_time-t0:.3f}', end_type, reason])
    # Keep the actual last pre-event samples, not the nearest 5-second metric row.
    def last_before(data):
        pts = [p for p in data if p[0] <= end_time - t0]
        return {'time': pts[-1][0], 'xyz': list(pts[-1][1:])} if pts else None
    if last_before(gt_dense) is None or last_before(vio_dense) is None:
        raise ValueError('No GT/VIO samples before the endpoint; raw failure evidence retained')
    metadata = dict(source_trial=str(trial), time_origin_ros=t0,
                    time_origin='first relevant bag message (historical LA/PURE convention)',
                    takeover_ros_time=result.get('takeover_ros_time'),
                    recorded_termination=result['termination'], valid_evaluation=result.get('valid_evaluation', False), endpoint_type=end_type,
                    endpoint_time=end_time-t0, endpoint_gt=last_before(gt_dense), endpoint_vio=last_before(vio_dense),
                    bag_last_gt=dict(ros_time=last_gt_stamp, xyz=gt_pos),
                    bag_last_vio=dict(ros_time=last_vio_stamp, xyz=vio_pos),
                    gt_cloud_messages=cloud_count, metric_rows=rows, bag_topics=counts,
                    gt_sha256=result['gt_sha256'], cadence_s=cadence,
                    surface_gt_policy='historical cloud accumulation with latest GT pose; no odometry interpolation',
                    camera_in_base_translation=cam_translation.tolist(),
                    camera_in_base_quaternion=Rotation.from_matrix(cam_rotation).as_quat().tolist(),
                    camera_in_base_rotation=cam_rotation.tolist(),
                    camera_frame=camera_frame,camera_calibration_policy=camera_policy,
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output / 'analysis_metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'{trial.name}: {rows} metric rows, {cloud_count} clouds, endpoint {end_type} at {end_time-t0:.3f}s', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gt-ply', type=Path, default=ROOT / 'data/assets/gt/ModernLivingroom_long_ros.ply')
    args = parser.parse_args()
    export(args.trial, args.output, args.gt_ply)


if __name__ == '__main__':
    main()

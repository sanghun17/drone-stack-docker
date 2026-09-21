#!/usr/bin/env python3
"""Replay PD on recorded control-camera inputs; no closed-loop dynamics model.

Run with ROS Noetic sourced and /usr/bin/python3. Bag receipt order approximates
callback availability. Compare the reconstructed old cap against logged commands.
Only logged DESCEND intervals with an active downward controller command count.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag
import yaml


def controller_configs(tree):
    if isinstance(tree, dict):
        for key, value in tree.items():
            if key == 'landing_trial_controller' and isinstance(value, dict):
                yield value
            yield from controller_configs(value)
    elif isinstance(tree, list):
        for value in tree:
            yield from controller_configs(value)


def summarize(values):
    return dict(zip(('median', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, [50, 95, 99, 100]))))


def replay(bag_path, output):
    manifest = bag_path.with_suffix('.runtime_manifest.yaml')
    configs = list(controller_configs(yaml.safe_load(manifest.read_text())))
    if not configs:
        raise ValueError('Missing recorded controller configuration')
    keys = ['kp_xy', 'kd_xy', 'derivative_filter_tau_s',
            'derivative_speed_limit_mps', 'max_pose_prediction_s',
            'latency_compensation_enabled', 'horizontal_speed_limit_mps',
            'horizontal_reference']
    config = {k: configs[0][k] for k in keys}
    assert all(all(c[k] == config[k] for k in keys) for c in configs)
    assert config['horizontal_reference'] == 'camera'
    pose_topic = '/landing/trial/camera_pose_pad'
    cmd_topic = '/landing/trial/landing_cmd_pad'
    status_topic = '/landing/trial/status'
    derivative = np.zeros(2)
    error = None
    stamp = None
    phase = None
    sources = set()
    rows = []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, receipt in bag.read_messages(topics=[pose_topic, cmd_topic, status_topic]):
            if topic == status_topic:
                status = json.loads(msg.data)
                new_phase = status['phase']
                # Reset is called when a new trial prepares, before approach.
                if new_phase == 'PRESTREAM' and phase != new_phase:
                    derivative = np.zeros(2)
                    error = stamp = None
                phase = new_phase
                if phase == 'DESCEND':
                    sources.add(status.get('estimation_source'))
            elif topic == pose_topic:
                pos = msg.pose.pose.position
                new_error = -np.array([pos.x, pos.y])
                new_stamp = msg.header.stamp.to_sec()
                if stamp is not None and 0 < new_stamp - stamp <= .25:
                    dt = new_stamp - stamp
                    alpha = dt / (config['derivative_filter_tau_s'] + dt)
                    derivative = (1-alpha)*derivative + alpha*(new_error-error)/dt
                    derivative *= min(1., config['derivative_speed_limit_mps'] /
                                      max(np.linalg.norm(derivative), 1e-12))
                error, stamp = new_error, new_stamp
            elif phase == 'DESCEND' and msg.twist.linear.z < 0 and error is not None:
                now = msg.header.stamp.to_sec()
                age = np.clip(now-stamp, 0, config['max_pose_prediction_s'])
                predicted = error + derivative*age if config['latency_compensation_enabled'] else error
                raw = config['kp_xy']*predicted + config['kd_xy']*derivative
                speed = float(np.linalg.norm(raw))
                old = raw*min(1., config['horizontal_speed_limit_mps']/max(speed, 1e-12))
                actual = np.array([msg.twist.linear.x, msg.twist.linear.y])
                rows.append(dict(time_s=now, error_m=float(np.linalg.norm(error)),
                                 raw_speed_mps=speed, cap2_speed_mps=min(2., speed),
                                 logged_speed_mps=float(np.linalg.norm(actual)),
                                 old_replay_error_mps=float(np.linalg.norm(old-actual)),
                                 pose_age_s=float(now-stamp)))
    if not rows:
        raise ValueError('No active descent samples: '+str(bag_path))
    with (output / (bag_path.stem+'.csv')).open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result = dict(flight=bag_path.stem, source_bag=str(bag_path),
                  runtime_manifest=str(manifest), config=config,
                  estimation_sources=sorted(sources), samples=len(rows))
    for key in ('error_m', 'raw_speed_mps', 'cap2_speed_mps', 'logged_speed_mps', 'old_replay_error_mps'):
        result[key] = summarize([r[key] for r in rows])
    speeds = np.array([r['raw_speed_mps'] for r in rows])
    result['fraction_above'] = {str(v): float(np.mean(speeds > v)) for v in [.5, 1., 1.5, 2.]}
    return result, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bags', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(args.bags), 1, figsize=(10, 2.5*len(args.bags)), squeeze=False)
    for axis, path in zip(axes[:, 0], args.bags):
        result, rows = replay(path, args.output)
        results.append(result)
        times = np.array([r['time_s'] for r in rows]); times -= times[0]
        axis.plot(times, [r['cap2_speed_mps'] for r in rows], label='Replayed cap 2.0')
        axis.plot(times, [r['logged_speed_mps'] for r in rows], alpha=.7, label='Recorded cap 0.5')
        axis.axhline(.5, color='gray', linestyle='--')
        axis.set(title=path.stem, ylabel='Horizontal command (m/s)', ylim=(0, 2.1))
        axis.legend(loc='upper right')
    axes[-1, 0].set_xlabel('Seconds from first active descent command')
    fig.tight_layout(); fig.savefig(args.output/'commands.png', dpi=150)
    report = dict(method='Fixed recorded camera trajectory, PD and latency replay at logged command timestamps; bag receipt order approximates callbacks.',
                  limitations='Not a closed-loop trajectory or stability prediction. Descent timing/gating remains recorded at 0.3 m/s. Pad-frame controller commands, before final frame transform and second cap. Exact reset/callback times unavailable; validate against old commands.',
                  trials=results)
    (args.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()

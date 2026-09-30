"""Independent, read-only review metrics for a frozen simulation campaign."""
import csv
import json
from pathlib import Path
import re
import sys
import math

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'data/analysis/risk-aware/planner-runtime'))


def rows(path):
    if not path.exists():
        return []
    with path.open() as stream:
        return [r for r in csv.DictReader(stream) if None not in r.values()]


def coverage_review(trial, policy):
    """Compare only actual samples before each checkpoint; never extrapolate."""
    control = trial / 'control.log'
    if not control.exists():
        return [], None
    match = re.search(r'\[INFO\] \[([0-9.]+), ([0-9.]+)\]: \[SO3-Control\] Takeover at new traj_id=',
                      control.read_text(errors='replace'))
    if not match:
        return [], None
    start = float(match.group(2))
    samples = [(float(r['ros_time']) - start, r) for r in rows(trial / 'metrics.csv')]
    motion = [(float(r['ros_time'])-start,tuple(float(r[k]) for k in 'xyz'))
              for r in rows(trial/'motion.csv')]
    checks = []
    consecutive = 0
    reason = None
    for checkpoint, reference in policy['checkpoints'].items():
        at = float(checkpoint)
        if not samples or samples[-1][0] < at:
            break
        available = [r for t, r in samples if 0 <= t <= at and at-t <= 7.5]
        if not available:
            consecutive = 0
            checks.append(dict(time_s=at, state='missing_sample'))
            continue
        values = {k: float(available[-1][k]) for k in reference['metrics'] if k!='gt_distance_m'}
        if 'gt_distance_m' in reference['metrics']:
            pts=[p for t,p in motion if 0<=t<=at]
            if len(pts)<2:
                raise ValueError('Missing GT motion for completed checkpoint')
            values['gt_distance_m']=sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))
        if any(not math.isfinite(v) for v in values.values()):
            raise ValueError('Non-finite coverage or motion at checkpoint '+str(at))
        outside = [k for k, v in values.items() if not
                   reference['metrics'][k]['review_lower'] <= v <= reference['metrics'][k]['review_upper']]
        checks.append(dict(time_s=at, reference_n=reference['n'], values=values, outside=outside))
        consecutive = consecutive + 1 if outside else 0
        if consecutive >= policy['consecutive_outside_checkpoints']:
            reason = 'observed_volume_or_rate_outside_reference'
    return checks, reason


def active_estimators(manifest):
    """Only estimators actually used for belief/planning/control can fail a trial."""
    names=[]
    if manifest.get('planner')=='rhem' and manifest.get('rhem_belief_mode','rovio')=='rovio':
        names.append('rovio')
    if 'fast-livo' in (manifest.get('planning_source'),manifest.get('control_source')):
        names.append('fast_livo')
    return names


def belief_review(sensors, policy, stop_ros=None, estimator='rovio'):
    # Reuse the historical scorer, including its initial yaw/translation alignment.
    # Ground truth is consumed only in this independent process.
    import numpy as np
    from score_rovio_replay import score
    states = {}
    for name in ('gt', estimator):
        records = rows(sensors / (name + '.csv'))
        if stop_ros is not None:
            records = [r for r in records if float(r['header_ns']) / 1e9 <= stop_ros]
        if len(records) < 2:
            return {}, None
        states[name] = np.array([[float(r['header_ns']) / 1e9,
                                 *[float(r[k]) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]]
                                for r in records])
    metrics, series = score(states)
    # A non-finite first estimate may leave no valid series to score.
    if metrics.get(estimator,{}).get('invalid_samples',0):
        return metrics, 'raw_'+estimator+'_nonfinite'
    if estimator not in series:
        return metrics, 'raw_'+estimator+'_unscorable'
    t, error, *_ = series[estimator]
    threshold=policy.get('localization_error_m',policy['raw_rovio_error_review_m'])
    hold=policy.get('localization_error_duration_s',policy['raw_rovio_error_duration_s'])
    healthy = np.flatnonzero(error <= threshold)
    first_bad = healthy[-1] + 1 if len(healthy) else 0
    duration = float(t[-1] - t[first_bad]) if first_bad < len(t) else 0.
    metrics['sustained_error_s'] = duration
    reason = ('raw_'+estimator+'_divergence' if duration >= hold else None)
    return metrics, reason

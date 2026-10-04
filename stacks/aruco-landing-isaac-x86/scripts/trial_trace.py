"""Small, simulation-time frame traces for per-trial landing evaluation."""
import numpy as np


SCHEMA_VERSION = 1
STATES = {'waiting': 0, 'descending': 1, 'touchdown': 2, 'aborted': 3}


class TrialTrace:
    def __init__(self, trial_id, body_from_camera, pad_ids, sample_period_s):
        self.trial_id = trial_id
        self.mount = np.asarray(body_from_camera, dtype=np.float64).copy()
        self.pad_ids = np.asarray(sorted(pad_ids), dtype=np.int32)
        self.period = sample_period_s
        self.frames = []

    def append(self, now, truth, velocity, observation, detected_ids, command, policy):
        self.frames.append(dict(
            time=float(now), truth=np.asarray(truth).copy(), velocity=np.asarray(velocity).copy(),
            estimate=np.full((4, 4), np.nan) if observation is None else
                     np.linalg.inv(observation['camera_from_pad']),
            decoded=list(detected_ids), inliers=[] if observation is None else list(observation['inlier_ids']),
            rms=np.nan if observation is None else float(observation['rms_px']),
            command=np.asarray(command).copy(), state=STATES[policy.state],
            controller_stamp=np.nan if policy.stamp is None else float(policy.stamp),
            controller_visible=bool(policy.visible)))

    def arrays(self):
        def ragged(key):
            rows=[f[key] for f in self.frames]
            offsets=np.concatenate(([0], np.cumsum([len(row) for row in rows])))
            values=np.asarray([value for row in rows for value in row], dtype=np.int32)
            return values, offsets
        decoded, decoded_offsets=ragged('decoded')
        inliers, inlier_offsets=ragged('inliers')
        return dict(schema_version=np.array(SCHEMA_VERSION), trial_id=np.array(self.trial_id),
            sample_period_s=np.array(self.period), body_from_camera=self.mount, pad_marker_ids=self.pad_ids,
            capture_time_s=np.asarray([f['time'] for f in self.frames], dtype=np.float64),
            gt_pad_from_camera=np.asarray([f['truth'] for f in self.frames], dtype=np.float64),
            estimated_pad_from_camera=np.asarray([f['estimate'] for f in self.frames], dtype=np.float64),
            body_velocity_w=np.asarray([f['velocity'] for f in self.frames]),
            command_world_velocity_yawrate=np.asarray([f['command'] for f in self.frames]),
            controller_state=np.asarray([f['state'] for f in self.frames], dtype=np.uint8),
            controller_capture_time_s=np.asarray([f['controller_stamp'] for f in self.frames]),
            controller_visible=np.asarray([f['controller_visible'] for f in self.frames], dtype=bool),
            pnp_reprojection_rms_px=np.asarray([f['rms'] for f in self.frames]),
            decoded_marker_ids=decoded, decoded_marker_offsets=decoded_offsets,
            inlier_marker_ids=inliers, inlier_marker_offsets=inlier_offsets)


def dropout_frames(available):
    longest=current=0
    for value in available:
        current=0 if value else current+1
        longest=max(longest, current)
    return longest


def pose_metrics(estimate, truth, valid):
    if not valid.any():
        return dict(valid_frames=0, position_rmse_m=None, position_axis_rmse_m=None,
                    xy_rmse_m=None, rotation_rmse_deg=None, position_max_m=None)
    a, b=estimate[valid], truth[valid]
    squared=(a[:, :3, 3]-b[:, :3, 3])**2
    relative=np.einsum('nji,njk->nik', b[:, :3, :3], a[:, :3, :3])
    angles=np.degrees(np.arccos(np.clip((np.trace(relative, axis1=1, axis2=2)-1)/2, -1, 1)))
    return dict(valid_frames=int(valid.sum()), position_rmse_m=float(np.sqrt(squared.sum(1).mean())),
                position_axis_rmse_m=np.sqrt(squared.mean(0)).tolist(),
                xy_rmse_m=float(np.sqrt(squared[:, :2].sum(1).mean())),
                rotation_rmse_deg=float(np.sqrt(np.mean(angles**2))),
                position_max_m=float(np.sqrt(squared.sum(1).max())))


def trace_metrics(trace):
    if int(trace['schema_version']) != SCHEMA_VERSION:
        raise ValueError('unsupported trial trace schema')
    times=trace['capture_time_s']
    if len(times)==0 or not np.isfinite(times).all() or np.any(np.diff(times)<=0):
        raise ValueError('trace needs strictly increasing finite simulation timestamps')
    truth, estimate=trace['gt_pad_from_camera'], trace['estimated_pad_from_camera']
    valid=np.isfinite(estimate).all(axis=(1, 2)) & np.isfinite(truth).all(axis=(1, 2))
    ids, offsets=trace['decoded_marker_ids'], trace['decoded_marker_offsets']
    pad=set(trace['pad_marker_ids'].tolist())
    available=np.asarray([bool(pad.intersection(ids[a:b].tolist()))
                          for a,b in zip(offsets[:-1], offsets[1:])])
    if len(available)!=len(times): raise ValueError('marker offsets/frame count mismatch')
    mount_inverse=np.linalg.inv(trace['body_from_camera'])
    return dict(capture_frames=len(times), decoded_pad_marker_frames=int(available.sum()),
                marker_detection_availability=float(available.mean()),
                pose_availability=float(valid.mean()),
                longest_marker_dropout_frames=dropout_frames(available),
                longest_pose_dropout_frames=dropout_frames(valid),
                longest_marker_dropout_s=dropout_frames(available)*float(trace['sample_period_s']),
                longest_pose_dropout_s=dropout_frames(valid)*float(trace['sample_period_s']),
                localization_camera=pose_metrics(estimate, truth, valid),
                localization_body=pose_metrics(estimate @ mount_inverse, truth @ mount_inverse, valid))

"""Stack-owned camera-center reference adapter; NumPy only, no GT pose input.

This module supplies velocity references to the existing Lee velocity loop.
It does not replace native physics or claim acceleration/jerk feedforward.
Sampled p/v/a/j describe the camera center. Rotation/quaternion/body rates are
the ideal fixed-heading body attitude derived from that point's acceleration,
not the camera optical-frame attitude. Compose the calibrated optical rotation
with body attitude for rendering; mount offset changes actual body dynamics.
"""
from pathlib import Path
import hashlib
import inspect
import json
import time
import math
import numpy as np

EXPECTED_PROFILE = {'profile_id': 'table_20261013_v2', 'reference_point': 'camera_center', 'pad_side_m': 0.7, 'epsilon_m': 0.05, 'lateral_speed_m_s': 0.12, 'descent_speed_m_s': 0.55, 'height_max_m': 2.0, 'height_min_m': 0.2, 'duration_s': 3.2727272727272725, 'middle_time_s': 1.8181818181818181, 'funnel_formula': 'max(0.05,0.4-(0.12/0.55)*(2-h))', 'funnel_halfwidth_top_m': 0.4, 'funnel_halfwidth_middle_m': 0.18181818181818188}


class CameraReferenceBank:
    def __init__(self, path, specification):
        self._sample_table = None
        self.sample_table_contract = None
        self._sample_rows_clock = None
        self._sample_rows = {}
        self.profile_reference_lookup = False
        self.sampling_statistics = dict(offline_lookup_calls=0, offline_sample_rows=0,
                                       offline_misses=0, analytic_calls=0,sample_wall_s=0.)
        self.path = Path(path)
        self.specification = dict(specification)
        spec = self.specification
        for key, expected in EXPECTED_PROFILE.items():
            if spec.get(key) != expected:
                raise ValueError('reference profile mismatch: '+key)
        self.descent_speed = spec['descent_speed_m_s']
        self.sha256 = hashlib.sha256(self.path.read_bytes()).hexdigest()
        if spec.get('sha256') != self.sha256:
            raise ValueError('reference bank hash differs from campaign specification')
        if spec.get('trajectory_type') not in ('Fully inside', 'Mixed', 'Fully outside'):
            raise ValueError('unknown trajectory type')
        with np.load(self.path, allow_pickle=False) as data:
            for key in (*EXPECTED_PROFILE, 'profile_sha256'):
                if key not in data or data[key].shape != () or data[key].item() != spec[key]:
                    raise ValueError('missing or mismatched embedded bank metadata: '+key)
            if 'trajectory_type' not in data or data['trajectory_type'].item() != spec['trajectory_type']:
                raise ValueError('embedded trajectory type differs from campaign label')
            self.coefficients = data['coefficients'].copy()
            durations = data['durations'].copy()
            self.candidate_ids = data['candidate_ids'].copy()
            counts = data['piece_counts'].copy() if 'piece_counts' in data else None
        if self.coefficients.ndim != 4 or self.coefficients.shape[2:] != (3, 6):
            raise ValueError('expected ascending local-time quintics [N,piece,3,6]')
        n, pieces = self.coefficients.shape[:2]
        if durations.shape == (pieces,):
            durations = np.broadcast_to(durations, (n, pieces)).copy()
        if durations.shape != (n, pieces):
            raise ValueError('durations must match shared or per-trajectory pieces')
        self.durations = durations
        self.piece_counts = np.full(n, pieces, int) if counts is None else counts
        if not np.issubdtype(self.piece_counts.dtype, np.integer) or self.piece_counts.shape != (n,) or np.any(self.piece_counts < 1) or np.any(self.piece_counts > pieces):
            raise ValueError('invalid piece counts')
        if self.candidate_ids.shape != (n,) or len(np.unique(self.candidate_ids)) != n:
            raise ValueError('bank requires unique raw pair IDs')
        active = np.arange(pieces)[None, :] < self.piece_counts[:, None]
        if not np.all(np.isfinite(durations)) or np.any(durations[active] <= 0) or np.any(durations[~active] != 0):
            raise ValueError('invalid durations or padding')
        if not np.all(np.isfinite(self.coefficients[active])):
            raise ValueError('nonfinite active coefficients')
        self.knots = np.concatenate((np.zeros((n, 1)), np.cumsum(durations, axis=1)), axis=1)
        self.duration = spec['duration_s']
        if np.max(np.abs(self.knots[:, -1] - self.duration)) > 1e-10:
            raise ValueError('reference duration differs from profile')
        # Do not hide a wrong z polynomial by replacing its evaluated values.
        expected_z = np.zeros((n, pieces, 6))
        expected_z[..., 0] = 2 - self.descent_speed*self.knots[:, :-1]
        expected_z[..., 1] = -self.descent_speed
        if np.max(np.abs(self.coefficients[:, :, 2][active] - expected_z[active])) > 1e-12:
            raise ValueError('raw coefficients violate exact constant camera descent')

    def __len__(self):
        return len(self.coefficients)

    def capture_clocks(self, physics_dt_s, control_decimation):
        # Keep native physics dt. Last physics step gets an extra capture;
        # its reference clock is clamped to the exact continuous endpoint.
        if physics_dt_s <= 0 or control_decimation < 1 or int(control_decimation) != control_decimation:
            raise ValueError('invalid capture timing')
        steps = math.ceil(self.duration / physics_dt_s - 1e-12)
        clocks = np.arange(0,steps,control_decimation)*physics_dt_s
        return np.r_[clocks,self.duration]

    def sampler_algorithm_sha256(self):
        # Hash the original arithmetic body; wrapper/IO edits do not change it.
        body = inspect.getsource(self._sample_uncached).split('\n', 1)[1]
        return hashlib.sha256(body.encode()).hexdigest()

    def load_sample_table(self, path, manifest_sha256, *, strict=False,
                          physics_dt_s=None, control_decimation=None, policy_sha256=None):
        """Verify once and mmap immutable offline camera samples; no runtime math."""
        directory = Path(path)
        manifest_path = directory/'manifest.json'
        raw = manifest_path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != manifest_sha256:
            raise ValueError('offline reference manifest hash differs from configuration')
        manifest = json.loads(raw)
        if (manifest.get('schema_version') != 1 or manifest.get('bank_sha256') != self.sha256
                or manifest.get('sampler_algorithm_sha256') != self.sampler_algorithm_sha256()
                or manifest.get('reference_point') != 'camera_center'
                or manifest.get('trajectory_count') != len(self)):
            raise ValueError('offline reference table does not match bank/sampler/schema')
        for key, expected in [('physics_dt_s',physics_dt_s),
                              ('control_decimation',control_decimation),
                              ('policy_sha256',policy_sha256)]:
            if expected is not None and manifest.get(key) != expected:
                raise ValueError('offline reference consumer contract mismatch: '+key)
        arrays = {}
        expected_fields = {'position','velocity','acceleration','jerk','rotation',
                           'quaternion_xyzw','body_rates','angular_velocity_world','thrust_per_mass',
                           'capture_time_s','reference_ids'}
        if set(manifest['files']) != expected_fields:
            raise ValueError('offline reference fields incomplete')
        for key, record in manifest['files'].items():
            filename = record['filename']
            if Path(filename).name != filename or not filename.endswith('.npy'):
                raise ValueError('offline reference filename must be a local NPY')
            file_path = directory/filename
            digest = hashlib.sha256()
            with file_path.open('rb') as stream:
                for chunk in iter(lambda:stream.read(1024*1024), b''):digest.update(chunk)
            if digest.hexdigest() != record['sha256']:
                raise ValueError('offline reference payload hash mismatch: '+key)
            value = np.load(file_path, mmap_mode='r', allow_pickle=False)
            if list(value.shape) != record['shape'] or str(value.dtype) != record['dtype']:
                raise ValueError('offline reference payload shape/dtype mismatch: '+key)
            arrays[key] = value
        if not np.array_equal(arrays['reference_ids'], np.arange(len(self), dtype=np.int64)):
            raise ValueError('offline reference ID order must cover the unchanged whole bank')
        clocks = self.capture_clocks(manifest['physics_dt_s'],manifest['control_decimation'])
        if not np.array_equal(arrays['capture_time_s'], clocks) or clocks[-1] != self.duration:
            raise ValueError('offline capture clock grid mismatch')
        shapes = dict(position=(3,),velocity=(3,),acceleration=(3,),jerk=(3,),rotation=(3,3),
                      quaternion_xyzw=(4,),body_rates=(3,),angular_velocity_world=(3,),thrust_per_mass=())
        for key, shape in shapes.items():
            if arrays[key].shape != (len(clocks),len(self))+shape or arrays[key].dtype != np.float64:
                raise ValueError('offline sample dimensions invalid: '+key)
        self._sample_table = {key:arrays[key] for key in shapes}
        self._sample_clock = {float(clock):index for index,clock in enumerate(clocks)}
        self._strict_sample_table = bool(strict)
        self.sample_table_contract = dict(schema_version=1,manifest_sha256=manifest_sha256,
            bank_sha256=self.sha256,sampler_algorithm_sha256=manifest['sampler_algorithm_sha256'],
            capture_frames=len(clocks),trajectory_count=len(self),fields=sorted(shapes),
            physics_dt_s=manifest['physics_dt_s'],control_decimation=manifest['control_decimation'],
            policy_sha256=manifest['policy_sha256'],runtime='readonly_mmap_exact_clock_lookup',
            strict_capture_clock=bool(strict))

    def sample(self, reference_ids, time_s):
        if not self.profile_reference_lookup:
            return self._sample_with_table(reference_ids,time_s)
        begin = time.perf_counter()
        try:
            return self._sample_with_table(reference_ids,time_s)
        finally:
            self.sampling_statistics['sample_wall_s'] += time.perf_counter()-begin

    def _sample_with_table(self, reference_ids, time_s):
        if self._sample_table is None:
            self.sampling_statistics['analytic_calls'] += 1
            return self._sample_uncached(reference_ids, time_s)
        ids0 = np.asarray(reference_ids)
        if not np.issubdtype(ids0.dtype, np.integer):
            raise ValueError('reference IDs must be integers')
        clocks0 = np.asarray(time_s,dtype=float)
        if ids0.ndim == 0 and clocks0.ndim == 0:
            rid, clock = int(ids0), float(clocks0)
            if rid < 0 or rid >= len(self):
                raise IndexError('reference ID outside bank')
            if not math.isfinite(clock) or clock < 0 or clock > self.duration + 1e-10:
                raise ValueError('reference clock outside profile interval')
            slot = self._sample_clock.get(clock)
            if slot is not None:
                if self._sample_rows_clock != clock:
                    self._sample_rows = {}
                    self._sample_rows_clock = clock
                if rid not in self._sample_rows:
                    self._sample_rows[rid] = {name:value[slot,rid]
                                             for name,value in self._sample_table.items()}
                self.sampling_statistics['offline_lookup_calls'] += 1
                self.sampling_statistics['offline_sample_rows'] += 1
                return {name:value.copy() for name,value in self._sample_rows[rid].items()}
        ids, clocks = np.broadcast_arrays(ids0,clocks0)
        if np.any(ids < 0) or np.any(ids >= len(self)):
            raise IndexError('reference ID outside bank')
        if not np.all(np.isfinite(clocks)) or np.any(clocks < 0) or np.any(clocks > self.duration + 1e-10):
            raise ValueError('reference clock outside profile interval')
        slots = [self._sample_clock.get(float(clock)) for clock in clocks.flat]
        if all(slot is not None for slot in slots):
            index = np.asarray(slots,dtype=np.int64).reshape(clocks.shape)
            self.sampling_statistics['offline_lookup_calls'] += 1
            self.sampling_statistics['offline_sample_rows'] += int(ids.size)
            return {name:value[index,ids].copy() for name,value in self._sample_table.items()}
        self.sampling_statistics['offline_misses'] += 1
        if self._strict_sample_table:
            raise ValueError('campaign reference clock missed offline table')
        self.sampling_statistics['analytic_calls'] += 1
        return self._sample_uncached(reference_ids,time_s)

    def _sample_uncached(self, reference_ids, time_s):
        ids0 = np.asarray(reference_ids)
        if not np.issubdtype(ids0.dtype, np.integer):
            raise ValueError('reference IDs must be integers')
        ids, t = np.broadcast_arrays(ids0, np.asarray(time_s, dtype=float))
        if np.any(ids < 0) or np.any(ids >= len(self)):
            raise IndexError('reference ID outside bank')
        if not np.all(np.isfinite(t)) or np.any(t < 0) or np.any(t > self.duration + 1e-10):
            raise ValueError('reference clock outside profile interval')
        t = np.minimum(t, self.duration)
        segment = np.sum(t[..., None] >= self.knots[ids, 1:], axis=-1)
        segment = np.minimum(segment, self.piece_counts[ids] - 1)
        tau = (t - self.knots[ids, segment])[..., None]
        c = self.coefficients[ids, segment]
        result = {}
        for derivative, name in enumerate(('position', 'velocity', 'acceleration', 'jerk')):
            cd = c.copy()
            for _ in range(derivative):
                cd = cd[..., 1:]*np.arange(1, cd.shape[-1])
            value = np.zeros(t.shape + (3,))
            for power in range(cd.shape[-1] - 1, -1, -1):
                value = value*tau + cd[..., power]
            result[name] = value
        # Coefficients have already been checked; retain exact linear arithmetic.
        result['position'][..., 2] = 2 - self.descent_speed*t
        result['velocity'][..., 2] = -self.descent_speed
        result['acceleration'][..., 2] = 0
        result['jerk'][..., 2] = 0
        a, j = result['acceleration'], result['jerk']
        force = a.copy(); force[..., 2] += 9.81
        thrust = np.linalg.norm(force, axis=-1)
        b3 = force/thrust[..., None]
        b3_dot = (j - b3*np.sum(b3*j, axis=-1)[..., None])/thrust[..., None]
        heading = np.zeros_like(force); heading[..., 0] = 1
        cross = np.cross(b3, heading); cross_dot = np.cross(b3_dot, heading)
        norm = np.linalg.norm(cross, axis=-1)
        b2 = cross/norm[..., None]
        b2_dot = (cross_dot - b2*np.sum(b2*cross_dot, axis=-1)[..., None])/norm[..., None]
        b1 = np.cross(b2, b3)
        b1_dot = np.cross(b2_dot, b3) + np.cross(b2, b3_dot)
        rotation = np.stack((b1, b2, b3), axis=-1)
        rotation_dot = np.stack((b1_dot, b2_dot, b3_dot), axis=-1)
        skew = np.swapaxes(rotation, -1, -2)@rotation_dot
        body_rates = np.stack((skew[..., 2, 1], skew[..., 0, 2], skew[..., 1, 0]), axis=-1)
        qw = np.sqrt(1 + np.trace(rotation, axis1=-2, axis2=-1))/2
        qxyz = np.stack((rotation[..., 2, 1] - rotation[..., 1, 2], rotation[..., 0, 2] - rotation[..., 2, 0], rotation[..., 1, 0] - rotation[..., 0, 1]), axis=-1)/(4*qw[..., None])
        result.update(rotation=rotation, quaternion_xyzw=np.concatenate((qxyz, qw[..., None]), axis=-1),
                      body_rates=body_rates, angular_velocity_world=np.einsum('...ij,...j->...i', rotation, body_rates), thrust_per_mass=thrust)
        return result

    def trace_sample(self, reference_id, time_s):
        r = self.sample(reference_id, time_s)
        return {name + '_pad_m': r[name] for name in ('position', 'velocity', 'acceleration', 'jerk')}

    def trial_metadata(self, reference_id):
        return dict(trajectory_type=self.specification['trajectory_type'], bank_id=self.sha256,
                    reference_id=int(reference_id), raw_pair_id=int(self.candidate_ids[reference_id]),
                    seed=int(self.specification['seed']))


def body_state_from_camera_reference(reference, body_from_camera_translation, marker_plane_z_m=0):
    """Coherent reset pose/world velocity, XYZW quaternion; includes mount motion."""
    r = np.asarray(body_from_camera_translation, dtype=float)
    rotation = reference['rotation']
    mount_position = np.einsum('...ij,j->...i', rotation, r)
    mount_velocity = np.einsum('...ij,...j->...i', rotation, np.cross(reference['body_rates'], r))
    position = reference['position'] - mount_position
    position = position.copy(); position[..., 2] += marker_plane_z_m
    velocity = reference['velocity'] - mount_velocity
    return dict(position=position, velocity=velocity, quaternion_xyzw=reference['quaternion_xyzw'],
                angular_velocity_world=reference['angular_velocity_world'], thrust_per_mass=reference['thrust_per_mass'])


def camera_command_to_body_velocity(camera_velocity, body_rotation, body_angular_velocity, mount_translation):
    """Convert origin velocities; native gyro/attitude, never GT camera position."""
    return np.asarray(camera_velocity) - np.einsum('...ij,...j->...i', body_rotation,
        np.cross(body_angular_velocity, np.asarray(mount_translation)))


class CameraReferencePolicy:
    """Optical reference tracking wrapper around the unchanged legacy policy.

    Position error uses the reference at image time; feedforward uses current
    simulation time. Original vision-height and pose-loss events are preserved.
    """
    def __init__(self, legacy_policy, bank, reference_id):
        self.legacy = legacy_policy
        self.bank = bank
        self.reference_id = int(reference_id)
        bank.sample(self.reference_id, 0.)
        self.events = {}

    def __getattr__(self, name):
        return getattr(self.legacy, name)

    def submit(self, capture_time, pose, latency):
        adjusted = None
        if pose is not None:
            adjusted = np.asarray(pose).copy()
            adjusted[:2, 3] -= self.bank.sample(self.reference_id, min(capture_time,self.bank.duration))['position'][:2]
        self.legacy.submit(capture_time, adjusted, latency)

    def bootstrap_ready_pose(self, pose):
        """Explicit frozen-render readiness gate; no physics or clock advances."""
        if pose is None:
            self.legacy.state='aborted'
            self.events['acquisition_failed']=0.
            return False
        self.submit(0.,pose,0.)
        self.legacy.command(0.)
        self.legacy.visible_count=0 # Readiness capture is outside active trace phase.
        self.events['acquisition_ready']=0.
        return True

    def command(self, now):
        if not np.isfinite(now) or now < 0:
            raise ValueError('invalid reference phase clock')
        command = self.legacy.command(now).copy()
        if self.legacy.state in ('touchdown', 'aborted'):
            self.events.setdefault(self.legacy.state, float(now))
        if now >= self.bank.duration - 1e-10:
            self.events.setdefault('reference_complete', self.bank.duration)
            return np.zeros(4, np.float32)
        if self.legacy.state == 'descending' and self.legacy.visible:
            command[:2] += self.bank.sample(self.reference_id, now)['velocity'][:2]
            norm = np.linalg.norm(command[:2])
            limit = self.legacy.cfg['speed_limit_m_s']
            if norm > limit:
                command[:2] *= limit/norm
            command[2] = -self.bank.descent_speed
        return command

    def phase_result(self, final_time_s):
        return dict(reference_duration_s=self.bank.duration, reference_final_time_s=float(min(final_time_s,self.bank.duration)),
                    simulation_final_time_s=float(final_time_s),
                    endpoint_capture_delay_s=float(max(0.,final_time_s-self.bank.duration)),
                    truncated_reference_phase=final_time_s < self.bank.duration - 1e-8,
                    vision_height_event_time_s=self.events.get('touchdown'),
                    aborted_event_time_s=self.events.get('aborted'),
                    acquisition_failure_time_s=self.events.get('acquisition_failed'),
                    acquisition_ready_time_s=self.events.get('acquisition_ready'),
                    reference_complete_time_s=self.events.get('reference_complete'))

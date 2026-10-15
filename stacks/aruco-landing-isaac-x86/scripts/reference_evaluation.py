"""Camera-reference contracts and the bank-only runtime entry check."""
from pathlib import Path
import json
import hashlib
import numpy as np
from trajectory_reference import CameraReferenceBank

def load_reference(config):
    contract=config.get('reference_bank')
    if contract is None:return None,None
    if config['policy']['horizontal_reference']!='camera' or config['policy']['height_reference']!='camera':
        raise ValueError('reference bank requires camera XY and height policy')
    spec=json.loads(Path(contract['specification_path']).read_text())
    bank=CameraReferenceBank(contract['path'],spec)
    if abs(config['policy']['descent_speed_m_s']-bank.descent_speed)>1e-12:
        raise ValueError('policy descent differs from reference profile')
    if contract.get('evaluation_interval_s')!=[0,bank.duration] or contract.get('reference_point')!='camera_center':
        raise ValueError('camera reference evaluation interval differs from bank')
    if contract.get('classification_interval_s')!=[0,spec['middle_time_s']]:
        raise ValueError('reference classification interval must be 2-to-1m prefix')
    if config['initial_protocol']['epsilon_m']!=spec['epsilon_m']:
        raise ValueError('initial protocol epsilon differs from reference profile')
    ids=np.asarray(contract['reference_ids'])
    if ids.ndim!=1 or not len(ids) or not np.issubdtype(ids.dtype,np.integer) or len(np.unique(ids))!=len(ids):
        raise ValueError('reference_ids must be unique integer selected-bank indices')
    if contract.get('sample_table_path') is not None:
        policy_sha = hashlib.sha256(json.dumps(config['policy'],sort_keys=True).encode()).hexdigest()
        bank.load_sample_table(contract['sample_table_path'], contract['sample_table_manifest_sha256'],
            strict=True, physics_dt_s=config['physics_dt_s'],control_decimation=config['control_decimation'],
            policy_sha256=policy_sha)
    bank.sample(ids,0.)
    return bank,ids

def controller_contract(bank):
    return dict(mode='clocked_camera_xy_velocity_feedforward',reference_point='camera_center',
        upper_feedback='optical estimated camera XY/body yaw; no GT camera position',
        feedforward='reference XY velocity only; native Lee velocity/yaw-rate API',
        native_mount_velocity_correction='world body gyro cross rotated camera mount offset',
        startup='three render-only warmups; valid ready optical frame primes t0 without physics advance; no pose means acquisition_failed',
        subsequent_measurements='original capture timestamps and configured sensor delivery latency',
        reference_interval_s=[0.,bank.duration],camera_descent_command_m_s=-bank.descent_speed,
        endpoint='reference-time completion or original vision-height/abort/bounds; no physical-contact claim')

def trace_reference(bank,reference_id,time_s):
    ref=bank.sample(reference_id,min(time_s,bank.duration))
    return dict(position_pad_m=ref['position'],velocity_pad_m=ref['velocity'],
        acceleration_pad_m=ref['acceleration'],jerk_pad_m=ref['jerk'])


def validate_runtime_config(config):
    """Reject retired initial-position modes before starting the simulator."""
    if not isinstance(config, dict) or config.get('initial_protocol', {}).get('kind') != 'reference_bank':
        raise ValueError('Only trajectory-bank simulation is supported; random/grid runtime is retired')
    contract = config.get('reference_bank')
    required = ('path', 'specification_path', 'reference_ids',
                'sample_table_path', 'sample_table_manifest_sha256')
    if not isinstance(contract, dict) or any(not contract.get(key) for key in required):
        raise ValueError('Explicit reference bank and hash-gated offline sample table are required')

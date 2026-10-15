#!/usr/bin/env python3
"""Native multirotor + Lee velocity controller, optical ArUco batch evaluation."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ARUCO = ROOT / 'ws/aruco-landing/src/aruco_landing'


def detector_parameter_contract(manifest):
    """Fingerprint the actual common detector parameters, not only Git HEAD."""
    import cv2
    from aruco_landing import physical_pad
    detector = physical_pad.PhysicalPadDetector(manifest)
    source_sha = hashlib.sha256(Path(physical_pad.__file__).read_bytes()).hexdigest()
    if detector.params is None:
        return dict(profile='not_applicable_fractal_author_detector',
                    physical_pad_source_sha256=source_sha, opencv_version=cv2.__version__, values=None)
    primitive = lambda p: {name:getattr(p,name) for name in dir(p)
        if not name.startswith('_') and isinstance(getattr(p,name),(bool,int,float,str))}
    values = primitive(detector.params)
    defaults = primitive(cv2.aruco.DetectorParameters() if hasattr(cv2.aruco,'ArucoDetector')
                         else cv2.aruco.DetectorParameters_create())
    if values != defaults:
        raise RuntimeError('Common DetectorParameters differ from installed OpenCV factory defaults')
    return dict(profile='opencv_factory_defaults', opencv_version=cv2.__version__,
                physical_pad_source_sha256=source_sha, values=values)


def main():
    process_started_wall = time.perf_counter()
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True, help='explicit trajectory-bank configuration')
    parser.add_argument('--num-envs', type=int, default=1)
    parser.add_argument('--trials', type=int, default=10)
    parser.add_argument('--trial-start', type=int, default=0, help='stable trial ID offset for separate GPU workers')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--detector', choices=['cpu','gpu-experimental','cpu-nested-apriltag','gpu-opencv-compat','cpu-fractal'])
    parser.add_argument('--save-audit-images-every',type=int,default=0,
                        help='save grayscale/GT evidence every N camera captures, plus up to 16 large-error batches')
    parser.add_argument('--save-native-diagnostics',action='store_true',help='save native motor/controller states each physics step without modifying control')
    parser.add_argument('--smoke', action='store_true', help='render and detect; fail unless every camera finds the pad')
    parser.add_argument('--isolation-smoke', action='store_true',
                        help='smoke with an env-1 visual occluder directly in front of env-0 camera')
    parser.add_argument('--physics', default='isaacsim_physx', choices=['isaacsim_physx'])
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True)
    args = parser.parse_args()
    args.process_started_wall = process_started_wall
    if args.isolation_smoke:
        if args.num_envs < 2: parser.error('isolation-smoke requires at least two environments')
        args.smoke = True
    if args.num_envs < 1 or args.trials < 1 or args.trial_start < 0:
        parser.error('num-envs/trials must be positive and trial-start nonnegative')
    if args.save_audit_images_every<0: parser.error('audit image interval must be nonnegative')
    if args.save_audit_images_every and args.resume:
        parser.error('camera evidence capture requires a fresh run')
    cfg = yaml.safe_load(args.config.read_text())
    from reference_evaluation import validate_runtime_config
    validate_runtime_config(cfg)
    if args.detector:
        cfg['detector_backend'] = args.detector
    if cfg['sensor_latency_s'] < 0 or cfg['physics_dt_s'] <= 0 or cfg['control_decimation'] < 1:
        parser.error('invalid simulation schedule')
    from preflight import check
    gpu_uuid = os.environ.get('NVIDIA_VISIBLE_DEVICES','')
    if not gpu_uuid.startswith('GPU-') or ',' in gpu_uuid:
        parser.error('run through the Isaac service with one explicit GPU UUID')
    hardware = check(gpu_uuid)
    if not hardware['rtx_driver_floor_passed']:
        parser.error('RTX renderer rejects driver '+hardware['driver']+'; minimum 550.90.07, recommended 580.95.05')
    from trial_store import TrialStore
    from trial_trace import SCHEMA_VERSION, STATES
    from pad_scene import MARKER_PLANE_Z_M, metric_pad_manifest
    from reference_evaluation import load_reference, controller_contract
    motor_compensation=cfg['native_controller'].get('motor_lag_compensation',{})
    if motor_compensation.get('enabled',False) and 'reference_bank' not in cfg:
        parser.error('Optional motor compensation is scoped to the explicit reference-bank controller revision')
    args.reference_bank,args.reference_ids=load_reference(cfg)
    if args.reference_bank is not None:
        args.reference_bank.profile_reference_lookup=bool(cfg.get('pipeline_profile',False))
    if args.reference_bank is not None and args.trial_start+args.trials>len(args.reference_ids):
        parser.error('requested trial IDs exceed configured reference_ids')
    manifest_root = HERE.parent if cfg.get('pad_manifest_root') == 'stack' else ARUCO
    manifest_path = manifest_root / cfg['pad_manifest']
    manifest = metric_pad_manifest(yaml.safe_load(manifest_path.read_text()),
                                   cfg.get('initial_protocol',{}).get('pad_side_m'))
    if manifest.get('family') == 'fractal':
        from aruco_landing.fractal import prepare_manifest
        manifest = prepare_manifest(manifest)
    source = subprocess.check_output(['git','-C',str(ARUCO),'rev-parse','HEAD'],text=True).strip()
    if subprocess.check_output(['git','-C',str(ARUCO),'status','--porcelain'],text=True).strip():
        parser.error('commit ArUco source before collecting reproducible evaluation results')
    lock = json.loads((ROOT/'config/modules.lock.json').read_text())
    if source != lock['modules']['planner/aruco-batch']['revision']:
        parser.error('ArUco workspace differs from planner/aruco-batch locked revision')
    application_sources = {path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(HERE.glob('*.py'))}
    metadata = dict(config=cfg, aruco_revision=source, application_sources_sha256=application_sources,
                    pad_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
                    native_image=lock['modules']['simulation/isaac-lab']['manifest']['container']['image'],
                    termination='vision camera height threshold, not physical ground contact',
                    trace_schema_version=SCHEMA_VERSION, trace_controller_states=STATES,
                    trace_sampling='active-trial image/control frames including terminal capture; no render warmup',
                    trace_frame='pad-local poses, world/pad-axis velocities; meters and simulation seconds',
                    environment_from_pad_translation_m=[0.,0.,MARKER_PLANE_Z_M],
                    terminal_positions_frame='environment-local; explicit *_pad_m fields use marker plane',
                    dropout_duration_convention='missing frame count times image/control period')
    if args.reference_bank is not None:
        metadata['reference_bank']=dict(args.reference_bank.specification,
            evaluation_interval_s=[0.,args.reference_bank.duration],classification_interval_s=cfg['reference_bank']['classification_interval_s'],
            reference_ids=args.reference_ids.tolist(),controller_contract=controller_contract(args.reference_bank))
        metadata['reference_bank']['controller_contract']['motor_lag_compensation']=dict(motor_compensation)
        metadata['termination']='reference-time completion, original vision camera height or failures; not physical contact'
        if args.reference_bank.sample_table_contract is not None:
            metadata['reference_bank']['sample_table']=args.reference_bank.sample_table_contract
    metadata['detector_parameters'] = detector_parameter_contract(manifest)
    cpu_workers=cfg.get('cpu_detector_workers',8)
    pinned_gray=cfg.get('cpu_pinned_gray_transfer',False)
    if isinstance(cpu_workers,bool) or not isinstance(cpu_workers,int) or cpu_workers<1:
        parser.error('cpu_detector_workers must be a positive integer')
    if not isinstance(pinned_gray,bool): parser.error('cpu_pinned_gray_transfer must be boolean')
    from aruco_landing import batched_detection
    metadata['detector_execution']=dict(cpu_detector_workers_requested=cpu_workers,
        cpu_detector_workers_effective=min(cpu_workers,args.num_envs) if cfg['detector_backend'].startswith('cpu') else None,
        pnp_execution='serial',cpu_gray_transfer='reusable_pinned_synchronous_stream' if pinned_gray else 'blocking_cpu',
        opencv_threads=1,batched_detection_source_sha256=hashlib.sha256(Path(batched_detection.__file__).read_bytes()).hexdigest())
    metadata['pipeline_profile_enabled']=bool(cfg.get('pipeline_profile',False))
    if cfg['detector_backend']=='gpu-experimental':
        library=Path(os.environ.get('ARUCO_CUDA_LIBRARY',''))
        if not library.is_file(): parser.error('gpu-experimental requires ARUCO_CUDA_LIBRARY')
        metadata['gpu_detector_library_sha256']=hashlib.sha256(library.read_bytes()).hexdigest()
    elif cfg['detector_backend']=='gpu-opencv-compat':
        library=Path(os.environ.get('ARUCO_OPENCV_CUDA_LIBRARY',''))
        if not library.is_file(): parser.error('gpu-opencv-compat requires ARUCO_OPENCV_CUDA_LIBRARY')
        metadata['gpu_detector_library_sha256']=hashlib.sha256(library.read_bytes()).hexdigest()
        metadata['gpu_compatibility_sources_sha256']={
            name:hashlib.sha256((ARUCO/'src/aruco_landing'/name).read_bytes()).hexdigest()
            for name in ['gpu_opencv.py','cuda/opencv_candidates.cu']}
        metadata['gpu_compatibility_scope']='GPU image processing; CPU grouping/dictionary/subpixel; opt-in prototype'
    if cfg['detector_backend']=='cpu-fractal':
        from aruco_landing.fractal import SOURCE_SHA256
        library=Path(os.environ['ARUCO_FRACTAL_LIBRARY'])
        author_library=library.parent/'build/src/libaruco.so'
        metadata['fractal']=dict(author_version='3.1.12',archive_sha256=SOURCE_SHA256,
            bridge_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
            author_library_sha256=hashlib.sha256(author_library.read_bytes()).hexdigest(),
            metric_models=manifest['marker_models'],
            scope='official stateless image detector and generator; common IPPE/LM pad pose estimator')
    if args.save_audit_images_every:
        metadata['camera_evidence_sampling']=dict(every_camera_batches=args.save_audit_images_every,
            extra_position_error_threshold_m=.3,max_extra_batches=16,
            scope='same-image grayscale audit evidence; may include warmup and inactive environments; never used by policy')
    store = TrialStore(args.output, metadata, args.resume)
    pending = [i for i in range(args.trial_start,args.trial_start+args.trials) if i not in store.completed]
    if args.smoke: pending = list(range(args.num_envs))
    if not pending and not args.smoke:
        print('All requested trials already completed.'); return
    args.enable_cameras = True
    launcher = AppLauncher(args)
    exit_code = 0
    try:
        run(args, cfg, manifest, pending, store)
    except BaseException as error:
        import traceback
        traceback.print_exc()
        exit_code = 130 if isinstance(error,KeyboardInterrupt) else 1
    finally:
        # SimulationApp uses os._exit() on shutdown. Preserve failures instead
        # of allowing its default exit code 0 to hide a failed smoke/run.
        launcher.app.close(exit_code=exit_code)
    if exit_code: raise SystemExit(exit_code)


def run(args, cfg, manifest, pending, store):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sensors import CameraCfg
    from isaaclab.utils import configclass
    import isaaclab.utils.math as math_utils
    from isaaclab_physx.renderers.isaac_rtx_renderer_cfg import IsaacRtxRendererCfg, IsaacRtxRendererGlobalSettingsCfg
    from native_runtime import robot_config, velocity_controller, reset_robot
    from aruco_landing.batch_landing import LandingPolicy
    from aruco_landing.batched_detection import BatchedPadDetector
    from aruco_landing.pose_alignment import pose_matrix
    from aruco_landing.physical_pad import inverse
    from pad_scene import add_pad, MARKER_PLANE_Z_M
    from trial_trace import TrialTrace, trace_metrics
    from trajectory_reference import CameraReferencePolicy, body_state_from_camera_reference
    from reference_evaluation import trace_reference
    bank=args.reference_bank

    dt = cfg['physics_dt_s']
    decimation = cfg['control_decimation']
    camera_cfg = cfg['camera']
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        dt=dt, device=args.device, render_interval=decimation, use_newton_actuators=False))
    robot_cfg = robot_config(dt)

    @configclass
    class SceneCfg(InteractiveSceneCfg):
        ground = AssetBaseCfg(prim_path='/World/Ground', spawn=sim_utils.GroundPlaneCfg())
        light = AssetBaseCfg(prim_path='/World/Light', spawn=sim_utils.DomeLightCfg(intensity=1000.))
        robot = robot_cfg
        camera = CameraCfg(prim_path='{ENV_REGEX_NS}/Camera', update_period=0.,
            width=camera_cfg['width'], height=camera_cfg['height'], data_types=['rgb'],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.,
                horizontal_aperture=24.*camera_cfg['width']/camera_cfg['fx'], clipping_range=(.01,8.)),
            renderer_cfg=IsaacRtxRendererCfg(enable_scene_partitioning=cfg.get('scene_partitioning',True),
                global_settings=IsaacRtxRendererGlobalSettingsCfg(
                    enable_shadows=False, enable_reflections=False, enable_global_illumination=False,
                    enable_ambient_occlusion=False, antialiasing_mode='Off')))

    scene = InteractiveScene(SceneCfg(num_envs=args.num_envs, env_spacing=cfg['env_spacing_m'],
                                     replicate_physics=True, filter_collisions=True))
    from isaaclab.sim import get_current_stage
    add_pad(get_current_stage(), args.num_envs, manifest)
    if args.isolation_smoke:
        # A visible foreign object within env-0's clipping range would cover
        # its pad completely without RTX partition culling. No physics is run.
        first_state = body_state_from_camera_reference(
            bank.sample(int(args.reference_ids[pending[0]]),0.),
            camera_cfg['body_position_m'],MARKER_PLANE_Z_M)
        first = dict(zip(('x','y','z'),first_state['position']))
        origins = scene.env_origins
        if not isinstance(origins,torch.Tensor): origins = origins.torch
        point = origins[0]-origins[1]+torch.tensor(
            [first['x'],first['y'],first['z']-.4],device=sim.device)
        occluder = sim_utils.CuboidCfg(size=(.8,.8,.05),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.,0.,0.)))
        occluder.func('/World/envs/env_1/IsolationOccluder',occluder,translation=tuple(point.cpu().tolist()))
    sim.reset()
    robot, camera = scene['robot'], scene['camera']
    K = np.array([[camera_cfg['fx'],0.,camera_cfg['cx']],
                  [0.,camera_cfg['fy'],camera_cfg['cy']],[0.,0.,1.]])
    camera.set_intrinsic_matrices(torch.tensor(K,device=sim.device,dtype=torch.float32).repeat(args.num_envs,1,1))
    gains = cfg['native_controller']
    native = velocity_controller(robot, robot_cfg, args.num_envs, sim.device, gains)
    native_allocation=torch.tensor(robot_cfg.allocation_matrix,device=sim.device)
    allocation = torch.linalg.pinv(native_allocation)
    motor_compensation=cfg['native_controller'].get('motor_lag_compensation',{})
    compensation_enabled=motor_compensation.get('enabled',False)
    if compensation_enabled:
        from motor_lag_compensation import compensate_motor_thrust,validate_native_actuator
        if len(robot.actuators)!=1:raise ValueError('Inspected compensation revision requires the native four-motor group')
        motor_actuator=next(iter(robot.actuators.values()))
        actuator_contract=validate_native_actuator(motor_actuator,motor_compensation['native_thruster_source_sha256'])
        store._atomic(store.directory/'motor_compensation_contract.json',actuator_contract)
    from pipeline_profile import PipelineProfile
    profile=PipelineProfile(cfg.get('pipeline_profile',False))
    detector = BatchedPadDetector(manifest,args.num_envs,K,cfg['detector_backend'],
        cpu_workers=cfg.get('cpu_detector_workers',8),cpu_pinned_gray_transfer=cfg.get('cpu_pinned_gray_transfer',False),pipeline_profile=profile)
    samples=None
    if args.save_audit_images_every:
        from camera_samples import CameraSamples
        samples=CameraSamples(store,manifest['dictionary'],args.save_audit_images_every)
    offset = torch.tensor(camera_cfg['body_position_m'],device=sim.device).expand(args.num_envs,-1)
    optical_quat = torch.tensor(camera_cfg['body_quaternion_xyzw'],device=sim.device).expand(args.num_envs,-1)
    body_from_camera = pose_matrix(camera_cfg['body_position_m'],camera_cfg['body_quaternion_xyzw'])
    camera_from_body = inverse(body_from_camera)
    environment_from_pad = np.array([0.,0.,MARKER_PLANE_Z_M])
    origins = scene.env_origins
    if not isinstance(origins,torch.Tensor): origins = origins.torch
    totals = dict(physics_s=0.,render_s=0.,device_and_transfer_s=0.,detect_s=0.,pnp_s=0.,
                  control_s=0.,transferred_bytes=0,markers=0,camera_batches=0,physics_steps=0)
    quality = dict(valid_poses=0,translation_sum_squared_m2=0.,translation_max_m=0.,rotation_max_deg=0.)
    start = time.perf_counter()
    new_rows = []
    sampled_truth = None
    sampled_velocity = None

    def capture():
        nonlocal sampled_truth, sampled_velocity
        # Camera-only headless runs have no visualizer to call forward(). Sync
        # PhysX articulation transforms into Fabric before RTX reads geometry,
        # including the first image after a trial reset. This does not step time.
        capture_start=time.perf_counter()
        sim.forward()
        pose = robot.data.root_link_pose_w.torch
        camera.set_world_poses(pose[:,:3]+math_utils.quat_apply(pose[:,3:7],offset),
            math_utils.quat_mul(pose[:,3:7],optical_quat), convention='ros')
        begin = time.perf_counter()
        sim.render()
        camera.update(0.,force_recompute=True)
        rgb = camera.data.output['rgb']
        if not isinstance(rgb,torch.Tensor): rgb = rgb.torch
        torch.cuda.synchronize(sim.device)
        totals['render_s'] += time.perf_counter()-begin
        profile.add('capture_forward_render',time.perf_counter()-capture_start)
        observations, stats = detector.detect(rgb)
        validation_start=time.perf_counter()
        # Independent rendered-pose validation. These values never feed policy.
        rotation_tensor=math_utils.matrix_from_quat(pose[:,3:7])
        gt_rotations=profile.copy('capture_gt_rotation',rotation_tensor,'D2H',lambda:rotation_tensor.cpu().numpy())
        position_tensor=pose[:,:3]-origins
        gt_positions=profile.copy('capture_gt_position',position_tensor,'D2H',lambda:position_tensor.cpu().numpy())-environment_from_pad
        sampled_truth = np.tile(np.eye(4), (args.num_envs,1,1))
        sampled_truth[:,:3,:3] = gt_rotations @ body_from_camera[:3,:3]
        sampled_truth[:,:3,3] = gt_positions + gt_rotations @ body_from_camera[:3,3]
        velocity_tensor=robot.data.root_link_vel_w.torch.detach()
        sampled_velocity=profile.copy('capture_gt_velocity',velocity_tensor,'D2H',lambda:velocity_tensor.cpu().numpy())
        if samples is not None:
            samples.capture(totals['camera_batches'],rgb,sampled_truth,observations,detector.last_detected_ids)
        for env, observation in enumerate(observations):
            if observation is None: continue
            estimated = inverse(observation['camera_from_pad'])
            gt_rotation = gt_rotations[env] @ body_from_camera[:3,:3]
            gt_position = gt_positions[env] + gt_rotations[env] @ body_from_camera[:3,3]
            delta = float(np.linalg.norm(estimated[:3,3]-gt_position))
            angle = math.degrees(math.acos(float(np.clip((np.trace(gt_rotation.T @ estimated[:3,:3])-1)/2,-1.,1.))))
            quality['valid_poses'] += 1
            quality['translation_sum_squared_m2'] += delta*delta
            quality['translation_max_m'] = max(quality['translation_max_m'],delta)
            quality['rotation_max_deg'] = max(quality['rotation_max_deg'],angle)
        for key,value in stats.items(): totals[key] += value
        profile.add('capture_gt_validation',time.perf_counter()-validation_start)
        profile.count('camera_captures')
        totals['camera_batches'] += 1
        return observations

    for base in range(0,len(pending),args.num_envs):
        cohort = pending[base:base+args.num_envs]
        reference_ids=[int(args.reference_ids[i]) for i in cohort] if bank is not None else None
        policies = [LandingPolicy(cfg['policy']) for _ in range(args.num_envs)]
        if bank is not None:
            policies[:len(cohort)]=[CameraReferencePolicy(policies[env],bank,rid) for env,rid in enumerate(reference_ids)]
            entry=body_state_from_camera_reference(bank.sample(np.array(reference_ids),0.),camera_cfg['body_position_m'],MARKER_PLANE_Z_M)
            initial=[dict(x=float(p[0]),y=float(p[1]),z=float(p[2]),yaw_deg=0.,
                camera_initial_position_pad_m=bank.sample(rid,0.)['position'].tolist(),
                reference=bank.trial_metadata(rid),initial_velocity='optimized nonhover; coherent camera-to-body transform')
                for p,rid in zip(entry['position'],reference_ids)]
        poses = robot.data.default_root_pose.torch.clone()
        poses[:,:3] += origins
        for env, row in enumerate(initial):
            poses[env,:3] = origins[env]+torch.tensor([row['x'],row['y'],row['z']],device=sim.device)
            angle = math.radians(row['yaw_deg'])/2
            poses[env,3:7] = torch.tensor([0.,0.,math.sin(angle),math.cos(angle)],device=sim.device)
            if bank is not None:
                poses[env,3:7]=torch.tensor(entry['quaternion_xyzw'][env],device=sim.device,dtype=poses.dtype)
        reset_robot(robot, native, poses, gains['trim_initial_motors'])
        if bank is not None:
            entry_velocity=torch.zeros((args.num_envs,6),device=sim.device,dtype=poses.dtype)
            entry_velocity[:len(cohort),:3]=torch.tensor(entry['velocity'],device=sim.device,dtype=poses.dtype)
            entry_velocity[:len(cohort),3:]=torch.tensor(entry['angular_velocity_world'],device=sim.device,dtype=poses.dtype)
            robot.write_root_velocity_to_sim_index(root_velocity=entry_velocity,
                env_ids=torch.arange(args.num_envs,device=sim.device))
        detector.reset()
        camera.reset()
        # Flush render startup/reset history without advancing physics or trial time.
        for _ in range(3): observations = capture()
        if bank is not None:
            # Physics and reference time stayed frozen during readiness renders.
            for env in range(len(cohort)):
                ready=None
                if observations[env] is not None:
                    ready=inverse(observations[env]['camera_from_pad'])
                    ready[:3,:3]=(ready@camera_from_body)[:3,:3]
                policies[env].bootstrap_ready_pose(ready)
        if args.smoke:
            found = sum(pose is not None for pose in observations[:len(cohort)])
            import cv2
            images = camera.data.output['rgb']
            if not isinstance(images, torch.Tensor): images = images.torch
            images = images[...,:3].cpu().numpy()
            for env, image in enumerate(images):
                cv2.imwrite(str(store.directory / ('camera-%03d.png' % env)), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            result = dict(smoke_detected=found, expected=len(cohort), statistics=totals,
                          pose_quality=quality, hardware=check_hardware(),
                          foreign_occluder=args.isolation_smoke,
                          scene_partitioning=cfg.get('scene_partitioning',True),
                          passed=found==len(cohort) and quality['translation_max_m']<=.03
                                 and quality['rotation_max_deg']<=5.)
            store._atomic(store.directory/'smoke.json', result)
            print(json.dumps(result),flush=True)
            if found != len(cohort): raise RuntimeError('pad detection smoke failed')
            if quality['translation_max_m']>.03 or quality['rotation_max_deg']>5.:
                raise RuntimeError('optical pose smoke exceeds .03 m / 5 degree limits')
            return
        active = set(range(len(cohort)))
        traces = [TrialTrace(trial_id, body_from_camera,
                            [marker['id'] for marker in manifest['markers']], dt*decimation)
                  for trial_id in cohort]
        native_diagnostics=[[] for _ in cohort]
        command = torch.zeros((args.num_envs,4),device=sim.device)
        host_command = np.zeros((args.num_envs,4),np.float32)
        steps = math.ceil(bank.duration/dt-1e-12) if bank is not None else math.ceil(cfg['max_duration_s']/dt)
        for step in range(steps+1):
            now = step*dt
            if step % decimation == 0 or (bank is not None and step == steps):
                observations = capture()
                begin = time.perf_counter()
                for env in sorted(active):
                    policy_start=time.perf_counter()
                    measurement = observations[env]
                    if measurement is None:
                        selected = None
                    else:
                        pad_from_camera = inverse(measurement['camera_from_pad'])
                        pad_from_body = pad_from_camera @ camera_from_body
                        selected = pad_from_camera.copy()
                        if cfg['policy']['horizontal_reference'] == 'vehicle':
                            selected[:2,3] = pad_from_body[:2,3]
                        if cfg['policy']['height_reference'] == 'vehicle':
                            selected[2,3] = pad_from_body[2,3]
                        selected[:3,:3] = pad_from_body[:3,:3]
                    policies[env].submit(now, selected, cfg['sensor_latency_s'])
                    velocity = policies[env].command(now)
                    host_command[env] = velocity
                    profile.add('optical_transform_policy_command',time.perf_counter()-policy_start)
                    trace_start=time.perf_counter()
                    reference_trace={} if bank is None else dict(reference_time_s=min(now,bank.duration),evaluation_phase=1,
                        reference_sample=trace_reference(bank,reference_ids[env],now))
                    traces[env].append(now, sampled_truth[env], sampled_velocity[env], measurement,
                        detector.last_detected_ids[env], velocity, policies[env],**reference_trace)
                    profile.add('reference_trace_append',time.perf_counter()-trace_start)
                host_command_tensor=torch.from_numpy(host_command)
                profile.copy('control_command',host_command_tensor,'H2D',lambda:command.copy_(host_command_tensor))
                totals['control_s'] += time.perf_counter()-begin
            # Ground truth is used ONLY for terminal metrics/bounds and native
            # dynamics feedback/frame conversion. Upper policy sees optical poses.
            root = robot.data.root_link_pose_w.torch
            positions_tensor=(root[:,:3]-origins).detach()
            positions=profile.copy('physics_bound_body_position',positions_tensor,'D2H',lambda:positions_tensor.cpu().numpy())
            camera_positions_tensor=(root[:,:3]+math_utils.quat_apply(root[:,3:7],offset)-origins).detach()
            camera_positions=profile.copy('physics_bound_camera_position',camera_positions_tensor,'D2H',lambda:camera_positions_tensor.cpu().numpy())
            completed = []
            for env in sorted(active):
                state = policies[env].state
                pos = positions[env]
                outside = not np.isfinite(pos).all() or np.linalg.norm(pos[:2]) > 3. or pos[2] < .05 or pos[2] > 5.
                if state in ('touchdown','aborted') or outside or step == steps:
                    if bank is None:outcome='bounds' if outside else ('timeout' if step==steps else state)
                    elif 'acquisition_failed' in policies[env].events:outcome='acquisition_failed'
                    elif outside:outcome='bounds'
                    elif state in ('touchdown','aborted'):outcome=state
                    elif bank is not None and step==steps:outcome='reference_complete'
                    else:outcome='timeout' if step==steps else state
                    finite = bool(np.isfinite(pos).all() and np.isfinite(camera_positions[env]).all())
                    target_position = camera_positions[env] if cfg['policy']['horizontal_reference']=='camera' else pos
                    error = float(np.linalg.norm(target_position[:2])) if finite else None
                    terminal_velocity_tensor=robot.data.root_link_vel_w.torch[env].detach()
                    terminal_velocity=profile.copy('terminal_velocity',terminal_velocity_tensor,'D2H',lambda:terminal_velocity_tensor.cpu().numpy())
                    completion_start=time.perf_counter()
                    trace = traces[env].arrays()
                    row = dict(trial_id=cohort[env],initial=initial[env],outcome=outcome,
                        simulation_duration_s=now, final_body_position_m=pos.tolist() if finite else None,
                        final_camera_position_m=camera_positions[env].tolist() if finite else None,
                        final_body_position_pad_m=(pos-environment_from_pad).tolist() if finite else None,
                        final_camera_position_pad_m=(camera_positions[env]-environment_from_pad).tolist() if finite else None,
                        lateral_error_m=error,
                        success=outcome=='touchdown' and finite and error<=cfg['success_radius_m'],
                        valid_pose_frames=policies[env].visible_count,
                        touchdown=dict(criterion='vision_height_threshold',
                            event_time_s=now if outcome=='touchdown' else None,
                            final_body_velocity_w=terminal_velocity.tolist() if np.isfinite(terminal_velocity).all() else None),
                        metrics=trace_metrics(trace))
                    if args.save_native_diagnostics:
                        diagnostic_path=store.directory/f'native-diagnostics-{cohort[env]:07d}.npz'
                        np.savez_compressed(diagnostic_path,samples=np.asarray(native_diagnostics[env]),
                            columns=np.array(['time_s','native_vx','native_vy','native_vz','body_b3z','requested_force_z','requested_tx','requested_ty','requested_tz','desired_motor0','desired_motor1','desired_motor2','desired_motor3','applied_motor0','applied_motor1','applied_motor2','applied_motor3']+([f'{name}_motor{i}' for name in ['raw_inverse','clamped_input','predicted_next','input_saturated','target_saturated','negative_inverse_speed','rate_limited'] for i in range(4)] if compensation_enabled else [])),
                            mass_kg=native.mass[env].detach().cpu().numpy(),gravity=native.gravity[env].detach().cpu().numpy(),gains=native.K_vel_current[env].detach().cpu().numpy())
                        row['native_diagnostics']=diagnostic_path.name
                    if bank is not None:
                        row['reference']=bank.trial_metadata(reference_ids[env])
                        row['reference_phase']=policies[env].phase_result(now)
                    profile.add('completion_metrics',time.perf_counter()-completion_start)
                    write_start=time.perf_counter()
                    store.write(row, trace=trace)
                    profile.add('trial_atomic_write',time.perf_counter()-write_start)
                    new_rows.append(row)
                    logging_start=time.perf_counter()
                    print(json.dumps(row),flush=True)
                    profile.add('trial_stdout_logging',time.perf_counter()-logging_start)
                    completed.append(env)
                    command[env] = 0.
                    host_command[env] = 0.
            active.difference_update(completed)
            if not active: break
            # Native controller rotates its input by vehicle yaw only. Applying
            # inverse full attitude here would introduce incorrect vertical terms.
            command_frame_start=time.perf_counter()
            _,_,yaw = math_utils.euler_xyz_from_quat(root[:,3:7])
            yaw_quat = math_utils.quat_from_euler_xyz(torch.zeros_like(yaw),torch.zeros_like(yaw),yaw)
            native_command = command.clone()
            if bank is not None:
                # Native gyro/attitude converts camera-origin velocity to body-origin velocity.
                # Upper XY feedback remains optical; no GT camera position enters it.
                angular_world=robot.data.root_link_vel_w.torch[:,3:6]
                mount_world=math_utils.quat_apply(root[:,3:7],offset)
                indices=profile.construct('active_env_indices',8*len(active),'H2D',lambda:torch.tensor(sorted(active),device=sim.device))
                native_command[indices,:3]-=torch.linalg.cross(angular_world[indices],mount_world[indices],dim=-1)
            native_command[:,:3] = math_utils.quat_apply_inverse(yaw_quat,native_command[:,:3])
            begin = time.perf_counter()
            profile.add('native_command_frame_enqueue',begin-command_frame_start)
            wrench=native.compute(native_command)
            desired_motors=(wrench @ allocation.T).clamp(min=0.)
            if compensation_enabled:
                lag_result=compensate_motor_thrust(desired_motors,motor_actuator.curr_thrust,motor_actuator.thrust_const,
                    motor_actuator.tau_inc_s,motor_actuator.tau_dec_s,motor_actuator.cfg.dt,motor_actuator.max_rate,
                    motor_actuator.min_thrust,motor_actuator.max_thrust,motor_actuator.cfg.integration_scheme,motor_actuator.cfg.use_discrete_approximation)
                # Raw inverse is passed through unchanged native force/rate limits.
                robot.set_thrust_target(lag_result['command_raw'])
            else:robot.set_thrust_target(desired_motors)
            profile.add('native_controller_motor_enqueue',time.perf_counter()-begin)
            physics_step_start=time.perf_counter()
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(dt)
            profile.add('physics_write_step_update_enqueue',time.perf_counter()-physics_step_start)
            physics_wait_start=time.perf_counter()
            if args.save_native_diagnostics:
                # Native requested wrench and AFTER-step applied motor state; phase time is step start.
                native_velocity=native._root_state_tensors()[2]
                actual_motors=torch.cat([a.applied_thrust for a in robot.actuators.values()],dim=1)
                b3z=math_utils.matrix_from_quat(robot.data.root_link_quat_w.torch)[:,2,2]
                diag=torch.cat((torch.full((args.num_envs,1),now,device=sim.device),native_velocity,b3z[:,None],wrench[:,2:],desired_motors,actual_motors),dim=1).detach().cpu().numpy()
                if compensation_enabled:
                    extras=torch.cat([lag_result[name].to(dtype=torch.float32) for name in ['command_raw','command_clamped','predicted_next','input_saturated','target_saturated','negative_inverse_speed','rate_limited']],dim=1).detach().cpu().numpy()
                    diag=np.concatenate((diag,extras),axis=1)
                for env in sorted(active):native_diagnostics[env].append(diag[env])
            torch.cuda.synchronize(sim.device)
            profile.add('physics_diagnostics_existing_sync',time.perf_counter()-physics_wait_start)
            totals['physics_s'] += time.perf_counter()-begin
            profile.count('physics_steps')
            totals['physics_steps'] += 1
    wall = time.perf_counter()-start
    durations = sum(row['simulation_duration_s'] for row in new_rows)
    summary = dict(num_envs=args.num_envs,completed_trials=len(store.completed), wall_s=wall,
        startup_wall_s=start-args.process_started_wall,
        evaluation_wall_s=time.perf_counter()-args.process_started_wall,
        new_trials=len(new_rows), trial_per_wall_hour=len(new_rows)/wall*3600, aggregate_simulation_s=durations,
        aggregate_simulation_per_wall_s=durations/wall, gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
        hardware=check_hardware(), timing=totals,pose_quality=quality)
    summary['pipeline_profile']=profile.report()
    if bank is not None: summary['reference_sampling']=bank.sampling_statistics
    store._atomic(store.directory/'summary.json',summary)
    print(json.dumps(summary,indent=2),flush=True)


def check_hardware():
    from preflight import check
    return check(os.environ['NVIDIA_VISIBLE_DEVICES'])


if __name__ == '__main__': main()

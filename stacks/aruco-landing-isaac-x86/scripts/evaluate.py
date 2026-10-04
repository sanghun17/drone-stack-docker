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


def main():
    process_started_wall = time.perf_counter()
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE.parent/'config/evaluation.yaml')
    parser.add_argument('--num-envs', type=int, default=1)
    parser.add_argument('--trials', type=int, default=10)
    parser.add_argument('--trial-start', type=int, default=0, help='stable trial ID offset for separate GPU workers')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--detector', choices=['cpu','gpu-experimental'])
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
    cfg = yaml.safe_load(args.config.read_text())
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
    from pad_scene import MARKER_PLANE_Z_M
    manifest_path = ARUCO / cfg['pad_manifest']
    manifest = yaml.safe_load(manifest_path.read_text())
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
    if cfg['detector_backend']=='gpu-experimental':
        library=Path(os.environ.get('ARUCO_CUDA_LIBRARY',''))
        if not library.is_file(): parser.error('gpu-experimental requires ARUCO_CUDA_LIBRARY')
        metadata['gpu_detector_library_sha256']=hashlib.sha256(library.read_bytes()).hexdigest()
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
    from aruco_landing.batch_landing import LandingPolicy, initial_condition
    from aruco_landing.batched_detection import BatchedPadDetector
    from aruco_landing.pose_alignment import pose_matrix
    from aruco_landing.physical_pad import inverse
    from pad_scene import add_pad, MARKER_PLANE_Z_M
    from trial_trace import TrialTrace, trace_metrics

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
        first = initial_condition(cfg['seed'],pending[0],cfg['initial_bounds'])
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
    allocation = torch.linalg.pinv(torch.tensor(robot_cfg.allocation_matrix,device=sim.device))
    detector = BatchedPadDetector(manifest,args.num_envs,K,cfg['detector_backend'])
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
        observations, stats = detector.detect(rgb)
        # Independent rendered-pose validation. These values never feed policy.
        gt_rotations = math_utils.matrix_from_quat(pose[:,3:7]).cpu().numpy()
        gt_positions = (pose[:,:3]-origins).cpu().numpy()-environment_from_pad
        sampled_truth = np.tile(np.eye(4), (args.num_envs,1,1))
        sampled_truth[:,:3,:3] = gt_rotations @ body_from_camera[:3,:3]
        sampled_truth[:,:3,3] = gt_positions + gt_rotations @ body_from_camera[:3,3]
        sampled_velocity = robot.data.root_link_vel_w.torch.detach().cpu().numpy()
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
        totals['camera_batches'] += 1
        return observations

    for base in range(0,len(pending),args.num_envs):
        cohort = pending[base:base+args.num_envs]
        policies = [LandingPolicy(cfg['policy']) for _ in range(args.num_envs)]
        initial = [initial_condition(cfg['seed'],i,cfg['initial_bounds']) for i in cohort]
        poses = robot.data.default_root_pose.torch.clone()
        poses[:,:3] += origins
        for env, row in enumerate(initial):
            poses[env,:3] = origins[env]+torch.tensor([row['x'],row['y'],row['z']],device=sim.device)
            angle = math.radians(row['yaw_deg'])/2
            poses[env,3:7] = torch.tensor([0.,0.,math.sin(angle),math.cos(angle)],device=sim.device)
        reset_robot(robot, native, poses, gains['trim_initial_motors'])
        camera.reset()
        # Flush render startup/reset history without advancing physics or trial time.
        for _ in range(3): observations = capture()
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
        command = torch.zeros((args.num_envs,4),device=sim.device)
        host_command = np.zeros((args.num_envs,4),np.float32)
        steps = math.ceil(cfg['max_duration_s']/dt)
        for step in range(steps+1):
            now = step*dt
            if step % decimation == 0:
                observations = capture()
                begin = time.perf_counter()
                for env in sorted(active):
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
                    traces[env].append(now, sampled_truth[env], sampled_velocity[env], measurement,
                        detector.last_detected_ids[env], velocity, policies[env])
                command.copy_(torch.from_numpy(host_command))
                totals['control_s'] += time.perf_counter()-begin
            # Ground truth is used ONLY for terminal metrics/bounds and native
            # dynamics feedback/frame conversion. Upper policy sees optical poses.
            root = robot.data.root_link_pose_w.torch
            positions = (root[:,:3]-origins).detach().cpu().numpy()
            camera_positions = (root[:,:3]+math_utils.quat_apply(root[:,3:7],offset)-origins).detach().cpu().numpy()
            completed = []
            for env in sorted(active):
                state = policies[env].state
                pos = positions[env]
                outside = not np.isfinite(pos).all() or np.linalg.norm(pos[:2]) > 3. or pos[2] < .05 or pos[2] > 5.
                if state in ('touchdown','aborted') or outside or step == steps:
                    outcome = 'bounds' if outside else ('timeout' if step == steps else state)
                    finite = bool(np.isfinite(pos).all() and np.isfinite(camera_positions[env]).all())
                    target_position = camera_positions[env] if cfg['policy']['horizontal_reference']=='camera' else pos
                    error = float(np.linalg.norm(target_position[:2])) if finite else None
                    terminal_velocity = robot.data.root_link_vel_w.torch[env].detach().cpu().numpy()
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
                    store.write(row, trace=trace)
                    new_rows.append(row)
                    print(json.dumps(row),flush=True)
                    completed.append(env)
                    command[env] = 0.
                    host_command[env] = 0.
            active.difference_update(completed)
            if not active: break
            # Native controller rotates its input by vehicle yaw only. Applying
            # inverse full attitude here would introduce incorrect vertical terms.
            _,_,yaw = math_utils.euler_xyz_from_quat(root[:,3:7])
            yaw_quat = math_utils.quat_from_euler_xyz(torch.zeros_like(yaw),torch.zeros_like(yaw),yaw)
            native_command = command.clone()
            native_command[:,:3] = math_utils.quat_apply_inverse(yaw_quat,command[:,:3])
            begin = time.perf_counter()
            robot.set_thrust_target((native.compute(native_command) @ allocation.T).clamp(min=0.))
            scene.write_data_to_sim()
            sim.step(render=False)
            scene.update(dt)
            torch.cuda.synchronize(sim.device)
            totals['physics_s'] += time.perf_counter()-begin
            totals['physics_steps'] += 1
    wall = time.perf_counter()-start
    durations = sum(row['simulation_duration_s'] for row in new_rows)
    summary = dict(num_envs=args.num_envs,completed_trials=len(store.completed), wall_s=wall,
        startup_wall_s=start-args.process_started_wall,
        evaluation_wall_s=time.perf_counter()-args.process_started_wall,
        new_trials=len(new_rows), trial_per_wall_hour=len(new_rows)/wall*3600, aggregate_simulation_s=durations,
        aggregate_simulation_per_wall_s=durations/wall, gpu_peak_allocated_bytes=torch.cuda.max_memory_allocated(),
        hardware=check_hardware(), timing=totals,pose_quality=quality)
    store._atomic(store.directory/'summary.json',summary)
    print(json.dumps(summary,indent=2),flush=True)


def check_hardware():
    from preflight import check
    return check(os.environ['NVIDIA_VISIBLE_DEVICES'])


if __name__ == '__main__': main()

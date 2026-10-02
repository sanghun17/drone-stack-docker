#!/usr/bin/env python3
"""Exercise native dynamics, velocity control and repeated vectorized resets.

No renderer/images or landing success claims. This diagnostic remains useful
when the current driver can run CUDA/PhysX but cannot initialize the RTX renderer.
"""
import argparse
import json
from pathlib import Path
import time
import yaml
from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--num-envs',type=int,default=1)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--physics',default='isaacsim_physx',choices=['isaacsim_physx'])
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True,enable_cameras=False)
args=parser.parse_args()
if args.num_envs<1: parser.error('num-envs must be positive')
app=AppLauncher(args)


def main():
    import numpy as np
    import torch
    import isaaclab.sim as sim_utils
    import isaaclab.utils.math as math_utils
    from isaaclab.scene import InteractiveScene,InteractiveSceneCfg
    from isaaclab.utils import configclass
    from native_runtime import robot_config,velocity_controller,reset_robot
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/evaluation.yaml').read_text())
    dt=cfg['physics_dt_s']
    sim=sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt,device=args.device,use_newton_actuators=False))
    robot_cfg=robot_config(dt)
    @configclass
    class SceneCfg(InteractiveSceneCfg):
        robot=robot_cfg
    scene=InteractiveScene(SceneCfg(num_envs=args.num_envs,env_spacing=20.,filter_collisions=True))
    sim.reset()
    robot=scene['robot']
    controller=velocity_controller(robot,robot_cfg,args.num_envs,sim.device,cfg['native_controller'])
    allocation=torch.linalg.pinv(torch.tensor(robot_cfg.allocation_matrix,device=sim.device))
    origins=scene.env_origins
    if not isinstance(origins,torch.Tensor): origins=origins.torch
    command=torch.zeros((args.num_envs,4),device=sim.device)
    rows=[]
    for cycle in range(3):
        poses=robot.data.default_root_pose.torch.clone()
        poses[:,:3]=origins
        poses[:,2]+=1.5
        reset_robot(robot,controller,poses)
        begin=time.perf_counter()
        for _ in range(120):
            robot.set_thrust_target((controller.compute(command)@allocation.T).clamp(min=0.))
            scene.write_data_to_sim();sim.step(render=False);scene.update(dt)
        torch.cuda.synchronize()
        final=(robot.data.root_link_pose_w.torch[:,:3]-origins).cpu().numpy()
        drift=float(np.linalg.norm(final-np.array([0.,0.,1.5]),axis=1).max())
        rows.append(dict(cycle=cycle,wall_s=time.perf_counter()-begin,max_hover_drift_m=drift))
        if not np.isfinite(final).all() or drift>.15:
            raise RuntimeError('native hover/reset smoke drift exceeds .15 m: '+str(drift))
    reset_robot(robot,controller,poses)
    desired=torch.tensor([.2,0.,-.2],device=sim.device).expand(args.num_envs,-1)
    command[:,3]=.1
    for _ in range(240):
        quat=robot.data.root_link_pose_w.torch[:,3:7]
        _,_,yaw=math_utils.euler_xyz_from_quat(quat)
        yaw_quat=math_utils.quat_from_euler_xyz(torch.zeros_like(yaw),torch.zeros_like(yaw),yaw)
        command[:,:3]=math_utils.quat_apply_inverse(yaw_quat,desired)
        robot.set_thrust_target((controller.compute(command)@allocation.T).clamp(min=0.))
        scene.write_data_to_sim();sim.step(render=False);scene.update(dt)
    velocities=robot.data.root_link_lin_vel_w.torch
    velocity_error=float(torch.linalg.vector_norm(velocities-desired,dim=1).max().item())
    if not np.isfinite(velocity_error) or velocity_error>.10:
        raise RuntimeError('native velocity tracking smoke exceeds .10 m/s: '+str(velocity_error))
    report=dict(num_envs=args.num_envs,cycles=rows,max_velocity_error_m_s=velocity_error,
                scope='native physics/control/reset only; no images',passed=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


try: main()
finally: app.app.close()

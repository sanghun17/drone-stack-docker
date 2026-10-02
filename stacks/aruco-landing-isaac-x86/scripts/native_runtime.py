"""Configuration/frame adapters around unchanged native Isaac drone code."""
import math


def robot_config(dt):
    from isaaclab_assets.robots.arl_robot_1 import ARL_ROBOT_1_CFG
    cfg=ARL_ROBOT_1_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
    actuator=cfg.actuators['thrusters']
    actuator.dt=dt
    for key in ('thrust_const_range','tau_inc_range','tau_dec_range'):
        midpoint=sum(getattr(actuator,key))/2
        setattr(actuator,key,(midpoint,midpoint))
    return cfg


def velocity_controller(robot, cfg, num_envs, device, gains):
    import torch
    from isaaclab_contrib.controllers.lee_velocity_control_cfg import LeeVelControllerCfg
    from isaaclab_contrib.controllers.lee_velocity_control import LeeVelController
    controller=LeeVelController(LeeVelControllerCfg(
        K_vel_range=(tuple(gains['K_vel']),)*2,K_rot_range=(tuple(gains['K_rot']),)*2,
        K_angvel_range=(tuple(gains['K_angvel']),)*2,
        max_inclination_angle_rad=math.pi/3,max_yaw_rate=math.pi/3),robot,num_envs,str(device))
    return controller


def reset_robot(robot, controller, poses, trim_initial_motors=True):
    import torch
    indices=torch.arange(len(poses),device=poses.device)
    # The pinned native Multirotor.reset(None) uses Warp _ALL_INDICES to index
    # a Torch buffer. Supplying its supported explicit Torch indices avoids that
    # upstream API mismatch without changing dynamics/controller/reset source.
    robot.reset(env_ids=indices)
    controller.reset_idx(indices)
    if trim_initial_motors:
        # Initial motor state is part of the trial fixture, like position and
        # velocity. Set the public native state buffer to balanced hover thrust;
        # every subsequent integration/force calculation stays native.
        for actuator in robot.actuators.values():
            actuator.curr_thrust[:] = controller.mass[:,None]*9.81/4
    robot.write_root_pose_to_sim_index(root_pose=poses,env_ids=indices)
    robot.write_root_velocity_to_sim_index(
        root_velocity=torch.zeros((len(poses),6),device=poses.device),env_ids=indices)

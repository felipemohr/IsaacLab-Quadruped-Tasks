import argparse

from isaaclab.app import AppLauncher
from carb.input import GamepadInput

parser = argparse.ArgumentParser(description="Test your trained agent.")
parser.add_argument("--policy_path", type=str, default=None, help="The path to the policy.pt file.")
parser.add_argument("--robot", type=str, choices=["anymal_d", "spot", "go2"], default="go2", 
                    help="Robot to use. Options: 'anymal_d', 'spot', 'go2'.")
parser.add_argument("--use_vision", action="store_true", default=False, help="Use height map from vision.")
parser.add_argument("--teleop", type=str, default=None, choices=["keyboard", "gamepad"], 
                    help="The teleop device to use. Options: 'keyboard', 'gamepad'.")
parser.add_argument("--use_predefined_cmds", action="store_true", default=False, help="Use predefined velocity commands.")
parser.add_argument("--use_higher_velocities", action="store_true", default=False, 
                    help="Test velocity commands higher than those seem during training. If true, a PI controller will be used")
parser.add_argument("--terrain", type=str, default=None, choices=["flat", "random", "waves", "boxes", "slope", "stairs"])
parser.add_argument("--terrain_difficulty", type=float, default=0.5, 
                    help="The difficulty of the terrain (Betwwen 0 and 1).")
parser.add_argument("--push_robot", action="store_true", default=False, 
                    help="Push the robot periodically by randomly increasing the root velocity.")
parser.add_argument("--push_interval", type=float, default=5.0, 
                    help="The interval between each push, in seconds.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--save_data", action="store_true", default=False, help="Save the data for inference.")
parser.add_argument("--save_path", type=str, default=None, help="The path to save the data for inference.")
parser.add_argument("--save_filename", type=str, default=None, help="The filename to save the data for inference.")
parser.add_argument("--save_interval", type=float, default=5.0, 
                    help="The interval between each data save, in seconds.")
parser.add_argument("--max_sim_time", type=float, default=None, 
                    help="The maximun simulation time, in seconds. After that, the simulation will close.")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import math
import os

from isaaclab.envs.mdp.events import push_by_setting_velocity
from isaaclab.assets import Articulation
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.devices import Se2Gamepad, Se2Keyboard
from isaaclab.terrains import (
    TerrainGeneratorCfg,
    MeshPlaneTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
    MeshRandomGridTerrainCfg,
    HfInvertedPyramidSlopedTerrainCfg,
    MeshInvertedPyramidStairsTerrainCfg,
)

from isaaclab_quadruped_tasks.robots.go2.go2_env_cfg import Go2CPGBaseEnvCfg
from isaaclab_quadruped_tasks.robots.spot.spot_env_cfg import SpotCPGBaseEnvCfg
from isaaclab_quadruped_tasks.robots.anymal_d.anymal_d_env_cfg import AnymalDCPGBaseEnvCfg

def main():
    # Set the save file path
    save_path = args_cli.save_path
    save_filename = args_cli.save_filename
    if args_cli.save_data:
        if save_path is None:
            save_path = "logs/inference"
        if save_filename is None:
            save_filename = f"{args_cli.robot}_cpg_{'vision' if args_cli.use_vision else 'blind'}.pt"

        if not os.path.exists(save_path):
            os.makedirs(save_path)

    # Defines wich environment will be used, according to the selected robot
    base_prim_path = "{ENV_REGEX_NS}/Robot/base"
    foot_prim_path = "{ENV_REGEX_NS}/Robot/.*_foot"
    if args_cli.robot == "anymal_d":
        env_cfg = AnymalDCPGBaseEnvCfg()
        foot_prim_path = "{ENV_REGEX_NS}/Robot/.*_FOOT"
    elif args_cli.robot == "spot":
        env_cfg = SpotCPGBaseEnvCfg()
        base_prim_path = "{ENV_REGEX_NS}/Robot/body"
    else:
        env_cfg = Go2CPGBaseEnvCfg()

    # Defines wich terrain will be used
    sub_terrains = dict()
    if args_cli.terrain == "flat":
        sub_terrains = {"flat": MeshPlaneTerrainCfg()}
    elif args_cli.terrain == "random":
        sub_terrains = {"random_rough": HfRandomUniformTerrainCfg(noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25)}
    elif args_cli.terrain == "waves":
        sub_terrains = {"waves": HfWaveTerrainCfg(amplitude_range=(0.02, 0.12), num_waves=50, border_width=0.25)}
    elif args_cli.terrain == "boxes":
        sub_terrains = {"boxes": MeshRandomGridTerrainCfg(grid_width=0.45, grid_height_range=(0.02, 0.16), platform_width=2.0)}
    elif args_cli.terrains == "slope":
        sub_terrains = {"slope":  HfInvertedPyramidSlopedTerrainCfg(slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25)}
    elif args_cli.terrain == "stairs":
        sub_terrains = {"pyramid_stairs_inv": MeshInvertedPyramidStairsTerrainCfg(step_height_range=(0.05, 0.23), step_width=0.3, platform_width=3.0, border_width=1.0, holes=False)}

    if args_cli.terrain is not None:
        TERRAIN_CFG = TerrainGeneratorCfg(
            seed=42,
            size=(50.0, 50.0),
            border_width=20.0,
            num_rows=1,
            num_cols=1,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            slope_threshold=0.75,
            use_cache=True,
            sub_terrains=sub_terrains,
            curriculum=False,
            difficulty_range=(args_cli.terrain_difficulty, args_cli.terrain_difficulty),
        )
        env_cfg.scene.terrain.terrain_type = "generator"
        env_cfg.scene.terrain.terrain_generator = TERRAIN_CFG

    # Defines if the task uses vision or not, and configure some action parameters according to it
    if not args_cli.use_vision:
        env_cfg.scene.height_scanner = None
        env_cfg.observations.policy.height_map = None
    else:
        if args_cli.robot == "anymal_d" or args_cli.robot == "spot":
            env_cfg.actions.action.ground_clearance = 0.2
            env_cfg.actions.action.ground_penetration = 0.02
        else:
            env_cfg.actions.action.body_height_offset = 0.05
            env_cfg.actions.action.ground_clearance = 0.15
            env_cfg.actions.action.ground_penetration = 0.015

    # Change some parameters of the environment for play
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.terminations.base_contact = None
    env_cfg.terminations.time_out = None
    env_cfg.observations.policy.enable_corruption = False
    env_cfg.curriculum = None
    env_cfg.events.change_gait = None
    env_cfg.events.add_base_mass = None
    env_cfg.events.change_actuator_gains = None
    env_cfg.events.change_vel_cmd = None
    env_cfg.events.push_robot = None
    env_cfg.events.physics_material = None
    env_cfg.events.reset_robot_base.params["pose_range"]["x"] = (-0.0, 0.0)
    env_cfg.events.reset_robot_base.params["pose_range"]["y"] = (-0.0, 0.0)
    env_cfg.events.reset_robot_base.params["pose_range"]["yaw"] = (torch.pi, torch.pi)

    # Defines the device to be used
    env_cfg.sim.device = args_cli.device
    if args_cli.device == "cpu":
        env_cfg.sim.use_fabric = False

    # Creates new sensors for getting the transform information from the feet to the robot's base
    foot_transforms_cfg = FrameTransformerCfg(
        prim_path=base_prim_path,
        target_frames=[FrameTransformerCfg.FrameCfg(prim_path=foot_prim_path)],
        update_period=0.0,
        debug_vis=True,
    )
    foot_transforms_cfg.visualizer_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    env_cfg.scene.foot_transforms = foot_transforms_cfg

    # Parses the configurations to create the environment   
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # Creates the teleop interface, if required
    teleop_interface = None
    if args_cli.teleop is not None:
        v_x = 1.0
        v_y = 1.0
        omega_z = math.pi / 2
        if args_cli.use_vision:
            v_x = 0.8
            v_y = 0.4
            omega_z = math.pi / 4

        if args_cli.teleop.lower() == "keyboard":
            teleop_interface = Se2Keyboard(v_x_sensitivity=v_x, v_y_sensitivity=-v_y, omega_z_sensitivity=-omega_z)
            teleop_interface.add_callback("ESCAPE", env.reset)
        elif args_cli.teleop.lower() == "gamepad":
            teleop_interface = Se2Gamepad(v_x_sensitivity=v_x, v_y_sensitivity=v_y, omega_z_sensitivity=omega_z, dead_zone=0.1)
            teleop_interface.add_callback(GamepadInput.A, env.reset)
        env.command_manager.set_debug_vis(False)

    if args_cli.use_predefined_cmds:
        env.command_manager.set_debug_vis(False)

    # Loads the trained policy
    policy = torch.jit.load(args_cli.policy_path, map_location=args_cli.device)

    # Initializes the variables for the PI controller, if use_higher_velocities is set to True
    sum_vel_error = torch.zeros(args_cli.num_envs, 3)
    kp = 2.0
    ki = 3.0

    # These lists will store all simulation data
    time_history = list()
    obs_history = list()
    actions_history = list()
    setpoint_vel_history = list()
    cpg_actions_processed_history = list()
    joints_actions_processed_history = list()
    feet_ik_pos_history = list()
    omnidirectional_offsets_history = list()
    root_pos_history = list()
    root_quat_history = list()
    foot_transforms_history = list()

    # Resets the environment and teleop interface
    obs, _ = env.reset()
    if teleop_interface is not None:
        teleop_interface.reset()
    # Starts the simulation loop
    t = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # print(t)

            cmd_vel = obs["policy"][:, :3]
            # If a teleop interface is being used, it will set the velocity commands
            if teleop_interface is not None:
                cmd_vel = torch.tensor(teleop_interface.advance()).to(args_cli.device)
                cmd_vel *= torch.tensor([1, -1, -1]).to(args_cli.device)
                cmd_vel = cmd_vel.expand(args_cli.num_envs, 3)

            # If the use_predefined_cmds is used, predefined the velocity commands will be set
            if args_cli.use_predefined_cmds:
                if t < 5:
                    cmd_vel = torch.tensor([1.0, 0.0, 0.0]).to(args_cli.device)
                elif t < 10:
                    cmd_vel = torch.tensor([0.0, 1.0, 0.0]).to(args_cli.device)
                elif t < 15:
                    cmd_vel = torch.tensor([0.0, 0.0, math.pi / 2]).to(args_cli.device)
                elif t < 20:
                    cmd_vel = torch.tensor([0.6, 0.6, -math.pi / 4]).to(args_cli.device)
                else:
                    t_rel = t - 20.0
                    angle = 2.0 * math.pi * 0.1 * t
                    if t_rel < 1:
                        alpha = t_rel
                        base = torch.tensor([0.6, 0.6, -math.pi / 4]).to(args_cli.device)
                        smooth = torch.tensor([
                            0.6 * math.cos(angle),
                            0.6 * math.sin(angle),
                            math.pi / 4 * math.sin(angle)
                        ]).to(args_cli.device)
                        cmd_vel = (1 - alpha) * base + alpha * smooth
                    else:
                        cmd_vel = torch.tensor([
                            0.6 * math.cos(angle),
                            0.6 * math.sin(angle),
                            math.pi / 4 * math.sin(angle)
                        ]).to(args_cli.device)
                cmd_vel = cmd_vel.expand(args_cli.num_envs, 3)
            
            # If the use_higher_velocities is used, a PI controller will set the velocity commands
            setpoint_vel = cmd_vel
            if args_cli.use_higher_velocities:
                setpoint_vel = torch.tensor([1.0, 0.0, 0.0], device=args_cli.device).repeat(args_cli.num_envs, 1)
                setpoint_vel[:, 0] += 0.25 * (t // 5.0)
                if t % 5.0 < env.step_dt:
                    print(f"Setting setpoint velocity to {setpoint_vel}")
                
                current_vel = torch.cat([obs["policy"][:, 3:5], obs["policy"][:, 8:9]], dim=1)
                vel_error = setpoint_vel - current_vel
                
                sum_vel_error += vel_error * env.step_dt
                sum_vel_error = torch.clamp(sum_vel_error, min=-4.0 / ki, max=4.0 / ki)

                cmd_vel = kp * vel_error + ki * sum_vel_error
                cmd_vel = torch.clamp(cmd_vel, min=-4.0, max=4.0)

            # Sets the velocity command for the policy
            obs["policy"][:, :3] = cmd_vel

            # Randomly pushes the robot base, if required
            if args_cli.push_robot:
                if t % args_cli.push_interval < env.step_dt:
                    print("Pushing the robot")
                    push_by_setting_velocity(env, velocity_range={"x": (-1.0, 1.0), "y": (-1.0, 1.0), "yaw": (-1.57, 1.57)}, 
                                             env_ids=torch.tensor([0]).to(args_cli.device))

            # Computes the action
            actions = torch.clamp(policy(obs["policy"]), -100.0, 100.0)

            # Stores data
            time_history.append(torch.tensor([t]).cpu())
            obs_history.append(obs["policy"].clone().detach().cpu())
            actions_history.append(actions.clone().detach().cpu())
            setpoint_vel_history.append(setpoint_vel.clone().detach().cpu())

            # Steps the environment
            obs, rew, terminated, truncated, info = env.step(actions)

            # Stores data
            robot : Articulation = env.scene["robot"]
            cpg_processed_actions = env.action_manager.get_term("action").get_cpg_processed_actions()
            joints_processed_actions = env.action_manager.get_term("action").processed_actions
            cpg_actions_processed_history.append(cpg_processed_actions.clone().detach().cpu())
            joints_actions_processed_history.append(joints_processed_actions.clone().detach().cpu())
            omnidirectional_offsets_history.append(env.action_manager.get_term("action").get_omnidirectional_offset().clone().detach().cpu())
            feet_ik_pos_history.append(env.action_manager.get_term("action").get_feet_ik_pos().clone().detach().cpu())
            foot_transforms_history.append(env.scene["foot_transforms"].data.target_pos_source.clone().detach().cpu())
            root_pos_history.append(robot.data.root_pos_w.clone().detach().cpu())
            root_quat_history.append(robot.data.root_quat_w.clone().detach().cpu())

            # Saves the data according to the save_interval
            if args_cli.save_data and t % args_cli.save_interval < env.step_dt:
                path = os.path.join(save_path, save_filename)
                print(f"Saving data in path: {path}")
                torch.save(
                    {
                        "time": torch.stack(time_history, dim=0),
                        "obs": torch.stack(obs_history, dim=0), 
                        "actions": torch.stack(actions_history, dim=0),
                        "setpoint_vel": torch.stack(setpoint_vel_history, dim=0), 
                        "cpg_processed_actions": torch.stack(cpg_actions_processed_history, dim=0),
                        "joints_processed_actions": torch.stack(joints_actions_processed_history, dim=0),
                        "omnidirectional_offsets": torch.stack(omnidirectional_offsets_history, dim=0),
                        "feet_ik_pos": torch.stack(feet_ik_pos_history, dim=0),
                        "foot_transforms": torch.stack(foot_transforms_history, dim=0),
                        "root_pos": torch.stack(root_pos_history, dim=0),
                        "root_quat": torch.stack(root_quat_history, dim=0),
                    },
                    path
                )
            
            # Increases the simulation time
            if args_cli.max_sim_time is not None:
                if t > args_cli.max_sim_time:
                    print("Stopping simulation due to time limit.")
                    break
            t += env.step_dt
    
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()

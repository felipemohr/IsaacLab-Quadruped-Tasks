
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Test your trained agent.")
parser.add_argument("--policy_path", type=str, default=None, help="The path to the policy.pt file.")
parser.add_argument("--robot", type=str, choices=["anymal_d", "spot", "go2"], default="go2", 
                    help="Robot to use. Options: 'anymal_d', 'spot', 'go2'.")
parser.add_argument("--use_vision", action="store_true", default=False, help="Use height map from vision.")
parser.add_argument("--teleop", type=str, default=None, choices=["keyboard", "gamepad"], 
                    help="The teleop device to use. Options: 'keyboard', 'gamepad'.")
parser.add_argument("--terrain", type=str, default=None, choices=["flat", "random", "waves", "boxes", "stairs"])
parser.add_argument("--terrain_difficulty", type=float, default=1.0, 
                    help="The difficulty of the terrain (Betwwen 0 and 1).")
parser.add_argument("--push_robot", action="store_true", default=False, 
                    help="Push the robot periodically by randomly increasing the root velocity.")
parser.add_argument("--push_interval", type=float, default=10.0, 
                    help="The interval between each push, in seconds.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--save_data", action="store_true", default=False, help="Save the data for inference.")
parser.add_argument("--save_path", type=str, default=None, help="The path to save the data for inference.")
parser.add_argument("--save_filename", type=str, default=None, help="The filename to save the data for inference.")
parser.add_argument("--save_interval", type=float, default=10.0, 
                    help="The interval between each data save, in seconds.")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
import os

from isaaclab.envs.mdp.events import push_by_setting_velocity
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.devices import Se2Gamepad, Se2Keyboard
from isaaclab.terrains import (
    TerrainGeneratorCfg,
    MeshPlaneTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
    MeshRandomGridTerrainCfg,
    MeshInvertedPyramidStairsTerrainCfg,
)

from isaaclab_quadruped_tasks.robots.go2.go2_env_cfg import Go2CPGBaseEnvCfg
from isaaclab_quadruped_tasks.robots.spot.spot_env_cfg import SpotCPGBaseEnvCfg
from isaaclab_quadruped_tasks.robots.anymal_d.anymal_d_env_cfg import AnymalDCPGBaseEnvCfg

def main():
    save_path = args_cli.save_path
    save_filename = args_cli.save_filename
    if args_cli.save_data:
        if save_path is None:
            save_path = "logs/inference"
        if save_filename is None:
            save_filename = f"{args_cli.robot}_cpg_{'vision' if args_cli.use_vision else 'blind'}.pt"

        if not os.path.exists(save_path):
            os.makedirs(save_path)

    if args_cli.robot == "anymal_d":
        env_cfg = AnymalDCPGBaseEnvCfg()
    elif args_cli.robot == "spot":
        env_cfg = SpotCPGBaseEnvCfg()
    else:
        env_cfg = Go2CPGBaseEnvCfg()

    teleop_interface = None
    if args_cli.teleop is not None:
        if args_cli.teleop.lower() == "keyboard":
            teleop_interface = Se2Keyboard(v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.57)
        elif args_cli.teleop.lower() == "gamepad":
            teleop_interface = Se2Gamepad(v_x_sensitivity=1.0, v_y_sensitivity=1.0, omega_z_sensitivity=1.57, dead_zone=0.1)
        env_cfg.commands.base_velocity.debug_vis = False
        env_cfg.events.change_vel_cmd = None
    
    sub_terrains = dict()
    if args_cli.terrain == "flat":
        sub_terrains = {"flat": MeshPlaneTerrainCfg()}
    elif args_cli.terrain == "random":
        sub_terrains = {"random_rough": HfRandomUniformTerrainCfg(noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25)}
    elif args_cli.terrain == "waves":
        sub_terrains = {"waves": HfWaveTerrainCfg(amplitude_range=(0.02, 0.12), num_waves=50, border_width=0.25)}
    elif args_cli.terrain == "boxes":
        sub_terrains = {"boxes": MeshRandomGridTerrainCfg(grid_width=0.45, grid_height_range=(0.02, 0.16), platform_width=2.0)}
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

    if not args_cli.use_vision:
        env_cfg.scene.height_scanner = None
        env_cfg.observations.policy.height_map = None

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

    env_cfg.sim.device = args_cli.device
    if args_cli.device == "cpu":
        env_cfg.sim.use_fabric = False

    env = ManagerBasedRLEnv(cfg=env_cfg)

    policy = torch.jit.load(args_cli.policy_path, map_location=args_cli.device)

    time_history = list()
    obs_history = list()
    actions_history = list()
    actions_processed_history = list()
    feet_ik_pos_history = list()

    obs, _ = env.reset()
    if teleop_interface is not None:
        teleop_interface.reset()
    t = 0 # TODO: Should it be a tensor?
    while simulation_app.is_running():
        with torch.inference_mode():
            if teleop_interface is not None:
                cmd_vel = torch.tensor(teleop_interface.advance()).to(args_cli.device)
                cmd_vel *= torch.tensor([1, -1, -1]).to(args_cli.device)
                obs["policy"][:, :3] = cmd_vel

            if args_cli.push_robot:
                if t % args_cli.push_interval < env.step_dt:
                    print("Pushing the robot")
                    push_by_setting_velocity(env, velocity_range={"x": (-1.0, 1.0), "y": (-1.0, 1.0), "yaw": (-1.57, 1.57)}, env_ids=torch.tensor([0]).to(args_cli.device))

            actions = torch.clamp(policy(obs["policy"]), -100.0, 100.0)
            processed_actions = env.action_manager.get_term("action").get_processed_actions()

            time_history.append(torch.tensor([t]).cpu())
            obs_history.append(obs["policy"].clone().detach().cpu())
            actions_processed_history.append(processed_actions)

            # print()
            # print(actions)
            # for key, value in processed_actions.items():
                # print(f"{key}: {value}")

            obs, rew, terminated, truncated, info = env.step(actions)
            
            actions_history.append(actions.clone().detach().cpu())
            feet_ik_pos_history.append(env.action_manager.get_term("action").get_feet_ik_pos())

            if args_cli.save_data and t % args_cli.save_interval < env.step_dt:
                path = os.path.join(save_path, save_filename)
                print(f"Saving data in path: {path}")
                actions_processed_stacked = {
                    key: torch.stack([d[key].view(-1) for d in actions_processed_history], dim=0)
                    for key in actions_processed_history[0]
                }
                torch.save({"time": torch.stack(time_history, dim=0), 
                            "obs": torch.stack(obs_history, dim=0), 
                            "actions": torch.stack(actions_history, dim=0),
                            "actions_processed": actions_processed_stacked,
                            "feet_ik_pos": torch.stack(feet_ik_pos_history, dim=0)}, path)
            
            t += env.step_dt
    
    env.close()

if __name__ == "__main__":
    main()
    simulation_app.close()

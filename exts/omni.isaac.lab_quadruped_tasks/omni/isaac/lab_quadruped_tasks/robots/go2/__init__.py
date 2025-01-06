"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks.agents.rsl_rl_cfg import QuadrupedPPORunnerCfg
from . import go2_env_cfg


##
# Create PPO runners for RSL-RL
##

go2_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
go2_blind_flat_runner_cfg.experiment_name = "go2_blind_flat"

go2_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
go2_blind_rough_runner_cfg.experiment_name = "go2_blind_rough"

go2_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
go2_blind_stairs_runner_cfg.experiment_name = "go2_blind_stairs"

go2_vision_runner_cfg = QuadrupedPPORunnerCfg()
go2_vision_runner_cfg.experiment_name = "go2_vision"
go2_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
go2_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

go2_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
go2_vision_stairs_runner_cfg.experiment_name = "go2_vision_stairs"
go2_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
go2_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##

############################
# Go2 Blind Flat Environment
############################

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": go2_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Flat-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": go2_blind_flat_runner_cfg,
    },
)

#############################
# Go2 Blind Rough Environment
#############################

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": go2_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Rough-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": go2_blind_rough_runner_cfg,
    },
)

##############################
# Go2 Blind Stairs Environment
##############################

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": go2_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Stairs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindStairsEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": go2_blind_stairs_runner_cfg,
    },
)

#########################
# Go2 Vision Environment
########################

gym.register(
    id="Isaac-Quadruped-Go2-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2VisionEnvCfg,
        "rsl_rl_cfg_entry_point": go2_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Vision-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2VisionEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": go2_vision_runner_cfg,
    },
)

###############################
# Go2 Vision Stairs Environment
###############################

gym.register(
    id="Isaac-Quadruped-Go2-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2VisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": go2_vision_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Vision-Stairs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2VisionStairsEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": go2_vision_stairs_runner_cfg,
    },
)

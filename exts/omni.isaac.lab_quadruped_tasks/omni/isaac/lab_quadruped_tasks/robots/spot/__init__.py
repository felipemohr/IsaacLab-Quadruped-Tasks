"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks.agents.rsl_rl_cfg import QuadrupedPPORunnerCfg
from . import spot_env_cfg


##
# Create PPO runners for RSL-RL
##

spot_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
spot_blind_flat_runner_cfg.experiment_name = "spot_blind_flat"

spot_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
spot_blind_rough_runner_cfg.experiment_name = "spot_blind_rough"

spot_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
spot_blind_stairs_runner_cfg.experiment_name = "spot_blind_stairs"

spot_vision_runner_cfg = QuadrupedPPORunnerCfg()
spot_vision_runner_cfg.experiment_name = "spot_vision"
spot_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
spot_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

spot_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
spot_vision_stairs_runner_cfg.experiment_name = "spot_vision_stairs"
spot_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
spot_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##

#############################
# Spot Blind Flat Environment
#############################

gym.register(
    id="Isaac-Quadruped-Spot-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": spot_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Blind-Flat-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotBlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": spot_blind_flat_runner_cfg,
    },
)

##############################
# Spot Blind Rough Environment
##############################

gym.register(
    id="Isaac-Quadruped-Spot-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": spot_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Blind-Rough-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotBlindRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": spot_blind_rough_runner_cfg,
    },
)

###################################
# Spot Blind Stairs Environment
###################################

gym.register(
    id="Isaac-Quadruped-Spot-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": spot_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Blind-Stairs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotBlindStairsEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": spot_blind_stairs_runner_cfg,
    },
)

#########################
# Spot Vision Environment
#########################

gym.register(
    id="Isaac-Quadruped-Spot-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotVisionEnvCfg,
        "rsl_rl_cfg_entry_point": spot_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Vision-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotVisionEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": spot_vision_runner_cfg,
    },
)

##################################
# Spot D Vision Stairs Environment
##################################

gym.register(
    id="Isaac-Quadruped-Spot-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": spot_vision_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Vision-Stairs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotVisionStairsEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": spot_vision_stairs_runner_cfg,
    },
)

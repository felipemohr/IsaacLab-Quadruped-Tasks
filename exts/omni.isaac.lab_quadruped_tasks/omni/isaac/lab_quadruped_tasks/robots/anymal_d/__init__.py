"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks.agents.rsl_rl_cfg import QuadrupedPPORunnerCfg
from . import anymal_d_env_cfg


##
# Create PPO runners for RSL-RL
##

anymal_d_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_blind_flat_runner_cfg.experiment_name = "anymal_d_blind_flat"

anymal_d_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_blind_rough_runner_cfg.experiment_name = "anymal_d_blind_rough"

anymal_d_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_blind_stairs_runner_cfg.experiment_name = "anymal_d_blind_stairs"

anymal_d_vision_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_vision_runner_cfg.experiment_name = "anymal_d_vision"
anymal_d_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
anymal_d_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

anymal_d_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_vision_stairs_runner_cfg.experiment_name = "anymal_d_vision_stairs"
anymal_d_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
anymal_d_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##

#################################
# Anymal D Blind Flat Environment
#################################

gym.register(
    id="Isaac-Quadruped-AnymalD-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Blind-Flat-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDBlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": anymal_d_blind_flat_runner_cfg,
    },
)

##################################
# Anymal D Blind Rough Environment
##################################

gym.register(
    id="Isaac-Quadruped-AnymalD-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Blind-Rough-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDBlindRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": anymal_d_blind_rough_runner_cfg,
    },
)

###################################
# Anymal D Blind Stairs Environment
###################################

gym.register(
    id="Isaac-Quadruped-AnymalD-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Blind-Stairs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDBlindStairsEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": anymal_d_blind_stairs_runner_cfg,
    },
)

#############################
# Anymal D Vision Environment
#############################

gym.register(
    id="Isaac-Quadruped-AnymalD-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDVisionEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Vision-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDVisionEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": anymal_d_vision_runner_cfg,
    },
)

####################################
# Anymal D Vision Stairs Environment
####################################

gym.register(
    id="Isaac-Quadruped-AnymalD-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_vision_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Vision-Stairs-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDVisionStairsEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": anymal_d_vision_stairs_runner_cfg,
    },
)

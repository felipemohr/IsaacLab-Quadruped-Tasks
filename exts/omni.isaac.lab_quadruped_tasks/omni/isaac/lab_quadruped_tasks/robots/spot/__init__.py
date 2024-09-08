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

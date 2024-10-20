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

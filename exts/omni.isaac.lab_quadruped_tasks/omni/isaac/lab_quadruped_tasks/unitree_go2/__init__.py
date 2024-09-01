"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from . import agents, go2_env_cfg

##
# Register Gym environments.
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
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2BlindFlatPPORunnerCfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Flat-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindFlatEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2BlindFlatPPORunnerCfg,
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
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2BlindRoughPPORunnerCfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Blind-Rough-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2BlindRoughEnvCfg_PLAY,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.Go2BlindRoughPPORunnerCfg,
    },
)

"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks import quadruped_env_cfg

from . import agents

# Register Gym environments
gym.register(
    id="Isaac-Quadruped-Go2-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": quadruped_env_cfg.QuadrupedEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.QuadrupedGo2PPORunnerCfg,
    },
)

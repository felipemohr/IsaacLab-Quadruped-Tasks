"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks.agents.rsl_rl_cfg import QuadrupedPPORunnerCfg
from . import spot_env_cfg


#########################
# Spot Joints Environment
#########################

##
# Create PPO runners for RSL-RL
##

spot_joints_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
spot_joints_blind_flat_runner_cfg.experiment_name = "spot_joints_blind_flat"

spot_joints_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
spot_joints_blind_rough_runner_cfg.experiment_name = "spot_joints_blind_rough"

spot_joints_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
spot_joints_blind_stairs_runner_cfg.experiment_name = "spot_joints_blind_stairs"

spot_joints_vision_runner_cfg = QuadrupedPPORunnerCfg()
spot_joints_vision_runner_cfg.experiment_name = "spot_joints_vision"
spot_joints_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
spot_joints_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

spot_joints_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
spot_joints_vision_stairs_runner_cfg.experiment_name = "spot_joints_vision_stairs"
spot_joints_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
spot_joints_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##

gym.register(
    id="Isaac-Quadruped-Spot-Joints-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotJointsBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": spot_joints_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Joints-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotJointsBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": spot_joints_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Joints-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotJointsBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": spot_joints_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Joints-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotJointsVisionEnvCfg,
        "rsl_rl_cfg_entry_point": spot_joints_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-Joints-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotJointsVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": spot_joints_vision_stairs_runner_cfg,
    },
)


######################
# Spot CPG Environment
######################

##
# Create PPO runners for RSL-RL
##


# CPG environments
spot_cpg_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
spot_cpg_blind_flat_runner_cfg.experiment_name = "spot_cpg_blind_flat"

spot_cpg_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
spot_cpg_blind_rough_runner_cfg.experiment_name = "spot_cpg_blind_rough"

spot_cpg_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
spot_cpg_blind_stairs_runner_cfg.experiment_name = "spot_cpg_blind_stairs"

spot_cpg_vision_runner_cfg = QuadrupedPPORunnerCfg()
spot_cpg_vision_runner_cfg.experiment_name = "spot_cpg_vision"
spot_cpg_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
spot_cpg_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

spot_cpg_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
spot_cpg_vision_stairs_runner_cfg.experiment_name = "spot_cpg_vision_stairs"
spot_cpg_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
spot_cpg_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##


gym.register(
    id="Isaac-Quadruped-Spot-CPG-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotCPGBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": spot_cpg_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-CPG-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotCPGBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": spot_cpg_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-CPG-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotCPGBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": spot_cpg_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-CPG-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotCPGVisionEnvCfg,
        "rsl_rl_cfg_entry_point": spot_cpg_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Spot-CPG-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": spot_env_cfg.SpotCPGVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": spot_cpg_vision_stairs_runner_cfg,
    },
)

"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks.agents.rsl_rl_cfg import QuadrupedPPORunnerCfg
from . import anymal_d_env_cfg


############################
# AnymalD Joints Environment
############################

##
# Create PPO runners for RSL-RL
##

anymal_d_joints_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_joints_blind_flat_runner_cfg.experiment_name = "anymal_d_joints_blind_flat"

anymal_d_joints_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_joints_blind_rough_runner_cfg.experiment_name = "anymal_d_joints_blind_rough"

anymal_d_joints_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_joints_blind_stairs_runner_cfg.experiment_name = "anymal_d_joints_blind_stairs"

anymal_d_joints_vision_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_joints_vision_runner_cfg.experiment_name = "anymal_d_joints_vision"
anymal_d_joints_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
anymal_d_joints_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

anymal_d_joints_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_joints_vision_stairs_runner_cfg.experiment_name = "anymal_d_joints_vision_stairs"
anymal_d_joints_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
anymal_d_joints_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##

gym.register(
    id="Isaac-Quadruped-AnymalD-Joints-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDJointsBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_joints_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Joints-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDJointsBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_joints_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Joints-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDJointsBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_joints_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Joints-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDJointsVisionEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_joints_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-Joints-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDJointsVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_joints_vision_stairs_runner_cfg,
    },
)


#########################
# AnymalD CPG Environment
#########################

##
# Create PPO runners for RSL-RL
##


# CPG environments
anymal_d_cpg_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_cpg_blind_flat_runner_cfg.experiment_name = "anymal_d_cpg_blind_flat"

anymal_d_cpg_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_cpg_blind_rough_runner_cfg.experiment_name = "anymal_d_cpg_blind_rough"

anymal_d_cpg_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_cpg_blind_stairs_runner_cfg.experiment_name = "anymal_d_cpg_blind_stairs"

anymal_d_cpg_vision_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_cpg_vision_runner_cfg.experiment_name = "anymal_d_cpg_vision"
anymal_d_cpg_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
anymal_d_cpg_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

anymal_d_cpg_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
anymal_d_cpg_vision_stairs_runner_cfg.experiment_name = "anymal_d_cpg_vision_stairs"
anymal_d_cpg_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
anymal_d_cpg_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##


gym.register(
    id="Isaac-Quadruped-AnymalD-CPG-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDCPGBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_cpg_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-CPG-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDCPGBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_cpg_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-CPG-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDCPGBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_cpg_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-CPG-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDCPGVisionEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_cpg_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-AnymalD-CPG-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": anymal_d_env_cfg.AnymalDCPGVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": anymal_d_cpg_vision_stairs_runner_cfg,
    },
)

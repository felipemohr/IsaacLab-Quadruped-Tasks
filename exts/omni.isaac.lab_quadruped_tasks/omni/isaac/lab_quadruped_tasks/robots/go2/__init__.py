"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

import gymnasium as gym

from omni.isaac.lab_quadruped_tasks.agents.rsl_rl_cfg import QuadrupedPPORunnerCfg
from . import go2_env_cfg


########################
# Go2 Joints Environment
########################

##
# Create PPO runners for RSL-RL
##

go2_joints_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
go2_joints_blind_flat_runner_cfg.experiment_name = "go2_joints_blind_flat"

go2_joints_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
go2_joints_blind_rough_runner_cfg.experiment_name = "go2_joints_blind_rough"

go2_joints_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
go2_joints_blind_stairs_runner_cfg.experiment_name = "go2_joints_blind_stairs"

go2_joints_vision_runner_cfg = QuadrupedPPORunnerCfg()
go2_joints_vision_runner_cfg.experiment_name = "go2_joints_vision"
go2_joints_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
go2_joints_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

go2_joints_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
go2_joints_vision_stairs_runner_cfg.experiment_name = "go2_joints_vision_stairs"
go2_joints_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
go2_joints_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##

gym.register(
    id="Isaac-Quadruped-Go2-Joints-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2JointsBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": go2_joints_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Joints-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2JointsBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": go2_joints_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Joints-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2JointsBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": go2_joints_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Joints-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2JointsVisionEnvCfg,
        "rsl_rl_cfg_entry_point": go2_joints_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-Joints-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2JointsVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": go2_joints_vision_stairs_runner_cfg,
    },
)


#####################
# Go2 CPG Environment
#####################

##
# Create PPO runners for RSL-RL
##


# CPG environments
go2_cpg_blind_flat_runner_cfg = QuadrupedPPORunnerCfg()
go2_cpg_blind_flat_runner_cfg.experiment_name = "go2_cpg_blind_flat"

go2_cpg_blind_rough_runner_cfg = QuadrupedPPORunnerCfg()
go2_cpg_blind_rough_runner_cfg.experiment_name = "go2_cpg_blind_rough"

go2_cpg_blind_stairs_runner_cfg = QuadrupedPPORunnerCfg()
go2_cpg_blind_stairs_runner_cfg.experiment_name = "go2_cpg_blind_stairs"

go2_cpg_vision_runner_cfg = QuadrupedPPORunnerCfg()
go2_cpg_vision_runner_cfg.experiment_name = "go2_cpg_vision"
go2_cpg_vision_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
go2_cpg_vision_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

go2_cpg_vision_stairs_runner_cfg = QuadrupedPPORunnerCfg()
go2_cpg_vision_stairs_runner_cfg.experiment_name = "go2_cpg_vision_stairs"
go2_cpg_vision_stairs_runner_cfg.policy.actor_hidden_dims = [512, 256, 128]
go2_cpg_vision_stairs_runner_cfg.policy.critic_hidden_dims = [512, 256, 128]

##
# Register Gym environments
##


gym.register(
    id="Isaac-Quadruped-Go2-CPG-Blind-Flat-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2CPGBlindFlatEnvCfg,
        "rsl_rl_cfg_entry_point": go2_cpg_blind_flat_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-CPG-Blind-Rough-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2CPGBlindRoughEnvCfg,
        "rsl_rl_cfg_entry_point": go2_cpg_blind_rough_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-CPG-Blind-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2CPGBlindStairsEnvCfg,
        "rsl_rl_cfg_entry_point": go2_cpg_blind_stairs_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-CPG-Vision-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2CPGVisionEnvCfg,
        "rsl_rl_cfg_entry_point": go2_cpg_vision_runner_cfg,
    },
)

gym.register(
    id="Isaac-Quadruped-Go2-CPG-Vision-Stairs-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": go2_env_cfg.Go2CPGVisionStairsEnvCfg,
        "rsl_rl_cfg_entry_point": go2_cpg_vision_stairs_runner_cfg,
    },
)

"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_quadruped_tasks.robots import base_envs_cfg as base_envs
from omni.isaac.lab_quadruped_tasks.cfg.quadruped_env_cfg import QuadrupedEnvCfg
from omni.isaac.lab_quadruped_tasks.cfg.quadruped_terrains_cfg import (
    ROUGH_TERRAINS_CFG,
    ROUGH_TERRAINS_PLAY_CFG,
    STAIRS_TERRAINS_CFG,
    STAIRS_TERRAINS_PLAY_CFG,
    FULL_TERRAINS_CFG,
    FULL_TERRAINS_PLAY_CFG,
)

from omni.isaac.lab_assets.unitree import UNITREE_GO2_CFG

import math

#######################
# Go2 Base Environments
#######################


@configclass
class Go2JointsBaseEnvCfg(base_envs.QuadrupedJointsEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": math.pi / 4,
            ".*calf_joint": -math.pi / 2,
        }

        self.actions.action.scale = 0.2

        self.rewards.rew_feet_air_time.weight = 0.75
        self.rewards.pen_joint_powers.weight = -3e-3
        self.rewards.pen_joint_deviation.weight = -0.1
        self.rewards.pen_undesired_contacts.weight = -0.25
        self.rewards.pen_offset_joints = None


@configclass
class Go2CPGBaseEnvCfg(base_envs.QuadrupedCPGEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": math.pi / 4,
            ".*calf_joint": -math.pi / 2,
        }
        # self.scene.robot.init_state.pos = (0, 0, 0.32)
        self.scene.robot.actuators["base_legs"].stiffness = 100.0
        self.scene.robot.actuators["base_legs"].damping = 2.0

        # self.events.change_gait = None
        # self.events.change_actuator_gains = None


########################
# Go2 Joints Environment
########################


@configclass
class Go2JointsBlindFlatEnvCfg(Go2JointsBaseEnvCfg, base_envs.QuadrupedBlindFlatEnvCfg):
    def __post_init__(self):
        Go2JointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindFlatEnvCfg.__post_init__(self)


@configclass
class Go2JointsBlindRoughEnvCfg(Go2JointsBaseEnvCfg, base_envs.QuadrupedBlindRoughEnvCfg):
    def __post_init__(self):
        Go2JointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindRoughEnvCfg.__post_init__(self)


@configclass
class Go2JointsBlindStairsEnvCfg(Go2JointsBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        Go2JointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)


@configclass
class Go2JointsVisionEnvCfg(Go2JointsBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        Go2JointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)


class Go2JointsVisionStairsEnvCfg(Go2JointsBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        Go2JointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)


#####################
# Go2 CPG Environment
#####################


@configclass
class Go2CPGBlindFlatEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedBlindFlatEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindFlatEnvCfg.__post_init__(self)
        self.actions.action.use_joints_offset = False


@configclass
class Go2CPGBlindRoughEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedBlindRoughEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindRoughEnvCfg.__post_init__(self)


@configclass
class Go2CPGBlindStairsEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)


@configclass
class Go2CPGVisionEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)


class Go2CPGVisionStairsEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)

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

from omni.isaac.lab_assets.anymal import ANYMAL_D_CFG

import math


########################
# AnymalD Base Environments
#######################


@configclass
class AnymalDJointsBaseEnvCfg(base_envs.QuadrupedJointsEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = ANYMAL_D_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        if self.rewards.rew_feet_air_time is not None:
            self.rewards.rew_feet_air_time.params["sensor_cfg"].body_names = ".*_FOOT"
        if self.rewards.pen_feet_slide is not None:
            self.rewards.pen_feet_slide.params["sensor_cfg"].body_names = ".*_FOOT"
            self.rewards.pen_feet_slide.params["asset_cfg"].body_names = ".*_FOOT"
        if self.rewards.pen_undesired_contacts is not None:
            self.rewards.pen_undesired_contacts.params["sensor_cfg"].body_names = ".*_THIGH"

        self.actions.action.scale = 0.5

        self.events.change_actuator_gains = None

        self.rewards.rew_feet_air_time.weight = 1.25
        self.rewards.pen_joint_deviation.weight = -0.125
        self.rewards.pen_undesired_contacts.weight = -1.0
        self.rewards.pen_feet_slide.weight = -0.1
        self.rewards.pen_joint_powers.weight = -5e-4


@configclass
class AnymalDCPGBaseEnvCfg(base_envs.QuadrupedCPGEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = ANYMAL_D_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/body"
        self.scene.robot.init_state.joint_pos = {
            ".*hx": 0.0,
            ".*hy": math.pi / 4,
            ".*kn": -math.pi / 2,
        }

        if self.events.add_base_mass is not None:
            self.events.add_base_mass.params["asset_cfg"].body_names = "body"
        if self.terminations.base_contact is not None:
            self.terminations.base_contact.params["sensor_cfg"].body_names = "body"
        if self.rewards.pen_undesired_contacts is not None:
            self.rewards.pen_undesired_contacts.params["sensor_cfg"].body_names = ".*_uleg"

        # TODO: Fix parameters and QuadrupedIKAction for anymal
        self.actions.action.front_left_joints = ["fl_.*"]
        self.actions.action.front_right_joints = ["fr_.*"]
        self.actions.action.rear_left_joints = ["hl_.*"]
        self.actions.action.rear_right_joints = ["hr_.*"]
        self.actions.action.hip_length = 0.11095
        self.actions.action.thigh_length = 0.3215
        self.actions.action.calf_length = 0.3365
        self.actions.action.foot_offset_x = 0.0
        self.actions.action.foot_offset_y = 0.11095
        self.actions.action.foot_offset_z = -0.465
        self.actions.action.step_size = 0.15

        self.events.change_gait = None
        self.events.change_actuator_gains = None


#########################
# AnymalD Joints Environment
########################


@configclass
class AnymalDJointsBlindFlatEnvCfg(AnymalDJointsBaseEnvCfg, base_envs.QuadrupedBlindFlatEnvCfg):
    def __post_init__(self):
        AnymalDJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindFlatEnvCfg.__post_init__(self)


@configclass
class AnymalDJointsBlindRoughEnvCfg(AnymalDJointsBaseEnvCfg, base_envs.QuadrupedBlindRoughEnvCfg):
    def __post_init__(self):
        AnymalDJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindRoughEnvCfg.__post_init__(self)


@configclass
class AnymalDJointsBlindStairsEnvCfg(AnymalDJointsBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        AnymalDJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)


@configclass
class AnymalDJointsVisionEnvCfg(AnymalDJointsBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        AnymalDJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)


class AnymalDJointsVisionStairsEnvCfg(AnymalDJointsBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        AnymalDJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)


######################
# AnymalD CPG Environment
#####################


@configclass
class AnymalDCPGBlindFlatEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedBlindFlatEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindFlatEnvCfg.__post_init__(self)
        self.actions.action.use_joints_offset = False
        self.rewards.pen_offset_joints = None


@configclass
class AnymalDCPGBlindRoughEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedBlindRoughEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindRoughEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"


@configclass
class AnymalDCPGBlindStairsEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)


@configclass
class AnymalDCPGVisionEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)


class AnymalDCPGVisionStairsEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)

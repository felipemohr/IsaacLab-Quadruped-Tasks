"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass
from omni.isaac.lab_quadruped_tasks.robots import base_envs_cfg as base_envs
from omni.isaac.lab_assets.anymal import ANYMAL_D_CFG, ANYDRIVE_3_SIMPLE_ACTUATOR_CFG


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
        self.scene.robot.actuators["legs"] = ANYDRIVE_3_SIMPLE_ACTUATOR_CFG
        self.scene.robot.actuators["legs"].stiffness = 100.0
        self.scene.robot.actuators["legs"].damping = 2.0

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/body"
        if self.observations.policy.feet_contact is not None:
            self.observations.policy.feet_contact.params["sensor_cfg"].body_names = ".*_FOOT"
        if self.rewards.rew_feet_air_time is not None:
            self.rewards.rew_feet_air_time.params["sensor_cfg"].body_names = ".*_FOOT"
        if self.rewards.pen_feet_slide is not None:
            self.rewards.pen_feet_slide.params["sensor_cfg"].body_names = ".*_FOOT"
            self.rewards.pen_feet_slide.params["asset_cfg"].body_names = ".*_FOOT"
        if self.rewards.pen_undesired_contacts is not None:
            self.rewards.pen_undesired_contacts.params["sensor_cfg"].body_names = ".*_THIGH"

        self.actions.action.front_left_joints = ["LF_.*"]
        self.actions.action.front_right_joints = ["RF_.*"]
        self.actions.action.rear_left_joints = ["LH_.*"]
        self.actions.action.rear_right_joints = ["RH_.*"]
        self.actions.action.front_legs_knee = False
        self.actions.action.rear_legs_knee = True
        self.actions.action.hip_length = 0.069
        self.actions.action.thigh_length = 0.3374
        self.actions.action.calf_length = 0.4056
        self.actions.action.foot_offset_x = -0.1
        self.actions.action.foot_offset_y = 0.0
        self.actions.action.foot_offset_z = -0.6
        self.actions.action.step_size = 0.2
        self.actions.action.ground_clearance = 0.2
        self.actions.action.ground_penetration = 0.02


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


@configclass
class AnymalDCPGBlindStairsEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"


@configclass
class AnymalDCPGVisionEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)
        self.events.change_gait = None


class AnymalDCPGVisionStairsEnvCfg(AnymalDCPGBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        AnymalDCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"

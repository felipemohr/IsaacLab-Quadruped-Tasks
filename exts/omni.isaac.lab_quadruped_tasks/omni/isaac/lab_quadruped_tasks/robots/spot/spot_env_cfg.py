"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_quadruped_tasks.robots import base_envs_cfg as base_envs

from omni.isaac.lab_assets.spot import SPOT_CFG

########################
# Spot Base Environments
########################


@configclass
class SpotJointsBaseEnvCfg(base_envs.QuadrupedJointsEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = SPOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/body"

        if self.events.add_base_mass is not None:
            self.events.add_base_mass.params["asset_cfg"].body_names = "body"
        if self.terminations.base_contact is not None:
            self.terminations.base_contact.params["sensor_cfg"].body_names = "body"
        if self.rewards.pen_undesired_contacts is not None:
            self.rewards.pen_undesired_contacts.params["sensor_cfg"].body_names = ".*_uleg"

        self.actions.action.scale = 0.2

        self.rewards.rew_feet_air_time.weight = 0.25
        self.rewards.pen_undesired_contacts.weight = -1.0
        self.rewards.pen_feet_slide.weight = -0.025
        self.rewards.pen_action_rate.weight = -0.05
        self.rewards.pen_joint_powers.weight = -1e-3


@configclass
class SpotCPGBaseEnvCfg(base_envs.QuadrupedCPGEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = SPOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.actuators["spot_hip"].stiffness = 100.0
        self.scene.robot.actuators["spot_hip"].damping = 2.0
        self.scene.robot.actuators["spot_knee"].stiffness = 100.0
        self.scene.robot.actuators["spot_knee"].damping = 2.0

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/body"
        if self.events.add_base_mass is not None:
            self.events.add_base_mass.params["asset_cfg"].body_names = "body"
        if self.terminations.base_contact is not None:
            self.terminations.base_contact.params["sensor_cfg"].body_names = "body"
        if self.rewards.pen_undesired_contacts is not None:
            self.rewards.pen_undesired_contacts.params["sensor_cfg"].body_names = ".*_uleg"

        self.actions.action.front_left_joints = ["fl_.*"]
        self.actions.action.front_right_joints = ["fr_.*"]
        self.actions.action.rear_left_joints = ["hl_.*"]
        self.actions.action.rear_right_joints = ["hr_.*"]
        self.actions.action.hip_length = 0.11095
        self.actions.action.thigh_length = 0.3215
        self.actions.action.calf_length = 0.3365
        self.actions.action.foot_offset_x = -0.1
        self.actions.action.foot_offset_y = 0.11095
        self.actions.action.foot_offset_z = -0.465
        self.actions.action.step_size = 0.2
        self.actions.action.ground_clearance = 0.2
        self.actions.action.ground_penetration = 0.02


#########################
# Spot Joints Environment
#########################


@configclass
class SpotJointsBlindFlatEnvCfg(SpotJointsBaseEnvCfg, base_envs.QuadrupedBlindFlatEnvCfg):
    def __post_init__(self):
        SpotJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindFlatEnvCfg.__post_init__(self)


@configclass
class SpotJointsBlindRoughEnvCfg(SpotJointsBaseEnvCfg, base_envs.QuadrupedBlindRoughEnvCfg):
    def __post_init__(self):
        SpotJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindRoughEnvCfg.__post_init__(self)


@configclass
class SpotJointsBlindStairsEnvCfg(SpotJointsBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        SpotJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)


@configclass
class SpotJointsVisionEnvCfg(SpotJointsBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        SpotJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)


class SpotJointsVisionStairsEnvCfg(SpotJointsBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        SpotJointsBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)


######################
# Spot CPG Environment
######################


@configclass
class SpotCPGBlindFlatEnvCfg(SpotCPGBaseEnvCfg, base_envs.QuadrupedBlindFlatEnvCfg):
    def __post_init__(self):
        SpotCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindFlatEnvCfg.__post_init__(self)
        self.actions.action.use_joints_offset = False
        self.rewards.pen_offset_joints = None


@configclass
class SpotCPGBlindRoughEnvCfg(SpotCPGBaseEnvCfg, base_envs.QuadrupedBlindRoughEnvCfg):
    def __post_init__(self):
        SpotCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindRoughEnvCfg.__post_init__(self)
        self.events.change_gait = None


@configclass
class SpotCPGBlindStairsEnvCfg(SpotCPGBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        SpotCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"

@configclass
class SpotCPGVisionEnvCfg(SpotCPGBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        SpotCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)
        self.events.change_gait = None


class SpotCPGVisionStairsEnvCfg(SpotCPGBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        SpotCPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"

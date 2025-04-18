"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from isaaclab.utils import configclass
from isaaclab_quadruped_tasks.robots import base_envs_cfg as base_envs
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG

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


@configclass
class Go2CPGBaseEnvCfg(base_envs.QuadrupedCPGEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.actuators["base_legs"].stiffness = 100.0
        self.scene.robot.actuators["base_legs"].damping = 2.0

        # IK Parameters
        self.actions.action.front_left_joints = ["FL_.*"]
        self.actions.action.front_right_joints = ["FR_.*"]
        self.actions.action.rear_left_joints = ["RL_.*"]
        self.actions.action.rear_right_joints = ["RR_.*"]
        self.actions.action.front_legs_knee = False
        self.actions.action.rear_legs_knee = False
        self.actions.action.hip_length = 0.0955
        self.actions.action.thigh_length = 0.2130
        self.actions.action.calf_length = 0.2130
        self.actions.action.foot_offset_x = -0.05
        self.actions.action.foot_offset_y = 0.0955
        self.actions.action.foot_offset_z = -0.3012
        # CPG Parameters
        self.actions.action.gait_type = "trot"
        self.actions.action.use_duty_cycle = False
        # self.actions.action.gait_frequency_limit = 3.0
        # self.actions.action.duty_cycle_limit = (0.5, 0.6)
        self.actions.action.convergence_factor = 50.0
        self.actions.action.swing_frequency_limit = 6.0
        self.actions.action.stance_frequency_limit = 6.0
        self.actions.action.oscilator_limit = (0.5, 2.0)
        self.actions.action.step_size = 0.05
        self.actions.action.ground_clearance = 0.1
        self.actions.action.ground_penetration = 0.01
        self.actions.action.feet_distance_x = 0.3868
        self.actions.action.body_height_offset = 0.0
        self.actions.action.body_pitch_offset = 0.0
        self.actions.action.use_joints_offset = True
        self.actions.action.joints_offset_scale = 0.1


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
        self.events.change_gait = None


@configclass
class Go2CPGBlindStairsEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedBlindStairsEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedBlindStairsEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"
        self.actions.action.body_height_offset = 0.05
        self.actions.action.ground_clearance = 0.15
        self.actions.action.ground_penetration = 0.015
        self.actions.action.swing_frequency_limit = 6.0
        self.actions.action.stance_frequency_limit = 2.0


@configclass
class Go2CPGVisionEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedVisionEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"
        self.actions.action.body_height_offset = 0.05
        self.actions.action.ground_clearance = 0.15
        self.actions.action.ground_penetration = 0.015
        self.actions.action.swing_frequency_limit = 6.0
        self.actions.action.stance_frequency_limit = 2.0


class Go2CPGVisionStairsEnvCfg(Go2CPGBaseEnvCfg, base_envs.QuadrupedVisionStairsEnvCfg):
    def __post_init__(self):
        Go2CPGBaseEnvCfg.__post_init__(self)
        base_envs.QuadrupedVisionStairsEnvCfg.__post_init__(self)
        self.events.change_gait = None
        self.actions.action.gait_type = "walk"
        self.actions.action.body_height_offset = 0.05
        self.actions.action.ground_clearance = 0.15
        self.actions.action.ground_penetration = 0.015
        self.actions.action.swing_frequency_limit = 6.0
        self.actions.action.stance_frequency_limit = 2.0

"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass

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


######################
# Go2 Base Environment
######################


@configclass
class Go2BaseEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": math.pi / 4,
            ".*calf_joint": -math.pi / 2,
        }
        self.scene.robot.init_state.pos = (0, 0, 0.32)
        self.scene.robot.actuators["base_legs"].stiffness = 100.0
        self.scene.robot.actuators["base_legs"].damping = 2.0

        self.rewards.pen_lin_vel_z.weight = -1.0
        self.rewards.pen_ang_vel_xy.weight = -0.05

        self.rewards.rew_lin_vel_xy.weight = 3.0
        self.rewards.rew_ang_vel_z.weight = 1.5
        self.rewards.pen_joint_powers.weight = -1e-3

        self.rewards.rew_feet_air_time = None
        self.rewards.pen_joint_deviation = None
        self.rewards.pen_undesired_contacts = None
        self.rewards.pen_feet_slide = None
        self.rewards.pen_action_rate = None
        self.rewards.pen_joint_accel = None
        self.rewards.pen_flat_orientation = None
        # self.terminations.base_contact = None

        # self.actions.joint_pos.scale = 0.2

        # self.rewards.rew_feet_air_time.weight = 0.75
        # self.rewards.pen_joint_powers.weight = -3e-3
        # self.rewards.pen_joint_deviation.weight = -0.1
        # self.rewards.pen_undesired_contacts.weight = -0.25

@configclass
class Go2BaseEnvCfg_PLAY(Go2BaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 64

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.push_robot = None
        # remove random base mass addition event
        self.events.add_base_mass = None


############################
# Go2 Blind Flat Environment
############################


@configclass
class Go2BlindFlatEnvCfg(Go2BaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.actions.cpg_action.use_joints_offset = False

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None
        self.curriculum.terrain_levels = None


@configclass
class Go2BlindFlatEnvCfg_PLAY(Go2BaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()
        self.actions.cpg_action.use_joints_offset = False

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None
        self.curriculum.terrain_levels = None


#############################
# Go2 Blind Rough Environment
#############################


@configclass
class Go2BlindRoughEnvCfg(Go2BaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = ROUGH_TERRAINS_CFG

        self.viewer.origin_type = "env"


@configclass
class Go2BlindRoughEnvCfg_PLAY(Go2BaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = ROUGH_TERRAINS_PLAY_CFG


##############################
# Go2 Blind Stairs Environment
##############################


@configclass
class Go2BlindStairsEnvCfg(Go2BaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-math.pi / 6, math.pi / 6)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (0.0, 0.0)

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_CFG

        self.viewer.origin_type = "env"


@configclass
class Go2BlindStairsEnvCfg_PLAY(Go2BaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (0.0, 0.0)

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_PLAY_CFG


########################
# Go2 Vision Environment
########################


@configclass
class Go2VisionEnvCfg(Go2BaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = FULL_TERRAINS_CFG

        self.events.change_vel_cmd = None

        self.viewer.origin_type = "env"


@configclass
class Go2VisionEnvCfg_PLAY(Go2BaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = FULL_TERRAINS_PLAY_CFG

        self.events.change_vel_cmd = None

        self.viewer.origin_type = "env"


###############################
# Go2 Vision Stairs Environment
###############################


@configclass
class Go2VisionStairsEnvCfg(Go2BaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-math.pi / 6, math.pi / 6)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (0.0, 0.0)

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_CFG

        self.viewer.origin_type = "env"


@configclass
class Go2VisionStairsEnvCfg_PLAY(Go2BaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (0.0, 0.0)

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_PLAY_CFG

        self.viewer.origin_type = "env"

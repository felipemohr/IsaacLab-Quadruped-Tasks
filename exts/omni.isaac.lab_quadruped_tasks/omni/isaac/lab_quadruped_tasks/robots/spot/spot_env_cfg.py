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

from omni.isaac.lab_assets.spot import SPOT_CFG

import math


#######################
# Spot Base Environment
#######################


@configclass
class SpotBaseEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = SPOT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/body"

        self.actions.joint_pos.scale = 0.2

        self.rewards.pen_undesired_contacts.params["sensor_cfg"].body_names = ".*_uleg"
        self.rewards.rew_feet_air_time.weight = 0.25
        self.rewards.pen_feet_slide.weight = -0.025
        self.rewards.pen_joint_powers.weight = -1e-3
        self.rewards.pen_action_rate.weight = -0.05
        self.rewards.pen_joint_deviation.weight = -0.1

        self.events.add_base_mass.params["asset_cfg"].body_names = "body"
        self.events.add_base_mass.params["mass_distribution_params"] = (-2.5, 5.0)

        self.terminations.base_contact.params["sensor_cfg"].body_names = "body"


@configclass
class SpotBaseEnvCfg_PLAY(SpotBaseEnvCfg):
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


#############################
# Spot Blind Flat Environment
#############################


@configclass
class SpotBlindFlatEnvCfg(SpotBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None
        self.curriculum.terrain_levels = None


@configclass
class SpotBlindFlatEnvCfg_PLAY(SpotBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None
        self.curriculum.terrain_levels = None


##############################
# Spot Blind Rough Environment
##############################


@configclass
class SpotBlindRoughEnvCfg(SpotBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = ROUGH_TERRAINS_CFG

        self.viewer.origin_type = "env"


@configclass
class SpotBlindRoughEnvCfg_PLAY(SpotBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = ROUGH_TERRAINS_PLAY_CFG


###############################
# Spot Blind Stairs Environment
###############################


@configclass
class SpotBlindStairsEnvCfg(SpotBaseEnvCfg):
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
class SpotBlindStairsEnvCfg_PLAY(SpotBaseEnvCfg_PLAY):
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


###############################
# Spot Blind Stairs Environment
###############################


@configclass
class SpotVisionEnvCfg(SpotBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = FULL_TERRAINS_CFG

        self.events.change_vel_cmd = None

        self.viewer.origin_type = "env"


@configclass
class SpotVisionEnvCfg_PLAY(SpotBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = FULL_TERRAINS_PLAY_CFG

        self.events.change_vel_cmd = None

        self.viewer.origin_type = "env"


##################################
# Spot Vision Stairs Environment
###################################


@configclass
class SpotVisionStairsEnvCfg(SpotBaseEnvCfg):
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
class SpotVisionStairsEnvCfg_PLAY(SpotBaseEnvCfg_PLAY):
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

"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_quadruped_tasks.cfg.quadruped_env_cfg import QuadrupedEnvCfg
from omni.isaac.lab_quadruped_tasks.cfg.quadruped_terrains_cfg import (
    BLIND_HARD_ROUGH_TERRAINS_CFG,
    BLIND_HARD_ROUGH_TERRAINS_PLAY_CFG,
    STAIRS_TERRAINS_CFG,
    STAIRS_TERRAINS_PLAY_CFG,
)

from omni.isaac.lab_assets.anymal import ANYMAL_D_CFG

import math


#########################
# Anymal Base Environment
#########################


@configclass
class AnymalDBaseEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = ANYMAL_D_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        self.rewards.pen_feet_slide.params["sensor_cfg"].body_names = ".*_FOOT"
        self.rewards.pen_feet_slide.params["asset_cfg"].body_names = ".*_FOOT"

        self.events.add_base_mass.params["mass_distribution_params"] = (-3.5, 7.5)

        self.rewards.pen_joint_powers.weight = -1e-4
        self.rewards.pen_joint_deviation.weight = -0.1


@configclass
class AnymalDBaseEnvCfg_PLAY(AnymalDBaseEnvCfg):
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


###############################
# Anymal Blind Flat Environment
###############################


@configclass
class AnymalDBlindFlatEnvCfg(AnymalDBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.rewards.pen_flat_orientation = None

        self.curriculum.terrain_levels = None


@configclass
class AnymalDBlindFlatEnvCfg_PLAY(AnymalDBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.curriculum.terrain_levels = None


################################
# Anymal Blind Rough Environment
################################


@configclass
class AnymalDBlindRoughEnvCfg(AnymalDBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.rewards.pen_flat_orientation = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = BLIND_HARD_ROUGH_TERRAINS_CFG

        # update viewport camera
        self.viewer.origin_type = "env"


@configclass
class AnymalDBlindRoughEnvCfg_PLAY(AnymalDBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = BLIND_HARD_ROUGH_TERRAINS_PLAY_CFG


#################################
# Anymal Blind Stairs Environment
#################################


@configclass
class AnymalDBlindStairsEnvCfg(AnymalDBaseEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-math.pi / 6, math.pi / 6)

        self.rewards.pen_joint_deviation.weight = -0.2

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_CFG.replace(difficulty_range=(0.5, 1.0))

        # update viewport camera
        self.viewer.origin_type = "env"


@configclass
class AnymalDBlindStairsEnvCfg_PLAY(AnymalDBaseEnvCfg_PLAY):
    def __post_init__(self):
        super().__post_init__()

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.0, 0.0)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (-0.0, 0.0)

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_PLAY_CFG

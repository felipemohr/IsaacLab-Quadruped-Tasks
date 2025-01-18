"""
License: MIT License
Copyright (c) 2025, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_quadruped_tasks import mdp
from omni.isaac.lab_quadruped_tasks.cfg.quadruped_env_cfg import QuadrupedEnvCfg
from omni.isaac.lab_quadruped_tasks.cfg.quadruped_terrains_cfg import (
    ROUGH_TERRAINS_CFG,
    STAIRS_TERRAINS_CFG,
    FULL_TERRAINS_CFG,
)

import math


#############################
# Quadruped Base Environments
#############################


@configclass
class QuadrupedJointsEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.actions.action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )

        self.rewards.pen_offset_joints = None

        self.observations.policy.feet_contact = None
        self.observations.policy.cpg_state = None

        self.events.change_gait = None


@configclass
class QuadrupedCPGEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.rewards.rew_lin_vel_xy.weight = 3.0
        self.rewards.rew_ang_vel_z.weight = 1.5
        self.rewards.pen_lin_vel_z.weight = -1.0
        self.rewards.pen_ang_vel_xy.weight = -0.1
        self.rewards.pen_offset_joints.weight = -0.01
        self.rewards.pen_joint_powers.weight = -1e-3

        self.rewards.rew_feet_air_time = None
        self.rewards.pen_joint_deviation = None
        self.rewards.pen_undesired_contacts = None
        self.rewards.pen_feet_slide = None
        self.rewards.pen_action_rate = None
        self.rewards.pen_joint_accel = None
        self.rewards.pen_flat_orientation = None

        # TODO: Randomize CPG parameters
        # These parameters are for go2 robot
        self.actions.action = mdp.QuadrupedCPGActionCfg(
            asset_name="robot",
            # IK Parameters
            front_left_joints=["FL_.*"],
            front_right_joints=["FR_.*"],
            rear_left_joints=["RL_.*"],
            rear_right_joints=["RR_.*"],
            hip_length=0.0955,  # 0.11095
            thigh_length=0.2130,  # 0.3215
            calf_length=0.2130,  # 0.3365
            foot_offset_x=0.0,
            foot_offset_y=0.0955,
            foot_offset_z=-0.3012,
            # CPG Parameters
            swing_frequency_limit=5.0,
            stance_frequency_limit=3.0,
            oscilator_limit=(0.5, 2.0),
            step_size=0.1,
            ground_clearance=0.1,
            ground_penetration=0.01,
            body_height_offset=0.0,
            use_joints_offset=True,
            joints_offset_scale=0.05,
            gait_type="trot",
        )


############################
# Quadruped Play Environment
############################


@configclass
class QuadrupedPlayEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 32

        # disable observation noise for play
        self.observations.policy.enable_corruption = False

        # disable randomizations for play
        self.events.physics_material = None
        self.events.add_base_mass = None
        self.events.change_actuator_gains = None
        self.events.push_robot = None

        # the play scene can start at any difficult level
        self.scene.terrain.max_init_terrain_level = None


##############################
# Quadruped Blind Environments
##############################


@configclass
class QuadrupedBlindFlatEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None
        self.curriculum.terrain_levels = None


@configclass
class QuadrupedBlindRoughEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.height_scanner = None
        self.observations.policy.height_map = None

        self.events.change_vel_cmd = None

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = ROUGH_TERRAINS_CFG

        self.viewer.origin_type = "env"


@configclass
class QuadrupedBlindStairsEnvCfg(QuadrupedEnvCfg):
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


###############################
# Quadruped Vision Environments
###############################


@configclass
class QuadrupedVisionEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = FULL_TERRAINS_CFG

        self.events.change_vel_cmd = None

        self.viewer.origin_type = "env"


@configclass
class QuadrupedVisionStairsEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-math.pi / 6, math.pi / 6)

        self.events.reset_robot_base.params["pose_range"]["yaw"] = (0.0, 0.0)

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = STAIRS_TERRAINS_CFG

        self.viewer.origin_type = "env"

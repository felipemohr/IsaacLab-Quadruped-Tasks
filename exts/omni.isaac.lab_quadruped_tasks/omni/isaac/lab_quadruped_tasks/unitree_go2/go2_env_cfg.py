"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass
from omni.isaac.lab.terrains import (
    TerrainGeneratorCfg,
    MeshPlaneTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
    MeshRandomGridTerrainCfg,
)

from omni.isaac.lab_quadruped_tasks.quadruped_env_cfg import QuadrupedEnvCfg

from omni.isaac.lab_assets.unitree import UNITREE_GO2_CFG

import math


#############################
# Rough Terrain Configuration
#############################


BLIND_ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=42,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=16,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=True,
    sub_terrains={
        "flat": MeshPlaneTerrainCfg(proportion=0.25),
        "waves": HfWaveTerrainCfg(proportion=0.25, amplitude_range=(0.01, 0.06), num_waves=10, border_width=0.25),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.25, grid_width=0.15, grid_height_range=(0.01, 0.04), platform_width=2.0
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.25, noise_range=(0.01, 0.06), noise_step=0.01, border_width=0.25
        ),
    },
    curriculum=True,
    difficulty_range=(0.0, 1.0),
)

BLIND_ROUGH_TERRAINS_PLAY_CFG = TerrainGeneratorCfg(
    seed=42,
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=4,
    num_cols=4,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=True,
    sub_terrains={
        "waves": HfWaveTerrainCfg(proportion=0.33, amplitude_range=(0.01, 0.06), num_waves=10, border_width=0.25),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.2, grid_width=0.33, grid_height_range=(0.01, 0.04), platform_width=2.0
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.34, noise_range=(0.01, 0.06), noise_step=0.01, border_width=0.25
        ),
    },
    curriculum=False,
    difficulty_range=(1.0, 1.0),
)


############################
# Go2 Blind Flat Environment
############################


@configclass
class Go2BlindFlatEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": math.pi / 4,
            ".*calf_joint": -math.pi / 2,
        }

        self.curriculum.terrain_levels = None


@configclass
class Go2BlindFlatEnvCfg_PLAY(Go2BlindFlatEnvCfg):
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
# Go2 Blind Rough Environment
#############################


@configclass
class Go2BlindRoughEnvCfg(QuadrupedEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.init_state.joint_pos = {
            ".*hip_joint": 0.0,
            ".*thigh_joint": math.pi / 4,
            ".*calf_joint": -math.pi / 2,
        }

        self.scene.terrain.terrain_type = "generator"
        self.scene.terrain.terrain_generator = BLIND_ROUGH_TERRAINS_CFG

        # update viewport camera
        self.viewer.eye = (0.0, 0.0, 75.0)


@configclass
class Go2BlindRoughEnvCfg_PLAY(Go2BlindRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 64

        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        self.scene.terrain.terrain_generator = BLIND_ROUGH_TERRAINS_PLAY_CFG

        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing event
        self.events.push_robot = None
        # remove random base mass addition event
        self.events.add_base_mass = None

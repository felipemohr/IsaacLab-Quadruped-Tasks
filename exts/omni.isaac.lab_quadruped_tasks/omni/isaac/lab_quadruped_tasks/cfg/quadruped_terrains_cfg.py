"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.terrains import (
    TerrainGeneratorCfg,
    MeshPlaneTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfWaveTerrainCfg,
    MeshRandomGridTerrainCfg,
    MeshPyramidStairsTerrainCfg,
    MeshInvertedPyramidStairsTerrainCfg,
    HfPyramidSlopedTerrainCfg,
    HfInvertedPyramidSlopedTerrainCfg,
)


#############################
# Rough Terrain Configuration
#############################

ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
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
        "waves": HfWaveTerrainCfg(proportion=0.25, amplitude_range=(0.02, 0.12), num_waves=8, border_width=0.25),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.25, grid_width=0.45, grid_height_range=(0.02, 0.16), platform_width=2.0
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.25, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
        ),
    },
    curriculum=True,
    difficulty_range=(0.0, 1.0),
)

ROUGH_TERRAINS_PLAY_CFG = ROUGH_TERRAINS_CFG.copy()
ROUGH_TERRAINS_PLAY_CFG.num_rows = 4
ROUGH_TERRAINS_PLAY_CFG.num_cols = 4
ROUGH_TERRAINS_PLAY_CFG.sub_terrains["flat"].proportion = 0.0
ROUGH_TERRAINS_PLAY_CFG.sub_terrains["waves"].proportion = 0.33
ROUGH_TERRAINS_PLAY_CFG.sub_terrains["boxes"].proportion = 0.33
ROUGH_TERRAINS_PLAY_CFG.sub_terrains["random_rough"].proportion = 0.34
ROUGH_TERRAINS_PLAY_CFG.curriculum = False
ROUGH_TERRAINS_PLAY_CFG.difficulty_range = (0.75, 0.75)


##############################
# Stairs Terrain Configuration
##############################

STAIRS_TERRAINS_CFG = TerrainGeneratorCfg(
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
        "pyramid_stairs_inv": MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.4,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "pyramid_stairs": MeshPyramidStairsTerrainCfg(
            proportion=0.4,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "hf_pyramid_slope_inv": HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
        "hf_pyramid_slope": HfPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
    },
    curriculum=True,
    difficulty_range=(0.0, 1.0),
)

STAIRS_TERRAINS_PLAY_CFG = STAIRS_TERRAINS_CFG.copy()
STAIRS_TERRAINS_PLAY_CFG.num_rows = 4
STAIRS_TERRAINS_PLAY_CFG.num_cols = 4
STAIRS_TERRAINS_PLAY_CFG.sub_terrains["pyramid_stairs_inv"].proportion = 0.5
STAIRS_TERRAINS_PLAY_CFG.sub_terrains["pyramid_stairs"].proportion = 0.5
STAIRS_TERRAINS_PLAY_CFG.curriculum = False
STAIRS_TERRAINS_PLAY_CFG.difficulty_range = (0.75, 0.75)


############################
# Full Terrain Configuration
############################

FULL_TERRAINS_CFG = TerrainGeneratorCfg(
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
        "waves": HfWaveTerrainCfg(proportion=0.1, amplitude_range=(0.02, 0.12), num_waves=8, border_width=0.25),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.2, grid_width=0.45, grid_height_range=(0.02, 0.16), platform_width=2.0
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.2, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
        ),
        "pyramid_stairs": MeshPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "pyramid_stairs_inv": MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=1.0,
            holes=False,
        ),
        "hf_pyramid_slope": HfPyramidSlopedTerrainCfg(
            proportion=0.05, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
        "hf_pyramid_slope_inv": HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.05, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
        ),
    },
    curriculum=True,
    difficulty_range=(0.0, 1.0),
)

FULL_TERRAINS_PLAY_CFG = FULL_TERRAINS_CFG.copy()
FULL_TERRAINS_PLAY_CFG.num_rows = 3
FULL_TERRAINS_PLAY_CFG.num_cols = 5
FULL_TERRAINS_PLAY_CFG.curriculum = False
FULL_TERRAINS_PLAY_CFG.difficulty_range = (0.75, 0.75)

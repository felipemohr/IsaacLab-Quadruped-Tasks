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
        "waves": HfWaveTerrainCfg(proportion=0.25, amplitude_range=(0.02, 0.10), num_waves=8, border_width=0.25),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.25, grid_width=0.45, grid_height_range=(0.02, 0.12), platform_width=2.0
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.25, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
        ),
    },
    curriculum=True,
    difficulty_range=(0.0, 1.0),
)

ROUGH_TERRAINS_PLAY_CFG = TerrainGeneratorCfg(
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
        "waves": HfWaveTerrainCfg(proportion=0.33, amplitude_range=(0.02, 0.10), num_waves=8, border_width=0.25),
        "boxes": MeshRandomGridTerrainCfg(
            proportion=0.33, grid_width=0.45, grid_height_range=(0.02, 0.12), platform_width=2.0
        ),
        "random_rough": HfRandomUniformTerrainCfg(
            proportion=0.34, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
        ),
    },
    curriculum=False,
    difficulty_range=(1.0, 1.0),
)


##############################
# Stairs Terrain Configuration
##############################

STAIRS_TERRAINS_CFG = TerrainGeneratorCfg(
    seed=42,
    size=(12.0, 12.0),
    border_width=20.0,
    num_rows=8,
    num_cols=10,
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

STAIRS_TERRAINS_PLAY_CFG = TerrainGeneratorCfg(
    seed=42,
    size=(12.0, 12.0),
    border_width=20.0,
    num_rows=4,
    num_cols=4,
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
    difficulty_range=(1.0, 1.0),
)

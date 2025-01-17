"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def joint_powers_l1(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize joint powers on the articulation using L1-kernel"""

    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(torch.abs(torch.mul(asset.data.applied_torque, asset.data.joint_vel)), dim=1)

def partial_action_l2(env: ManagerBasedRLEnv, first_idx: int, last_idx: int) -> torch.Tensor:
    """Penalize parcial actions using L2 squared kernel."""
    return torch.sum(torch.square(env.action_manager.action[:, first_idx:last_idx+1]), dim=1)

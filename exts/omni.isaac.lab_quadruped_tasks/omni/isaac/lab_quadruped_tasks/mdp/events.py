"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def invert_vel_cmd(env: ManagerBasedRLEnv, env_ids: torch.Tensor | None, command_name: str):
    """ """
    # obtain command settings
    cmd_cfg = env.command_manager.get_term(command_name)
    # update command settings
    min_vel = cmd_cfg.cfg.ranges.lin_vel_x[0]
    max_vel = cmd_cfg.cfg.ranges.lin_vel_x[1]
    cmd_cfg.cfg.ranges.lin_vel_x = (-max_vel, -min_vel)

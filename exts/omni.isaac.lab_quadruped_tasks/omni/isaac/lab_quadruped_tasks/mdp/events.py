"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from omni.isaac.lab_quadruped_tasks.mdp.actions.quadruped_actions import QuadrupedCPGAction


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


def change_gait_type(
    env: ManagerBasedRLEnv, env_ids: torch.Tensor | None, action_name: str, gaits: Sequence[str] | None = None
):
    """ """
    # extract the used quantities (to enable type-hinting)
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=env.device)
    cpg_action: QuadrupedCPGAction = env.action_manager.get_term(action_name)

    if gaits == None:
        gait_idxs = torch.randint_like(env_ids, 0, len(cpg_action.supported_gaits))
    else:
        gaits_to_use = torch.tensor([cpg_action.supported_gaits.index(gait) for gait in gaits], device=env.device)
        random_gaits = torch.randint_like(env_ids, 0, gaits_to_use.size(0))
        gait_idxs = gaits_to_use[random_gaits]

    cpg_action.set_gait_type(gait_idxs, env_ids)

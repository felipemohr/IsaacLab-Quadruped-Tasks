"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from omni.isaac.lab.managers import SceneEntityCfg
from omni.isaac.lab.sensors import ContactSensor
from omni.isaac.lab_quadruped_tasks.mdp.actions.quadruped_actions import QuadrupedCPGAction

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedRLEnv


def feet_contact_bools(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg, threshold: float) -> torch.Tensor:
    """Feet contact booleans. The foot is in contact when the force sensor exceeds the threshold"""

    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w
    # check which contact forces exceed the threshold
    return torch.norm(net_contact_forces[:, sensor_cfg.body_ids], dim=-1) > threshold


def time_sine_cossine(env: ManagerBasedRLEnv) -> torch.Tensor:
    """The time represented as sine and cossine."""

    sine_cossine = torch.concatenate(
        [torch.sin(torch.Tensor([env.sim.current_time])), torch.cos(torch.Tensor([env.sim.current_time]))]
    ).to(env.device)
    return sine_cossine.unsqueeze(0).repeat(env.num_envs, 1)


def cpg_states(env: ManagerBasedRLEnv, cpg_action_name: str) -> torch.Tensor:
    """The Central Pattern Generator states from cpg action term in the action manager with the given name."""

    # extract the used quantities (to enable type-hinting)
    cpg_action: QuadrupedCPGAction = env.action_manager.get_term(cpg_action_name)
    # return the states of the Central Pattern Generator
    return cpg_action.get_cpg_states()

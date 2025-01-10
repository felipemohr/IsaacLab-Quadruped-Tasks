"""
License: MIT License
Copyright (c) 2025, Felipe Mohr Santos
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

import math
import omni.log

import omni.isaac.lab.utils.string as string_utils
from omni.isaac.lab.assets.articulation import Articulation
from omni.isaac.lab.managers.action_manager import ActionTerm

if TYPE_CHECKING:
    from omni.isaac.lab.envs import ManagerBasedEnv

    from . import actions_cfg


class JointCPGAction(ActionTerm):
    """Joint CPG action term that applies the processed actions to the articulation's joints as position commands"""

    cfg: actions_cfg.JointCPGActionCfg
    """The configuration of the action term."""
    _asset: Articulation
    """The articulation asset on which the action term is applied."""
    _scale: torch.Tensor | float
    """The scaling factor applied to the input action."""
    _offset: torch.Tensor | float
    """The offset applied to the input action."""
    _frequency_omega: torch.Tensor | float
    """The frequency of the oscilator applied to the input action."""
    _phase_beta: torch.Tensor | float
    """The phase offset of the oscilator applied to the input action."""

    def __init__(self, cfg: actions_cfg.JointCPGActionCfg, env: ManagerBasedEnv) -> None:
        # initialize the action term
        super().__init__(cfg, env)

        # resolve the joints over which the action term is applied
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names, preserve_order=self.cfg.preserve_order
        )
        self._num_joints = len(self._joint_ids)
        # log the resolved joint names for debugging
        omni.log.info(
            f"Resolved joint names for the action term {self.__class__.__name__}:"
            f" {self._joint_names} [{self._joint_ids}]"
        )

        # Avoid indexing across all joints for efficiency
        if self._num_joints == self._asset.num_joints and not self.cfg.preserve_order:
            self._joint_ids = slice(None)

        # create tensors for raw and processed actions
        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, self._num_joints, device=self.device)
        # self._processed_actions = torch.zeros_like(self.raw_actions)

        # Create tensor for time elapsed
        self._time_elapsed = torch.zeros(self.num_envs, self._num_joints, device=self.device)
        self._control_period = env.sim.cfg.dt * env.cfg.decimation

        # parse scale
        if isinstance(cfg.scale, (float, int)):
            self._scale = float(cfg.scale)
        elif isinstance(cfg.scale, dict):
            self._scale = torch.ones(self.num_envs, self._num_joints, device=self.device)
            # resolve the dictionary config
            index_list, _, value_list = string_utils.resolve_matching_names_values(self.cfg.scale, self._joint_names)
            self._scale[:, index_list] = torch.tensor(value_list, device=self.device)
        else:
            raise ValueError(f"Unsupported scale type: {type(cfg.scale)}. Supported types are float and dict.")

        # parse offset
        if cfg.use_default_offset:
            self._offset = self._asset.data.default_joint_pos[:, self._joint_ids].clone()
        else:
            if isinstance(cfg.offset, (float, int)):
                self._offset = float(cfg.offset)
            elif isinstance(cfg.offset, dict):
                self._offset = torch.zeros_like(self._processed_actions)
                # resolve the dictionary config
                index_list, _, value_list = string_utils.resolve_matching_names_values(
                    self.cfg.offset, self._joint_names
                )
                self._offset[:, index_list] = torch.tensor(value_list, device=self.device)
            else:
                raise ValueError(f"Unsupported offset type: {type(cfg.offset)}. Supported types are float and dict.")
        # parse frequency omega
        if isinstance(cfg.frequency_omega, (float, int)):
            self._frequency_omega = float(cfg.frequency_omega)
        elif isinstance(cfg.frequency_omega, dict):
            self._frequency_omega = torch.ones(self.num_envs, self._num_joints, device=self.device)
            # resolve the dictionary config
            index_list, _, value_list = string_utils.resolve_matching_names_values(
                self.cfg.frequency_omega, self._joint_names
            )
            self._frequency_omega[:, index_list] = torch.tensor(value_list, device=self.device)
        else:
            raise ValueError(
                f"Unsupported frequency_omega type: {type(cfg.frequency_omega)}. Supported types are float and dict."
            )
        # parse phase_beta
        if isinstance(cfg.phase_beta, (float, int)):
            self._phase_beta = float(cfg.phase_beta)
        elif isinstance(cfg.phase_beta, dict):
            self._phase_beta = torch.zeros_like(self._processed_actions)
            # resolve the dictionary config
            index_list, _, value_list = string_utils.resolve_matching_names_values(
                self.cfg.phase_beta, self._joint_names
            )
            self._phase_beta[:, index_list] = torch.tensor(value_list, device=self.device)
        else:
            raise ValueError(
                f"Unsupported phase_beta type: {type(cfg.phase_beta)}. Supported types are float and dict."
            )

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return self._num_joints

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor):
        # store the raw actions
        self._raw_actions[:] = actions
        # update time elapsed
        self._time_elapsed += self._control_period

        # apply the affine transformations
        self._processed_actions = self._offset + self._scale * self._raw_actions * torch.cos(
            self._frequency_omega * self._time_elapsed + self._phase_beta
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0
        self._time_elapsed[env_ids] = 0.0

    def apply_actions(self):
        # set position targets
        self._asset.set_joint_position_target(self.processed_actions, joint_ids=self._joint_ids)

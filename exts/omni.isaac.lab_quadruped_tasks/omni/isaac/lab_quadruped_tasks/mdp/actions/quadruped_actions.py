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


class QuadrupedIKAction(ActionTerm):
    """Quadruped Inverse Kinematics action term that computes all the joints of a quadruped robot and
    applies the processed actions to the articulation's joints as position commands"""

    cfg: actions_cfg.QuadrupedIKActionCfg
    """The configuration of the action term."""
    _asset: Articulation
    """The articulation asset on which the action term is applied."""
    _leg_dimensions: dict[str, float]
    """The leg dimensions of the quadruped used to compute inverse kinematics."""
    _foot_offsets: dict[str, float]
    """The offset applied to the input action to compute inverse kinematics."""
    _action_limits: dict[str, float]
    """The limits applied to the input action to compute inverse kinematics, disregarding the foot offset."""

    def __init__(self, cfg: actions_cfg.QuadrupedIKActionCfg, env: ManagerBasedEnv) -> None:
        # initialize the action term
        super().__init__(cfg, env)

        # resolve the joints over which the action term is applied
        self._fl_joint_ids, self._fl_joint_names = self._asset.find_joints(
            self.cfg.front_left_joints, preserve_order=self.cfg.preserve_order
        )
        self._fr_joint_ids, self._fr_joint_names = self._asset.find_joints(
            self.cfg.front_right_joints, preserve_order=self.cfg.preserve_order
        )
        self._rl_joint_ids, self._rl_joint_names = self._asset.find_joints(
            self.cfg.rear_left_joints, preserve_order=self.cfg.preserve_order
        )
        self._rr_joint_ids, self._rr_joint_names = self._asset.find_joints(
            self.cfg.rear_right_joints, preserve_order=self.cfg.preserve_order
        )
        self._num_joints = (
            len(self._fl_joint_ids) + len(self._fr_joint_ids) + len(self._rl_joint_ids) + len(self._rr_joint_ids)
        )
        if self._num_joints != 12:
            raise ValueError(f"Unsupported number of joints: {self._num_joints}. Supported number is only 12.")

        # log the resolved joint names for debugging
        omni.log.info(
            f"Resolved joint names for the action term {self.__class__.__name__}:"
            f" Front Left: {self._fl_joint_names} [{self._fl_joint_ids}]"
            f" Front Right: {self._fr_joint_names} [{self._fr_joint_ids}]"
            f" Rear Left: {self._rl_joint_names} [{self._rl_joint_ids}]"
            f" Rear Right: {self._rr_joint_names} [{self._rr_joint_ids}]"
        )

        # create tensors for raw and processed actions
        self._raw_actions = torch.zeros(self.num_envs, self.action_dim, device=self.device)
        self._processed_actions = torch.zeros(self.num_envs, self._num_joints, device=self.device)
        # used to avoid sending NaN values to the joints
        self._last_processed_actions = torch.zeros_like(self._processed_actions)

        # parse leg dimensions
        self._leg_dimensions = dict()
        if isinstance(cfg.hip_length, (float, int)):
            self._leg_dimensions["L1"] = float(cfg.hip_length)
        else:
            raise ValueError(f"Unsupported hip_length type: {type(cfg.hip_length)}. Supported type is float.")
        if isinstance(cfg.thigh_length, (float, int)):
            self._leg_dimensions["L2"] = float(cfg.thigh_length)
        else:
            raise ValueError(f"Unsupported thigh_length type: {type(cfg.thigh_length)}. Supported type is float.")
        if isinstance(cfg.calf_length, (float, int)):
            self._leg_dimensions["L3"] = float(cfg.calf_length)
        else:
            raise ValueError(f"Unsupported calf_length type: {type(cfg.calf_length)}. Supported type is float.")

        # parse foot offsets
        self._foot_offsets = dict()
        if isinstance(cfg.foot_offset_x, (float, int)):
            self._foot_offsets["x"] = float(cfg.foot_offset_x)
        else:
            raise ValueError(f"Unsupported foot_offset_x type: {type(cfg.foot_offset_x)}. Supported type is float.")
        if isinstance(cfg.foot_offset_y, (float, int)):
            self._foot_offsets["y"] = float(cfg.foot_offset_y)
        else:
            raise ValueError(f"Unsupported foot_offset_y type: {type(cfg.foot_offset_y)}. Supported type is float.")
        if isinstance(cfg.foot_offset_z, (float, int)):
            self._foot_offsets["z"] = float(cfg.foot_offset_z)
        else:
            raise ValueError(f"Unsupported foot_offset_z type: {type(cfg.foot_offset_z)}. Supported type is float.")

        # parse action limits
        self._action_limits = dict()
        if isinstance(cfg.action_limit_x, (float, int)):
            self._action_limits["x"] = float(abs(cfg.action_limit_x))
        else:
            raise ValueError(f"Unsupported action_limit_x type: {type(cfg.action_limit_x)}. Supported type is float.")
        if isinstance(cfg.action_limit_y, (float, int)):
            self._action_limits["y"] = float(abs(cfg.action_limit_y))
        else:
            raise ValueError(f"Unsupported action_limit_y type: {type(cfg.action_limit_y)}. Supported type is float.")
        if isinstance(cfg.action_limit_z, (float, int)):
            self._action_limits["z"] = float(abs(cfg.action_limit_z))
        else:
            raise ValueError(f"Unsupported action_limit_z type: {type(cfg.action_limit_z)}. Supported type is float.")

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        return 12  # (x, y, z) for each foot

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

        front_left_action = self._compute_leg_ik(self._raw_actions[:, :3], True)
        front_right_action = self._compute_leg_ik(self._raw_actions[:, 3:6], False)
        rear_left_action = self._compute_leg_ik(self._raw_actions[:, 6:9], True)
        rear_right_action = self._compute_leg_ik(self._raw_actions[:, 9:], False)

        self._processed_actions = torch.concatenate(
            [front_left_action, front_right_action, rear_left_action, rear_right_action], dim=1
        )
        self._processed_actions = torch.where(
            self._processed_actions.isnan(), self._last_processed_actions, self._processed_actions
        )

        self._last_processed_actions = self._processed_actions

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0
        self._last_processed_actions = torch.zeros_like(self._processed_actions)

    def apply_actions(self):
        # set position targets
        self._asset.set_joint_position_target(self.processed_actions[:, :3], joint_ids=self._fl_joint_ids)
        self._asset.set_joint_position_target(self.processed_actions[:, 3:6], joint_ids=self._fr_joint_ids)
        self._asset.set_joint_position_target(self.processed_actions[:, 6:9], joint_ids=self._rl_joint_ids)
        self._asset.set_joint_position_target(self.processed_actions[:, 9:], joint_ids=self._rr_joint_ids)

    """
    Helper functions.
    """

    def _compute_leg_ik(self, foot_action: torch.Tensor, is_left: bool = False) -> torch.Tensor:
        L1, L2, L3 = self._leg_dimensions["L1"], self._leg_dimensions["L2"], self._leg_dimensions["L3"]
        max_x, max_y, max_z = self._action_limits["x"], self._action_limits["y"], self._action_limits["z"]
        reflect = 1.0 if is_left else -1.0

        x, y, z = foot_action.T
        if max_x < torch.inf:
            x = max_x * torch.tanh(x / max_x)
        if max_y < torch.inf:
            y = max_y * torch.tanh(y / max_y)
        if max_z < torch.inf:
            z = max_z * torch.tanh(z / max_z)

        x += self._foot_offsets["x"]
        y += reflect * self._foot_offsets["x"]
        z += self._foot_offsets["z"]

        a = torch.sqrt(y**2 + z**2 - L1**2)
        A = (a**2 + x**2 + L2**2 - L3**2) / (2 * L2 * torch.sqrt(a**2 + x**2))
        B = (a**2 + x**2 - L2**2 - L3**2) / (2 * L2 * L3)

        theta1 = torch.atan2(y, -z) - torch.atan2(torch.tensor([reflect * L1], device=self.device), a)
        theta2 = math.pi / 2 - torch.atan2(a, x) - torch.atan2(torch.sqrt(1.0 - A**2), A)
        theta3 = torch.atan2(torch.sqrt(1.0 - B**2), B)

        return torch.concatenate([theta1.unsqueeze(1), -theta2.unsqueeze(1), -theta3.unsqueeze(1)], dim=1)

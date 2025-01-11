"""
License: MIT License
Copyright (c) 2025, Felipe Mohr Santos
"""

from dataclasses import MISSING

from omni.isaac.lab_quadruped_tasks.mdp import JointActionCfg
from omni.isaac.lab.managers.action_manager import ActionTerm, ActionTermCfg
from omni.isaac.lab.utils import configclass

import torch

from . import quadruped_actions

##
# Quadruped actions.
##


@configclass
class JointCPGActionCfg(JointActionCfg):
    """Configuration for the Joint CPG action term.

    See :class:`JointCPGAction` for more details.
    """

    class_type: type[ActionTerm] = quadruped_actions.JointCPGAction

    use_default_offset: bool = True
    """Whether to use default joint positions configured in the articulation asset as offset.
    Defaults to True.

    If True, this flag results in overwriting the values of :attr:`offset` to the default joint positions
    from the articulation asset.
    """
    frequency_omega: float | dict[str, float] = 2 * torch.pi
    """The frequency for the oscilatory signal action (float or dict of regex expressions). Defaults to 2*pi."""
    phase_beta: float | dict[str, float] = 0.0
    """The phase offset for the oscilatory signal action (float or dict of regex expressions). Defaults to 0.0."""


@configclass
class QuadrupedIKActionCfg(ActionTermCfg):
    """Configuration for the base joint action term.

    See :class:`QuadrupedIKAction` for more details.
    """

    class_type: type[ActionTerm] = quadruped_actions.QuadrupedIKAction

    front_left_joints: list[str] = MISSING
    """List of front left joint names or regex expressions that the action will be mapped to."""
    front_right_joints: list[str] = MISSING
    """List of front right joint names or regex expressions that the action will be mapped to."""
    rear_left_joints: list[str] = MISSING
    """List of rear left joint names or regex expressions that the action will be mapped to."""
    rear_right_joints: list[str] = MISSING
    """List of rear right joint names or regex expressions that the action will be mapped to."""

    preserve_order: bool = False
    """Whether to preserve the order of the joint names in the action output. Defaults to False."""

    hip_length: float = MISSING
    """The length of the hip joint of the quadruped robot."""
    thigh_length: float = MISSING
    """The length of the thigh joint of the quadruped robot."""
    calf_length: float = MISSING
    """The length of the calf joint of the quadruped robot."""

    foot_offset_x: float = 0.0
    """The offset in x axis for the action to compute inverse kinematics, w.r.t hip link. Defaults to 0.0."""
    foot_offset_y: float = 0.0
    """The offset in y axis for the action to compute inverse kinematics, w.r.t hip link. Defaults to 0.0."""
    foot_offset_z: float = 0.0
    """The offset in z axis for the action to compute inverse kinematics, w.r.t hip link. Defaults to 0.0."""

    action_limit_x: float = torch.inf
    """The limit in x axis to compute inverse kinematics, disregarding the foot offset. Defaults to infinity."""
    action_limit_y: float = torch.inf
    """The limit in y axis to compute inverse kinematics, disregarding the foot offset. Defaults to infinity."""
    action_limit_z: float = torch.inf
    """The limit in z axis to compute inverse kinematics, disregarding the foot offset. Defaults to infinity."""

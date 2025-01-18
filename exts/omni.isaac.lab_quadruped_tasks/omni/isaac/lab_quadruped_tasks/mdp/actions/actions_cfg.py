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

    convergence_factor: float = 50.0
    """The convergence factor used to generate the CPG signals. Defaults to 50.0."""

    phase_offset: float | dict[str, float] = 0.0
    """Phase offset factor for the action (float or dict of regex expressions). Defaults to 0.0."""

    frequency_limit: float = torch.inf
    """The limit frequency in Hz to use in Central Pattern Generator. Defaults to infinity."""
    oscilator_limit: float = torch.inf
    """The limit of the oscilator amplitude to use in Central Pattern Generator. Defaults to infinity."""


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


@configclass
class QuadrupedCPGActionCfg(QuadrupedIKActionCfg):
    """Configuration for the base joint action term.

    See :class:`QuadrupedCPGAction` for more details.
    """

    class_type: type[ActionTerm] = quadruped_actions.QuadrupedCPGAction

    swing_frequency_limit: float = torch.inf
    """The limit swing frequency in Hz to use in Central Pattern Generator in swing phase. Defaults to infinity."""
    stance_frequency_limit: float = torch.inf
    """The limit stance frequency in Hz to use in Central Pattern Generator in stance phase. Defaults to infinity."""
    oscilator_limit: float | tuple[float, float] = torch.inf
    """The limit of the oscilator amplitude to use in Central Pattern Generator. Defaults to infinity."""

    convergence_factor: float = 50.0
    """The convergence factor used to generate the CPG signals. Defaults to 50.0."""
    step_size: float = 0.1
    """The step size in meters used to convert the CPG space to cartesian space. Defaults to 0.2."""
    ground_clearance: float = 0.1
    """The maximum height of the foot when in swing fase in meters. Defaults to 0.1."""
    ground_penetration: float = 0.005
    """The maximum penetration of the foot in the ground when in swing fase in meters. Defaults to 0.005."""

    body_height_offset: float = 0.0
    """The offset of the body height in meters w.r.t its default height. Defaults to 0.0."""

    use_joints_offset: bool = True
    """Whether to compute an offset in the position of the joints to change the values generated by the CPG or not.
    This will also define the size of the output action vector.
    """
    joints_offset_scale: float = 0.05
    """Scale factor for the offset of joints in the action. Only used if use_joints_offset is True. Defaults to 0.05."""

    gait_type: str = "trot"
    """The gait type used to generate the gait pattern. Available options are ['trot', 'walk', 'pace', 'gallop'].
    Defaults to 'trot'.
    """

"""
License: MIT License
Copyright (c) 2025, Felipe Mohr Santos
"""

from dataclasses import MISSING

from omni.isaac.lab_quadruped_tasks.mdp import JointActionCfg
from omni.isaac.lab.managers.action_manager import ActionTerm, ActionTermCfg
from omni.isaac.lab.utils import configclass

import math

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
    frequency_omega: float | dict[str, float] = 2 * math.pi
    """The frequency for the oscilatory signal action (float or dict of regex expressions). Defaults to 2*pi."""
    phase_beta: float | dict[str, float] = 0.0
    """The phase offset for the oscilatory signal action (float or dict of regex expressions). Defaults to 0.0."""

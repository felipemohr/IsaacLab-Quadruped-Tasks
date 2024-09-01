"""
License: MIT License
Copyright (c) 2024, Felipe Mohr Santos
"""

from omni.isaac.lab.utils import configclass
from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class QuadrupedGo2PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 40000
    save_interval = 1000
    experiment_name = "quadruped_go2"
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        class_name="ActorCritic",
        init_noise_std=1.0,
        actor_hidden_dims=[128, 128, 128],
        critic_hidden_dims=[128, 128, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class Go2BlindFlatPPORunnerCfg(QuadrupedGo2PPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.experiment_name = "go2_blind_flat"


@configclass
class Go2BlindRoughPPORunnerCfg(QuadrupedGo2PPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.experiment_name = "go2_blind_rough"

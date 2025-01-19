Changelog
---------

X.0.0 (Unreleased)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Add ``physics_material`` and ``change_actuator_gains`` event terms
* Create new `partial_action_l2` reward term
* Create new `time_sine_cossine` observation term
* Create new `change_gait_type` event term
* Create new actions for quadruped robots - `JointCPGAction`, `QuadrupedIKAction` and `QuadrupedCPGAction`
* Create `base_envs_cfg.py`
* Create new tasks for ``go2``, ``anymal_d`` and ``spot`` using CPG


Changed
^^^^^

* Change ``add_base_mass`` to scale the robot mass
* Change simulation frequency to 400 Hz
* Refactor quadruped tasks

4.0.0 (2025-01-06)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Create ``vision`` and ``vision_stairs`` tasks for the three robots
* Create ``invert_vel_cmd`` event
* Create `FULL_TERRAINS_CFG` terrain

Changed
^^^^^

* Update pre-trained models for all tasks and robots
* Update for Isaac Lab 1.4.0
* Update reward terms weights
* Use only one `ROUGH_TERRAINS_CFG` for all robots
* Change simulation frequency to 200 Hz
* Change ``robots/unitree_go2`` folder to just ``go2``
* Use the same reward terms and weights for all tasks
* Include ``rew_feet_air_time`` reward term
* Remove ``pen_feet_slide`` penalization term


3.3.0 (2024-10-20)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Create `STAIRS_TERRAINS_CFG` and `STAIRS_TERRAINS_PLAY_CFG` terrain configurations
* Create ``pen_flat_orientation`` penalization term for `QuadrupedEnvCfg`
* Create `Go2BlindStairsEnvCfg`, `AnymalDBlindStairsEnvCfg` and `SpotBlindStairsEnvCfg` environment configurations
* Create tasks for Stairs terrains for Go2, AnymalD and Spot robots
* Add pre-trained models for ``go2_blind_stairs``, ``anymal_d_blind_stairs`` and ``spot_blind_stairs`` experiments
* Add ``--checkpoint_path`` argument to rsl_rl train script

Changed
^^^^^

* Create `Go2BaseEnvCfg`, `AnymalDBaseEnvCfg` and `SpotBaseEnvCfg` environment base classes


3.2.0 (2024-09-26)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Create `BLIND_HARD_ROUGH_TERRAINS_CFG` and `BLIND_HARD_ROUGH_TERRAINS_PLAY_CFG` terrain configurations
* Create ``pen_undesired_contacts`` penalization term for `QuadrupedEnvCfg`

Changed
^^^^^

* Update for Isaac Sim 4.2.0 and Isaac Lab 1.2.0
* Change ``scale`` of ``joint_pos`` action to 0.2
* Use new `BLIND_HARD_ROUGH_TERRAINS_CFG` in AnymalD and Spot tasks
* Update pre-trained models for ``go2_blind_flat`` and ``go2_blind_rough`` experiments
* Update pre-trained models for ``spot_blind_flat`` and ``spot_blind_rough`` experiments
* Update pre-trained models for ``anymal_d_blind_flat`` and ``anymal_d_blind_rough`` experiments


3.1.0 (2024-09-14)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Add pre-trained models for ``anymal_d_blind_flat`` and ``anymal_d_blind_rough`` experiments
* Add pre-trained models for ``spot_blind_flat`` and ``spot_blind_rough`` experiments

Changed
^^^^^

* Update pre-trained models for ``go2_blind_flat`` and ``go2_blind_rough`` experiments
* Update ``pen_joint_powers`` weight for `QuadrupedEnvCfg`
* Update ``pen_joint_powers`` weight for `Spot` environments
* Update ``pen_joint_powers`` and ``pen_joint_deviation`` weight fors `ANYmalD` environments
* Remove ``increase_push_vel`` curriculum from `QuadrupedEnvCfg`
* Remove ``bad_orientation`` termination from `QuadrupedEnvCfg`


3.0.0 (2024-09-08)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Create `cfg`, `robot` and `agent` folders
* Create tasks for Spot robot from Boston Dynamics
* Create tasks for ANYmal D robot from ANYbotics

Changed
^^^^^

* Move `unitree_go2` to `robots` folder
* Move `rsl_rl_cfg.py` to `agent` folder
* Move `quadruped_env_cfg.py` to `cfg` folder
* Move `go2_env_cfg.py` to `robots` folder
* Move `BLIND_ROUGH_TERRAINS_CFG` and `BLIND_ROUGH_TERRAINS_PLAY_CFG` to `quadruped_terrains_cfg.py`


2.1.0 (2024-09-06)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Add pre-trained models for ``go2_blind_flat`` and ``go2_blind_rough`` experiments
* Create ``modify_event_parameter`` curriculum and include it in `QuadrupedEnvCfg` to modify ``push_robot`` event 
* Create ``disable_termination`` curriculum
* Set terrain generator seed in `go2_env_cfg.py`

Changed
^^^^^

* Increase ``lin_vel_x`` range of ``vel_command`` command in `QuadrupedEnvCfg`
* Modify viewport camera of `Go2BlindRoughEnvCfg`
* Update terrains proportion in `BLIND_ROUGH_TERRAINS_CFG` in `go2_env_cfg.py`


2.0.0 (2024-09-01)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Include ``terrain_levels`` curriculum in `QuadrupedEnvCfg`
* Create terrain configurations for training Go2 in rough terrains
* Create `Go2BlindFlatPPORunnerCfg` and `Go2BlindRoughPPORunnerCfg` configurations
* Create `Go2BlindFlatEnvCfg` and `Go2BlindRoughEnvCfg` environment configurations for training Go2 robot
* Create `Go2BlindFlatEnvCfg_PLAY` and `Go2BlindRoughEnvCfg_PLAY` environment configurations for playing Go2 robot

Changed
^^^^^

* Update README.md with information for training and playing agents
* Remove robot initialization from `QuadrupedSceneCfg`
* Use `TerrainImporterCfg` in `QuadrupedSceneCfg` instead of `GroundPlaneCfg`
* Replace `Isaac-Quadruped-Go2-Flat-v0` with `Isaac-Quadruped-Go2-Blind-Flat-v0`

1.1.0 (2024-08-30)
~~~~~~~~~~~~~~~~~~

Changed
^^^^^

* Update robot joint positions initial states in `QuadrupedEnvCfg`
* Include ``pen_joint_deviation`` and ``pen_feet_slide`` penalization terms in `QuadrupedEnvCfg`
* Remove ``rew_feet_air_time`` reward term from `QuadrupedEnvCfg`
* Remove ``joint_vel`` and ``feet_contact`` observations from `QuadrupedEnvCfg`

Fixed
^^^^^

* Fix `RSL-RL` training and playing scripts for Isaac Sim 4.1.0 and Isaac Lab 1.1.0
* Fix ``randomize_rigid_body_mass`` event for Isaac Sim 4.1.0 and Isaac Lab 1.1.0


1.0.0 (2024-07-04)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Create `RSL-RL` training and playing scripts
* Create Unitree Go2 `RSL-RL` agent
* Create `QuadrupedEnvCfg` environment
* Create `omni.isaac.lab_quadruped_tasks` extension for Isaac Lab

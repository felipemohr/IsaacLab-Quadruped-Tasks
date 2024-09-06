Changelog
---------

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

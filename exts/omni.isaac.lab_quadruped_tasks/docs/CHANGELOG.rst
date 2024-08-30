Changelog
---------

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

* Fixed `RSL-RL` training and playing scripts for Isaac Sim 4.1.0 and Isaac Lab 1.1.0
* Fixed ``randomize_rigid_body_mass`` event for Isaac Sim 4.1.0 and Isaac Lab 1.1.0


1.0.0 (2024-07-04)
~~~~~~~~~~~~~~~~~~

Added
^^^^^

* Created `RSL-RL` training and playing scripts
* Created Unitree Go2 `RSL-RL` agent
* Created `QuadrupedEnvCfg` environment
* Created `omni.isaac.lab_quadruped_tasks` extension for Isaac Lab

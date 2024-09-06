# Isaac Lab Quadruped Tasks Extension

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.1.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.1.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview

This repository contains an extension with tasks for training quadruped robots using Reinforcement Learning in Isaac Lab.

So far, there are 2 tasks that can be used to train Unitree's Go2 robot:

- `Isaac-Quadruped-Go2-Blind-Flat-v0`: blind locomotion, only on flat terrain
- `Isaac-Quadruped-Go2-Blind-Rough-v0`: blind locomotion, on irregular terrain

## Installation
1. Begin by installing NVIDIA's [Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html) and [Isaac Lab](https://isaac-sim.github.io/IsaacLab/source/setup/installation/binaries_installation.html).
2. This repository includes an Isaac Lab extension with the quadruped tasks. To install it, follow these steps:

```bash
$ git clone git@github.com:felipemohr/IsaacLab-Quadruped-Tasks.git
$ cd IsaacLab-Quadruped-Tasks
$ conda activate isaaclab
$ python -m pip install -e exts/omni.isaac.lab_quadruped_tasks
```

## Training the Quadruped Agent

Use the `rsl_rl/train.py` script to train the robot, specifying the task:

```bash
$ python scripts/rsl_rl/train.py --task Isaac-Quadruped-Go2-Blind-Flat-v0 --headless
```

The available tasks are `Isaac-Quadruped-Go2-Blind-Flat-v0` and `Isaac-Quadruped-Go2-Blind-Rough-v0`, and the `--headless` flag is used to disable the viewport, to speed up the training significantly.

The following arguments are optional, but can be used to specify the training configurations:

- `--num_envs` - Number of environments to simulate (default is `1024`)
- `--max_iterations` - Maximum number of iterations to train (default is `20000`)
- `--save_interval` - The number of iterations between saves (default is `1000`)
- `--seed` - Seed used for the environment (default is `42`)

If you want to enable video clips recording during training, you can include the following arguments, along with `--enable_cameras` and `--video` flags:

- `--video_length` - Length of each recorded video, in steps (default is `200`)
- `--video_interval` - Interval between each video recording, in steps (default is `20000`)

The entire command would be something like:

```bash
$ python scripts/rsl_rl/train.py --task Isaac-Quadruped-Go2-Blind-Flat-v0 --num_envs 1024 --max_iterations 20000 --save_interval 1000 --seed 42 --headless --enable_cameras --video --video_length 200 --video_interval 20000
```

To resume the training from a checkpoint, you can set the `--reset` to `True` and specify the run directory and checkpoint.  

- `--resume` - Whether to resume the training (default is `False`)
- `--load_run` - The run directory to load (default is `".*"`, the latest in alphabetical order matching run will be loaded)
- `--load_checkpoint` - The checkpoint file to load (default is `"model_.*.pt"`, the latest in alphabetical order matching file will be loaded)

Training logs will be generated in the directory where the training script was executed. Visualize these logs using TensorBoard:

```bash
$ python -m tensorboard.main --logdir=$PATH_TO_YOUR_LOGS_DIR$
```

## Playing the Trained Agent

Use the `rsl_rl/play.py` script to play the trained agent, specifying the task and the model path:

```bash
$ python scripts/rsl_rl/play.py --task Isaac-Quadruped-Go2-Blind-Flat-Play-v0 --num_envs 64 --checkpoint_path logs/rsl_rl/go2_blind_flat/XXXX-XX-XX_XX-XX-XX/model_XXXX.pt
```

The `--num_envs` argument is optional and can also be used to define the number of environments to simulate (default is `64`).

Note that the task used ends with `-Play-v0` instead of just `-v0`. This task is exactly the same as the one used for training, but excluding the randomization terms used to make the agent more robust.

You can also use the pre-trained models present in `models` folder:

```bash
$ python scripts/rsl_rl/play.py --task Isaac-Quadruped-Go2-Blind-Rough-Play-v0 --checkpoint_path models/go2_blind_rough/model_20k.pt
```

# Universal Robots UR5 environment for reinforcement learning

# Project on UR5-gym: MISIS

MISIS: Reinforcement learning for UR5 robotic manipulator

# Table of content
-  [General description](#general-description)
-  [Structure](#struct)
-  [Installation](#install)

# General description <a name="general-description"></a>
This is a robotic Environment with [UR5](https://github.com/ros-industrial/universal_robot) robot arm. 
It uses Bullet to simulate physical interactions between robot and environment.
Reinforcement learning is realized via [Stable Baselines 3](https://github.com/DLR-RM/stable-baselines3).

## Structure <a name="struct"></a>
You can find more information:
- Descriptions of each skill (State, Action, Reward, ...) is given in [envs/*_env.py](ur5_gym/envs).
- Running scripts are given in [examples](examples)

## Installation <a name="install"></a>

Install with:  
```bash
pip3 install -e .
```

**Note**: *-e* flag is for editable mode, so that changes to the code are reflected immediately.

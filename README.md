# Universal Robots UR5 environment for reinforcement learning

# Project on UR5-gym: MISIS

MISIS: Reinforcement learning for UR5 robotic manipulator

# Table of content
- [Table of content](#table-of-content)
- [General description ](#general-description-)
  - [Simulation ](#simulation-)
- [Structure ](#structure-)
- [Installation ](#installation-)

# General description <a name="general-description"></a>
This is a robotic Environment with [UR5](https://github.com/ros-industrial/universal_robot) robot arm. 
It uses Bullet to simulate physical interactions between robot and environment.
Reinforcement learning is realized via [Stable Baselines 3](https://github.com/DLR-RM/stable-baselines3).

## Simulation <a name="simulation"></a>
This is a simulation of the UR10 robot arm in the [PyBullet](https://pybullet.org/wordpress/) simulator. We have already implemented the following skills:

|                                  |                                                |
| :------------------------------: | :--------------------------------------------: |
|         `Reach skill`            |                 `Pick skill`           |
|  ![Reach](/readme-assets/reach.gif)        |         ![Pick](/readme-assets/pick.gif)             |
|         `Push skill`             |             `Slide skill`                      |
|  ![Push](/readme-assets/push.gif)          |                      ![Slide](/readme-assets/slide.gif)  |

**Note**: Now we are working on the new version of the environment for the [**UR Sim**](https://www.universal-robots.com/download/software-cb-series/simulator-non-linux/offline-simulator-cb-series-non-linux-ursim-3143/). The new version will be available soon.
## Structure <a name="struct"></a>
You can find more information:
- Descriptions of each skill (State, Action, Reward, ...) is given in [envs/*_env.py](/ur5_gym/envs).
- Running scripts are given in [examples](/examples)
- Dockerfiles and simulators are given in [docker](/docker)

## Installation <a name="install"></a>
We recommend using [Docker](/docker) to run the environment. By the way, you can install the environment manually. We provide the python package [ur5_gym](/ur5_gym) for this purpose.
Install with:  
```bash
pip3 install -e .
```

**Note**: *-e* flag is for editable mode, so that changes to the code are reflected immediately.

    The package was tested for the Ubuntu 20.04 LTS.
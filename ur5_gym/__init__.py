"""
Gym environment for the UR5 robot.
"""

__version__ = "0.0.1"

from ur5_gym.envs.ur5_sim import UR5,  JOINT_TYPE, Joint as UR5Joint
from ur5_gym.envs.tasks.reach import Env as ReachEnv
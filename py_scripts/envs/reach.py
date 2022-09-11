import gym
import numpy as np
from envs.ur5_sim import UR5
from collections import namedtuple

class Env(UR5, gym.Env):
    action_space = gym.spaces.Box(low=-1, high=1, shape=(3,), dtype='float32')
    obs_space = gym.spaces.Dict(dict(
            observation=gym.spaces.Box(-np.inf, np.inf, shape=(6,), dtype='float32')
        ))
    reward_range = (-np.inf, 0)

    def __init__(self, headless=False):
        UR5.__init__(self, headless)
    
    def step(self, action = None):
        self._step_id += 1
        if not action is None:
            action = self._compute_action(action)
            self._step_sim(action)
        else:
            self._step_sim(None)
        output = self._compute_output()
        return output.state, output.reward, output.is_done, output.info

    def _compute_action(self, agent_input):
        return agent_input
       
    def _compute_output(self):
        output = namedtuple('output', ["state", "reward", "is_done", "info"])
        observation = self._observation()
        output.state = self._compute_state(observation)
        output.reward = self._compute_reward(observation)
        output.is_done = self._is_done(observation)
        if output.is_done:
            self._is_success = True
        output.info = {
            "is_done": output.is_done,
            "is_success": self._is_success,
            "reward": output.reward,
            "observation": observation
        }
        return output

    def _compute_state(self, observation):
        return np.concatenate((observation.gripper_pos, observation.obj_pos))

    def _compute_reward(self, observation):
        return -np.linalg.norm(observation.gripper_pos - observation.obj_pos)

    def _is_done(self, observation):
        if np.linalg.norm(observation.gripper_pos - observation.obj_pos) < 0.05:
            self._steps_at_target += 1
        else:
            self._steps_at_target = 0
        return self._step_id == 500 or self._steps_at_target >= 50 

    def reset(self):
        self._reset_world()
        self._steps_at_target = 0
        self._step_id = 0
        self._reward = 0
        self._is_success = False
        state = self._compute_state(self._observation())
        return state

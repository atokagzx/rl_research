'''
Gym environment for the reach skill of the robot arm.
'''

import gym
import numpy as np
from ur5_gym.envs.ur5_sim import UR5
from collections import namedtuple

class Env(UR5, gym.Env):
    action_space = gym.spaces.Box(low=-1, high=1, shape=(3,), dtype='float32')
    observation_space = gym.spaces.Dict(dict(
            observation=gym.spaces.Box(-np.inf, np.inf, shape=(6,), dtype='float32')
        ))
    reward_range = (-np.inf, 0)

    def __init__(self, headless=False):
        '''
        @param headless: if True, the simulation is run in headless mode
        '''
        UR5.__init__(self, headless, gripper_state=False)
    
    def step(self, action = None):
        '''
        Perform a step in the environment.
        @param action: output of the agent
        @return: namedtuple with the following fields:
            state: current state of the environment
            reward: reward for the current step
            is_done: True if the episode is done
            info: additional information
        '''
        self._step_id += 1
        if not action is None:
            action = self._compute_action(action)
            self._step_sim(action)
        else:
            self._step_sim(None)
        output = self._compute_output()
        return output.state, output.reward, output.is_done, output.info

    def _compute_action(self, agent_input):
        '''
        Convert the agent input to the action.
        @param agent_input: agent input
        @return: action
        '''
        return agent_input
       
    def _compute_output(self):
        '''
        Compute the output of the environment for the agent.
        Info contains the following fields:
            is_success: True if the episode is successful
            observation: observation of the environment
            cumulative_reward: cumulative reward of the episode
        @return: namedtuple with the following fields:
            state: current state of the environment
            reward: reward for the current step
            is_done: True if the episode is done
            info: additional information
        '''
        output = namedtuple('output', ["state", "reward", "is_done", "info"])
        observation = self._observation()
        output.state = self._compute_state(observation)
        output.reward = self._compute_reward(observation)
        is_success = self._is_success(observation)
        output.is_done = self._is_done(observation, is_success)
        output.info = {
            "is_success": is_success,
            "done": output.is_done,
            "cumulutive_reward": self._cumulative_reward,
            "observation": observation
        }
        return output

    def _compute_state(self, observation):
        '''
        Compute the state.
        State: [gripper_pos, obj_pos]
        '''
        return {"observation": np.concatenate((observation.gripper_pos, observation.obj_pos))}

    def _compute_reward(self, observation):
        '''
        Compute the reward.
        Reward: -dst(gripper, object)
        @param observation: observation of the environment
        @return: reward for the current step
        '''
        rew = -np.linalg.norm(observation.gripper_pos - observation.obj_pos)
        self._cumulative_reward += rew
        return rew

    def _is_done(self, observation, is_success):
        '''
        Check if the episode is done.
        Conditions: more than 500 steps for current episode or success is achieved
        '''
        return self._step_id == 500 or is_success

    def _is_success(self, observation):
        '''
        Check if the episode is successful.
        Conditions: distance between gripper and object is less than 0.05 for 50 consecutive steps
        @param observation: observation of the environment
        @return: True if the episode is successful
        '''
        if np.linalg.norm(observation.gripper_pos - observation.obj_pos) < 0.05:
            self._steps_at_target += 1
        else:
            self._steps_at_target = 0
        return self._steps_at_target >= 50 

    def reset(self):
        '''
        Reset the environment.
        @return: initial state of the environment
        '''
        self._reset_world()
        self._steps_at_target = 0
        self._step_id = 0
        self._cumulative_reward = 0
        state = self._compute_state(self._observation())
        return state

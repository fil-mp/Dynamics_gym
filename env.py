import math
import numpy as np
from typing import Optional
import abc

import gym
from gym import spaces
from gym.utils import seeding
import pytest

class QuadRotorEnv(gym.Env):

    action_space = spaces.Box(0, 1, (4,), dtype=np.float32)

    def __init__(self, params: Optional[CopterParams]=None):
        
        # set to supplied copter params, or to default value
        if params is None:
            params = CopterParams()
        self.setup = params
        self.reward=0
        self._error_target = 1 * np.pi / 180
        self._in_target_reward = 0.1
        self._boundary_penalty = 1.0
        # just initialize the state to default, the rest will be done by reset
        self._state = DynamicsState()
        self.random_state = None
        self.seed()

    # env functions
    def seed(self, seed=None):
        self.random_state, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        action = np.clip(action, 0.0, 1.0)
        assert action.shape == (4,)
        
        self._state.desired_rotor_speeds = np.sqrt(action) * self.setup.max_rotor_speed
 
        simulate_quadrotor(self.setup, self._state, 0.02)
        
        
        reward, done, info = self._step_copter(action)
        return self._get_state(), reward, done, info

    def _step_copter(self, action: np.ndarray):
        #ensure_fixed_position(self._state, 1.0)
        #project_2d(self._state)
        attitude = self._state.attitude
        angle_error = attitude.pitch ** 2

        velocity_error = np.sum(self._state.angular_velocity ** 2)
        reward = self._calculate_reward(angle_error)

        return reward, False, {}

    def _calculate_reward(self, angle_error):
        reward = -angle_error

        # check whether error is below bound and add bonus reward
        if angle_error < self._error_target * self._error_target:
            reward += self._in_target_reward
        else:
            reward +=0
        return reward

    def _get_state(self):
        s = self._state
        rate = angvel_to_euler(s.attitude, s.angular_velocity)
        state = [s.attitude.roll, s.attitude.pitch, angle_difference(s.attitude.yaw, 0.0),
                 rate[0], rate[1], rate[2], s.position[0], s.position[1], s.position[2]]
        return np.array(state)

    def reset(self):
        #self._state = DynamicsState()
        self._reset_copter()

    def _reset_copter(self):
        print(self._state.attitude.roll)
        mpr = 20 * math.pi / 180

        # small pitch, roll values, random yaw angle
        
        self._state.attitude.roll=self.random_state.uniform(low=-mpr, high=mpr)
        self._state.attitude.pitch=self.random_state.uniform(low=-mpr, high=mpr)
        self._state.attitude.yaw = self.random_state.uniform(low=-math.pi, high=math.pi)
        #self.randomize_velocity(2.0)
        self.randomize_angular_velocity(2.0)
        
        '''
        self._state.pqr[0]=self.random_state.uniform(low=0, high=1)
        self._state.pqr[1]=self.random_state.uniform(low=0, high=1)
        self._state.pqr[2]=self.random_state.uniform(low=0, high=1)
        '''
        #project_2d(self._state)

        self._state.position[2] = 1
        self._correct_counter = 0

    def randomize_velocity(self, max_speed: float):
        self._state.velocity[:] = self.random_state.uniform(low=-max_speed, high=max_speed, size=(3,))

    def randomize_angular_velocity(self, max_speed: float):
        self._state.angular_velocity[:] = self.random_state.uniform(low=-max_speed, high=max_speed, size=(3,))


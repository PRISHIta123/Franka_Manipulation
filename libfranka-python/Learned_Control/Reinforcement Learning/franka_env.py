# franka_env.py

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time

class FrankaJointEnv(gym.Env):
    def __init__(self, robot, rt):
        super(FrankaJointEnv, self).__init__()
        self.robot = robot
        self.rt = rt
        self.dt = 0.05

        self.q = np.zeros(7, dtype=np.float32)
        self.dq = np.zeros(7, dtype=np.float32)
        self.q_target = np.array([0.3,-0.5,0.5,0.4,-0.3,0.2,0.5], dtype=np.float32)

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(14,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(7,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        self.q = np.zeros(7, dtype=np.float32)
        self.dq = np.zeros(7, dtype=np.float32)
        obs = np.concatenate([self.q, self.dq])
        info = {}
        return obs, info

    def step(self, action):
        # Example: simple joint position update
        self.q += action * self.dt
        self.dq = action

        # Observation is current q and dq
        obs = np.concatenate([self.q, self.dq]).astype(np.float32)

        # Reward based on distance to target
        reward = -np.linalg.norm(self.q - self.q_target)

        # Termination condition: close enough to target
        terminated = bool(np.linalg.norm(self.q - self.q_target) < 0.05)
        truncated = bool(False)  # No time truncation for now

        info = {}
        return obs, float(reward), terminated, truncated, info

    def _get_obs(self):
        state = self.rt.get_current_state()
        q = np.array(state.q)
        dq = np.array(state.dq)
        return np.concatenate([q, dq]).astype(np.float32)

    def close(self):
        try:
            self.robot.stop()
        except:
            pass

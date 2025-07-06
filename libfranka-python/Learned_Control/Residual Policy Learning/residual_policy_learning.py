import gymnasium as gym
import numpy as np
from gymnasium import spaces
from stable_baselines3 import PPO
import time
from franka_bindings import Robot

class ResidualFrankaEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.robot = Robot("127.0.0.1")
        self.robot.read_once()
        self.robot.start_realtime_control()
        time.sleep(1.0)
        self.rt = self.robot.get_realtime_control()

        self.dt = 0.04
        self.q_home = np.array(self.rt.get_current_state().q)
        self.t0 = time.time()

        # Observation: joint positions + velocities
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(14,), dtype=np.float32)
        # Action: residual joint velocity (7 DOF)
        self.action_space = spaces.Box(low=-0.2, high=0.2, shape=(7,), dtype=np.float32)

    def base_controller(self, t):
        """ Simple oscillating policy as base controller """
        q_base = self.q_home.copy()
        q_base[4] += 0.3 * np.sin(2 * np.pi * 0.2 * t)
        return q_base

    def reset(self, *, seed=None, options=None):
        self.t0 = time.time()
        state = self.rt.get_current_state()
        self.q = np.array(state.q)
        self.dq = np.array(state.dq)
        obs = np.concatenate([self.q, self.dq]).astype(np.float32)
        return obs, {}

    def step(self, action):
        t = time.time() - self.t0
        q_base = self.base_controller(t)
        dq_residual = np.array(action)
        q_target = q_base + dq_residual * self.dt

        self.rt.set_target_position(q_target.tolist())
        time.sleep(self.dt)

        state = self.rt.get_current_state()
        self.q = np.array(state.q)
        self.dq = np.array(state.dq)
        obs = np.concatenate([self.q, self.dq]).astype(np.float32)

        # Reward: track a fixed goal (or smoothness)
        q_goal = self.q_home.copy()
        q_goal[4] += 0.3
        reward = -np.linalg.norm(self.q - q_goal)

        terminated = bool(np.linalg.norm(self.q - q_goal) < 0.05)
        truncated = False

        return obs, float(reward), terminated, truncated, {}

    def close(self):
        try:
            self.robot.stop()
        except Exception as e:
            print(f"[WARNING] robot.stop() failed: {e}")

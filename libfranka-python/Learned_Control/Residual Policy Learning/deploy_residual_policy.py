import time
from stable_baselines3 import PPO
from residual_policy_learning import ResidualFrankaEnv

env = ResidualFrankaEnv()
model = PPO.load("ppo_residual_franka")

obs, _ = env.reset()
for _ in range(1000):
    action, _ = model.predict(obs)
    obs, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        obs, _ = env.reset()

env.close()

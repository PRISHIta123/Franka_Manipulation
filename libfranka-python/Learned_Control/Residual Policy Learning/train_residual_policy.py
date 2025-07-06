from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO

from residual_policy_learning import ResidualFrankaEnv

env = ResidualFrankaEnv()
check_env(env)

vec_env = DummyVecEnv([lambda: Monitor(env)])
model = PPO("MlpPolicy", vec_env, verbose=1)

model.learn(total_timesteps=10000)
model.save("ppo_residual_franka")

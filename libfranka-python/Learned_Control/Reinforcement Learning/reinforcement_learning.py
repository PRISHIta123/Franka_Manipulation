from franka_bindings import Robot
from franka_env import FrankaJointEnv
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_checker import check_env

# === Only start realtime control ONCE ===
robot = Robot("127.0.0.1")
robot.read_once()
robot.start_realtime_control()
rt = robot.get_realtime_control()

# === Pass robot and rt to the env ===
env = FrankaJointEnv(robot, rt)

check_env(env)

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)

model.save("ppo_franka_joint")
env.close()

# Reuse the same robot and rt for evaluation
env = FrankaJointEnv(robot, rt)
obs, _ = env.reset()

for _ in range(1000):
    action, _ = model.predict(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    done = terminated or truncated
    if done:
        obs, _ = env.reset()

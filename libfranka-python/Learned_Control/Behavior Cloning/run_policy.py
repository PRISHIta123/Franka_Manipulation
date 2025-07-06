import torch
import time
import numpy as np
from franka_bindings import Robot
from train_policy import TrajectoryPolicy

# Load policy
policy = TrajectoryPolicy()
policy.load_state_dict(torch.load("trajectory_policy.pth"))
policy.eval()

robot = Robot("127.0.0.1")
robot.start_realtime_control()
time.sleep(1.0)
rt = robot.get_realtime_control()

print("Executing learned trajectory policy...")
t0 = time.time()
while time.time() - t0 < 12.0:
    state = rt.get_current_state()
    q = np.array(state.q)

    with torch.no_grad():
        q_input = torch.tensor(q, dtype=torch.float32).unsqueeze(0)
        q_pred = policy(q_input).squeeze(0).numpy()

    rt.set_target_position(q_pred.tolist())
    time.sleep(0.01)

robot.stop()
print("Policy execution complete.")

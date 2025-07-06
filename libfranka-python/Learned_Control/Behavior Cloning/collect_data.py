import numpy as np
import time
import pickle
from franka_bindings import Robot

data = []

def collect_trajectory():
    robot = Robot("127.0.0.1")
    robot.start_realtime_control()
    time.sleep(1.0)
    rt = robot.get_realtime_control()

    t0 = time.time()
    duration = 12.0
    print("Collecting expert trajectory...")

    while time.time() - t0 < duration:
        t = time.time() - t0
        state = rt.get_current_state()
        q = np.array(state.q)

        # Create expert motion in multiple joints

        q_target = q.copy()
        q_target[0] += 0.4
        q_target[1] -= 0.3   # shoulder
        q_target[2] += 0.2
        q_target[3] += 0.5  # elbow
        q_target[4] -= 0.2
        q_target[5] += 0.25  # wrist
        q_target[6] -= 0.4

        rt.set_target_position(q_target.tolist())

        # Save (state, action) pairs
        data.append((q.tolist(), q_target.tolist()))

        time.sleep(0.01)

    robot.stop()
    with open("trajectory_data.pkl", "wb") as f:
        pickle.dump(data, f)
    print("Expert trajectory saved.")

if __name__ == "__main__":
    collect_trajectory()

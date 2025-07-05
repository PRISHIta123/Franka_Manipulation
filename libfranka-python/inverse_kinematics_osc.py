import numpy as np
import time
from franka_bindings import Robot
from modern_robotics import FKinSpace, IKinSpace

# Franka Panda screw axes (Slist) and M (end-effector home configuration)
Slist = np.array([
    [0, 0, 1, 0, 0, 0],
    [0, -1, 0, -0.333, 0, 0],
    [0, -1, 0, -0.316, 0, 0],
    [0, -1, 0, -0.0825, 0, 0],
    [1, 0, 0, 0, 0.384, 0],
    [0, -1, 0, -0.088, 0, 0],
    [1, 0, 0, 0, 0.107, 0]
]).T

M = np.array([
    [1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, -1, 0.107],
    [0, 0, 0, 1]
])

def move_to_target(rt, q_start, q_target, duration=4.0):
    steps = int(duration * 100)
    for i in range(steps):
        alpha = i / steps
        q_cmd = (1 - alpha) * q_start + alpha * q_target
        rt.set_target_position(q_cmd.tolist())
        time.sleep(0.01)

def run_inverse_kinematics():
    robot = Robot("127.0.0.1")
    robot.read_once()
    robot.start_realtime_control()
    time.sleep(1.0)

    rt = robot.get_realtime_control()
    state = rt.get_current_state()
    q_start = np.array(state.q)

    print("Reading current joint state...")

    T_start = FKinSpace(M, Slist, q_start)

    # Apply a safer and more reachable target offset
    delta_z = 0.005  # 5 mm upward
    xyz_target = T_start[:3, 3] + np.array([0.0, 0.0, delta_z])
    T_target = T_start.copy()
    T_target[:3, 3] = xyz_target

    print("Solving IK for target pose...")

    # Try solving IK with relaxed tolerance
    q_sol, success = IKinSpace(Slist, M, T_target, q_start, 1e-2, 0.01)

    if not success:
        print("[ERROR] IK failed to converge after 200 iterations.")
        try:
            robot.stop()
        except Exception as e:
            print(f"[WARNING] robot.stop() failed: {e}")
        return

    # Optional: prevent huge jumps
    if np.linalg.norm(q_sol - q_start) > 1.5:
        print("[ERROR] IK returned an unsafe joint configuration. Aborting.")
        try:
            robot.stop()
        except Exception as e:
            print(f"[WARNING] robot.stop() failed: {e}")
        return

    print("Starting IK Cartesian motion...")
    try:
        move_to_target(rt, q_start, q_sol)
    finally:
        print("Stopping robot...")
        try:
            robot.stop()
        except Exception as e:
            print(f"[WARNING] robot.stop() failed: {e}")

    print("Motion complete.")

if __name__ == "__main__":
    run_inverse_kinematics()

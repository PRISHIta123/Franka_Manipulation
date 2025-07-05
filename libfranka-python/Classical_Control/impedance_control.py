from franka_bindings import Robot, Torques
import time
import numpy as np
import threading

def impedance_control_loop(ctrl, run_flag):
    K = np.array([300, 300, 300, 10, 10, 10, 5])
    D = np.array([20, 20, 20, 2, 2, 2, 1])

    # Desired target pose
    q_d = np.zeros(7)
    # q_d = np.array([0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.5])

    while run_flag[0]:
        state, success = ctrl.readOnce()
        if not success:
            continue
        q = np.array(state.q)
        dq = np.array(state.dq)

        tau = -K * (q - q_d) - D * dq
        ctrl.writeOnce(Torques(tau.tolist()))
        time.sleep(0.001)

def run_impedance_control():
    robot = Robot("127.0.0.1")

    # ✅ DO NOT call robot.read_once() or start_realtime_control() before this
    print("[INFO] Starting torque control...")
    ctrl = robot.start_torque_control()

    run_flag = [True]
    control_thread = threading.Thread(target=impedance_control_loop, args=(ctrl, run_flag))
    control_thread.start()

    try:
        time.sleep(10)
    finally:
        run_flag[0] = False
        control_thread.join(timeout=2)
        robot.stop()
        print("Impedance control complete.")

if __name__ == "__main__":
    run_impedance_control()

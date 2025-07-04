import numpy as np
import time
from franka_bindings import Robot, ControllerMode, JointPositions

#Move all 7 joints in the franka robot arm using pd control
#PD controller u = Kp(q-q_des)+ Kd(q*)
#u is the torquw
#Kp is the proportional gain
#Kd is the derivative gain
#q is current joint position, q_des is desired joint position, q* is current joint velocity

def main():
    try:
        # Connect to the robot (simulated)
        robot = Robot("127.0.0.1")
        time.sleep(1)  # Let connection stabilize

        # Set collision behavior
        lower_torque = [20.0] * 7
        upper_torque = [40.0] * 7
        lower_force = [10.0] * 6
        upper_force = [20.0] * 6

        robot.set_collision_behavior(lower_torque, upper_torque, lower_force, upper_force)

        # Start joint position control
        print("[INFO] Starting PD control (position-mode emulation)...")
        control = robot.start_joint_position_control(ControllerMode.JointImpedance)

        # Read initial state
        state, _ = control.readOnce()
        q = np.array(state.q)
        print("[INFO] Initial joint positions:", q)
        dq = np.array(state.dq)
        print("[INFO] Initial joint velocity:", dq)

        # Desired position: move joints a bit
        q_desired = q.copy()
        q_desired[0] -= 0.5
        q_desired[1] += 0.3  # lift a bit
        q_desired[2] -= 0.2
        q_desired[3] += 0.5
        q_desired[4] -= 0.4  # elbow bend
        q_desired[5] += 0.1
        q_desired[6] -= 0.3  # wrist bend

        # PD gains
        Kp = np.array([10.0,5.0,5.0,20.0,30.0,5.0,10.0])
        Kd = np.array([2.0,1.0,1.0,2.0,2.0,1.0,2.0])

        print("[INFO] Starting PD control (with velocity damping)...")
        print(f"[INFO] Target joint positions: {q_desired.tolist()}")

        start_time = time.time()
        duration = 5.0  # seconds

        while time.time() - start_time < duration:
            state, dt = control.readOnce()
            q_curr = np.array(state.q)
            dq_curr = np.array(state.dq)

            # Compute PD control (emulated by generating new position)
            error = q_desired - q_curr

            tau = Kp * error + Kd * dq_curr

            next_q = q_curr + tau
            command = JointPositions(next_q.tolist())
            control.writeOnce(command)

            time.sleep(0.005)  # simulate real-time loop

        # Final stop command
        final_cmd = JointPositions(q_desired.tolist())
        final_cmd.motion_finished = True
        control.writeOnce(final_cmd)

        print("[INFO] PD control finished.")

    except Exception as e:
        print(f"[ERROR] {e}")
        robot.stop()

if __name__ == "__main__":
    main()

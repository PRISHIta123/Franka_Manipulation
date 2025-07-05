import numpy as np
from franka_bindings import Robot, ControllerMode, JointPositions


def main():
    try:
        # Connect to robot (use "127.0.0.1" for simulation or the robot's IP for real hardware)
        robot = Robot("127.0.0.1")

        # Set collision behavior
        lower_torque_thresholds = [20.0] * 7  # Nm
        upper_torque_thresholds = [40.0] * 7  # Nm
        lower_force_thresholds = [10.0] * 6  # N (linear) and Nm (angular)
        upper_force_thresholds = [20.0] * 6  # N (linear) and Nm (angular)

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds
        )

        # Start joint position control
        control = robot.start_joint_position_control(ControllerMode.JointImpedance)

        # Get initial position
        state, duration = control.readOnce()
        initial_position = list(state.q)

        # Move the robot with a sinusoidal motion
        amplitude = np.pi / 8.0
        frequency = 0.4  # Hz
        run_time = 5.0  # seconds
        elapsed_time = 0.0

        while elapsed_time < run_time:
            state, duration = control.readOnce()
            elapsed_time += duration.to_sec()

            # Calculate desired position
            desired_position = initial_position.copy()
            delta_angle = amplitude * (1.0 - np.cos(2.0 * np.pi * frequency * elapsed_time))
            desired_position[3] += delta_angle  # Move joint 4

            # Send command
            joint_positions = JointPositions(desired_position)
            if elapsed_time >= run_time - 0.1:
                joint_positions.motion_finished = True

            control.writeOnce(joint_positions)

    except Exception as e:
        print(f"Error: {e}")
        robot.stop()


if __name__ == "__main__":
    main()
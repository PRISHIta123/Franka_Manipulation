#!/usr/bin/env python3
import numpy as np
from franka_bindings import Robot
import time
import argparse


def verify_joint_limits(positions):
    """Verify joint positions are within safe limits"""
    # Franka Research 3 joint limits in radians
    JOINT_LIMITS = [
        [-2.7437, 2.7437],  # Joint 1
        [-1.7837, 1.7837],  # Joint 2
        [-2.9007, 2.9007],  # Joint 3
        [-3.0421, -0.1518], # Joint 4
        [-2.8065, 2.8065],  # Joint 5
        [0.5445, 4.5169],   # Joint 6
        [-3.0159, 3.0159],  # Joint 7
    ]

    for i, (pos, (lower, upper)) in enumerate(zip(positions, JOINT_LIMITS)):
        if not (lower <= pos <= upper):
            raise ValueError(
                f"Joint {i+1} position {pos:.4f} exceeds limits [{lower:.4f}, {upper:.4f}]"
            )


def wait_for_motion_completion(rt_control, target_pos, timeout=30.0, threshold=0.01):
    """Wait for motion to complete with timeout"""
    start_time = time.time()
    while True:
        current_pos = rt_control.get_current_position()
        error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))

        # print(f"Current position: {[f'{p:.4f}' for p in current_pos]}")
        # print(f"Position error: {error:.4f}")

        if error < threshold:
            return True

        if time.time() - start_time > timeout:
            print("Motion timed out!")
            return False

        time.sleep(0.1)


def main():
    try:
        args = argparse.ArgumentParser()
        args.add_argument("--ip", type=str, default="localhost")
        args = args.parse_args()
        # if no args, use localhost
        if not args.ip:
            args.ip = "127.0.0.1"
        robot = Robot(args.ip)  # Replace with your robot's IP
        # Set collision behavior
        lower_torque_thresholds = [1000.0] * 7  # Nm
        upper_torque_thresholds = [1000.0] * 7  # Nm
        lower_force_thresholds = [1000.0] * 6  # N (linear) and Nm (angular)
        upper_force_thresholds = [1000.0] * 6  # N (linear) and Nm (angular)

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )
    
        robot.start_realtime_control()
        rt_control = robot.get_realtime_control()
        print("Successfully connected to robot and started realtime control")

        # Give the control thread time to initialize
        time.sleep(1.0)

        # Get initial state
        initial_pos = np.array(rt_control.get_current_position())
        print(f"Initial joint positions: {[f'{p:.4f}' for p in initial_pos]}")

        # Define a sequence of interesting joint configurations
        poses = [
            # Home position - slightly raised
            [0.0, -0.2, 0.0, -1.5, 0.0, 1.5, 0.0],
            # Left side reach
            [np.pi / 3, -0.5, 0.3, -2.0, 0.2, 2.0, np.pi / 6],
            # Right side reach
            [-np.pi / 3, -0.3, -0.3, -1.8, -0.2, 1.8, -np.pi / 6],
            # Forward extended position
            [0.0, -0.1, 0.0, -2.5, 0.0, 2.8, 0.0],
            # Compact folded position
            [0.0, -1.5, 0.0, -1.2, 0.0, 1.0, 0.0],
            # Diagonal reach
            [np.pi / 4, -0.4, np.pi / 4, -1.7, np.pi / 4, 2.2, np.pi / 4],
            # Complex twist
            [np.pi / 6, -0.8, np.pi / 3, -1.5, -np.pi / 4, 1.8, np.pi / 2],
        ]

        # Move through each pose and wait for completion
        for i, target_pose in enumerate(poses):
            verify_joint_limits(target_pose)
            # print(f"\nMoving to pose {i+1}: {[f'{p:.4f}' for p in target_pose]}")
            rt_control.set_target_position(target_pose)

            # Wait for motion completion with timeout
            if not wait_for_motion_completion(rt_control, target_pose):
                print(f"Motion to pose {i+1} did not complete in time!")
                return

            print(f"Successfully reached pose {i+1}")
            time.sleep(0.5)  # Short pause at each position

        print("\nMotion sequence completed!")

        # Return to a neutral position
        neutral_pose = [0.0, -0.3, 0.0, -1.5, 0.0, 1.8, 0.0]
        verify_joint_limits(neutral_pose)
        print("\nReturning to neutral position...")
        rt_control.set_target_position(neutral_pose)

        if not wait_for_motion_completion(rt_control, neutral_pose):
            print("Return to neutral position did not complete in time!")
            return

        print("Successfully returned to neutral position!")

    except ValueError as ve:
        print(f"Joint limit error: {ve}")
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        robot.stop()
        print("Stopped robot control")


if __name__ == "__main__":
    main()

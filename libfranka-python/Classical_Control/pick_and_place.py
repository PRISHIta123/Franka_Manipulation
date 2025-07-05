import time
from franka_bindings import Robot, ControllerMode, JointPositions

def move_joint(robot, target_position):
    print(f"Moving to position: {target_position}")
    control = robot.start_joint_position_control(ControllerMode.JointImpedance)

    while True:
        state, duration = control.readOnce()
        desired_position = target_position.copy()

        joint_command = JointPositions(desired_position)

        # Check if we're close to desired
        if all(abs(q_curr - q_des) < 1e-2 for q_curr, q_des in zip(state.q, desired_position)):
            joint_command.motion_finished = True
            print("Final joint command sent with motion_finished = True")
            control.writeOnce(joint_command)
            break

        control.writeOnce(joint_command)
        time.sleep(0.01)

    time.sleep(1.0)

def main():
    try:
        robot = Robot("127.0.0.1")

        # Base position
        initial_position = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785]

        # Move both lower joint (joint 0) and mid joint (joint 4)
        pick_position = [-0.5, 0.0, 0.0, -1.57, -0.5, 1.57, 0.785]
        place_position = [0.5, 0.0, 0.0, -1.57, 0.5, 1.57, 0.785]

        print("Moving to initial position...")
        move_joint(robot, initial_position)

        print("Moving to pick position...")
        move_joint(robot, pick_position)

        print("Moving to place position...")
        move_joint(robot, place_position)

        print("Pick and place complete.")

    except Exception as e:
        print(f"Error: {e}")
        robot.stop()

if __name__ == "__main__":
    main()

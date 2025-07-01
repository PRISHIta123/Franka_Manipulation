#pragma once

#include <array>

namespace franka_bindings {

struct RobotState {
    std::array<double, 7> q;    // joint positions
    std::array<double, 7> dq;   // joint velocities
    std::array<double, 16> O_T_EE;  // end-effector pose as 4x4 transformation matrix (column-major)
};

}  // namespace franka_bindings
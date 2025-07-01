#pragma once

#include <memory>
#include <optional>
#include <array>

#include <Eigen/Core>
#include <franka/robot.h>
#include <franka/active_control_base.h>
#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/active_torque_control.h>
#include "realtime_control/realtime_control.hpp"
#include "realtime_control/shared_memory.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

namespace franka_bindings {

class PyRobot {
public:
    explicit PyRobot(const std::string& franka_address);
    ~PyRobot() = default;

    // Active control methods
    std::unique_ptr<franka::ActiveControlBase> startTorqueControl();
    std::unique_ptr<franka::ActiveControlBase> startJointPositionControl(const franka::ControllerMode&);
    std::unique_ptr<franka::ActiveControlBase> startJointVelocityControl(const franka::ControllerMode&);

    // Realtime control methods
    void startRealtimeControl();
    RealtimeControl* getRealtimeControl();

    // Configuration methods
    void setCollisionBehavior(const std::array<double, 7>& lower_torque_thresholds,
                             const std::array<double, 7>& upper_torque_thresholds,
                             const std::array<double, 6>& lower_force_thresholds,
                             const std::array<double, 6>& upper_force_thresholds);

    void setJointImpedance(const std::array<double, 7>& K_theta);
    void setCartesianImpedance(const std::array<double, 6>& K_x);

    // State methods
    franka::RobotState readOnce();
    void stop();

private:
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<RealtimeControl> realtime_control_;
};

} // namespace franka_bindings 
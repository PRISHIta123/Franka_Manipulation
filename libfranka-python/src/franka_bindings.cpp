#include "franka_bindings.h"
#include <franka/exception.h>
#include <franka/duration.h>
#include "realtime_control/realtime_control.hpp"
#include "realtime_control/shared_memory.hpp"
#include "realtime_control/robot_state.hpp"
#include <research_interface/robot/service_types.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

namespace franka_bindings {

PyRobot::PyRobot(const std::string& franka_address) 
    : robot_(std::make_unique<franka::Robot>(franka_address, franka::RealtimeConfig::kIgnore)) {}

std::unique_ptr<franka::ActiveControlBase> PyRobot::startTorqueControl() {
    return robot_->startTorqueControl();
}

std::unique_ptr<franka::ActiveControlBase> PyRobot::startJointPositionControl(
    const franka::ControllerMode& control_type) {
    research_interface::robot::Move::ControllerMode mode;
    if (control_type == franka::ControllerMode::kJointImpedance) {
        mode = research_interface::robot::Move::ControllerMode::kJointImpedance;
    } else {
        mode = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
    }
    return robot_->startJointPositionControl(mode);
}

std::unique_ptr<franka::ActiveControlBase> PyRobot::startJointVelocityControl(
    const franka::ControllerMode& control_type) {
    research_interface::robot::Move::ControllerMode mode;
    if (control_type == franka::ControllerMode::kJointImpedance) {
        mode = research_interface::robot::Move::ControllerMode::kJointImpedance;
    } else {
        mode = research_interface::robot::Move::ControllerMode::kCartesianImpedance;
    }
    return robot_->startJointVelocityControl(mode);
}

void PyRobot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds,
    const std::array<double, 7>& upper_torque_thresholds,
    const std::array<double, 6>& lower_force_thresholds,
    const std::array<double, 6>& upper_force_thresholds) {
    robot_->setCollisionBehavior(
        lower_torque_thresholds, upper_torque_thresholds,
        lower_torque_thresholds, upper_torque_thresholds,
        lower_force_thresholds, upper_force_thresholds,
        lower_force_thresholds, upper_force_thresholds);
}

void PyRobot::setJointImpedance(const std::array<double, 7>& K_theta) {
    robot_->setJointImpedance(K_theta);
}

void PyRobot::setCartesianImpedance(const std::array<double, 6>& K_x) {
    robot_->setCartesianImpedance(K_x);
}

franka::RobotState PyRobot::readOnce() {
    return robot_->readOnce();
}

void PyRobot::startRealtimeControl() {
    realtime_control_ = std::make_unique<RealtimeControl>(*robot_);
}

void PyRobot::stop() {
    robot_->stop();
}

RealtimeControl* PyRobot::getRealtimeControl() {
    return realtime_control_.get();
}

PYBIND11_MODULE(franka_bindings, m) {
    // Bind exceptions
    py::register_exception<franka::Exception>(m, "FrankaException");
    py::register_exception<franka::CommandException>(m, "CommandException", PyExc_RuntimeError);
    py::register_exception<franka::NetworkException>(m, "NetworkException", PyExc_RuntimeError);
    py::register_exception<franka::ControlException>(m, "ControlException", PyExc_RuntimeError);
    py::register_exception<franka::InvalidOperationException>(m, "InvalidOperationException", PyExc_RuntimeError);
    py::register_exception<franka::RealtimeException>(m, "RealtimeException", PyExc_RuntimeError);

    // Bind Duration
    py::class_<franka::Duration>(m, "Duration")
        .def("to_sec", &franka::Duration::toSec);

    // Bind enums
    py::enum_<franka::ControllerMode>(m, "ControllerMode")
        .value("JointImpedance", franka::ControllerMode::kJointImpedance)
        .value("CartesianImpedance", franka::ControllerMode::kCartesianImpedance);

    // First bind our custom RobotState (before RealtimeControl)
    py::class_<RobotState>(m, "CustomRobotState")
        .def(py::init<>())
        .def_readwrite("q", &RobotState::q)
        .def_readwrite("dq", &RobotState::dq)
        .def_readwrite("O_T_EE", &RobotState::O_T_EE);

    // Bind franka::RobotState
    py::class_<franka::RobotState>(m, "RobotState")
        .def_readwrite("q", &franka::RobotState::q)
        .def_readwrite("q_d", &franka::RobotState::q_d)
        .def_readwrite("dq", &franka::RobotState::dq)
        .def_readwrite("dq_d", &franka::RobotState::dq_d)
        .def_readwrite("tau_J", &franka::RobotState::tau_J)
        .def_readwrite("tau_J_d", &franka::RobotState::tau_J_d)
        .def_readwrite("O_T_EE", &franka::RobotState::O_T_EE)
        .def_readwrite("O_T_EE_d", &franka::RobotState::O_T_EE_d)
        .def_readwrite("F_T_EE", &franka::RobotState::F_T_EE)
        .def_readwrite("EE_T_K", &franka::RobotState::EE_T_K);

    // Bind ActiveControlBase
    py::class_<franka::ActiveControlBase>(m, "ActiveControlBase")
        .def("readOnce", [](franka::ActiveControlBase& self) {
            auto result = self.readOnce();
            return py::make_tuple(result.first, result.second);
        })
        .def("writeOnce", py::overload_cast<const franka::Torques&>(&franka::ActiveControlBase::writeOnce))
        .def("writeOnce", py::overload_cast<const franka::JointPositions&>(&franka::ActiveControlBase::writeOnce))
        .def("writeOnce", py::overload_cast<const franka::JointVelocities&>(&franka::ActiveControlBase::writeOnce))
        .def("writeOnce", py::overload_cast<const franka::CartesianPose&>(&franka::ActiveControlBase::writeOnce))
        .def("writeOnce", py::overload_cast<const franka::CartesianVelocities&>(&franka::ActiveControlBase::writeOnce));

    // Bind control types
    py::class_<franka::Torques>(m, "Torques")
        .def(py::init<const std::array<double, 7>&>())
        .def_readwrite("tau_J", &franka::Torques::tau_J)
        .def_readwrite("motion_finished", &franka::Torques::motion_finished);

    py::class_<franka::JointPositions>(m, "JointPositions")
        .def(py::init<const std::array<double, 7>&>())
        .def_readwrite("q", &franka::JointPositions::q)
        .def_readwrite("motion_finished", &franka::JointPositions::motion_finished);

    py::class_<franka::JointVelocities>(m, "JointVelocities")
        .def(py::init<const std::array<double, 7>&>())
        .def_readwrite("dq", &franka::JointVelocities::dq)
        .def_readwrite("motion_finished", &franka::JointVelocities::motion_finished);

    py::class_<franka::CartesianPose>(m, "CartesianPose")
        .def(py::init<const std::array<double, 16>&>())
        .def_readwrite("O_T_EE", &franka::CartesianPose::O_T_EE)
        .def_readwrite("motion_finished", &franka::CartesianPose::motion_finished);

    py::class_<franka::CartesianVelocities>(m, "CartesianVelocities")
        .def(py::init<const std::array<double, 6>&>())
        .def_readwrite("O_dP_EE", &franka::CartesianVelocities::O_dP_EE)
        .def_readwrite("motion_finished", &franka::CartesianVelocities::motion_finished);

    // Bind RealtimeControl
    py::class_<RealtimeControl>(m, "RealtimeControl")
        .def("get_current_position", &RealtimeControl::getCurrentPosition)
        .def("set_target_position", &RealtimeControl::setTargetPosition)
        .def("enable_rate_limiter", &RealtimeControl::enableRateLimiter)
        .def("get_current_state", &RealtimeControl::get_current_state,
             py::return_value_policy::copy);  // Ensure we return a copy

    // Bind PyRobot
    py::class_<PyRobot>(m, "Robot")
        .def(py::init<const std::string&>())
        .def("start_torque_control", &PyRobot::startTorqueControl)
        .def("start_joint_position_control", &PyRobot::startJointPositionControl)
        .def("start_joint_velocity_control", &PyRobot::startJointVelocityControl)
        .def("set_collision_behavior", &PyRobot::setCollisionBehavior)
        .def("set_joint_impedance", &PyRobot::setJointImpedance)
        .def("set_cartesian_impedance", &PyRobot::setCartesianImpedance)
        .def("read_once", &PyRobot::readOnce)
        .def("stop", &PyRobot::stop)
        .def("start_realtime_control", [](PyRobot& self) {
            self.startRealtimeControl();
        })
        .def("get_realtime_control", &PyRobot::getRealtimeControl, py::return_value_policy::reference);

    // Add docstring
    m.doc() = "Python bindings for libfranka realtime control";
}

} // namespace franka_bindings 
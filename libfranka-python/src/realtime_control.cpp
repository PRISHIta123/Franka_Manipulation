#include "realtime_control/realtime_control.hpp"
#include "realtime_control/motion_generator.hpp"
#include "realtime_control/shared_memory.hpp"
#include <atomic>
#include <chrono>
#include <franka/active_control_base.h>
#include <franka/duration.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <iostream>
#include <mutex>
#include <research_interface/robot/service_types.h>
#include <thread>

namespace franka_bindings {

std::array<double, 7> RealtimeControl::applyRateLimiter(
    const std::array<double, 7> &upper_velocity_limits,
    const std::array<double, 7> &lower_velocity_limits,
    const std::array<double, 7> &max_acceleration,
    const std::array<double, 7> &max_jerk,
    const std::array<double, 7> &commanded_positions,
    const std::array<double, 7> &last_positions,
    const std::array<double, 7> &last_velocities,
    const std::array<double, 7> &last_accelerations, double delta_t) {

  std::array<double, 7> limited_positions{};

  for (size_t i = 0; i < 7; i++) {
    if (!std::isfinite(commanded_positions[i])) {
      throw std::invalid_argument("commanded_position is infinite or NaN.");
    }

    // Calculate commanded velocity
    double commanded_velocity =
        (commanded_positions[i] - last_positions[i]) / delta_t;

    // Calculate commanded jerk
    double commanded_jerk =
        (((commanded_velocity - last_velocities[i]) / delta_t) -
         last_accelerations[i]) /
        delta_t;

    // Limit jerk and integrate to get acceleration
    double commanded_acceleration =
        last_accelerations[i] +
        std::max(std::min(commanded_jerk, max_jerk[i]), -max_jerk[i]) * delta_t;

    // Compute acceleration limits
    double safe_max_acceleration =
        std::min((max_jerk[i] / max_acceleration[i]) *
                     (upper_velocity_limits[i] - last_velocities[i]),
                 max_acceleration[i]);
    double safe_min_acceleration =
        std::max((max_jerk[i] / max_acceleration[i]) *
                     (lower_velocity_limits[i] - last_velocities[i]),
                 -max_acceleration[i]);

    // Limit acceleration and integrate to get velocity
    double limited_velocity =
        last_velocities[i] +
        std::max(std::min(commanded_acceleration, safe_max_acceleration),
                 safe_min_acceleration) *
            delta_t;

    // Integrate velocity to get position
    limited_positions[i] = last_positions[i] + limited_velocity * 0.001;
  }

  return limited_positions;
}

RealtimeControl::RealtimeControl(franka::Robot &robot) : robot_(robot) {
  shm_ = std::make_shared<SharedMemory>();

  auto state = robot_.readOnce();
  shm_->current_joint_position = state.q;
  shm_->current_joint_velocity = state.dq;
  shm_->current_ee_pose = state.O_T_EE;
  std::array<double, 7> target_pose = {0.0, -0.2, 0.0, -1.5, 0.0, 1.5, 0.0};
  shm_->target_joint_position = target_pose;

  control_thread_ = std::thread([this]() {
    try {
      // Start joint position control with joint impedance mode
      auto control = robot_.startJointPositionControl(
          research_interface::robot::Move::ControllerMode::kJointImpedance);

      // Create initial motion generator with current position as target
      auto initial_state = control->readOnce().first;
      motion_generator_ =
          std::make_unique<MotionGenerator>(0.1, initial_state.q);

      while (!stop_thread_) {
        // Read current state and duration
        auto [state, duration] = control->readOnce();

        // Update shared state
        {
          std::lock_guard<std::mutex> lock(shm_mutex_);
          shm_->current_joint_position = state.q;
          shm_->current_joint_velocity = state.dq;
          shm_->current_ee_pose = state.O_T_EE;
          shm_->state_sequence++;
        }

        if (shm_->emergency_stop) {
          control->writeOnce(franka::JointPositions(state.q));
          continue;
        }

        // Get target position and check if it changed
        std::array<double, 7> target_position;
        {
          std::lock_guard<std::mutex> lock(shm_mutex_);
          target_position = shm_->target_joint_position;
        }

        // Create new motion generator if target changed
        if (!motion_generator_ || target_position != last_target_) {
          motion_generator_ =
              std::make_unique<MotionGenerator>(0.5, target_position);
          last_target_ = target_position;
        }

        // Generate next position using motion generator
        if (motion_generator_) {
          franka::JointPositions output = (*motion_generator_)(state, duration);
          output.motion_finished = false;
          rate_limiter_enabled_ = true;
          if (rate_limiter_enabled_) {
            for (size_t i = 0; i < 7; i++) {
              output.q[i] = franka::lowpassFilter(duration.toSec(), output.q[i],
                                                  state.q_d[i], 100);
            }

            std::array<double, 7> upper_velocity_limits =
                franka::computeUpperLimitsJointVelocity(state.q_d);
            std::array<double, 7> lower_velocity_limits =
                franka::computeLowerLimitsJointVelocity(state.q_d);

            std::array<double, 7> limited_positions = applyRateLimiter(
                upper_velocity_limits, lower_velocity_limits,
                franka::kMaxJointAcceleration, franka::kMaxJointJerk, output.q,
                state.q_d, state.dq_d, state.ddq_d,
                0.1 // TODO: change to 0.001 for real robot control
            );

            last_commanded_position_ = limited_positions;
            control->writeOnce(franka::JointPositions(limited_positions));
          } else {
            // Bypass rate limiting
            control->writeOnce(output);
          }
        }
      }
    } catch (const std::exception &e) {
      std::cerr << "[RealtimeControl] Control thread exception: " << e.what()
                << std::endl;
      shm_->emergency_stop = true;
    }
  });
}

void RealtimeControl::setTargetPosition(const std::array<double, 7> &target) {
  std::lock_guard<std::mutex> lock(shm_mutex_);
  shm_->target_joint_position = target;
  shm_->command_sequence++;
}

std::array<double, 7> RealtimeControl::getCurrentPosition() {
  std::lock_guard<std::mutex> lock(shm_mutex_);
  return shm_->current_joint_position;
}

RealtimeControl::~RealtimeControl() {
  stop_thread_ = true;
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

void RealtimeControl::enableRateLimiter(bool enable) {
  std::lock_guard<std::mutex> lock(shm_mutex_);
  rate_limiter_enabled_ = enable;
}

RobotState RealtimeControl::get_current_state() {
  std::lock_guard<std::mutex> lock(shm_mutex_);
  RobotState state;
  state.q = shm_->current_joint_position;
  state.dq = shm_->current_joint_velocity;
  state.O_T_EE = shm_->current_ee_pose;
  return state;
}

} // namespace franka_bindings
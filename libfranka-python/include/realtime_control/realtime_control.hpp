#pragma once

#include "realtime_control/robot_state.hpp"
#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

// Forward declarations from franka namespace
namespace franka {
class Robot;
}

// Forward declaration for MotionGenerator
class MotionGenerator;

namespace franka_bindings {

struct SharedMemory;

class RealtimeControl {
public:
  explicit RealtimeControl(franka::Robot &robot);
  ~RealtimeControl();

  // Delete copy constructor and assignment operator
  RealtimeControl(const RealtimeControl &) = delete;
  RealtimeControl &operator=(const RealtimeControl &) = delete;
  // Delete move constructor and assignment operator
  RealtimeControl(RealtimeControl &&) = delete;
  RealtimeControl &operator=(RealtimeControl &&) = delete;

  void setTargetPosition(const std::array<double, 7> &target);
  std::array<double, 7> getCurrentPosition();
  void enableRateLimiter(bool enable);
  RobotState get_current_state();

private:
  franka::Robot &robot_;
  std::shared_ptr<SharedMemory> shm_;
  std::thread control_thread_;
  std::array<double, 7> last_commanded_;
  std::array<double, 7> last_velocity_;
  std::array<double, 7> last_acceleration_;
  std::array<double, 7> last_commanded_position_;

  std::mutex shm_mutex_;
  std::unique_ptr<MotionGenerator> motion_generator_;
  std::array<double, 7> last_target_;
  std::atomic<bool> stop_thread_{false};
  bool rate_limiter_enabled_{true};

  std::array<double, 7>
  applyRateLimiter(const std::array<double, 7> &upper_velocity_limits,
                   const std::array<double, 7> &lower_velocity_limits,
                   const std::array<double, 7> &max_acceleration,
                   const std::array<double, 7> &max_jerk,
                   const std::array<double, 7> &commanded_positions,
                   const std::array<double, 7> &last_positions,
                   const std::array<double, 7> &last_velocities,
                   const std::array<double, 7> &last_accelerations,
                   double delta_t);
};

} // namespace franka_bindings
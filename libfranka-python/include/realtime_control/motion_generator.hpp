#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <franka/control_types.h>
#include <franka/robot_state.h>

class MotionGenerator {
public:
  MotionGenerator(double speed_factor, const std::array<double, 7> &q_goal)
      : q_goal_(q_goal), speed_factor_(speed_factor), time_(0.0),
        initialized_(false) {

    // Constants from Python implementation
    for (size_t i = 0; i < 7; i++) {
      dq_max_[i] = (i < 4 ? 2.0 : 2.5) * speed_factor;
      ddq_max_start_[i] = 5.0 * speed_factor;
      ddq_max_goal_[i] = 5.0 * speed_factor;
    }

    delta_q_motion_finished_ = 1e-6;
  }

  franka::JointPositions operator()(const franka::RobotState &robot_state,
                                    const franka::Duration &duration) {
    time_ += duration.toSec();

    if (!initialized_) {
      q_start_ = robot_state.q;
      for (size_t i = 0; i < 7; i++) {
        delta_q_[i] = q_goal_[i] - q_start_[i];
      }
      calculateSynchronizedValues();
      initialized_ = true;
    }

    std::array<double, 7> delta_q_d;
    bool motion_finished = calculateDesiredValues(time_, delta_q_d);

    std::array<double, 7> joint_positions;
    for (size_t i = 0; i < 7; i++) {
      joint_positions[i] = q_start_[i] + delta_q_d[i];
    }

    franka::JointPositions output(joint_positions);
    output.motion_finished = motion_finished;
    return output;
  }

private:
  std::array<double, 7> q_goal_;
  double speed_factor_;
  double time_;
  bool initialized_;

  std::array<double, 7> q_start_{};
  std::array<double, 7> delta_q_{};
  std::array<double, 7> dq_max_{};
  std::array<double, 7> ddq_max_start_{};
  std::array<double, 7> ddq_max_goal_{};

  std::array<double, 7> dq_max_sync_{};
  std::array<double, 7> t_1_sync_{};
  std::array<double, 7> t_2_sync_{};
  std::array<double, 7> t_f_sync_{};
  std::array<double, 7> q_1_{};

  double delta_q_motion_finished_;

  void calculateSynchronizedValues() {
    std::array<double, 7> dq_max_reach = dq_max_;
    std::array<double, 7> t_f{};
    std::array<double, 7> delta_t_2{};
    std::array<double, 7> t_1{};
    std::array<double, 7> delta_t_2_sync{};
    std::array<int, 7> sign_delta_q;

    for (size_t i = 0; i < 7; i++) {
      sign_delta_q[i] = (delta_q_[i] > 0) ? 1 : ((delta_q_[i] < 0) ? -1 : 0);

      if (std::abs(delta_q_[i]) > delta_q_motion_finished_) {
        // Calculate maximum reachable velocity
        if (std::abs(delta_q_[i]) <
            (3.0 / 4.0 * (dq_max_[i] * dq_max_[i] / ddq_max_start_[i]) +
             3.0 / 4.0 * (dq_max_[i] * dq_max_[i] / ddq_max_goal_[i]))) {
          dq_max_reach[i] = std::sqrt(4.0 / 3.0 * std::abs(delta_q_[i]) *
                                      (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                      (ddq_max_start_[i] + ddq_max_goal_[i]));
        }

        t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
        delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
        t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 +
                 std::abs(delta_q_[i]) / dq_max_reach[i];
      }
    }

    double max_t_f = 0.0;
    for (size_t i = 0; i < 7; i++) {
      if (t_f[i] > max_t_f) {
        max_t_f = t_f[i];
      }
    }

    for (size_t i = 0; i < 7; i++) {
      if (std::abs(delta_q_[i]) > delta_q_motion_finished_) {
        double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
        double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
        double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];

        double delta = b * b - 4.0 * a * c;
        if (delta < 0.0) {
          delta = 0.0;
        }

        dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
        delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
        t_f_sync_[i] = t_1_sync_[i] / 2.0 + delta_t_2_sync[i] / 2.0 +
                       std::abs(delta_q_[i] / dq_max_sync_[i]);
        t_2_sync_[i] = t_f_sync_[i] - delta_t_2_sync[i];
        q_1_[i] = dq_max_sync_[i] * sign_delta_q[i] * (0.5 * t_1_sync_[i]);
      }
    }
  }

  bool calculateDesiredValues(double t, std::array<double, 7> &delta_q_d) {
    std::array<int, 7> sign_delta_q;
    std::array<double, 7> t_d;
    std::array<double, 7> delta_t_2_sync;
    std::array<bool, 7> joint_motion_finished;

    for (size_t i = 0; i < 7; i++) {
      sign_delta_q[i] = (delta_q_[i] > 0) ? 1 : ((delta_q_[i] < 0) ? -1 : 0);
      t_d[i] = t_2_sync_[i] - t_1_sync_[i];
      delta_t_2_sync[i] = t_f_sync_[i] - t_2_sync_[i];
      joint_motion_finished[i] = false;

      if (std::abs(delta_q_[i]) < delta_q_motion_finished_) {
        delta_q_d[i] = 0;
        joint_motion_finished[i] = true;
      } else {
        if (t < t_1_sync_[i]) {
          delta_q_d[i] = -1.0 / (std::pow(t_1_sync_[i], 3)) * dq_max_sync_[i] *
                         sign_delta_q[i] * (0.5 * t - t_1_sync_[i]) *
                         std::pow(t, 3);
        } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
          delta_q_d[i] =
              q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
        } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
          delta_q_d[i] =
              delta_q_[i] +
              0.5 *
                  (1.0 / (std::pow(delta_t_2_sync[i], 3)) *
                       (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                       std::pow((t - t_1_sync_[i] - t_d[i]), 3) +
                   (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] -
                    2.0 * t_d[i])) *
                  dq_max_sync_[i] * sign_delta_q[i];
        } else {
          delta_q_d[i] = delta_q_[i];
          joint_motion_finished[i] = true;
        }
      }
    }

    bool all_finished = true;
    for (size_t i = 0; i < 7; i++) {
      if (!joint_motion_finished[i]) {
        all_finished = false;
        break;
      }
    }

    return all_finished;
  }
};
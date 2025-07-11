// Copyright (c) 2024 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <research_interface/robot/service_types.h>

namespace research_interface {
namespace robot {

template <typename T>
struct CommandTraits {};

template <>
struct CommandTraits<GetRobotModel> {
  static constexpr const char *kName = "Get Robot Model";
};

template <>
struct CommandTraits<Move> {
  static constexpr const char *kName = "Move";
};

template <>
struct CommandTraits<StopMove> {
  static constexpr const char *kName = "Stop Move";
};

template <>
struct CommandTraits<SetCollisionBehavior> {
  static constexpr const char *kName = "Set Collision Behavior";
};

template <>
struct CommandTraits<SetJointImpedance> {
  static constexpr const char *kName = "Set Joint Impedance";
};

template <>
struct CommandTraits<SetCartesianImpedance> {
  static constexpr const char *kName = "Set Cartesian Impedance";
};

template <>
struct CommandTraits<SetGuidingMode> {
  static constexpr const char *kName = "Set Guiding Mode";
};

template <>
struct CommandTraits<SetEEToK> {
  static constexpr const char *kName = "Set EE to K";
};

template <>
struct CommandTraits<SetNEToEE> {
  static constexpr const char *kName = "Set NE to EE";
};

template <>
struct CommandTraits<SetLoad> {
  static constexpr const char *kName = "Set Load";
};

template <>
struct CommandTraits<AutomaticErrorRecovery> {
  static constexpr const char *kName = "Automatic Error Recovery";
};

}  // namespace robot
}  // namespace research_interface

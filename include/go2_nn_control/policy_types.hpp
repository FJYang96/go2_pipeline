#pragma once

#include <array>
#include <functional>
#include <optional>

namespace go2_nn_control {

constexpr std::size_t kJointCount = 12;
constexpr std::size_t kObservationSize = 33;

struct PolicyObservation {
  std::array<double, kJointCount> joint_position{};
  std::array<double, kJointCount> joint_velocity{};
  std::array<double, 4> relative_quaternion_wxyz{{1.0, 0.0, 0.0, 0.0}};
  std::array<double, 3> body_angular_velocity{};
  std::array<double, 2> phase_cos_sin{{1.0, 0.0}};
};

struct PolicyOutput {
  std::array<double, kJointCount> desired_position{};
  std::array<double, kJointCount> desired_velocity{};
  std::optional<std::array<double, kJointCount>> feedforward_torque;
};

using PolicyFunction = std::function<PolicyOutput(const PolicyObservation &)>;

}  // namespace go2_nn_control

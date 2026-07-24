#include "go2_nn_control/observation_wrappers.hpp"

#include <cmath>
#include <stdexcept>

namespace go2_nn_control {
namespace {

QuaternionWxyz multiply(const QuaternionWxyz &a, const QuaternionWxyz &b) {
  return {{
      a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  }};
}

}  // namespace

QuaternionWxyz normalize_quaternion(const QuaternionWxyz &q) {
  double squared_norm = 0.0;
  for (const double value : q) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("quaternion contains a non-finite value");
    }
    squared_norm += value * value;
  }
  if (squared_norm < 1.0e-12) {
    throw std::invalid_argument("quaternion norm is too small");
  }
  const double inverse_norm = 1.0 / std::sqrt(squared_norm);
  QuaternionWxyz result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = q[i] * inverse_norm;
  }
  if (result[0] < 0.0) {
    for (double &value : result) {
      value = -value;
    }
  }
  return result;
}

QuaternionWxyz relative_quaternion_initial_frame(
    const QuaternionWxyz &start_body_to_world,
    const QuaternionWxyz &current_body_to_world) {
  const auto start = normalize_quaternion(start_body_to_world);
  const auto current = normalize_quaternion(current_body_to_world);
  const QuaternionWxyz inverse_start{{start[0], -start[1], -start[2],
                                      -start[3]}};
  return normalize_quaternion(multiply(inverse_start, current));
}

std::array<double, kJointCount> reorder_unitree_to_policy(
    const std::array<double, kJointCount> &unitree_values) {
  // Unitree: FR, FL, RR, RL. Policy: FL, FR, RL, RR.
  constexpr std::array<std::size_t, kJointCount> permutation{
      {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}};
  std::array<double, kJointCount> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = unitree_values[permutation[i]];
  }
  return result;
}

std::array<double, kObservationSize> gather_policy_observation_33(
    const QuaternionWxyz &relative_quaternion_wxyz,
    const std::array<double, kJointCount> &unitree_joint_position,
    const std::array<double, 3> &body_angular_velocity,
    const std::array<double, kJointCount> &unitree_joint_velocity,
    const std::array<double, 2> &phase_cos_sin) {
  std::array<double, kObservationSize> result{};
  std::size_t cursor = 0;
  const auto quaternion = normalize_quaternion(relative_quaternion_wxyz);
  const auto position = reorder_unitree_to_policy(unitree_joint_position);
  const auto velocity = reorder_unitree_to_policy(unitree_joint_velocity);
  for (const double value : quaternion) result[cursor++] = value;
  for (const double value : position) result[cursor++] = value;
  for (const double value : body_angular_velocity) result[cursor++] = value;
  for (const double value : velocity) result[cursor++] = value;
  for (const double value : phase_cos_sin) result[cursor++] = value;
  return result;
}

}  // namespace go2_nn_control

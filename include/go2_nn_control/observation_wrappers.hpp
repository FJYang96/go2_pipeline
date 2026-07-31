#pragma once

#include <array>

#include "go2_nn_control/policy_types.hpp"

namespace go2_nn_control {

using QuaternionWxyz = std::array<double, 4>;

QuaternionWxyz normalize_quaternion(const QuaternionWxyz &q);
QuaternionWxyz relative_quaternion_initial_frame(
    const QuaternionWxyz &start_body_to_world,
    const QuaternionWxyz &current_body_to_world);

// Produces:
// quat[4], q[FL,FR,RL,RR][3], gyro[3], dq[FL,FR,RL,RR][3], phase[cos,sin].
std::array<double, kObservationSize> gather_policy_observation_33(
    const QuaternionWxyz &relative_quaternion_wxyz,
    const std::array<double, kJointCount> &unitree_joint_position,
    const std::array<double, 3> &body_angular_velocity,
    const std::array<double, kJointCount> &unitree_joint_velocity,
    const std::array<double, 2> &phase_cos_sin);

std::array<double, kJointCount> reorder_unitree_to_policy(
    const std::array<double, kJointCount> &unitree_values);

std::array<double, kJointCount> reorder_policy_to_unitree(
    const std::array<double, kJointCount> &policy_values);

}  // namespace go2_nn_control

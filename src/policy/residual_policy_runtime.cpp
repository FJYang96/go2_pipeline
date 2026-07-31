#include "go2_nn_control/policy/residual_policy_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "go2_nn_control/observation_wrappers.hpp"

namespace go2_nn_control {
namespace {

constexpr double kActionRangeSoftTolerance = 1.0e-3;

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error("ResidualPolicyRuntime: " + message);
}

}  // namespace

ResidualPolicyRuntime::ResidualPolicyRuntime(
    const std::filesystem::path &policy_dir)
    : manifest_(PolicyManifest::Load(policy_dir)),
      reference_(ReferenceTrajectory::Load(manifest_)),
      onnx_(std::make_unique<OnnxPolicy>(manifest_)) {}

RuntimeResult ResidualPolicyRuntime::evaluate_packed(
    const std::array<double, kObservationSize> &observation,
    double policy_time_seconds) {
  if (!(policy_time_seconds >= 0.0) || !std::isfinite(policy_time_seconds)) {
    fail("policy_time_seconds must be finite and >= 0");
  }
  for (const double value : observation) {
    if (!std::isfinite(value)) {
      fail("observation contains a non-finite value");
    }
  }

  const auto index = static_cast<std::size_t>(
      std::floor(policy_time_seconds / manifest_.control_dt()));
  if (index >= reference_.size()) {
    fail("reference index " + std::to_string(index) +
         " is outside trajectory of size " +
         std::to_string(reference_.size()));
  }

  const auto action = onnx_->infer(observation);
  for (const float value : action) {
    if (!std::isfinite(value)) {
      fail("ONNX action contains a non-finite value");
    }
    if (std::abs(value) > 1.0 + kActionRangeSoftTolerance) {
      fail("ONNX action exceeds normalized range [-1, 1]");
    }
  }

  const auto &sample = reference_.at(index);
  JointArray q_policy{};
  JointArray qd_policy = sample.velocity;
  JointArray tau_policy = sample.feedforward_torque;
  const double limit = manifest_.position_correction_limit_rad();
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double clipped =
        std::max(-1.0, std::min(1.0, static_cast<double>(action[i])));
    q_policy[i] = sample.position[i] + clipped * limit;
  }

  RuntimeResult result;
  result.reference_index = index;
  result.raw_action = action;
  result.output.desired_position = reorder_policy_to_unitree(q_policy);
  result.output.desired_velocity = reorder_policy_to_unitree(qd_policy);
  result.output.feedforward_torque = reorder_policy_to_unitree(tau_policy);
  return result;
}

RuntimeResult ResidualPolicyRuntime::evaluate(
    const PolicyObservation &observation, double policy_time_seconds) {
  const auto packed = gather_policy_observation_33(
      observation.relative_quaternion_wxyz, observation.joint_position,
      observation.body_angular_velocity, observation.joint_velocity,
      observation.phase_cos_sin);
  return evaluate_packed(packed, policy_time_seconds);
}

}  // namespace go2_nn_control

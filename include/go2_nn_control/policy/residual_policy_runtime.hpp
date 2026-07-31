#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>

#include "go2_nn_control/policy/onnx_policy.hpp"
#include "go2_nn_control/policy/policy_manifest.hpp"
#include "go2_nn_control/policy/reference_trajectory.hpp"
#include "go2_nn_control/policy_types.hpp"

namespace go2_nn_control {

struct RuntimeResult {
  PolicyOutput output;
  std::size_t reference_index = 0;
  PolicyActionVector raw_action{};
};

class ResidualPolicyRuntime {
 public:
  explicit ResidualPolicyRuntime(const std::filesystem::path &policy_dir);

  const PolicyManifest &manifest() const { return manifest_; }
  const ReferenceTrajectory &reference() const { return reference_; }

  RuntimeResult evaluate_packed(
      const std::array<double, kObservationSize> &observation,
      double policy_time_seconds);

  RuntimeResult evaluate(const PolicyObservation &observation,
                         double policy_time_seconds);

 private:
  PolicyManifest manifest_;
  ReferenceTrajectory reference_;
  std::unique_ptr<OnnxPolicy> onnx_;
};

}  // namespace go2_nn_control

#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "go2_nn_control/policy/policy_manifest.hpp"

namespace go2_nn_control {

struct ReferenceSample {
  JointArray position{};
  JointArray velocity{};
  JointArray feedforward_torque{};
};

class ReferenceTrajectory {
 public:
  static ReferenceTrajectory Load(const PolicyManifest &manifest);

  const ReferenceSample &at(std::size_t index) const;
  std::size_t size() const { return samples_.size(); }

 private:
  explicit ReferenceTrajectory(std::vector<ReferenceSample> samples)
      : samples_(std::move(samples)) {}

  std::vector<ReferenceSample> samples_;
};

}  // namespace go2_nn_control

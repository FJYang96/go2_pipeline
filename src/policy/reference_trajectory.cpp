#include "go2_nn_control/policy/reference_trajectory.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "go2_nn_control/policy/npy_array.hpp"

namespace go2_nn_control {
namespace {

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error("ReferenceTrajectory: " + message);
}

void copy_row(const NpyArray &array, std::size_t row, JointArray &out) {
  const std::size_t offset = row * kJointCount;
  for (std::size_t j = 0; j < kJointCount; ++j) {
    const double value = array.data[offset + j];
    if (!std::isfinite(value)) {
      fail("non-finite value at row " + std::to_string(row) + " joint " +
           std::to_string(j));
    }
    out[j] = value;
  }
}

}  // namespace

ReferenceTrajectory ReferenceTrajectory::Load(const PolicyManifest &manifest) {
  const auto q = load_npy_f64_matrix(manifest.reference_q_path(), kJointCount);
  const auto qd =
      load_npy_f64_matrix(manifest.reference_qd_path(), kJointCount);
  const auto tau =
      load_npy_f64_matrix(manifest.reference_tau_path(), kJointCount);

  if (q.shape[0] != manifest.horizon() || qd.shape[0] != manifest.horizon() ||
      tau.shape[0] != manifest.horizon()) {
    fail("reference array horizon does not match policy_meta horizon_N");
  }
  if (q.shape[0] != qd.shape[0] || q.shape[0] != tau.shape[0]) {
    fail("reference array horizons are inconsistent");
  }

  std::vector<ReferenceSample> samples(q.shape[0]);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    copy_row(q, i, samples[i].position);
    copy_row(qd, i, samples[i].velocity);
    copy_row(tau, i, samples[i].feedforward_torque);
  }
  return ReferenceTrajectory(std::move(samples));
}

const ReferenceSample &ReferenceTrajectory::at(std::size_t index) const {
  if (index >= samples_.size()) {
    throw std::out_of_range("ReferenceTrajectory::at index " +
                            std::to_string(index) + " out of range for size " +
                            std::to_string(samples_.size()));
  }
  return samples_[index];
}

}  // namespace go2_nn_control

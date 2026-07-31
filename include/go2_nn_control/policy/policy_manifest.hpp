#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include "go2_nn_control/policy_types.hpp"

namespace go2_nn_control {

enum class CompletionBehavior {
  kHoldCurrent,
  kMoveToNeutral,
};

using JointArray = std::array<double, kJointCount>;

class PolicyManifest {
 public:
  static PolicyManifest Load(const std::filesystem::path &policy_dir);

  const std::filesystem::path &policy_dir() const { return policy_dir_; }
  std::filesystem::path onnx_path() const;
  std::filesystem::path reference_q_path() const;
  std::filesystem::path reference_qd_path() const;
  std::filesystem::path reference_tau_path() const;
  std::filesystem::path reference_metadata_path() const;
  std::filesystem::path policy_meta_path() const;
  std::filesystem::path parity_npz_path() const;

  double control_dt() const { return control_dt_; }
  std::size_t horizon() const { return horizon_; }
  double duration_seconds() const { return duration_seconds_; }
  CompletionBehavior completion_behavior() const {
    return completion_behavior_;
  }
  const char *completion_behavior_string() const;

  const JointArray &policy_kp() const { return policy_kp_; }
  const JointArray &policy_kd() const { return policy_kd_; }
  double position_correction_limit_rad() const {
    return position_correction_limit_rad_;
  }
  const JointArray &ready_position_policy() const {
    return ready_position_policy_;
  }

  const std::string &model_input_name() const { return model_input_name_; }
  const std::string &model_output_name() const { return model_output_name_; }
  const std::vector<std::string> &joint_order() const { return joint_order_; }
  const std::map<std::string, std::string> &provenance() const {
    return provenance_;
  }
  const std::vector<double> &normalizer_mean() const {
    return normalizer_mean_;
  }
  const std::vector<double> &normalizer_std() const { return normalizer_std_; }
  double normalizer_eps() const { return normalizer_eps_; }

 private:
  PolicyManifest() = default;

  std::filesystem::path policy_dir_;
  double control_dt_ = 0.0;
  std::size_t horizon_ = 0;
  double duration_seconds_ = 0.0;
  CompletionBehavior completion_behavior_ = CompletionBehavior::kHoldCurrent;
  JointArray policy_kp_{};
  JointArray policy_kd_{};
  double position_correction_limit_rad_ = 0.0;
  JointArray ready_position_policy_{};
  std::string model_input_name_;
  std::string model_output_name_;
  std::vector<std::string> joint_order_;
  std::map<std::string, std::string> provenance_;
  std::vector<double> normalizer_mean_;
  std::vector<double> normalizer_std_;
  double normalizer_eps_ = 0.0;
};

}  // namespace go2_nn_control

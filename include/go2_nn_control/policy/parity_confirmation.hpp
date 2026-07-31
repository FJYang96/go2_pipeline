#pragma once

#include <filesystem>
#include <string>

#include "go2_nn_control/policy/residual_policy_runtime.hpp"

namespace go2_nn_control {

struct ParityConfirmationResult {
  std::size_t num_samples = 0;
  double max_action_abs_error = 0.0;
  double max_q_abs_error = 0.0;
  double max_qd_abs_error = 0.0;
  double max_tau_abs_error = 0.0;
};

// Loads <policy_dir>/parity.npz (or an explicit path) and compares C++ runtime
// outputs against the packaged expectations.
ParityConfirmationResult confirm_policy_parity(
    ResidualPolicyRuntime &runtime,
    const std::filesystem::path &parity_npz_path);

ParityConfirmationResult confirm_policy_parity(ResidualPolicyRuntime &runtime);

}  // namespace go2_nn_control

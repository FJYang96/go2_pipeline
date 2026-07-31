#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "go2_nn_control/policy/parity_confirmation.hpp"
#include "go2_nn_control/policy/policy_manifest.hpp"
#include "go2_nn_control/policy/reference_trajectory.hpp"
#include "go2_nn_control/policy/residual_policy_runtime.hpp"

namespace {

void print_joint_array(const char *label,
                       const go2_nn_control::JointArray &values) {
  std::cout << "  " << label << ": [";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      std::cout << ", ";
    }
    std::cout << values[i];
  }
  std::cout << "]\n";
}

void print_sample(const char *label,
                  const go2_nn_control::ReferenceSample &sample) {
  std::cout << label << "\n";
  print_joint_array("q", sample.position);
  print_joint_array("qd", sample.velocity);
  print_joint_array("tau_ff", sample.feedforward_torque);
}

void print_usage() {
  std::cerr << "Usage: inspect_policy_package <policy_dir> [--parity]\n";
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2 || argc > 3) {
    print_usage();
    return 2;
  }

  bool run_parity = false;
  std::filesystem::path policy_dir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--parity") {
      run_parity = true;
    } else if (arg.rfind("-", 0) == 0) {
      print_usage();
      return 2;
    } else {
      policy_dir = arg;
    }
  }
  if (policy_dir.empty()) {
    print_usage();
    return 2;
  }

  try {
    const auto manifest = go2_nn_control::PolicyManifest::Load(policy_dir);
    const auto reference = go2_nn_control::ReferenceTrajectory::Load(manifest);

    std::cout << std::setprecision(17);
    std::cout << "Policy package OK\n";
    std::cout << "policy_dir: " << manifest.policy_dir() << "\n";
    std::cout << "onnx_path: " << manifest.onnx_path() << "\n";
    std::cout << "model: " << manifest.model_input_name() << " -> "
              << manifest.model_output_name() << "\n";
    std::cout << "control_dt: " << manifest.control_dt() << "\n";
    std::cout << "horizon_N: " << manifest.horizon() << "\n";
    std::cout << "duration_s: " << manifest.duration_seconds() << "\n";
    std::cout << "completion_behavior: "
              << manifest.completion_behavior_string() << "\n";
    std::cout << "position_correction_limit_rad: "
              << manifest.position_correction_limit_rad() << "\n";
    std::cout << "policy_rate_hz: " << (1.0 / manifest.control_dt()) << "\n";
    print_joint_array("policy_kp", manifest.policy_kp());
    print_joint_array("policy_kd", manifest.policy_kd());
    print_joint_array("ready_position_policy",
                      manifest.ready_position_policy());
    std::cout << "normalizer_eps: " << manifest.normalizer_eps() << "\n";
    std::cout << "reference_size: " << reference.size() << "\n";
    if (!manifest.provenance().empty()) {
      std::cout << "provenance:\n";
      for (const auto &item : manifest.provenance()) {
        std::cout << "  " << item.first << ": " << item.second << "\n";
      }
    }
    if (reference.size() > 0) {
      print_sample("first_reference_sample", reference.at(0));
      print_sample("last_reference_sample",
                   reference.at(reference.size() - 1));
    }

    if (run_parity) {
      if (!std::filesystem::is_regular_file(manifest.parity_npz_path())) {
        throw std::runtime_error("parity.npz is missing (required by --parity)");
      }
      go2_nn_control::ResidualPolicyRuntime runtime(policy_dir);
      const auto summary = go2_nn_control::confirm_policy_parity(runtime);
      std::cout << "Parity confirmation OK\n";
      std::cout << "parity_samples: " << summary.num_samples << "\n";
      std::cout << "max_action_abs_error: " << summary.max_action_abs_error
                << "\n";
      std::cout << "max_q_abs_error: " << summary.max_q_abs_error << "\n";
      std::cout << "max_qd_abs_error: " << summary.max_qd_abs_error << "\n";
      std::cout << "max_tau_abs_error: " << summary.max_tau_abs_error << "\n";
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "inspect_policy_package failed: " << error.what() << "\n";
    return 1;
  }
}

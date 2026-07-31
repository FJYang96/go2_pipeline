#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"
#include "go2_nn_control/policy/parity_confirmation.hpp"
#include "go2_nn_control/policy/residual_policy_runtime.hpp"

namespace {

std::filesystem::path hop_policy_dir() {
  if (const char *env = std::getenv("GO2_HOP_POLICY_DIR")) {
    return std::filesystem::path(env);
  }
  return std::filesystem::path(GO2_HOP_POLICY_DIR);
}

bool hop_available() {
  const auto dir = hop_policy_dir();
  return std::filesystem::is_regular_file(dir / "policy.onnx") &&
         std::filesystem::is_regular_file(dir / "policy_meta.yaml");
}

TEST(ResidualPolicyRuntime, LoadsHopPackageAndInfers) {
  if (!hop_available()) {
    GTEST_SKIP() << "hop policy package not mounted at " << hop_policy_dir();
  }
  go2_nn_control::ResidualPolicyRuntime runtime(hop_policy_dir());
  std::array<double, 33> obs{};
  obs[0] = 1.0;
  const auto result = runtime.evaluate_packed(obs, 0.0);
  EXPECT_EQ(result.reference_index, 0u);
  EXPECT_TRUE(result.output.feedforward_torque.has_value());
  for (const float value : result.raw_action) {
    EXPECT_TRUE(std::isfinite(value));
    EXPECT_LE(std::abs(value), 1.0 + 1.0e-3);
  }
}

TEST(ResidualPolicyRuntime, RejectsOutOfHorizonTime) {
  if (!hop_available()) {
    GTEST_SKIP() << "hop policy package not mounted at " << hop_policy_dir();
  }
  go2_nn_control::ResidualPolicyRuntime runtime(hop_policy_dir());
  std::array<double, 33> obs{};
  obs[0] = 1.0;
  EXPECT_THROW(runtime.evaluate_packed(obs, runtime.manifest().duration_seconds()),
               std::runtime_error);
}

TEST(ResidualPolicyRuntime, ConfirmsParityNpzWhenPresent) {
  if (!hop_available()) {
    GTEST_SKIP() << "hop policy package not mounted at " << hop_policy_dir();
  }
  const auto parity = hop_policy_dir() / "parity.npz";
  if (!std::filesystem::is_regular_file(parity)) {
    GTEST_SKIP() << "parity.npz not present; generate with "
                    "test/scripts/generate_parity_npz.py --in-place";
  }
  go2_nn_control::ResidualPolicyRuntime runtime(hop_policy_dir());
  const auto summary = go2_nn_control::confirm_policy_parity(runtime, parity);
  EXPECT_GT(summary.num_samples, 0u);
  EXPECT_LT(summary.max_action_abs_error, 1.0e-4);
  EXPECT_LT(summary.max_q_abs_error, 1.0e-4);
}

}  // namespace

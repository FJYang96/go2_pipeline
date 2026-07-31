#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "go2_nn_control/policy/policy_manifest.hpp"
#include "policy_test_utils.hpp"

namespace {

std::filesystem::path fixture(const std::string &name) {
  return std::filesystem::path(GO2_POLICY_TEST_FIXTURES_DIR) / name;
}

std::filesystem::path make_temp_copy(const std::string &name) {
  const auto destination =
      std::filesystem::temp_directory_path() / ("go2_policy_" + name);
  go2_policy_test::copy_directory(fixture("valid_package"), destination);
  return destination;
}

TEST(PolicyManifest, LoadsValidPackage) {
  const auto manifest =
      go2_nn_control::PolicyManifest::Load(fixture("valid_package"));
  EXPECT_DOUBLE_EQ(manifest.control_dt(), 0.01);
  EXPECT_EQ(manifest.horizon(), 4u);
  EXPECT_DOUBLE_EQ(manifest.duration_seconds(), 0.04);
  EXPECT_EQ(manifest.completion_behavior(),
            go2_nn_control::CompletionBehavior::kHoldCurrent);
  EXPECT_DOUBLE_EQ(manifest.position_correction_limit_rad(), 0.6);
  EXPECT_DOUBLE_EQ(manifest.policy_kp()[0], 50.0);
  EXPECT_DOUBLE_EQ(manifest.policy_kd()[0], 3.0);
  EXPECT_EQ(manifest.model_input_name(), "obs");
  EXPECT_EQ(manifest.model_output_name(), "actions");
  EXPECT_TRUE(std::filesystem::is_regular_file(manifest.onnx_path()));
  ASSERT_FALSE(manifest.provenance().empty());
  EXPECT_NE(manifest.provenance().at("checkpoint").find("/tmp/"),
            std::string::npos);
  EXPECT_EQ(manifest.onnx_path().filename(), "policy.onnx");
  EXPECT_EQ(manifest.reference_q_path().filename(), "q_des.npy");
}

TEST(PolicyManifest, IgnoresProvenancePathsForAssetResolution) {
  const auto manifest =
      go2_nn_control::PolicyManifest::Load(fixture("valid_package"));
  const auto dir = manifest.policy_dir().string();
  EXPECT_EQ(manifest.onnx_path().string().rfind(dir, 0), 0u);
  EXPECT_EQ(manifest.reference_q_path().string().rfind(dir, 0), 0u);
  EXPECT_TRUE(manifest.provenance().count("source_onnx_dir") > 0);
  EXPECT_EQ(manifest.onnx_path().string().find("/tmp/does/not/exist"),
            std::string::npos);
}

TEST(PolicyManifest, RejectsMissingAsset) {
  const auto pkg = make_temp_copy("missing_asset");
  std::filesystem::remove(pkg / "reference" / "q_des.npy");
  EXPECT_THROW(go2_nn_control::PolicyManifest::Load(pkg), std::runtime_error);
  std::filesystem::remove_all(pkg);
}

TEST(PolicyManifest, RejectsHashMismatch) {
  const auto pkg = make_temp_copy("hash_mismatch");
  std::ofstream(pkg / "policy.onnx", std::ios::binary | std::ios::app) << 'x';
  EXPECT_THROW(go2_nn_control::PolicyManifest::Load(pkg), std::runtime_error);
  std::filesystem::remove_all(pkg);
}

TEST(PolicyManifest, RejectsPathEscape) {
  const auto pkg = make_temp_copy("path_escape");
  std::ofstream(pkg / "MANIFEST.sha256")
      << go2_policy_test::sha256_file(pkg / "policy_meta.yaml")
      << "  ../outside.yaml\n"
      << go2_policy_test::sha256_file(pkg / "policy.onnx") << "  policy.onnx\n"
      << go2_policy_test::sha256_file(pkg / "policy_meta.yaml")
      << "  policy_meta.yaml\n"
      << go2_policy_test::sha256_file(pkg / "reference" / "metadata.yaml")
      << "  reference/metadata.yaml\n"
      << go2_policy_test::sha256_file(pkg / "reference" / "q_des.npy")
      << "  reference/q_des.npy\n"
      << go2_policy_test::sha256_file(pkg / "reference" / "qd_des.npy")
      << "  reference/qd_des.npy\n"
      << go2_policy_test::sha256_file(pkg / "reference" / "tau_ff.npy")
      << "  reference/tau_ff.npy\n";
  EXPECT_THROW(go2_nn_control::PolicyManifest::Load(pkg), std::runtime_error);
  std::filesystem::remove_all(pkg);
}

TEST(PolicyManifest, RejectsContractViolation) {
  // One representative metadata-contract failure is enough: packages are
  // produced by an upstream exporter, so exhaustive YAML matrix tests are
  // low value compared with integrity and happy-path coverage.
  const auto pkg = make_temp_copy("baked_false");
  std::ifstream input(pkg / "policy_meta.yaml");
  std::string text{
      std::istreambuf_iterator<char>(input),
      std::istreambuf_iterator<char>()};
  const auto needle = std::string("baked_into_onnx: true");
  const auto pos = text.find(needle);
  ASSERT_NE(pos, std::string::npos);
  text.replace(pos, needle.size(), "baked_into_onnx: false");
  std::ofstream(pkg / "policy_meta.yaml") << text;
  go2_policy_test::write_manifest(pkg);
  EXPECT_THROW(go2_nn_control::PolicyManifest::Load(pkg), std::runtime_error);
  std::filesystem::remove_all(pkg);
}

}  // namespace

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "go2_nn_control/policy/policy_manifest.hpp"
#include "go2_nn_control/policy/reference_trajectory.hpp"
#include "policy_test_utils.hpp"

namespace {

std::filesystem::path fixture(const std::string &name) {
  return std::filesystem::path(GO2_POLICY_TEST_FIXTURES_DIR) / name;
}

std::filesystem::path make_temp_copy(const std::string &name) {
  const auto destination =
      std::filesystem::temp_directory_path() / ("go2_policy_ref_" + name);
  go2_policy_test::copy_directory(fixture("valid_package"), destination);
  return destination;
}

void write_npy_f64(const std::filesystem::path &path, std::size_t rows,
                   std::size_t cols, bool insert_nan) {
  // Minimal NumPy v1.0 little-endian float64 C-order writer for tests.
  std::ostringstream header_dict;
  header_dict << "{'descr': '<f8', 'fortran_order': False, 'shape': (" << rows
              << ", " << cols << "), }";
  std::string dict = header_dict.str();
  while ((10 + 2 + dict.size()) % 64 != 0) {
    dict.push_back(' ');
  }
  dict.back() = '\n';

  std::ofstream output(path, std::ios::binary);
  const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y'};
  output.write(magic, 6);
  const unsigned char version[2] = {1, 0};
  output.write(reinterpret_cast<const char *>(version), 2);
  const std::uint16_t header_len = static_cast<std::uint16_t>(dict.size());
  output.write(reinterpret_cast<const char *>(&header_len), 2);
  output.write(dict.data(), static_cast<std::streamsize>(dict.size()));

  std::vector<double> values(rows * cols);
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = static_cast<double>(i);
  }
  if (insert_nan && !values.empty()) {
    values[0] = std::numeric_limits<double>::quiet_NaN();
  }
  output.write(reinterpret_cast<const char *>(values.data()),
               static_cast<std::streamsize>(values.size() * sizeof(double)));
}

TEST(ReferenceTrajectory, LoadsFloat64Package) {
  const auto manifest =
      go2_nn_control::PolicyManifest::Load(fixture("valid_package"));
  const auto trajectory =
      go2_nn_control::ReferenceTrajectory::Load(manifest);
  ASSERT_EQ(trajectory.size(), 4u);
  EXPECT_DOUBLE_EQ(trajectory.at(0).position[0], 0.0);
  EXPECT_DOUBLE_EQ(trajectory.at(0).position[1], 1.0);
  EXPECT_DOUBLE_EQ(trajectory.at(1).position[0], 12.0);
  EXPECT_DOUBLE_EQ(trajectory.at(3).velocity[11], 3.0 * 12.0 + 11.0);
  EXPECT_DOUBLE_EQ(trajectory.at(2).feedforward_torque[5], 2.0 * 12.0 + 5.0);
  EXPECT_THROW(trajectory.at(4), std::out_of_range);
}

TEST(ReferenceTrajectory, LoadsFloat32Package) {
  const auto manifest =
      go2_nn_control::PolicyManifest::Load(fixture("valid_package_f32"));
  const auto trajectory =
      go2_nn_control::ReferenceTrajectory::Load(manifest);
  ASSERT_EQ(trajectory.size(), 4u);
  EXPECT_DOUBLE_EQ(trajectory.at(0).position[0], 0.0);
  EXPECT_DOUBLE_EQ(trajectory.at(0).position[11], 11.0);
}

TEST(ReferenceTrajectory, RejectsCorruptReferenceArray) {
  const auto pkg = make_temp_copy("nan_reference");
  write_npy_f64(pkg / "reference" / "q_des.npy", 4, 12, true);
  go2_policy_test::write_manifest(pkg);
  const auto manifest = go2_nn_control::PolicyManifest::Load(pkg);
  EXPECT_THROW(go2_nn_control::ReferenceTrajectory::Load(manifest),
               std::runtime_error);
  std::filesystem::remove_all(pkg);
}

TEST(ReferenceTrajectory, RejectsWrongShape) {
  const auto pkg = make_temp_copy("wrong_shape");
  write_npy_f64(pkg / "reference" / "q_des.npy", 4, 11, false);
  go2_policy_test::write_manifest(pkg);
  const auto manifest = go2_nn_control::PolicyManifest::Load(pkg);
  EXPECT_THROW(go2_nn_control::ReferenceTrajectory::Load(manifest),
               std::runtime_error);
  std::filesystem::remove_all(pkg);
}

TEST(ReferenceTrajectory, BoundaryIndices) {
  const auto manifest =
      go2_nn_control::PolicyManifest::Load(fixture("valid_package"));
  const auto trajectory =
      go2_nn_control::ReferenceTrajectory::Load(manifest);
  EXPECT_NO_THROW(trajectory.at(0));
  EXPECT_NO_THROW(trajectory.at(trajectory.size() - 1));
  EXPECT_THROW(trajectory.at(trajectory.size()), std::out_of_range);
}

}  // namespace

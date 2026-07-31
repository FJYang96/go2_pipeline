#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "go2_nn_control/observation_wrappers.hpp"

namespace {

constexpr double kTolerance = 1.0e-9;

TEST(QuaternionWrapper, StartOrientationMapsToIdentity) {
  const double half = std::sqrt(0.5);
  const go2_nn_control::QuaternionWxyz start{{half, 0.0, 0.0, half}};
  const auto relative =
      go2_nn_control::relative_quaternion_initial_frame(start, start);
  EXPECT_NEAR(relative[0], 1.0, kTolerance);
  EXPECT_NEAR(relative[1], 0.0, kTolerance);
  EXPECT_NEAR(relative[2], 0.0, kTolerance);
  EXPECT_NEAR(relative[3], 0.0, kTolerance);
}

TEST(QuaternionWrapper, UsesInitialFrameCompositionAndCanonicalSign) {
  const double half = std::sqrt(0.5);
  const go2_nn_control::QuaternionWxyz start{{half, half, 0.0, 0.0}};
  const go2_nn_control::QuaternionWxyz delta{{half, 0.0, half, 0.0}};
  // current = start * delta, so inverse(start) * current must recover delta.
  const go2_nn_control::QuaternionWxyz current{{0.5, 0.5, 0.5, 0.5}};
  const auto relative =
      go2_nn_control::relative_quaternion_initial_frame(start, current);
  for (std::size_t i = 0; i < delta.size(); ++i) {
    EXPECT_NEAR(relative[i], delta[i], kTolerance);
  }
  const go2_nn_control::QuaternionWxyz negative{{-1.0, 0.0, 0.0, 0.0}};
  EXPECT_NEAR(go2_nn_control::normalize_quaternion(negative)[0], 1.0,
              kTolerance);
}

TEST(QuaternionWrapper, RejectsInvalidInput) {
  EXPECT_THROW(
      go2_nn_control::normalize_quaternion({{0.0, 0.0, 0.0, 0.0}}),
      std::invalid_argument);
  EXPECT_THROW(
      go2_nn_control::normalize_quaternion(
          {{1.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}}),
      std::invalid_argument);
}

TEST(ObservationWrapper, ReordersAndFlattensExactly) {
  std::array<double, 12> q{};
  std::array<double, 12> dq{};
  for (std::size_t i = 0; i < 12; ++i) {
    q[i] = 100.0 + static_cast<double>(i);
    dq[i] = 200.0 + static_cast<double>(i);
  }
  const auto result = go2_nn_control::gather_policy_observation_33(
      {{2.0, 0.0, 0.0, 0.0}}, q, {{10.0, 11.0, 12.0}}, dq,
      {{0.25, -0.75}});
  const std::array<double, 12> expected_q{
      {103, 104, 105, 100, 101, 102, 109, 110, 111, 106, 107, 108}};
  const std::array<double, 12> expected_dq{
      {203, 204, 205, 200, 201, 202, 209, 210, 211, 206, 207, 208}};

  EXPECT_DOUBLE_EQ(result[0], 1.0);
  for (std::size_t i = 0; i < 12; ++i) {
    EXPECT_DOUBLE_EQ(result[4 + i], expected_q[i]);
    EXPECT_DOUBLE_EQ(result[19 + i], expected_dq[i]);
  }
  EXPECT_DOUBLE_EQ(result[16], 10.0);
  EXPECT_DOUBLE_EQ(result[17], 11.0);
  EXPECT_DOUBLE_EQ(result[18], 12.0);
  EXPECT_DOUBLE_EQ(result[31], 0.25);   // cos
  EXPECT_DOUBLE_EQ(result[32], -0.75);  // sin
}

TEST(JointOrder, PolicyToUnitreeIsInverseOfUnitreeToPolicy) {
  std::array<double, 12> unitree{};
  for (std::size_t i = 0; i < unitree.size(); ++i) {
    unitree[i] = 10.0 + static_cast<double>(i);
  }
  const auto policy = go2_nn_control::reorder_unitree_to_policy(unitree);
  const auto round_trip = go2_nn_control::reorder_policy_to_unitree(policy);
  for (std::size_t i = 0; i < unitree.size(); ++i) {
    EXPECT_DOUBLE_EQ(round_trip[i], unitree[i]);
  }

  const std::array<double, 12> expected_policy{
      {13, 14, 15, 10, 11, 12, 19, 20, 21, 16, 17, 18}};
  for (std::size_t i = 0; i < expected_policy.size(); ++i) {
    EXPECT_DOUBLE_EQ(policy[i], expected_policy[i]);
  }

  const auto back_to_policy =
      go2_nn_control::reorder_unitree_to_policy(round_trip);
  for (std::size_t i = 0; i < policy.size(); ++i) {
    EXPECT_DOUBLE_EQ(back_to_policy[i], policy[i]);
  }
}

}  // namespace

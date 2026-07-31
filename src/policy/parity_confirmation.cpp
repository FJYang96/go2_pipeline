#include "go2_nn_control/policy/parity_confirmation.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

#include "go2_nn_control/policy/npy_array.hpp"

namespace go2_nn_control {
namespace {

constexpr double kTimingTolerance = 1.0e-9;
constexpr double kPhaseTolerance = 1.0e-5;
constexpr double kPi = 3.14159265358979323846;

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error("ParityConfirmation: " + message);
}

const NpyArray &require_key(const std::map<std::string, NpyArray> &arrays,
                            const std::string &key) {
  const auto it = arrays.find(key);
  if (it == arrays.end()) {
    fail("missing key '" + key + "' in parity.npz");
  }
  return it->second;
}

std::int64_t as_scalar_int64(const NpyArray &array, const std::string &key) {
  if (array.dtype != NpyDType::kInt64) {
    fail(key + " must be int64");
  }
  if (!(array.shape.empty() ||
        (array.shape.size() == 1 && array.shape[0] == 1))) {
    fail(key + " must be a scalar");
  }
  if (array.int64_data.size() != 1) {
    fail(key + " must contain exactly one value");
  }
  return array.int64_data[0];
}

double as_scalar_f64(const NpyArray &array, const std::string &key) {
  if (array.dtype == NpyDType::kInt64) {
    fail(key + " must be floating-point");
  }
  if (!(array.shape.empty() ||
        (array.shape.size() == 1 && array.shape[0] == 1))) {
    fail(key + " must be a scalar");
  }
  if (array.data.size() != 1) {
    fail(key + " must contain exactly one value");
  }
  return array.data[0];
}

void require_shape(const NpyArray &array, const std::vector<std::size_t> &shape,
                   const std::string &key) {
  if (array.shape != shape) {
    fail(key + " has unexpected shape");
  }
}

double max_abs_diff(const double *a, const float *b, std::size_t n) {
  double max_err = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    max_err = std::max(max_err, std::abs(a[i] - static_cast<double>(b[i])));
  }
  return max_err;
}

double max_abs_diff(const double *a, const double *b, std::size_t n) {
  double max_err = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    max_err = std::max(max_err, std::abs(a[i] - b[i]));
  }
  return max_err;
}

bool close(double actual, double expected, double atol, double rtol) {
  return std::abs(actual - expected) <= atol + rtol * std::abs(expected);
}

}  // namespace

ParityConfirmationResult confirm_policy_parity(
    ResidualPolicyRuntime &runtime,
    const std::filesystem::path &parity_npz_path) {
  const auto arrays = load_npz(parity_npz_path);
  const auto schema = as_scalar_int64(require_key(arrays, "schema_version"),
                                      "schema_version");
  if (schema != 1) {
    fail("unsupported parity schema_version");
  }
  const auto batch_size =
      as_scalar_int64(require_key(arrays, "batch_size"), "batch_size");
  if (batch_size <= 0) {
    fail("batch_size must be > 0");
  }
  const auto B = static_cast<std::size_t>(batch_size);
  const double duration_s =
      as_scalar_f64(require_key(arrays, "duration_s"), "duration_s");
  if (std::abs(duration_s - runtime.manifest().duration_seconds()) >
      kTimingTolerance) {
    fail("duration_s does not match policy_meta");
  }

  double atol = 1.0e-5;
  double rtol = 1.0e-5;
  if (arrays.count("atol")) {
    atol = as_scalar_f64(arrays.at("atol"), "atol");
  }
  if (arrays.count("rtol")) {
    rtol = as_scalar_f64(arrays.at("rtol"), "rtol");
  }

  const auto &times = require_key(arrays, "policy_time_seconds");
  const auto &obs = require_key(arrays, "obs");
  const auto &actions = require_key(arrays, "actions");
  const auto &q_des = require_key(arrays, "q_des_unitree");
  const auto &qd_des = require_key(arrays, "qd_des_unitree");
  const auto &tau_ff = require_key(arrays, "tau_ff_unitree");
  const auto &ref_index = require_key(arrays, "reference_index");

  require_shape(times, {B}, "policy_time_seconds");
  require_shape(obs, {B, kObservationSize}, "obs");
  require_shape(actions, {B, kJointCount}, "actions");
  require_shape(q_des, {B, kJointCount}, "q_des_unitree");
  require_shape(qd_des, {B, kJointCount}, "qd_des_unitree");
  require_shape(tau_ff, {B, kJointCount}, "tau_ff_unitree");
  require_shape(ref_index, {B}, "reference_index");
  if (times.dtype == NpyDType::kInt64 || obs.dtype == NpyDType::kInt64 ||
      actions.dtype == NpyDType::kInt64 || q_des.dtype == NpyDType::kInt64 ||
      qd_des.dtype == NpyDType::kInt64 || tau_ff.dtype == NpyDType::kInt64) {
    fail("floating arrays must not be int64");
  }
  if (ref_index.dtype != NpyDType::kInt64) {
    fail("reference_index must be int64");
  }

  ParityConfirmationResult summary;
  summary.num_samples = B;

  for (std::size_t i = 0; i < B; ++i) {
    const double t = times.data[i];
    std::array<double, kObservationSize> obs_row{};
    for (std::size_t j = 0; j < kObservationSize; ++j) {
      obs_row[j] = obs.data[i * kObservationSize + j];
    }

    const double angle = 2.0 * kPi * t / duration_s;
    const double expected_cos = std::cos(angle);
    const double expected_sin = std::sin(angle);
    if (!close(obs_row[31], expected_cos, kPhaseTolerance, 0.0) ||
        !close(obs_row[32], expected_sin, kPhaseTolerance, 0.0)) {
      fail("obs phase does not match cos/sin(2*pi*t/duration_s) at sample " +
           std::to_string(i));
    }

    const auto result = runtime.evaluate_packed(obs_row, t);
    if (static_cast<std::int64_t>(result.reference_index) !=
        ref_index.int64_data[i]) {
      fail("reference_index mismatch at sample " + std::to_string(i));
    }

    const double action_err = max_abs_diff(
        actions.data.data() + i * kJointCount, result.raw_action.data(),
        kJointCount);
    const double q_err = max_abs_diff(
        q_des.data.data() + i * kJointCount,
        result.output.desired_position.data(), kJointCount);
    const double qd_err = max_abs_diff(
        qd_des.data.data() + i * kJointCount,
        result.output.desired_velocity.data(), kJointCount);
    if (!result.output.feedforward_torque) {
      fail("missing feedforward torque in runtime output");
    }
    const double tau_err = max_abs_diff(
        tau_ff.data.data() + i * kJointCount,
        result.output.feedforward_torque->data(), kJointCount);

    summary.max_action_abs_error =
        std::max(summary.max_action_abs_error, action_err);
    summary.max_q_abs_error = std::max(summary.max_q_abs_error, q_err);
    summary.max_qd_abs_error = std::max(summary.max_qd_abs_error, qd_err);
    summary.max_tau_abs_error = std::max(summary.max_tau_abs_error, tau_err);

    auto check = [&](double err, const char *name) {
      // Use atol-dominant check vs representative magnitude.
      if (err > atol + rtol) {
        fail(std::string(name) + " mismatch at sample " + std::to_string(i) +
             " (max abs err=" + std::to_string(err) + ")");
      }
    };
    check(action_err, "actions");
    check(q_err, "q_des_unitree");
    check(qd_err, "qd_des_unitree");
    check(tau_err, "tau_ff_unitree");
  }

  return summary;
}

ParityConfirmationResult confirm_policy_parity(ResidualPolicyRuntime &runtime) {
  return confirm_policy_parity(runtime, runtime.manifest().parity_npz_path());
}

}  // namespace go2_nn_control

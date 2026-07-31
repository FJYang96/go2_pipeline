#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "go2_nn_control/policy/policy_manifest.hpp"
#include "go2_nn_control/policy_types.hpp"

namespace Ort {
struct Env;
struct Session;
struct SessionOptions;
struct MemoryInfo;
}  // namespace Ort

namespace go2_nn_control {

using PolicyObservationVector = std::array<float, kObservationSize>;
using PolicyActionVector = std::array<float, kJointCount>;

class OnnxPolicy {
 public:
  explicit OnnxPolicy(const PolicyManifest &manifest);
  ~OnnxPolicy();

  OnnxPolicy(const OnnxPolicy &) = delete;
  OnnxPolicy &operator=(const OnnxPolicy &) = delete;
  OnnxPolicy(OnnxPolicy &&) noexcept;
  OnnxPolicy &operator=(OnnxPolicy &&) noexcept;

  PolicyActionVector infer(const PolicyObservationVector &observation);
  PolicyActionVector infer(const std::array<double, kObservationSize> &observation);

  const std::string &input_name() const { return input_name_; }
  const std::string &output_name() const { return output_name_; }

 private:
  void warm_up();

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::SessionOptions> session_options_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_info_;
  std::string input_name_;
  std::string output_name_;
  std::vector<float> input_buffer_;
  std::vector<float> output_buffer_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
};

}  // namespace go2_nn_control

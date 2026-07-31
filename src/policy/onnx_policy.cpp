#include "go2_nn_control/policy/onnx_policy.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <onnxruntime_cxx_api.h>
#pragma GCC diagnostic pop

#include <stdexcept>
#include <utility>

namespace go2_nn_control {
namespace {

[[noreturn]] void fail(const std::string &message) {
  throw std::runtime_error("OnnxPolicy: " + message);
}

}  // namespace

OnnxPolicy::OnnxPolicy(const PolicyManifest &manifest)
    : input_name_(manifest.model_input_name()),
      output_name_(manifest.model_output_name()),
      input_buffer_(kObservationSize, 0.0f),
      output_buffer_(kJointCount, 0.0f),
      input_shape_{1, static_cast<int64_t>(kObservationSize)},
      output_shape_{1, static_cast<int64_t>(kJointCount)} {
  try {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING,
                                      "go2_nn_control");
    session_options_ = std::make_unique<Ort::SessionOptions>();
    session_options_->SetIntraOpNumThreads(1);
    session_options_->SetInterOpNumThreads(1);
    session_options_->SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_->SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    session_options_->DisableCpuMemArena();

    session_ = std::make_unique<Ort::Session>(
        *env_, manifest.onnx_path().c_str(), *session_options_);
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::AllocatorWithDefaultOptions allocator;
    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
      fail("expected exactly one model input and one model output");
    }
    {
      auto name = session_->GetInputNameAllocated(0, allocator);
      if (input_name_ != name.get()) {
        fail("input name mismatch: metadata '" + input_name_ + "' vs model '" +
             name.get() + "'");
      }
    }
    {
      auto name = session_->GetOutputNameAllocated(0, allocator);
      if (output_name_ != name.get()) {
        fail("output name mismatch: metadata '" + output_name_ + "' vs model '" +
             name.get() + "'");
      }
    }

    const Ort::TypeInfo input_type_info = session_->GetInputTypeInfo(0);
    const Ort::TypeInfo output_type_info = session_->GetOutputTypeInfo(0);
    const auto input_info = input_type_info.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type_info.GetTensorTypeAndShapeInfo();
    const auto input_elem = input_info.GetElementType();
    const auto output_elem = output_info.GetElementType();
    if (input_elem != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output_elem != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
      fail("model tensors must be float32 (input type=" +
           std::to_string(static_cast<int>(input_elem)) + ", output type=" +
           std::to_string(static_cast<int>(output_elem)) + ")");
    }
    const auto in_shape = input_info.GetShape();
    const auto out_shape = output_info.GetShape();
    if (in_shape.size() != 2 || out_shape.size() != 2) {
      fail("model tensors must be rank-2");
    }
    auto matches = [](int64_t actual, int64_t expected) {
      return actual == expected || actual == -1;
    };
    if (!matches(in_shape[0], 1) || !matches(in_shape[1], kObservationSize) ||
        !matches(out_shape[0], 1) || !matches(out_shape[1], kJointCount)) {
      fail("unexpected model tensor shapes");
    }

    warm_up();
  } catch (const Ort::Exception &error) {
    fail(std::string("ORT exception: ") + error.what());
  }
}

OnnxPolicy::~OnnxPolicy() = default;

OnnxPolicy::OnnxPolicy(OnnxPolicy &&) noexcept = default;
OnnxPolicy &OnnxPolicy::operator=(OnnxPolicy &&) noexcept = default;

void OnnxPolicy::warm_up() {
  PolicyObservationVector zeros{};
  (void)infer(zeros);
}

PolicyActionVector OnnxPolicy::infer(
    const std::array<double, kObservationSize> &observation) {
  PolicyObservationVector converted{};
  for (std::size_t i = 0; i < observation.size(); ++i) {
    converted[i] = static_cast<float>(observation[i]);
  }
  return infer(converted);
}

PolicyActionVector OnnxPolicy::infer(
    const PolicyObservationVector &observation) {
  try {
    for (std::size_t i = 0; i < observation.size(); ++i) {
      input_buffer_[i] = observation[i];
    }
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        *memory_info_, input_buffer_.data(), input_buffer_.size(),
        input_shape_.data(), input_shape_.size());

    const char *input_names[] = {input_name_.c_str()};
    const char *output_names[] = {output_name_.c_str()};
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, input_names,
                                 &input_tensor, 1, output_names, 1);
    if (outputs.empty() || !outputs[0].IsTensor()) {
      fail("model did not return an output tensor");
    }
    float *out = outputs[0].GetTensorMutableData<float>();
    const auto info = outputs[0].GetTensorTypeAndShapeInfo();
    if (info.GetElementCount() != static_cast<size_t>(kJointCount)) {
      fail("unexpected output element count");
    }
    PolicyActionVector action{};
    for (std::size_t i = 0; i < action.size(); ++i) {
      action[i] = out[i];
      output_buffer_[i] = out[i];
    }
    return action;
  } catch (const Ort::Exception &error) {
    fail(std::string("ORT exception during infer: ") + error.what());
  }
}

}  // namespace go2_nn_control

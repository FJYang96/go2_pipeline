#pragma once

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "go2_nn_control/msg/policy_action.hpp"
#include "go2_nn_control/msg/policy_observation.hpp"
#include "go2_nn_control/policy/residual_policy_runtime.hpp"
#include "go2_nn_control/policy_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace go2_nn_control {

class PolicyRunner final : public rclcpp::Node {
 public:
  PolicyRunner()
      : Node("policy_runner",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)) {
    const auto policy_dir = required_parameter<std::string>("policy_dir");
    if (policy_dir.empty()) {
      throw std::runtime_error("policy_dir must be a non-empty path");
    }
    runtime_ = std::make_unique<ResidualPolicyRuntime>(policy_dir);

    const auto input_topic = required_parameter<std::string>("observation_topic");
    const auto output_topic =
        required_parameter<std::string>("policy_action_topic");
    const auto qos_depth =
        static_cast<int>(required_parameter<int64_t>("qos_depth"));
    action_publisher_ =
        create_publisher<msg::PolicyAction>(output_topic, qos_depth);
    observation_subscription_ = create_subscription<msg::PolicyObservation>(
        input_topic, qos_depth,
        [this](msg::PolicyObservation::SharedPtr message) { evaluate(*message); });
    RCLCPP_INFO(get_logger(), "Policy runner loaded package from %s",
                policy_dir.c_str());
  }

 private:
  template <typename T>
  T required_parameter(const std::string &name) const {
    rclcpp::Parameter parameter;
    if (!get_parameter(name, parameter)) {
      throw std::runtime_error("missing required YAML parameter: " + name);
    }
    return parameter.get_value<T>();
  }

  void evaluate(const msg::PolicyObservation &message) {
    if (!message.policy_active) {
      return;
    }

    PolicyObservation observation;
    observation.joint_position = message.joint_position;
    observation.joint_velocity = message.joint_velocity;
    observation.relative_quaternion_wxyz = message.relative_quaternion_wxyz;
    observation.body_angular_velocity = message.body_angular_velocity;
    observation.phase_cos_sin = message.phase_cos_sin;

    const auto started = std::chrono::steady_clock::now();
    RuntimeResult result;
    try {
      result = runtime_->evaluate(observation, message.policy_time_seconds);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Policy evaluation failed: %s", error.what());
      return;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Policy evaluation threw an unknown exception");
      return;
    }
    const auto finished = std::chrono::steady_clock::now();

    msg::PolicyAction action;
    action.header.stamp = now();
    action.header.frame_id = "go2_policy";
    action.observation_sequence = message.sequence;
    action.policy_epoch = message.policy_epoch;
    action.reference_index = static_cast<uint32_t>(result.reference_index);
    action.desired_position = result.output.desired_position;
    action.desired_velocity = result.output.desired_velocity;
    action.has_feedforward_torque = result.output.feedforward_torque.has_value();
    if (result.output.feedforward_torque) {
      action.feedforward_torque = *result.output.feedforward_torque;
    } else {
      action.feedforward_torque.fill(0.0);
    }
    action.evaluation_time_ms =
        std::chrono::duration<double, std::milli>(finished - started).count();
    action_publisher_->publish(action);
  }

  std::unique_ptr<ResidualPolicyRuntime> runtime_;
  rclcpp::Publisher<msg::PolicyAction>::SharedPtr action_publisher_;
  rclcpp::Subscription<msg::PolicyObservation>::SharedPtr
      observation_subscription_;
};

}  // namespace go2_nn_control

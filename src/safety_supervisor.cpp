#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <thread>

#include "go2_nn_control/msg/applied_command.hpp"
#include "go2_nn_control/msg/policy_action.hpp"
#include "go2_nn_control/msg/policy_observation.hpp"
#include "go2_nn_control/msg/supervisor_status.hpp"
#include "go2_nn_control/observation_wrappers.hpp"
#include "go2_nn_control/unitree_crc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

namespace go2_nn_control {
namespace {

using SteadyClock = std::chrono::steady_clock;
constexpr double kPi = 3.14159265358979323846;

enum class ControlState {
  kPassive,
  kMoveToReady,
  kReadyHold,
  kPolicy,
  kHoldCurrent,
  kMoveToNeutral,
  kNeutralHold,
  kEstop,
};

const char *state_name(ControlState state) {
  switch (state) {
    case ControlState::kPassive: return "PASSIVE";
    case ControlState::kMoveToReady: return "MOVE_TO_READY";
    case ControlState::kReadyHold: return "READY_HOLD";
    case ControlState::kPolicy: return "POLICY";
    case ControlState::kHoldCurrent: return "HOLD_CURRENT";
    case ControlState::kMoveToNeutral: return "MOVE_TO_NEUTRAL";
    case ControlState::kNeutralHold: return "NEUTRAL_HOLD";
    case ControlState::kEstop: return "ESTOP";
  }
  return "UNKNOWN";
}

double minimum_jerk(double progress) {
  const double x = std::clamp(progress, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

bool is_active(ControlState state) {
  return state != ControlState::kPassive && state != ControlState::kEstop;
}

}  // namespace

class SafetySupervisor final : public rclcpp::Node {
 public:
  SafetySupervisor()
      : Node("safety_supervisor",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)) {
    load_configuration();
    initialize_low_command();

    low_command_publisher_ = create_publisher<unitree_go::msg::LowCmd>(
        low_command_topic_, qos_depth_);
    observation_publisher_ = create_publisher<msg::PolicyObservation>(
        observation_topic_, qos_depth_);
    applied_command_publisher_ = create_publisher<msg::AppliedCommand>(
        applied_command_topic_, qos_depth_);
    status_publisher_ = create_publisher<msg::SupervisorStatus>(
        status_topic_, qos_depth_);

    low_state_subscription_ = create_subscription<unitree_go::msg::LowState>(
        low_state_topic_, qos_depth_,
        [this](unitree_go::msg::LowState::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          low_state_ = *message;
          have_low_state_ = true;
          last_low_state_time_ = SteadyClock::now();
        });
    policy_subscription_ = create_subscription<msg::PolicyAction>(
        policy_action_topic_, qos_depth_,
        [this](msg::PolicyAction::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          policy_action_ = *message;
          have_policy_action_ = true;
          last_policy_time_ = SteadyClock::now();
        });
    estop_subscription_ = create_subscription<std_msgs::msg::Bool>(
        estop_topic_, qos_depth_, [this](std_msgs::msg::Bool::SharedPtr message) {
          if (message->data) latch_fault("MANUAL_ESTOP", "E-stop topic asserted");
        });

    create_services();
    command_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / command_rate_hz_)),
        [this] { command_tick(); });
    observation_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / policy_rate_hz_)),
        [this] { observation_tick(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                      [this] { publish_status(); });
    RCLCPP_INFO(get_logger(), "Safety supervisor initialized in PASSIVE mode");
  }

  ~SafetySupervisor() override {
    // Best effort only: signals or process/network failure can prevent delivery.
    const auto deadline =
        SteadyClock::now() + std::chrono::duration<double>(shutdown_passive_seconds_);
    do {
      publish_passive();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (SteadyClock::now() < deadline);
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

  template <typename T>
  std::array<double, kJointCount> get_array_parameter(const std::string &name) {
    (void)sizeof(T);
    const auto values = required_parameter<std::vector<double>>(name);
    if (values.size() != kJointCount) {
      throw std::runtime_error(name + " must contain exactly 12 values");
    }
    std::array<double, kJointCount> result{};
    std::copy(values.begin(), values.end(), result.begin());
    for (const double value : result) {
      if (!std::isfinite(value)) {
        throw std::runtime_error(name + " contains a non-finite value");
      }
    }
    return result;
  }

  void load_configuration() {
    hardware_mode_ = required_parameter<bool>("hardware_mode");
    hardware_profile_complete_ =
        required_parameter<bool>("hardware_profile_complete");
    ownership_acknowledged_ =
        required_parameter<bool>("ownership_acknowledged");
    command_rate_hz_ = required_parameter<double>("command_rate_hz");
    policy_rate_hz_ = required_parameter<double>("policy_rate_hz");
    phase_period_seconds_ = required_parameter<double>("phase_period_seconds");
    low_state_timeout_seconds_ =
        required_parameter<double>("low_state_timeout_seconds");
    policy_timeout_seconds_ =
        required_parameter<double>("policy_timeout_seconds");
    ready_transition_seconds_ =
        required_parameter<double>("ready_transition_seconds");
    neutral_transition_seconds_ =
        required_parameter<double>("neutral_transition_seconds");
    transition_timeout_seconds_ =
        required_parameter<double>("transition_timeout_seconds");
    position_tolerance_ = required_parameter<double>("position_tolerance");
    velocity_tolerance_ = required_parameter<double>("velocity_tolerance");
    quaternion_min_norm_ = required_parameter<double>("quaternion_min_norm");
    quaternion_max_norm_ = required_parameter<double>("quaternion_max_norm");
    shutdown_passive_seconds_ =
        required_parameter<double>("shutdown_passive_seconds");

    kp_ = get_array_parameter<double>("kp");
    kd_ = get_array_parameter<double>("kd");
    ready_position_ = get_array_parameter<double>("ready_position");
    neutral_position_ = get_array_parameter<double>("neutral_position");
    joint_position_min_ = get_array_parameter<double>("joint_position_min");
    joint_position_max_ = get_array_parameter<double>("joint_position_max");
    desired_velocity_limit_ =
        get_array_parameter<double>("desired_velocity_limit");
    target_rate_limit_ = get_array_parameter<double>("target_rate_limit");
    feedforward_torque_limit_ =
        get_array_parameter<double>("feedforward_torque_limit");

    low_state_topic_ = required_parameter<std::string>("low_state_topic");
    low_command_topic_ = required_parameter<std::string>("low_command_topic");
    observation_topic_ = required_parameter<std::string>("observation_topic");
    policy_action_topic_ =
        required_parameter<std::string>("policy_action_topic");
    applied_command_topic_ =
        required_parameter<std::string>("applied_command_topic");
    status_topic_ = required_parameter<std::string>("status_topic");
    estop_topic_ = required_parameter<std::string>("estop_topic");
    qos_depth_ = static_cast<int>(required_parameter<int64_t>("qos_depth"));

    if (hardware_mode_ &&
        (!hardware_profile_complete_ || !ownership_acknowledged_)) {
      throw std::runtime_error(
          "hardware mode requires a complete profile and per-run ownership "
          "acknowledgment");
    }
    if (hardware_mode_ &&
        (low_state_topic_ != "/lowstate" || low_command_topic_ != "/lowcmd")) {
      throw std::runtime_error(
          "hardware mode requires explicit /lowstate and /lowcmd topics");
    }
    if (command_rate_hz_ <= 0.0 || policy_rate_hz_ <= 0.0 ||
        phase_period_seconds_ <= 0.0 || low_state_timeout_seconds_ <= 0.0 ||
        policy_timeout_seconds_ <= 0.0 || shutdown_passive_seconds_ < 0.0) {
      throw std::runtime_error("rates, period, and watchdogs must be positive");
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (joint_position_min_[i] >= joint_position_max_[i] ||
          kp_[i] < 0.0 || kd_[i] < 0.0 ||
          desired_velocity_limit_[i] < 0.0 ||
          target_rate_limit_[i] <= 0.0 ||
          feedforward_torque_limit_[i] < 0.0 ||
          ready_position_[i] < joint_position_min_[i] ||
          ready_position_[i] > joint_position_max_[i] ||
          neutral_position_[i] < joint_position_min_[i] ||
          neutral_position_[i] > joint_position_max_[i]) {
        throw std::runtime_error("invalid per-joint configuration at index " +
                                 std::to_string(i));
      }
    }
  }

  void initialize_low_command() {
    low_command_.head[0] = 0xFE;
    low_command_.head[1] = 0xEF;
    low_command_.level_flag = 0xFF;
    low_command_.gpio = 0;
    for (auto &motor : low_command_.motor_cmd) {
      motor.mode = kPassiveMotorMode;
      motor.q = kPositionStop;
      motor.dq = kVelocityStop;
      motor.kp = 0.0F;
      motor.kd = 0.0F;
      motor.tau = 0.0F;
    }
  }

  void create_services() {
    using Trigger = std_srvs::srv::Trigger;
    ownership_service_ = create_service<Trigger>(
        "/ws_control/acknowledge_ownership",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          if (hardware_mode_) {
            response->success = ownership_acknowledged_;
            response->message = ownership_acknowledged_
                                    ? "ownership already acknowledged at launch"
                                    : "relaunch with the required acknowledgment";
          } else {
            ownership_acknowledged_ = true;
            response->success = true;
            response->message = "local-mode ownership acknowledged";
          }
        });
    arm_service_ = create_service<Trigger>(
        "/ws_control/arm",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kPassive || !healthy_low_state_locked() ||
              !ownership_acknowledged_ ||
              count_publishers(low_command_topic_) > 1U) {
            response->success = false;
            response->message =
                "arm requires PASSIVE, healthy lowstate, ownership acknowledgment, "
                "and no competing ROS /lowcmd publisher";
            return;
          }
          capture_measured_positions_locked(transition_start_position_);
          transition_started_ = SteadyClock::now();
          state_ = ControlState::kMoveToReady;
          response->success = true;
          response->message = "moving to ready stance";
        });
    start_service_ = create_service<Trigger>(
        "/ws_control/start_policy",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kReadyHold || !healthy_low_state_locked()) {
            response->success = false;
            response->message = "policy start requires READY_HOLD and healthy state";
            return;
          }
          try {
            start_quaternion_ = current_quaternion_locked();
          } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
            return;
          }
          policy_started_ = SteadyClock::now();
          have_policy_action_ = false;
          state_ = ControlState::kPolicy;
          response->success = true;
          response->message = "policy started; phase and orientation reset";
        });
    stop_service_ = create_service<Trigger>(
        "/ws_control/stop_policy",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kPolicy) {
            response->success = false;
            response->message = "normal stop is only valid in POLICY";
            return;
          }
          capture_measured_positions_locked(hold_position_);
          state_ = ControlState::kHoldCurrent;
          response->success = true;
          response->message = "holding measured pose";
        });
    recover_service_ = create_service<Trigger>(
        "/ws_control/recover",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kHoldCurrent) {
            response->success = false;
            response->message = "recovery requires HOLD_CURRENT";
            return;
          }
          capture_measured_positions_locked(transition_start_position_);
          transition_started_ = SteadyClock::now();
          state_ = ControlState::kMoveToNeutral;
          response->success = true;
          response->message = "moving to neutral stance";
        });
    estop_service_ = create_service<Trigger>(
        "/ws_control/estop",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          latch_fault("MANUAL_ESTOP", "E-stop service called");
          response->success = true;
          response->message = "E-stop latched";
        });
    reset_service_ = create_service<Trigger>(
        "/ws_control/reset_estop",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kEstop || !healthy_low_state_locked()) {
            response->success = false;
            response->message = "reset requires ESTOP and healthy lowstate";
            return;
          }
          fault_code_.clear();
          fault_message_.clear();
          state_ = ControlState::kPassive;
          response->success = true;
          response->message = "fault cleared; full arm sequence required";
        });
  }

  bool healthy_low_state_locked() const {
    return have_low_state_ &&
           std::chrono::duration<double>(SteadyClock::now() -
                                         last_low_state_time_)
                   .count() <= low_state_timeout_seconds_;
  }

  QuaternionWxyz current_quaternion_locked() const {
    QuaternionWxyz q{};
    for (std::size_t i = 0; i < q.size(); ++i) {
      q[i] = low_state_.imu_state.quaternion[i];
    }
    double squared_norm = 0.0;
    for (const double value : q) squared_norm += value * value;
    const double norm = std::sqrt(squared_norm);
    if (!std::isfinite(norm) || norm < quaternion_min_norm_ ||
        norm > quaternion_max_norm_) {
      throw std::invalid_argument("IMU quaternion failed norm validation");
    }
    return normalize_quaternion(q);
  }

  void capture_measured_positions_locked(
      std::array<double, kJointCount> &destination) const {
    for (std::size_t i = 0; i < destination.size(); ++i) {
      destination[i] = low_state_.motor_state[i].q;
    }
  }

  void latch_fault(const std::string &code, const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ControlState::kEstop) {
      RCLCPP_ERROR(get_logger(), "Latched fault %s: %s", code.c_str(),
                   message.c_str());
    }
    fault_code_ = code;
    fault_message_ = message;
    state_ = ControlState::kEstop;
  }

  bool validate_policy_locked(std::string &error) const {
    if (!have_policy_action_) {
      if (std::chrono::duration<double>(SteadyClock::now() - policy_started_)
              .count() <= policy_timeout_seconds_) {
        return true;
      }
      error = "no policy action received before startup watchdog expired";
      return false;
    }
    if (std::chrono::duration<double>(SteadyClock::now() - last_policy_time_)
            .count() > policy_timeout_seconds_) {
      error = "policy action watchdog expired";
      return false;
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double q = policy_action_.desired_position[i];
      const double dq = policy_action_.desired_velocity[i];
      const double tau = policy_action_.has_feedforward_torque
                             ? policy_action_.feedforward_torque[i]
                             : 0.0;
      if (!std::isfinite(q) || !std::isfinite(dq) || !std::isfinite(tau)) {
        error = "policy produced a non-finite value";
        return false;
      }
      if (q < joint_position_min_[i] || q > joint_position_max_[i] ||
          std::abs(dq) > desired_velocity_limit_[i] ||
          std::abs(tau) > feedforward_torque_limit_[i]) {
        error = "policy action exceeded configured hard limit at joint " +
                std::to_string(i);
        return false;
      }
    }
    return true;
  }

  void command_tick() {
    std::string deferred_fault;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_active(state_) && !healthy_low_state_locked()) {
        deferred_fault = "lowstate watchdog expired";
      } else if (state_ == ControlState::kPolicy) {
        std::string policy_error;
        if (!validate_policy_locked(policy_error)) deferred_fault = policy_error;
      }
      if (deferred_fault.empty()) {
        build_command_locked();
        set_unitree_crc(low_command_);
        low_command_publisher_->publish(low_command_);
        publish_applied_locked();
      }
    }
    if (!deferred_fault.empty()) {
      latch_fault("WATCHDOG_OR_COMMAND", deferred_fault);
      publish_passive();
    }
  }

  void build_command_locked() {
    if (state_ == ControlState::kPassive || state_ == ControlState::kEstop) {
      set_passive_locked();
      return;
    }

    std::array<double, kJointCount> target{};
    std::array<double, kJointCount> velocity{};
    std::array<double, kJointCount> torque{};
    if (state_ == ControlState::kMoveToReady ||
        state_ == ControlState::kMoveToNeutral) {
      const bool ready = state_ == ControlState::kMoveToReady;
      const auto &destination = ready ? ready_position_ : neutral_position_;
      const double duration =
          ready ? ready_transition_seconds_ : neutral_transition_seconds_;
      const double elapsed =
          std::chrono::duration<double>(SteadyClock::now() - transition_started_)
              .count();
      if (elapsed > transition_timeout_seconds_) {
        fault_code_ = "TRANSITION_TIMEOUT";
        fault_message_ = "stance transition exceeded configured timeout";
        state_ = ControlState::kEstop;
        set_passive_locked();
        return;
      }
      const double blend = minimum_jerk(elapsed / duration);
      for (std::size_t i = 0; i < kJointCount; ++i) {
        target[i] = transition_start_position_[i] +
                    blend * (destination[i] - transition_start_position_[i]);
      }
      if (elapsed >= duration) {
        bool within_tolerance = true;
        for (std::size_t i = 0; i < kJointCount; ++i) {
          within_tolerance =
              within_tolerance &&
              std::abs(low_state_.motor_state[i].q - destination[i]) <=
                  position_tolerance_ &&
              std::abs(low_state_.motor_state[i].dq) <= velocity_tolerance_;
        }
        if (within_tolerance) {
          state_ = ready ? ControlState::kReadyHold
                         : ControlState::kNeutralHold;
        }
      }
    } else if (state_ == ControlState::kReadyHold) {
      target = ready_position_;
    } else if (state_ == ControlState::kNeutralHold) {
      target = neutral_position_;
    } else if (state_ == ControlState::kHoldCurrent) {
      target = hold_position_;
    } else if (state_ == ControlState::kPolicy) {
      if (!have_policy_action_) {
        target = ready_position_;
      } else {
        target = policy_action_.desired_position;
        velocity = policy_action_.desired_velocity;
        if (policy_action_.has_feedforward_torque) {
          torque = policy_action_.feedforward_torque;
        }
      }
    }

    const double step_seconds = 1.0 / command_rate_hz_;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double maximum_step = target_rate_limit_[i] * step_seconds;
      const double limited_target =
          std::clamp(target[i], last_target_[i] - maximum_step,
                     last_target_[i] + maximum_step);
      last_target_[i] = limited_target;
      auto &motor = low_command_.motor_cmd[i];
      motor.mode = kServoMotorMode;
      motor.q = static_cast<float>(limited_target);
      motor.dq = static_cast<float>(velocity[i]);
      motor.kp = static_cast<float>(kp_[i]);
      motor.kd = static_cast<float>(kd_[i]);
      motor.tau = static_cast<float>(torque[i]);
    }
    for (std::size_t i = kJointCount; i < low_command_.motor_cmd.size(); ++i) {
      auto &motor = low_command_.motor_cmd[i];
      motor.mode = kPassiveMotorMode;
      motor.q = kPositionStop;
      motor.dq = kVelocityStop;
      motor.kp = motor.kd = motor.tau = 0.0F;
    }
  }

  void set_passive_locked() {
    for (auto &motor : low_command_.motor_cmd) {
      motor.mode = kPassiveMotorMode;
      motor.q = kPositionStop;
      motor.dq = kVelocityStop;
      motor.kp = motor.kd = motor.tau = 0.0F;
    }
    if (have_low_state_) capture_measured_positions_locked(last_target_);
  }

  void publish_passive() {
    std::lock_guard<std::mutex> lock(mutex_);
    set_passive_locked();
    set_unitree_crc(low_command_);
    if (low_command_publisher_) low_command_publisher_->publish(low_command_);
  }

  void observation_tick() {
    msg::PolicyObservation observation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_low_state_) return;
      observation.header.stamp = now();
      observation.header.frame_id = "go2_body";
      observation.sequence = ++observation_sequence_;
      for (std::size_t i = 0; i < kJointCount; ++i) {
        observation.joint_position[i] = low_state_.motor_state[i].q;
        observation.joint_velocity[i] = low_state_.motor_state[i].dq;
      }
      for (std::size_t i = 0; i < 3; ++i) {
        observation.body_angular_velocity[i] =
            low_state_.imu_state.gyroscope[i];
      }
      try {
        const auto current = current_quaternion_locked();
        const auto relative =
            state_ == ControlState::kPolicy
                ? relative_quaternion_initial_frame(start_quaternion_, current)
                : QuaternionWxyz{{1.0, 0.0, 0.0, 0.0}};
        observation.relative_quaternion_wxyz = relative;
      } catch (const std::exception &error) {
        fault_code_ = "INVALID_QUATERNION";
        fault_message_ = error.what();
        state_ = ControlState::kEstop;
        return;
      }
      if (state_ == ControlState::kPolicy) {
        const double elapsed =
            std::chrono::duration<double>(SteadyClock::now() - policy_started_)
                .count();
        const double angle = 2.0 * kPi * elapsed / phase_period_seconds_;
        observation.phase_cos_sin = {{std::cos(angle), std::sin(angle)}};
      } else {
        observation.phase_cos_sin = {{1.0, 0.0}};
      }
    }
    observation_publisher_->publish(observation);
  }

  void publish_applied_locked() {
    msg::AppliedCommand message;
    message.header.stamp = now();
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const auto &motor = low_command_.motor_cmd[i];
      message.mode[i] = motor.mode;
      message.desired_position[i] = motor.q;
      message.desired_velocity[i] = motor.dq;
      message.kp[i] = motor.kp;
      message.kd[i] = motor.kd;
      message.feedforward_torque[i] = motor.tau;
    }
    applied_command_publisher_->publish(message);
  }

  void publish_status() {
    msg::SupervisorStatus message;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto current = SteadyClock::now();
    message.header.stamp = now();
    message.state = state_name(state_);
    message.hardware_mode = hardware_mode_;
    message.ownership_acknowledged = ownership_acknowledged_;
    message.low_state_healthy = healthy_low_state_locked();
    message.policy_healthy =
        have_policy_action_ &&
        std::chrono::duration<double>(current - last_policy_time_).count() <=
            policy_timeout_seconds_;
    message.estop_latched = state_ == ControlState::kEstop;
    message.fault_code = fault_code_;
    message.fault_message = fault_message_;
    message.low_state_age_ms =
        have_low_state_
            ? std::chrono::duration<double, std::milli>(
                  current - last_low_state_time_)
                  .count()
            : std::numeric_limits<double>::infinity();
    message.policy_age_ms =
        have_policy_action_
            ? std::chrono::duration<double, std::milli>(
                  current - last_policy_time_)
                  .count()
            : std::numeric_limits<double>::infinity();
    status_publisher_->publish(message);
  }

  std::mutex mutex_;
  ControlState state_{ControlState::kPassive};
  bool hardware_mode_{false};
  bool hardware_profile_complete_{false};
  bool ownership_acknowledged_{false};
  bool have_low_state_{false};
  bool have_policy_action_{false};
  double command_rate_hz_{};
  double policy_rate_hz_{};
  double phase_period_seconds_{};
  double low_state_timeout_seconds_{};
  double policy_timeout_seconds_{};
  double ready_transition_seconds_{};
  double neutral_transition_seconds_{};
  double transition_timeout_seconds_{};
  double position_tolerance_{};
  double velocity_tolerance_{};
  double quaternion_min_norm_{};
  double quaternion_max_norm_{};
  double shutdown_passive_seconds_{};
  int qos_depth_{};
  std::array<double, kJointCount> kp_{}, kd_{}, ready_position_{},
      neutral_position_{}, joint_position_min_{}, joint_position_max_{},
      desired_velocity_limit_{}, target_rate_limit_{},
      feedforward_torque_limit_{}, transition_start_position_{},
      hold_position_{}, last_target_{};
  QuaternionWxyz start_quaternion_{{1.0, 0.0, 0.0, 0.0}};
  std::string low_state_topic_, low_command_topic_, observation_topic_,
      policy_action_topic_, applied_command_topic_, status_topic_, estop_topic_;
  std::string fault_code_, fault_message_;
  uint64_t observation_sequence_{0};
  SteadyClock::time_point last_low_state_time_{}, last_policy_time_{},
      transition_started_{}, policy_started_{};
  unitree_go::msg::LowState low_state_;
  unitree_go::msg::LowCmd low_command_;
  msg::PolicyAction policy_action_;

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_command_publisher_;
  rclcpp::Publisher<msg::PolicyObservation>::SharedPtr observation_publisher_;
  rclcpp::Publisher<msg::AppliedCommand>::SharedPtr applied_command_publisher_;
  rclcpp::Publisher<msg::SupervisorStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr
      low_state_subscription_;
  rclcpp::Subscription<msg::PolicyAction>::SharedPtr policy_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ownership_service_,
      arm_service_, start_service_, stop_service_, recover_service_,
      estop_service_, reset_service_;
  rclcpp::TimerBase::SharedPtr command_timer_, observation_timer_, status_timer_;
};

}  // namespace go2_nn_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_nn_control::SafetySupervisor>());
  } catch (const std::exception &error) {
    std::cerr << "Safety supervisor refused to start: " << error.what()
              << std::endl;
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

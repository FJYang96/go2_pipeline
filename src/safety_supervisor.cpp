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
#include "go2_nn_control/policy/policy_manifest.hpp"
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
          if (message->policy_epoch != policy_epoch_) {
            return;
          }
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
    RCLCPP_INFO(get_logger(),
                "Safety supervisor initialized in PASSIVE mode "
                "(policy_rate=%.3f Hz, duration=%.3f s)",
                policy_rate_hz_, duration_s_);
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

  bool try_get_array_parameter(const std::string &name,
                               std::array<double, kJointCount> &destination) {
    rclcpp::Parameter parameter;
    if (!get_parameter(name, parameter)) {
      return false;
    }
    const auto values = parameter.get_value<std::vector<double>>();
    if (values.size() != kJointCount) {
      throw std::runtime_error(name + " must contain exactly 12 values");
    }
    std::copy(values.begin(), values.end(), destination.begin());
    for (const double value : destination) {
      if (!std::isfinite(value)) {
        throw std::runtime_error(name + " contains a non-finite value");
      }
    }
    return true;
  }

  void load_configuration() {
    hardware_mode_ = required_parameter<bool>("hardware_mode");
    hardware_profile_complete_ =
        required_parameter<bool>("hardware_profile_complete");
    ownership_acknowledged_ =
        required_parameter<bool>("ownership_acknowledged");
    command_rate_hz_ = required_parameter<double>("command_rate_hz");
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

    transition_kp_ = get_array_parameter<double>("transition_kp");
    transition_kd_ = get_array_parameter<double>("transition_kd");
    // YAML ready_position is retained as a schema fallback; active arming uses
    // the policy package ready pose loaded below.
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

    const auto policy_dir = required_parameter<std::string>("policy_dir");
    if (policy_dir.empty()) {
      throw std::runtime_error("policy_dir must be a non-empty path");
    }
    const auto manifest = PolicyManifest::Load(policy_dir);
    if (manifest.completion_behavior() !=
        CompletionBehavior::kMoveToNeutral) {
      throw std::runtime_error(
          "safety supervisor currently requires "
          "execution.completion_behavior: move_to_neutral");
    }
    if (!(manifest.control_dt() > 0.0) ||
        !(manifest.duration_seconds() > 0.0)) {
      throw std::runtime_error("policy control_dt and duration_s must be > 0");
    }
    policy_rate_hz_ = 1.0 / manifest.control_dt();
    duration_s_ = manifest.duration_seconds();
    policy_kp_ = manifest.policy_kp();
    policy_kd_ = manifest.policy_kd();
    ready_position_ =
        reorder_policy_to_unitree(manifest.ready_position_policy());

    std::array<double, kJointCount> policy_kp_max{};
    std::array<double, kJointCount> policy_kd_max{};
    const bool have_kp_max =
        try_get_array_parameter("policy_kp_max", policy_kp_max);
    const bool have_kd_max =
        try_get_array_parameter("policy_kd_max", policy_kd_max);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (have_kp_max && policy_kp_[i] > policy_kp_max[i]) {
        throw std::runtime_error(
            "policy kp exceeds policy_kp_max at joint " + std::to_string(i));
      }
      if (have_kd_max && policy_kd_[i] > policy_kd_max[i]) {
        throw std::runtime_error(
            "policy kd exceeds policy_kd_max at joint " + std::to_string(i));
      }
    }

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
        duration_s_ <= 0.0 || low_state_timeout_seconds_ <= 0.0 ||
        policy_timeout_seconds_ <= 0.0 || shutdown_passive_seconds_ < 0.0) {
      throw std::runtime_error("rates, duration, and watchdogs must be positive");
    }
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (joint_position_min_[i] >= joint_position_max_[i] ||
          transition_kp_[i] < 0.0 || transition_kd_[i] < 0.0 ||
          policy_kp_[i] < 0.0 || policy_kd_[i] < 0.0 ||
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

  void transition_to_locked(ControlState next_state,
                            const std::string &reason) {
    if (state_ == next_state) return;
    const ControlState previous_state = state_;
    state_ = next_state;
    RCLCPP_INFO(get_logger(), "State transition: %s -> %s (%s)",
                state_name(previous_state), state_name(next_state),
                reason.c_str());
  }

  void latch_fault_locked(const std::string &code,
                          const std::string &message) {
    if (state_ != ControlState::kEstop) {
      RCLCPP_ERROR(get_logger(), "Latched fault %s: %s", code.c_str(),
                   message.c_str());
    }
    fault_code_ = code;
    fault_message_ = message;
    transition_to_locked(ControlState::kEstop, code + ": " + message);
  }

  void log_service_result_locked(
      const char *service_name,
      const std_srvs::srv::Trigger::Response &response) const {
    if (response.success) {
      RCLCPP_INFO(get_logger(), "Service %s succeeded in state %s: %s",
                  service_name, state_name(state_), response.message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Service %s failed in state %s: %s",
                  service_name, state_name(state_), response.message.c_str());
    }
  }

  void begin_move_to_neutral_locked(const std::string &reason) {
    capture_measured_positions_locked(transition_start_position_);
    transition_started_ = SteadyClock::now();
    have_policy_action_ = false;
    transition_to_locked(ControlState::kMoveToNeutral, reason);
  }

  void create_services() {
    using Trigger = std_srvs::srv::Trigger;
    ownership_service_ = create_service<Trigger>(
        "/ws_control/acknowledge_ownership",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
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
          log_service_result_locked("/ws_control/acknowledge_ownership",
                                    *response);
        });
    arm_service_ = create_service<Trigger>(
        "/ws_control/arm",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kPassive) {
            response->success = false;
            response->message = "arm requires PASSIVE; current state is " +
                                std::string(state_name(state_));
            if (state_ == ControlState::kMoveToReady) {
              response->message +=
                  " (ready transition still in progress; wait for READY_HOLD "
                  "or ESTOP)";
            } else if (state_ == ControlState::kEstop) {
              response->message += " (call /ws_control/reset_estop first)";
            }
          } else if (!healthy_low_state_locked()) {
            response->success = false;
            response->message =
                have_low_state_
                    ? "arm requires a healthy lowstate (stale /lowstate)"
                    : "arm requires a healthy lowstate (no /lowstate yet)";
          } else if (!ownership_acknowledged_) {
            response->success = false;
            response->message = "arm requires ownership acknowledgment";
          } else if (count_publishers(low_command_topic_) > 1U) {
            response->success = false;
            response->message =
                "arm rejected because another ROS publisher is using " +
                low_command_topic_ + " (publisher count=" +
                std::to_string(count_publishers(low_command_topic_)) + ")";
          } else {
            capture_measured_positions_locked(transition_start_position_);
            transition_started_ = SteadyClock::now();
            transition_to_locked(ControlState::kMoveToReady,
                                 "/ws_control/arm accepted");
            response->success = true;
            response->message = "moving to ready stance";
          }
          log_service_result_locked("/ws_control/arm", *response);
        });
    start_service_ = create_service<Trigger>(
        "/ws_control/start_policy",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kReadyHold) {
            response->success = false;
            response->message =
                "policy start requires READY_HOLD; current state is " +
                std::string(state_name(state_));
            log_service_result_locked("/ws_control/start_policy", *response);
            return;
          }
          if (!healthy_low_state_locked()) {
            response->success = false;
            response->message = "policy start requires a healthy lowstate";
            log_service_result_locked("/ws_control/start_policy", *response);
            return;
          }
          try {
            start_quaternion_ = current_quaternion_locked();
          } catch (const std::exception &error) {
            response->success = false;
            response->message = error.what();
            log_service_result_locked("/ws_control/start_policy", *response);
            return;
          }
          ++policy_epoch_;
          policy_started_ = SteadyClock::now();
          have_policy_action_ = false;
          transition_to_locked(ControlState::kPolicy,
                               "/ws_control/start_policy accepted");
          response->success = true;
          response->message = "policy started; phase and orientation reset";
          log_service_result_locked("/ws_control/start_policy", *response);
        });
    stop_service_ = create_service<Trigger>(
        "/ws_control/stop_policy",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kPolicy) {
            response->success = false;
            response->message = "normal stop requires POLICY; current state is " +
                                std::string(state_name(state_));
            log_service_result_locked("/ws_control/stop_policy", *response);
            return;
          }
          capture_measured_positions_locked(hold_position_);
          have_policy_action_ = false;
          transition_to_locked(ControlState::kHoldCurrent,
                               "/ws_control/stop_policy accepted");
          response->success = true;
          response->message = "holding measured pose";
          log_service_result_locked("/ws_control/stop_policy", *response);
        });
    recover_service_ = create_service<Trigger>(
        "/ws_control/recover",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kHoldCurrent) {
            response->success = false;
            response->message =
                "recovery requires HOLD_CURRENT; current state is " +
                std::string(state_name(state_));
            log_service_result_locked("/ws_control/recover", *response);
            return;
          }
          begin_move_to_neutral_locked("/ws_control/recover accepted");
          response->success = true;
          response->message = "moving to neutral stance";
          log_service_result_locked("/ws_control/recover", *response);
        });
    estop_service_ = create_service<Trigger>(
        "/ws_control/estop",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          latch_fault_locked("MANUAL_ESTOP", "E-stop service called");
          response->success = true;
          response->message = "E-stop latched";
          log_service_result_locked("/ws_control/estop", *response);
        });
    reset_service_ = create_service<Trigger>(
        "/ws_control/reset_estop",
        [this](const std::shared_ptr<Trigger::Request>,
               std::shared_ptr<Trigger::Response> response) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != ControlState::kEstop) {
            response->success = false;
            response->message = "reset requires ESTOP; current state is " +
                                std::string(state_name(state_));
            log_service_result_locked("/ws_control/reset_estop", *response);
            return;
          }
          if (!healthy_low_state_locked()) {
            response->success = false;
            response->message = "reset requires a healthy lowstate";
            log_service_result_locked("/ws_control/reset_estop", *response);
            return;
          }
          fault_code_.clear();
          fault_message_.clear();
          transition_to_locked(ControlState::kPassive,
                               "/ws_control/reset_estop accepted");
          response->success = true;
          response->message = "fault cleared; full arm sequence required";
          log_service_result_locked("/ws_control/reset_estop", *response);
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
    latch_fault_locked(code, message);
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
        error = "policy produced a non-finite value at joint " +
                std::to_string(i) + " (q=" + std::to_string(q) +
                ", dq=" + std::to_string(dq) + ", tau=" + std::to_string(tau) +
                ")";
        return false;
      }
      if (q < joint_position_min_[i] || q > joint_position_max_[i]) {
        error = "policy desired_position exceeded joint_position limits at "
                "joint " +
                std::to_string(i) + " (q=" + std::to_string(q) + ", min=" +
                std::to_string(joint_position_min_[i]) + ", max=" +
                std::to_string(joint_position_max_[i]) +
                ", ref_index=" + std::to_string(policy_action_.reference_index) +
                ")";
        return false;
      }
      if (std::abs(dq) > desired_velocity_limit_[i]) {
        error = "policy desired_velocity exceeded desired_velocity_limit at "
                "joint " +
                std::to_string(i) + " (dq=" + std::to_string(dq) + ", limit=" +
                std::to_string(desired_velocity_limit_[i]) +
                ", ref_index=" + std::to_string(policy_action_.reference_index) +
                ")";
        return false;
      }
      if (std::abs(tau) > feedforward_torque_limit_[i]) {
        error = "policy feedforward_torque exceeded "
                "feedforward_torque_limit at joint " +
                std::to_string(i) + " (tau=" + std::to_string(tau) +
                ", limit=" + std::to_string(feedforward_torque_limit_[i]) +
                ", ref_index=" + std::to_string(policy_action_.reference_index) +
                ")";
        return false;
      }
    }
    return true;
  }

  std::string transition_progress_message_locked(
      const std::array<double, kJointCount> &destination) const {
    std::size_t worst_joint = 0;
    double worst_position_error = 0.0;
    double worst_velocity = 0.0;
    for (std::size_t i = 0; i < kJointCount; ++i) {
      const double position_error =
          std::abs(low_state_.motor_state[i].q - destination[i]);
      const double velocity = std::abs(low_state_.motor_state[i].dq);
      if (position_error > worst_position_error) {
        worst_position_error = position_error;
        worst_joint = i;
        worst_velocity = velocity;
      }
    }
    return "worst joint " + std::to_string(worst_joint) +
           " |q_err|=" + std::to_string(worst_position_error) +
           " (tol=" + std::to_string(position_tolerance_) +
           "), |dq|=" + std::to_string(worst_velocity) +
           " (tol=" + std::to_string(velocity_tolerance_) +
           "), measured_q=" +
           std::to_string(low_state_.motor_state[worst_joint].q) +
           ", destination_q=" + std::to_string(destination[worst_joint]);
  }

  void command_tick() {
    std::string deferred_fault;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (is_active(state_) && !healthy_low_state_locked()) {
        deferred_fault = "lowstate watchdog expired";
      } else if (state_ == ControlState::kPolicy) {
        const double elapsed =
            std::chrono::duration<double>(SteadyClock::now() - policy_started_)
                .count();
        if (elapsed >= duration_s_) {
          begin_move_to_neutral_locked("policy horizon completed");
        } else {
          std::string policy_error;
          if (!validate_policy_locked(policy_error)) {
            deferred_fault = policy_error;
          }
        }
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
        latch_fault_locked(
            "TRANSITION_TIMEOUT",
            std::string("stance transition exceeded configured timeout; ") +
                transition_progress_message_locked(destination));
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
          transition_to_locked(
              ready ? ControlState::kReadyHold : ControlState::kNeutralHold,
              ready ? "ready stance reached" : "neutral stance reached");
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

    const auto &kp =
        (state_ == ControlState::kPolicy) ? policy_kp_ : transition_kp_;
    const auto &kd =
        (state_ == ControlState::kPolicy) ? policy_kd_ : transition_kd_;

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
      motor.kp = static_cast<float>(kp[i]);
      motor.kd = static_cast<float>(kd[i]);
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
      observation.policy_active = (state_ == ControlState::kPolicy);
      observation.policy_epoch = policy_epoch_;
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
        latch_fault_locked("INVALID_QUATERNION", error.what());
        return;
      }
      if (state_ == ControlState::kPolicy) {
        const double elapsed =
            std::chrono::duration<double>(SteadyClock::now() - policy_started_)
                .count();
        observation.policy_time_seconds = elapsed;
        const double angle = 2.0 * kPi * elapsed / duration_s_;
        observation.phase_cos_sin = {{std::cos(angle), std::sin(angle)}};
      } else {
        observation.policy_time_seconds = 0.0;
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
  double duration_s_{};
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
  uint64_t policy_epoch_{0};
  std::array<double, kJointCount> transition_kp_{}, transition_kd_{},
      policy_kp_{}, policy_kd_{}, ready_position_{},
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

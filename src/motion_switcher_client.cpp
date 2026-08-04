#include "go2_nn_control/motion_switcher_client.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <thread>

#include "yaml-cpp/yaml.h"

namespace go2_nn_control {
namespace {

constexpr char kRequestTopic[] = "/api/motion_switcher/request";
constexpr char kResponseTopic[] = "/api/motion_switcher/response";

}  // namespace

MotionSwitcherClient::MotionSwitcherClient(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr response_group)
    : node_(node) {
  request_publisher_ = node_->create_publisher<unitree_api::msg::Request>(
      kRequestTopic, rclcpp::QoS(10));
  rclcpp::SubscriptionOptions options;
  options.callback_group = std::move(response_group);
  response_subscription_ =
      node_->create_subscription<unitree_api::msg::Response>(
          kResponseTopic, rclcpp::QoS(20),
          [this](unitree_api::msg::Response::SharedPtr message) {
            std::lock_guard<std::mutex> lock(response_mutex_);
            if (pending_request_id_ == 0 ||
                message->header.identity.id != pending_request_id_ ||
                message->header.identity.api_id != pending_api_id_) {
              return;
            }
            response_ = *message;
            response_ready_ = true;
            response_cv_.notify_one();
          },
          options);
}

bool MotionSwitcherClient::endpoints_ready() const {
  return node_->count_subscribers(kRequestTopic) > 0U &&
         node_->count_publishers(kResponseTopic) > 0U;
}

MotionSwitcherResult MotionSwitcherClient::acquire_ownership(
    std::chrono::milliseconds response_timeout,
    std::chrono::milliseconds verify_timeout,
    std::chrono::milliseconds poll_interval) {
  MotionSwitcherResult result;
  std::string form;
  std::string mode;
  std::string error;
  if (!check_mode(response_timeout, form, mode, error)) {
    result.message = "unable to check firmware motion mode: " + error;
    return result;
  }
  if (mode.empty()) {
    result.success = true;
    result.message = "hardware ownership verified; firmware motion mode was "
                     "already inactive";
    return result;
  }
  result.mode = mode;
  if (!release_mode(response_timeout, error)) {
    result.message = "failed to release firmware motion mode '" + mode +
                     "': " + error;
    return result;
  }

  result.release_accepted = true;
  const auto deadline = std::chrono::steady_clock::now() + verify_timeout;
  std::string last_error;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    const auto timeout = std::min(response_timeout, remaining);
    std::string verified_form;
    std::string verified_mode;
    std::string check_error;
    if (check_mode(timeout, verified_form, verified_mode, check_error)) {
      if (verified_mode.empty()) {
        result.success = true;
        result.message = "hardware ownership acquired; firmware motion mode '" +
                         mode + "' released and verified inactive";
        return result;
      }
      last_error = "firmware mode remains active as '" + verified_mode + "'";
    } else {
      last_error = check_error;
    }
    std::this_thread::sleep_for(poll_interval);
  }

  result.message =
      "firmware accepted ReleaseMode but deactivation was not verified";
  if (!last_error.empty()) result.message += ": " + last_error;
  result.message += "; firmware may already be released; retry ownership";
  return result;
}

MotionSwitcherResult MotionSwitcherClient::restore_firmware_control(
    const std::string &mode, std::chrono::milliseconds response_timeout,
    std::chrono::milliseconds verify_timeout,
    std::chrono::milliseconds poll_interval) {
  MotionSwitcherResult result;
  result.mode = mode;
  std::string error;
  if (!select_mode(mode, response_timeout, error)) {
    result.message = "failed to select firmware motion mode '" + mode +
                     "': " + error;
    return result;
  }

  const auto deadline = std::chrono::steady_clock::now() + verify_timeout;
  std::string last_error;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    const auto timeout = std::min(response_timeout, remaining);
    std::string form;
    std::string active_mode;
    std::string check_error;
    if (check_mode(timeout, form, active_mode, check_error)) {
      if (active_mode == mode) {
        result.success = true;
        result.message = "firmware motion mode '" + mode +
                         "' selected and verified active";
        return result;
      }
      last_error = "firmware reports mode '" + active_mode + "'";
    } else {
      last_error = check_error;
    }
    std::this_thread::sleep_for(poll_interval);
  }

  result.message = "firmware mode '" + mode +
                   "' was selected but not verified";
  if (!last_error.empty()) result.message += ": " + last_error;
  return result;
}

int64_t MotionSwitcherClient::next_request_id() {
  const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
  int64_t previous = request_sequence_.load();
  while (true) {
    const int64_t candidate = std::max<int64_t>(now, previous + 1);
    if (request_sequence_.compare_exchange_weak(previous, candidate)) {
      return candidate;
    }
  }
}

bool MotionSwitcherClient::call(int64_t api_id,
                                std::chrono::milliseconds timeout,
                                std::string &data, std::string &error,
                                const std::string &parameter) {
  std::lock_guard<std::mutex> call_lock(call_mutex_);
  if (!endpoints_ready()) {
    error = "motion-switcher request/response endpoints are not ready";
    return false;
  }

  unitree_api::msg::Request request;
  request.header.identity.id = next_request_id();
  request.header.identity.api_id = api_id;
  request.parameter = parameter;
  request.binary.clear();
  {
    std::lock_guard<std::mutex> response_lock(response_mutex_);
    pending_request_id_ = request.header.identity.id;
    pending_api_id_ = api_id;
    response_ready_ = false;
  }
  request_publisher_->publish(request);

  std::unique_lock<std::mutex> response_lock(response_mutex_);
  const bool received = response_cv_.wait_for(
      response_lock, timeout, [this] { return response_ready_; });
  if (!received) {
    pending_request_id_ = 0;
    pending_api_id_ = 0;
    error = "timed out waiting for motion-switcher API " +
            std::to_string(api_id) + " response";
    return false;
  }

  const auto response = response_;
  pending_request_id_ = 0;
  pending_api_id_ = 0;
  response_ready_ = false;
  if (response.header.status.code != 0) {
    error = "motion-switcher API " + std::to_string(api_id) +
            " returned status " +
            std::to_string(response.header.status.code);
    return false;
  }
  data = response.data;
  return true;
}

bool MotionSwitcherClient::check_mode(std::chrono::milliseconds timeout,
                                      std::string &form, std::string &name,
                                      std::string &error) {
  std::string data;
  if (!call(kCheckModeApiId, timeout, data, error)) return false;
  try {
    const YAML::Node response = YAML::Load(data);
    if (!response || !response.IsMap() || !response["name"] ||
        !response["name"].IsScalar()) {
      error = "motion-switcher CheckMode response is missing string field 'name'";
      return false;
    }
    name = response["name"].as<std::string>();
    form = response["form"] && response["form"].IsScalar()
               ? response["form"].as<std::string>()
               : std::string{};
  } catch (const std::exception &exception) {
    error = std::string("invalid motion-switcher CheckMode response: ") +
            exception.what();
    return false;
  }
  return true;
}

bool MotionSwitcherClient::release_mode(std::chrono::milliseconds timeout,
                                        std::string &error) {
  std::string ignored;
  return call(kReleaseModeApiId, timeout, ignored, error);
}

bool MotionSwitcherClient::select_mode(const std::string &mode,
                                       std::chrono::milliseconds timeout,
                                       std::string &error) {
  if (mode.empty() ||
      !std::all_of(mode.begin(), mode.end(), [](unsigned char character) {
        return std::isalnum(character) || character == '-' || character == '_';
      })) {
    error = "firmware motion mode contains unsupported characters";
    return false;
  }
  std::string ignored;
  return call(kSelectModeApiId, timeout, ignored, error,
              "{\"name\":\"" + mode + "\"}");
}

}  // namespace go2_nn_control

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

// ROS 2 Foxy treats any field named `header` as std_msgs/Header when compiling
// topic statistics. Unitree API messages use a different header without a
// timestamp, so provide the same compatibility specialization as Unitree's
// examples (for both directions used by the client tests).
namespace libstatistics_collector::topic_statistics_collector {
template <>
struct TimeStamp<unitree_api::msg::Request> {
  static std::pair<bool, int64_t> value(
      const unitree_api::msg::Request & /*message*/) {
    return std::make_pair(true, 0);
  }
};
template <>
struct TimeStamp<unitree_api::msg::Response> {
  static std::pair<bool, int64_t> value(
      const unitree_api::msg::Response & /*message*/) {
    return std::make_pair(true, 0);
  }
};
}  // namespace libstatistics_collector::topic_statistics_collector

namespace go2_nn_control {

struct MotionSwitcherResult {
  bool success{false};
  bool release_accepted{false};
  std::string mode;
  std::string message;
};

class MotionSwitcherClient {
 public:
  static constexpr int64_t kCheckModeApiId = 1001;
  static constexpr int64_t kSelectModeApiId = 1002;
  static constexpr int64_t kReleaseModeApiId = 1003;

  MotionSwitcherClient(rclcpp::Node *node,
                       rclcpp::CallbackGroup::SharedPtr response_group);

  bool endpoints_ready() const;
  MotionSwitcherResult acquire_ownership(
      std::chrono::milliseconds response_timeout,
      std::chrono::milliseconds verify_timeout,
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(250));
  MotionSwitcherResult restore_firmware_control(
      const std::string &mode, std::chrono::milliseconds response_timeout,
      std::chrono::milliseconds verify_timeout,
      std::chrono::milliseconds poll_interval = std::chrono::milliseconds(250));
  bool check_mode(std::chrono::milliseconds timeout, std::string &form,
                  std::string &name, std::string &error);
  bool release_mode(std::chrono::milliseconds timeout, std::string &error);
  bool select_mode(const std::string &mode, std::chrono::milliseconds timeout,
                   std::string &error);

 private:
  bool call(int64_t api_id, std::chrono::milliseconds timeout,
            std::string &data, std::string &error,
            const std::string &parameter = std::string{});
  int64_t next_request_id();

  rclcpp::Node *node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_publisher_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr
      response_subscription_;

  std::atomic<int64_t> request_sequence_{0};
  std::mutex call_mutex_;
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  int64_t pending_request_id_{0};
  int64_t pending_api_id_{0};
  bool response_ready_{false};
  unitree_api::msg::Response response_;
};

}  // namespace go2_nn_control

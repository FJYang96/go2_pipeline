#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "go2_nn_control/motion_switcher_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "yaml-cpp/yaml.h"

namespace go2_nn_control {
namespace {

using namespace std::chrono_literals;

class FakeMotionSwitcher : public rclcpp::Node {
 public:
  FakeMotionSwitcher() : Node("fake_motion_switcher") {
    response_publisher_ = create_publisher<unitree_api::msg::Response>(
        "/api/motion_switcher/response", 20);
    request_subscription_ = create_subscription<unitree_api::msg::Request>(
        "/api/motion_switcher/request", 10,
        [this](unitree_api::msg::Request::SharedPtr request) {
          unitree_api::msg::Response response;
          response.header.identity = request->header.identity;
          std::lock_guard<std::mutex> lock(mutex_);
          if (request->header.identity.api_id ==
              MotionSwitcherClient::kCheckModeApiId) {
            ++check_count_;
            if (mismatched_response_) ++response.header.identity.id;
            if (malformed_check_response_) {
              response.data = "{}";
            } else {
              if (select_seen_) {
                if (active_checks_after_select_ > 0) {
                  --active_checks_after_select_;
                } else {
                  mode_ = selected_mode_;
                  select_seen_ = false;
                }
              } else if (release_seen_) {
                if (active_checks_after_release_ > 0) {
                  --active_checks_after_release_;
                } else {
                  mode_.clear();
                  release_seen_ = false;
                }
              }
              response.data =
                  "{\"form\":\"0\",\"name\":\"" + mode_ + "\"}";
            }
          } else if (request->header.identity.api_id ==
                     MotionSwitcherClient::kSelectModeApiId) {
            ++select_count_;
            last_select_parameter_ = request->parameter;
            response.header.status.code = select_status_;
            if (select_status_ == 0) {
              const auto parameter = YAML::Load(request->parameter);
              selected_mode_ = parameter["name"].as<std::string>();
              select_seen_ = true;
            }
          } else if (request->header.identity.api_id ==
                     MotionSwitcherClient::kReleaseModeApiId) {
            ++release_count_;
            release_seen_ = true;
            response.header.status.code = release_status_;
          }
          response_publisher_->publish(response);
        });
  }

  void set_mode(const std::string &mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = mode;
  }
  void set_active_checks_after_release(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_checks_after_release_ = count;
  }
  void set_release_status(int32_t status) {
    std::lock_guard<std::mutex> lock(mutex_);
    release_status_ = status;
  }
  void set_select_status(int32_t status) {
    std::lock_guard<std::mutex> lock(mutex_);
    select_status_ = status;
  }
  void set_active_checks_after_select(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    active_checks_after_select_ = count;
  }
  void set_malformed_check_response(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    malformed_check_response_ = enabled;
  }
  void set_mismatched_response(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    mismatched_response_ = enabled;
  }
  int check_count() const { return check_count_.load(); }
  int release_count() const { return release_count_.load(); }
  int select_count() const { return select_count_.load(); }
  std::string last_select_parameter() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_select_parameter_;
  }

 private:
  mutable std::mutex mutex_;
  std::string mode_;
  int active_checks_after_release_{0};
  int active_checks_after_select_{0};
  int32_t release_status_{0};
  int32_t select_status_{0};
  bool release_seen_{false};
  bool select_seen_{false};
  bool malformed_check_response_{false};
  bool mismatched_response_{false};
  std::atomic<int> check_count_{0};
  std::atomic<int> release_count_{0};
  std::atomic<int> select_count_{0};
  std::string selected_mode_;
  std::string last_select_parameter_;
  rclcpp::Publisher<unitree_api::msg::Response>::SharedPtr response_publisher_;
  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr
      request_subscription_;
};

class MotionSwitcherClientTest : public ::testing::Test {
 protected:
  static void SetUpTestCase() {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestCase() { rclcpp::shutdown(); }

  void SetUp() override {
    client_node_ = std::make_shared<rclcpp::Node>("motion_switcher_test_client");
    fake_ = std::make_shared<FakeMotionSwitcher>();
    auto response_group = client_node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = std::make_unique<MotionSwitcherClient>(client_node_.get(),
                                                     response_group);
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = rclcpp::contexts::get_global_default_context();
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
        executor_options, 2U);
    executor_->add_node(client_node_);
    executor_->add_node(fake_);
    spin_thread_ = std::thread([this] { executor_->spin(); });
    for (int attempt = 0; attempt < 100 && !client_->endpoints_ready(); ++attempt) {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_TRUE(client_->endpoints_ready());
  }

  void TearDown() override {
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    client_.reset();
    fake_.reset();
    client_node_.reset();
    executor_.reset();
  }

  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<FakeMotionSwitcher> fake_;
  std::unique_ptr<MotionSwitcherClient> client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(MotionSwitcherClientTest, AlreadyInactiveIsIdempotent) {
  fake_->set_mode("");
  const auto result = client_->acquire_ownership(500ms, 500ms, 10ms);
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.release_accepted);
  EXPECT_EQ(fake_->release_count(), 0);
}

TEST_F(MotionSwitcherClientTest, ReleasesAndPollsUntilInactive) {
  fake_->set_mode("normal");
  fake_->set_active_checks_after_release(2);
  const auto result = client_->acquire_ownership(500ms, 1000ms, 10ms);
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.release_accepted);
  EXPECT_EQ(result.mode, "normal");
  EXPECT_EQ(fake_->release_count(), 1);
  EXPECT_GE(fake_->check_count(), 4);
}

TEST_F(MotionSwitcherClientTest, SelectsAndVerifiesFirmwareMode) {
  fake_->set_mode("");
  fake_->set_active_checks_after_select(2);
  const auto result =
      client_->restore_firmware_control("normal", 500ms, 1000ms, 10ms);
  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.mode, "normal");
  EXPECT_EQ(fake_->select_count(), 1);
  EXPECT_EQ(fake_->last_select_parameter(), "{\"name\":\"normal\"}");
}

TEST_F(MotionSwitcherClientTest, SelectModeFirmwareErrorFailsClosed) {
  fake_->set_select_status(37);
  const auto result =
      client_->restore_firmware_control("normal", 500ms, 500ms, 10ms);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("status 37"), std::string::npos);
}

TEST_F(MotionSwitcherClientTest, FirmwareErrorIsRetryableFailure) {
  fake_->set_mode("normal");
  fake_->set_release_status(42);
  const auto result = client_->acquire_ownership(500ms, 500ms, 10ms);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.release_accepted);
  EXPECT_NE(result.message.find("status 42"), std::string::npos);
}

TEST_F(MotionSwitcherClientTest, MalformedCheckResponseFailsClosed) {
  fake_->set_malformed_check_response(true);
  const auto result = client_->acquire_ownership(500ms, 500ms, 10ms);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(fake_->release_count(), 0);
  EXPECT_NE(result.message.find("missing string field 'name'"),
            std::string::npos);
}

TEST_F(MotionSwitcherClientTest, MismatchedResponseTimesOut) {
  fake_->set_mismatched_response(true);
  const auto result = client_->acquire_ownership(50ms, 100ms, 10ms);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("timed out"), std::string::npos);
}

}  // namespace
}  // namespace go2_nn_control

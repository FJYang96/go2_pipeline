#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace go2_nn_control {

class TerminalMode {
 public:
  TerminalMode() {
    if (!isatty(STDIN_FILENO)) return;
    if (tcgetattr(STDIN_FILENO, &original_) != 0) return;
    termios raw = original_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
  }
  ~TerminalMode() {
    if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &original_);
  }

 private:
  termios original_{};
  bool active_{false};
};

class KeyboardControl final : public rclcpp::Node {
 public:
  KeyboardControl() : Node("keyboard_control") {
    key_to_service_ = {
        {'o', "/ws_control/acknowledge_ownership"},
        {'a', "/ws_control/arm"},
        {'p', "/ws_control/start_policy"},
        {'s', "/ws_control/stop_policy"},
        {'r', "/ws_control/recover"},
        {'e', "/ws_control/estop"},
        {' ', "/ws_control/estop"},
        {'x', "/ws_control/reset_estop"},
    };
    for (const auto &entry : key_to_service_) {
      clients_[entry.second] =
          create_client<std_srvs::srv::Trigger>(entry.second);
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this] { poll_keyboard(); });
    std::cout << "\nGo2 controls: [o] ownership  [a] arm  [p] policy  "
                 "[s] stop/hold  [r] recover  [e/SPACE] E-STOP  [x] reset\n"
              << std::flush;
  }

 private:
  void poll_keyboard() {
    char key = '\0';
    const ssize_t count = ::read(STDIN_FILENO, &key, 1);
    if (count != 1) return;
    const auto service = key_to_service_.find(key);
    if (service == key_to_service_.end()) return;
    auto client = clients_.at(service->second);
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s is not ready", service->second.c_str());
      return;
    }
    auto future =
        client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    (void)future;
    RCLCPP_INFO(get_logger(), "sent key '%c' to %s", key,
                service->second.c_str());
  }

  TerminalMode terminal_mode_;
  std::map<char, std::string> key_to_service_;
  std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr>
      clients_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go2_nn_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_nn_control::KeyboardControl>());
  rclcpp::shutdown();
  return 0;
}

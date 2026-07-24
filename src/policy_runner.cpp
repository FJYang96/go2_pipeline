#include <memory>

#include "go2_nn_control/policy_runner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_nn_control::PolicyRunner>(
      go2_nn_control::hold_current_policy));
  rclcpp::shutdown();
  return 0;
}

#include <iostream>
#include <memory>

#include "go2_nn_control/policy_runner.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<go2_nn_control::PolicyRunner>());
  } catch (const std::exception &error) {
    std::cerr << "Policy runner refused to start: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}

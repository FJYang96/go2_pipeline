#include "gtest/gtest.h"
#include "go2_nn_control/unitree_crc.hpp"

namespace {

unitree_go::msg::LowCmd passive_command() {
  unitree_go::msg::LowCmd message;
  message.head[0] = 0xFE;
  message.head[1] = 0xEF;
  message.level_flag = 0xFF;
  for (auto &motor : message.motor_cmd) {
    motor.mode = go2_nn_control::kPassiveMotorMode;
    motor.q = go2_nn_control::kPositionStop;
    motor.dq = go2_nn_control::kVelocityStop;
    motor.kp = 0.0F;
    motor.kd = 0.0F;
    motor.tau = 0.0F;
  }
  return message;
}

TEST(UnitreeCrc, IsDeterministicAndCoversCommands) {
  auto first = passive_command();
  auto second = passive_command();
  go2_nn_control::set_unitree_crc(first);
  go2_nn_control::set_unitree_crc(second);
  EXPECT_NE(first.crc, 0U);
  EXPECT_EQ(first.crc, second.crc);

  second.motor_cmd[3].mode = go2_nn_control::kServoMotorMode;
  second.motor_cmd[3].q = 0.25F;
  go2_nn_control::set_unitree_crc(second);
  EXPECT_NE(first.crc, second.crc);
}

}  // namespace

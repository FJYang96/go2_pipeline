#pragma once

#include <cstdint>

#include "unitree_go/msg/low_cmd.hpp"

namespace go2_nn_control {

constexpr float kPositionStop = 2.146E+9F;
constexpr float kVelocityStop = 16000.0F;
constexpr uint8_t kPassiveMotorMode = 0x00;
constexpr uint8_t kServoMotorMode = 0x01;

uint32_t crc32_core(uint32_t *words, uint32_t length);
void set_unitree_crc(unitree_go::msg::LowCmd &message);

}  // namespace go2_nn_control

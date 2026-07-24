#include "go2_nn_control/unitree_crc.hpp"

#include <array>
#include <cstring>

namespace go2_nn_control {
namespace {

struct RawBmsCommand {
  uint8_t off;
  std::array<uint8_t, 3> reserve;
};

struct RawMotorCommand {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  std::array<uint32_t, 3> reserve;
};

struct RawLowCommand {
  std::array<uint8_t, 2> head;
  uint8_t level_flag;
  uint8_t frame_reserve;
  std::array<uint32_t, 2> serial_number;
  std::array<uint32_t, 2> version;
  uint16_t bandwidth;
  std::array<RawMotorCommand, 20> motor_command;
  RawBmsCommand bms;
  std::array<uint8_t, 40> wireless_remote;
  std::array<uint8_t, 12> led;
  std::array<uint8_t, 2> fan;
  uint8_t gpio;
  uint32_t reserve;
  uint32_t crc;
};

}  // namespace

uint32_t crc32_core(uint32_t *words, uint32_t length) {
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < length; ++i) {
    uint32_t bit = 1U << 31;
    const uint32_t data = words[i];
    for (uint32_t count = 0; count < 32; ++count) {
      crc = (crc & 0x80000000U) ? (crc << 1U) ^ polynomial : crc << 1U;
      if (data & bit) crc ^= polynomial;
      bit >>= 1U;
    }
  }
  return crc;
}

void set_unitree_crc(unitree_go::msg::LowCmd &message) {
  RawLowCommand raw{};
  std::memcpy(raw.head.data(), message.head.data(), 2);
  raw.level_flag = message.level_flag;
  raw.frame_reserve = message.frame_reserve;
  std::memcpy(raw.serial_number.data(), message.sn.data(), 8);
  std::memcpy(raw.version.data(), message.version.data(), 8);
  raw.bandwidth = message.bandwidth;
  for (std::size_t i = 0; i < raw.motor_command.size(); ++i) {
    auto &destination = raw.motor_command[i];
    const auto &source = message.motor_cmd[i];
    destination.mode = source.mode;
    destination.q = source.q;
    destination.dq = source.dq;
    destination.tau = source.tau;
    destination.kp = source.kp;
    destination.kd = source.kd;
    std::memcpy(destination.reserve.data(), source.reserve.data(), 12);
  }
  raw.bms.off = message.bms_cmd.off;
  std::memcpy(raw.bms.reserve.data(), message.bms_cmd.reserve.data(), 3);
  std::memcpy(raw.wireless_remote.data(), message.wireless_remote.data(), 40);
  std::memcpy(raw.led.data(), message.led.data(), 12);
  std::memcpy(raw.fan.data(), message.fan.data(), 2);
  raw.gpio = message.gpio;
  raw.reserve = message.reserve;
  raw.crc = crc32_core(reinterpret_cast<uint32_t *>(&raw),
                       (sizeof(RawLowCommand) / sizeof(uint32_t)) - 1);
  message.crc = raw.crc;
}

}  // namespace go2_nn_control

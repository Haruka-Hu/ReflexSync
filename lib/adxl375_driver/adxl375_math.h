#pragma once
#include <stdint.h>

namespace adxl375 {
constexpr uint8_t kDeviceId = 0xE5;
constexpr uint8_t kRate1600 = 0x0E;
constexpr uint8_t kReadBit = 0x80;
constexpr uint8_t kMultiBit = 0x40;

inline int16_t decodeAxis(uint8_t lo, uint8_t hi) {
  return static_cast<int16_t>(static_cast<uint16_t>(lo) |
                              (static_cast<uint16_t>(hi) << 8));
}
}  // namespace adxl375

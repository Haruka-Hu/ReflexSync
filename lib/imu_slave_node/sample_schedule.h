#pragma once
#include <stdint.h>

namespace imu_sampling {
struct Tick {
  bool due;
  uint32_t missed;
  uint32_t nextUs;
};

// A delayed task reads the latest value once, rather than inventing catch-up samples.
inline Tick advance(uint32_t nowUs, uint32_t nextUs, uint32_t periodUs) {
  const int32_t lateUs = static_cast<int32_t>(nowUs - nextUs);
  if (lateUs < 0) return {false, 0, nextUs};
  const uint32_t missed = static_cast<uint32_t>(lateUs) / periodUs;
  return {true, missed, nextUs + (missed + 1U) * periodUs};
}
}  // namespace imu_sampling

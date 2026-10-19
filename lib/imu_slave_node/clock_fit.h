#pragma once
#include <algorithm>
#include <cmath>
#include <stdint.h>

namespace imu_clock {
constexpr unsigned kCapacity = 128;
struct Point { uint32_t localUs; uint32_t masterUs; };
struct Model {
  bool valid = false;
  double slope = 1;
  double intercept = 0;
  double residualUs = 0;
};

// Input is an immutable snapshot in reception order. No locks are needed for the math.
inline Model linear(const Point *points, unsigned count, const bool *include = nullptr) {
  Model result;
  if (count < 2) return result;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  unsigned used = 0;
  for (unsigned i = 0; i < count; ++i) {
    if (include && !include[i]) continue;
    // A reply midpoint can precede a just-received beacon by a few microseconds.
    // Unsigned subtraction would turn that into a false 71-minute separation.
    const double x = static_cast<int32_t>(points[i].localUs - points[0].localUs);
    const double y = static_cast<int32_t>(points[i].masterUs - points[0].masterUs);
    sx += x; sy += y; sxx += x * x; sxy += x * y; ++used;
  }
  const double denominator = used * sxx - sx * sx;
  if (used < 2 || denominator == 0) return result;
  result.slope = (used * sxy - sx * sy) / denominator;
  result.intercept = points[0].masterUs + (sy - result.slope * sx) / used -
                     result.slope * points[0].localUs;
  double residualSq = 0;
  for (unsigned i = 0; i < count; ++i) {
    if (include && !include[i]) continue;
    const double error = result.slope * points[i].localUs + result.intercept - points[i].masterUs;
    residualSq += error * error;
  }
  result.residualUs = std::sqrt(residualSq / used);
  result.valid = true;
  return result;
}

inline Model robust(const Point *points, unsigned count, const Model &previous) {
  if (count > kCapacity) return {};
  Model result = linear(points, count);
  if (!result.valid || count < 8) return result;
  double residual[kCapacity] = {}, sorted[kCapacity] = {};
  bool include[kCapacity] = {};
  for (unsigned i = 0; i < count; ++i) {
    residual[i] = std::fabs(result.slope * points[i].localUs + result.intercept - points[i].masterUs);
    sorted[i] = residual[i];
  }
  std::sort(sorted, sorted + count);
  const unsigned keep = std::min(count, std::max(8U, count * 80U / 100U));
  const double limit = std::min(sorted[keep - 1], 5000.0);
  unsigned included = 0;
  for (unsigned i = 0; i < count; ++i) {
    include[i] = residual[i] <= limit;
    included += include[i];
  }
  if (included < 8) return result;
  Model refined = linear(points, count, include);
  if (!refined.valid) return result;
  if (previous.valid) {
    double sx = 0, sy = 0;
    for (unsigned i = 0; i < count; ++i) {
      if (!include[i]) continue;
      sx += static_cast<int32_t>(points[i].localUs - points[0].localUs);
      sy += static_cast<int32_t>(points[i].masterUs - points[0].masterUs);
    }
    const double cx = points[0].localUs + sx / included;
    const double cy = points[0].masterUs + sy / included;
    refined.slope = 0.8 * previous.slope + 0.2 * refined.slope;
    refined.intercept = cy - refined.slope * cx;
  }
  return refined;
}

inline bool plausible(const Model &model) {
  // Reject startup fits that imply oscillator errors greater than 0.5%.
  return model.valid && std::isfinite(model.slope) && std::isfinite(model.intercept) &&
         std::fabs(model.slope - 1.0) <= 0.005;
}
}  // namespace imu_clock

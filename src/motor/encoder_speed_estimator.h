#pragma once
#include <math.h>
#include <stdint.h>

// Reciprocal speed over four same-direction edges, with startup/reversal fallback.
inline float reciprocalEncoderSpeed(float countSpeed, float countsPerDegree,
        uint32_t fourEdgeUs, uint32_t lastEdgeAgeUs, int direction, bool valid) {
    if (!valid || !fourEdgeUs || countsPerDegree <= 0) return countSpeed;
    if (lastEdgeAgeUs >= 30000U) return 0;
    const float periodSpeed = 4.0e6f / (countsPerDegree * fourEdgeUs);
    const float ageBound = lastEdgeAgeUs
        ? 1.5e6f / (countsPerDegree * lastEdgeAgeUs) : periodSpeed;
    return direction * fminf(periodSpeed, ageBound);
}

inline unsigned encoderSpeedSelfTest() {
    unsigned fail = 0;
    if (fabsf(reciprocalEncoderSpeed(0, 4.8f, 10417, 1000, 1, true) - 80) > 0.1f) fail |= 1;
    if (fabsf(reciprocalEncoderSpeed(0, 4.8f, 10417, 1000, -1, true) + 80) > 0.1f) fail |= 2;
    if (reciprocalEncoderSpeed(23, 4.8f, 0, 0, 1, false) != 23) fail |= 4;
    if (reciprocalEncoderSpeed(80, 4.8f, 10417, 30000, 1, true) != 0) fail |= 8;
    if (fabsf(reciprocalEncoderSpeed(0, 4.8f, 10417, 10000, 1, true) - 31.25f) > 0.01f) fail |= 16;
    return fail;
}

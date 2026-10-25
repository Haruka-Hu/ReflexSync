#pragma once
#include <stdint.h>

// PCNT resets to zero at either configured limit, rather than wrapping int16.
// Sample every 1ms; a delta must be less than half the limit (3125 degrees).
// At the GUI maximum 3000 deg/s even a 50ms control gap is only 720 counts.
constexpr int16_t EncoderPcntLimit = 30000;
inline int32_t pcntDelta(int16_t current, int16_t previous) {
    int32_t delta = int32_t(current) - previous;
    if (delta > EncoderPcntLimit/2) delta -= EncoderPcntLimit;
    else if (delta < -EncoderPcntLimit/2) delta += EncoderPcntLimit;
    return delta;
}

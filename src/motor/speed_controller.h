#pragma once
#include <math.h>
#include "control_config.h"

struct SpeedTuning {
    float kp = HammerConfig::SpeedKp, ki = HammerConfig::SpeedKi;
    float ks = HammerConfig::FeedforwardKs, kv = HammerConfig::FeedforwardKv;
    float kg = HammerConfig::FeedforwardKg;
    bool valid() const {
        return isfinite(kp) && isfinite(ki) && isfinite(ks) && isfinite(kv) && isfinite(kg)
            && kp >= 0 && kp <= 100 && ki >= 0 && ki <= 1000
            && fabsf(ks) <= 1000 && kv >= 0 && kv <= 10 && fabsf(kg) <= 1000;
    }
};

class SpeedController {
public:
    void reset() { integral_ = 0; }
    float update(float target, float actual, float angle, float dt, const SpeedTuning& p) {
        const float err = target - actual;
        const float ff = p.ks + p.kv * target + p.kg * sinf(angle * .01745329252f);
        const float raw = ff + p.kp * err + integral_;
        // Conditional integration allows recovery from either output rail.
        const bool nearTarget = fabsf(err) <= fmaxf(20.0f, fabsf(target)*.2f);
        const bool unwinding = integral_ * err < 0;
        if ((nearTarget || unwinding) && (fabsf(raw) <= HammerConfig::MaxPwm || raw * err < 0)) {
            integral_ = fmaxf(-HammerConfig::SpeedIntegralLimit,
                fminf(HammerConfig::SpeedIntegralLimit, integral_ + p.ki * err * dt));
        }
        return fmaxf(-HammerConfig::MaxPwm,
            fminf(HammerConfig::MaxPwm, ff + p.kp * err + integral_));
    }
    float integral() const { return integral_; }
private:
    float integral_ = 0;
};

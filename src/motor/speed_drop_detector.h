#pragma once
#include <stdint.h>
#include <math.h>
#include "control_config.h"

// Encoder-only contact candidate. Thresholds are bench hypotheses, not force limits.
// A short position window avoids differentiating individual quantized single-period speeds.
class SpeedDropDetector {
public:
    struct Config {
        uint32_t windowUs = HammerConfig::DropWindowUs;
        uint32_t peakAgeUs = HammerConfig::DropPeakAgeUs;
        uint32_t armUs = HammerConfig::DropArmUs;
        float minSpeed = HammerConfig::DropMinSpeed;
        float absoluteDrop = HammerConfig::DropAbsolute;
        float relativeDrop = HammerConfig::DropRelative;
    };

    explicit SpeedDropDetector(const Config& cfg) : cfg_(cfg) { reset(); }
    void reset() {
        head_ = size_ = 0;
        armed_ = latched_ = building_ = false;
        fastSince_ = 0;
        speed_ = baseline_ = 0;
    }
    bool update(float angle, uint32_t now, bool allowArm = true,
                bool allowTrigger = true, float baselineCeiling = INFINITY,
                float triggerSpeedCeiling = INFINITY, float feedbackSpeed = NAN) {
        if (latched_) return true;
        if (!isfinite(angle)) return false;
        bool valid = false;
        float measured = 0;
        // Search from newest to oldest for the shortest available full window.
        for (unsigned n = 0; n < size_; ++n) {
            const Sample& old = samples_[(head_ + Capacity - 1 - n) % Capacity];
            const uint32_t age = now - old.time;
            if (age >= cfg_.windowUs) {
                if (age <= cfg_.windowUs + 10000U) {
                    measured = (angle - old.angle) * 1000000.0f / age;
                    valid = true;
                }
                break;
            }
        }
        speed_ = measured;
        baseline_ = valid ? measured : 0;
        for (unsigned n = 0; valid && n < size_; ++n) {
            const Sample& old = samples_[(head_ + Capacity - 1 - n) % Capacity];
            if (now - old.time > cfg_.peakAgeUs) break;
            if (old.valid && old.speed > baseline_) baseline_ = old.speed;
        }
        baseline_ = fminf(baseline_, baselineCeiling);
        if (valid && !armed_) {
            if (allowArm && measured >= cfg_.minSpeed) {
                if (!building_) { fastSince_ = now; building_ = true; }
                if (now - fastSince_ >= cfg_.armUs) armed_ = true;
            } else {
                building_ = false;
            }
        }
        if (allowTrigger && valid && armed_ && measured <= triggerSpeedCeiling && baseline_ >= cfg_.minSpeed &&
            baseline_ - measured >= cfg_.absoluteDrop &&
            baseline_ - measured >= baseline_ * cfg_.relativeDrop &&
            (!isfinite(feedbackSpeed) ||
             (baseline_ - feedbackSpeed >= cfg_.absoluteDrop &&
              baseline_ - feedbackSpeed >= baseline_ * cfg_.relativeDrop))) {
            latched_ = true;
        }
        samples_[head_] = {now, angle, measured, valid};
        head_ = (head_ + 1) % Capacity;
        if (size_ < Capacity) ++size_;
        return latched_;
    }
    bool armed() const { return armed_; }
    float speed() const { return speed_; }
    float baseline() const { return baseline_; }

private:
    struct Sample { uint32_t time; float angle; float speed; bool valid; };
    static constexpr unsigned Capacity = 64;
    Config cfg_;
    Sample samples_[Capacity]{};
    unsigned head_, size_;
    uint32_t fastSince_;
    bool armed_, latched_, building_;
    float speed_, baseline_;
};

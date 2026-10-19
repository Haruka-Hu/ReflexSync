#pragma once
#include <Arduino.h>
#include "drv8871_motor.h"
#include "hall_encoder.h"
#include "speed_drop_detector.h"
#include "speed_controller.h"

class MotionController {
public:
    enum class State : uint8_t { Idle, Retract, Ready, Strike };
    struct Settings {
        float retractAngleDeg = HammerConfig::DefaultRetractAngle;
        float strikeSpeedDps = HammerConfig::DefaultStrikeSpeed;
    };
    struct TraceSample {
        uint32_t timeUs;
        float angle, speed, rawSpeed, pwm, dropSpeed, baseline;
    };
    MotionController(Drv8871Motor& motor, HallEncoder& encoder);
    void begin();
    void update();
    bool zeroHere();
    bool home();
    bool jog(float signedSpeed);
    bool homing() const { return homing_; }
    bool setSettings(const Settings& settings);
    bool setTuning(const SpeedTuning& tuning);
    bool retract();
    bool start();
    bool fire();
    void stop();
    State state() const { return state_; }
    bool zeroed() const { return zeroed_; }
    const char* lastReason() const { return lastReason_; }
    float positionError() const { return targetAngle_ - angle(); }
    uint32_t settledMs() const {
        return state_==State::Retract && !homing_ && !jogging_ && stableSinceMs_ ? millis()-stableSinceMs_ : 0;
    }
    uint32_t phaseMs() const { return state_==State::Retract || state_==State::Strike ? millis()-phaseStartMs_ : 0; }
    bool autoFire() const { return autoFire_; }
    float angle() const { return encoder_.getAngleDeg(); }
    float speed() const { return encoder_.getSpeedDegPerSec(); }
    float rawSpeed() const { return encoder_.getRawSpeedDegPerSec(); }
    float pwm() const { return appliedPwm_; }
    const Settings& settings() const { return settings_; }
    const SpeedTuning& tuning() const { return tuning_; }
    unsigned traceCount() const { return traceCount_; }
    unsigned shotId() const { return shotId_; }
    float traceTarget() const { return traceTarget_; }
    const Settings& traceSettings() const { return traceSettings_; }
    const SpeedTuning& traceTuning() const { return traceTuning_; }
    const char* traceReason() const { return traceReason_; }
    bool traceContact() const { return traceContact_; }
    const TraceSample& trace(unsigned index) const { return trace_[index]; }
    static const char* stateName(State state);
private:
    Drv8871Motor& motor_;
    HallEncoder& encoder_;
    Settings settings_{};
    SpeedTuning tuning_{};
    SpeedController speedController_;
    State state_ = State::Idle;
    bool zeroed_ = false, autoFire_ = false, homing_ = false, jogging_ = false;
    uint32_t jogRefreshMs_ = 0;
    float jogTarget_ = 0;
    float targetAngle_ = 0, holdIntegral_ = 0, appliedPwm_ = 0;
    uint32_t lastUpdateUs_ = 0, phaseStartMs_ = 0, strikeStartUs_ = 0;
    uint32_t stableSinceMs_ = 0, lastProgressMs_ = 0;
    uint32_t lastReverseUs_ = 0;
    bool reversedThisStrike_ = false;
    int32_t lastProgressCount_ = 0, homeLastCount_ = 0;
    uint32_t homeStillSinceMs_ = 0;
    float homeIntegral_ = 0, homeStartAngle_ = 0, homePeakSpeed_ = 0;
    SpeedDropDetector dropDetector_;
    TraceSample trace_[HammerConfig::TraceCapacity]{};
    unsigned traceCount_ = 0, shotId_ = 0;
    float traceTarget_ = 0;
    Settings traceSettings_{};
    SpeedTuning traceTuning_{};
    const char* traceReason_ = "NONE";
    const char* lastReason_ = "NONE";
    bool traceContact_ = false;
    float holdControl(float position, float velocity, float dt);
    void recordTrace(uint32_t nowUs);
    void drive(float signedPwm);
    void endStrike(const char* reason, bool contactCandidate);
    void abortMotion(const char* reason);
};

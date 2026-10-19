#include "motion_controller.h"
#include "host_output.h"
#include <math.h>
using namespace HammerConfig;

MotionController::MotionController(Drv8871Motor& m, HallEncoder& e)
    : motor_(m), encoder_(e), dropDetector_(SpeedDropDetector::Config{}) {}
void MotionController::begin() {
    motor_.begin(); encoder_.begin(); motor_.stop(); lastUpdateUs_ = micros();
}
const char* MotionController::stateName(State s) {
    switch (s) {
        case State::Idle: return "IDLE";
        case State::Retract: return "RETRACT";
        case State::Ready: return "READY";
        case State::Strike: return "STRIKE";
    }
    return "UNKNOWN";
}
bool MotionController::zeroHere() {
    if (state_ != State::Idle || fabsf(speed()) > StillSpeed) return false;
    encoder_.reset(); zeroed_ = true; autoFire_ = false; homing_ = false; lastReason_="ZEROED";
    speedController_.reset(); holdIntegral_ = 0; dropDetector_.reset();
    return true;
}
bool MotionController::home() {
    if (state_ != State::Idle || fabsf(speed()) > StillSpeed) return false;
    autoFire_ = false; homing_ = true; zeroed_ = false; lastReason_="HOMING";
    phaseStartMs_ = millis(); homeStillSinceMs_ = phaseStartMs_;
    homeLastCount_ = encoder_.getCount(); homeStartAngle_ = angle();
    homeIntegral_ = 0; homePeakSpeed_ = 0;
    state_ = State::Retract; // same four states; HOME is an approach mode
    hostPrintf("OK HOME speed=%.1f forward_cap=%.0f reverse_cap=%.0f\n",
               HomeSpeed, HomeForwardPwm, HomeReversePwm);
    return true;
}
bool MotionController::jog(float signedSpeed) {
    if (!isfinite(signedSpeed) || fabsf(signedSpeed)<1 || fabsf(signedSpeed)>80) return false;
    if (!jogging_) {
        if (state_!=State::Idle || fabsf(speed())>StillSpeed) return false;
        homing_=false; autoFire_=false; zeroed_=false; homeIntegral_=0;
        phaseStartMs_=millis(); state_=State::Retract; jogging_=true; lastReason_="JOGGING";
    }
    jogTarget_=signedSpeed; jogRefreshMs_=millis();
    hostPrintf("OK JOG speed=%.1f\n",signedSpeed);
    return true;
}
bool MotionController::setSettings(const Settings& s) {
    if (state_ != State::Idle || !isfinite(s.retractAngleDeg) || !isfinite(s.strikeSpeedDps)
        || s.retractAngleDeg <= 0 || s.strikeSpeedDps <= 0) return false;
    settings_ = s; return true;
}
bool MotionController::setTuning(const SpeedTuning& p) {
    if (state_ != State::Idle || !p.valid()) return false;
    tuning_ = p; speedController_.reset(); return true;
}
bool MotionController::retract() {
    if (state_ != State::Idle || !zeroed_ || fabsf(speed()) > StillSpeed) return false;
    targetAngle_ = -settings_.retractAngleDeg;
    speedController_.reset(); holdIntegral_ = 0; autoFire_ = false;
    phaseStartMs_ = millis(); stableSinceMs_ = 0;
    lastProgressMs_ = phaseStartMs_; lastProgressCount_ = encoder_.getCount();
    state_ = State::Retract; lastReason_="RETRACTING";
    hostPrintf("OK RETRACT target=%.2f\n", targetAngle_);
    return true;
}
bool MotionController::start() {
    if (!retract()) return false;
    autoFire_ = true;
    hostPrintf("OK START\n"); return true;
}
bool MotionController::fire() {
    if (state_ != State::Ready || !zeroed_ || fabsf(targetAngle_ - angle()) > AngleTolerance
        || fabsf(speed()) > StillSpeed) return false;
    speedController_.reset(); autoFire_ = false;
    traceCount_ = 0; ++shotId_; traceTarget_ = settings_.strikeSpeedDps;
    traceSettings_ = settings_; traceTuning_ = tuning_;
    traceReason_ = "IN_PROGRESS"; traceContact_ = false;
    phaseStartMs_ = millis(); strikeStartUs_ = micros();
    dropDetector_.reset(); reversedThisStrike_ = false; state_ = State::Strike; lastReason_="STRIKING";
    hostPrintf("OK FIRE shot=%u target_speed=%.2f\n", shotId_, traceTarget_);
    return true;
}
void MotionController::drive(float pwm) {
    if (!isfinite(pwm)) { abortMotion("INVALID_PWM"); return; }
    const float cap = fminf(MaxPwm, float(motor_.maxPwm()));
    const float limited = constrain(pwm, -cap, cap);
    const int duty = int(fabsf(limited));
    appliedPwm_ = duty < 5 ? 0 : (limited > 0 ? float(duty) : -float(duty));
    if (duty < 5) motor_.stop();
    else if (limited > 0) motor_.driveForward(duty);
    else motor_.driveReverse(duty);
}
float MotionController::holdControl(float pos, float vel, float dt) {
    const float err = targetAngle_ - pos;
    // Preserve the calibrated approach effort, but taper it continuously near
    // zero error. The same damping and integral remain active in READY.
    const float effort = PositionFrictionPwm + (err < 0 ? PositionRetractPerDeg * fmaxf(0, -pos) : 0);
    const float friction = effort * constrain(err / PositionFrictionWidthDeg, -1.0f, 1.0f);
    const float base = PositionKp * err - PositionKd * vel + friction;
    const float raw = base + holdIntegral_;
    const bool nearAndSlow = fabsf(err) <= PositionIntegralZoneDeg && fabsf(vel) <= PositionIntegralSpeed;
    const bool unwinding = holdIntegral_ * err < 0;
    if ((nearAndSlow || unwinding) && (fabsf(raw) <= MaxPwm || raw * err < 0)) {
        holdIntegral_ = constrain(holdIntegral_ + (fabsf(vel) < 5 ? PositionStaticKi : PositionKi) * err * dt,
                                  -PositionIntegralLimit, PositionIntegralLimit);
    }
    return constrain(base + holdIntegral_, -MaxPwm, MaxPwm);
}
void MotionController::recordTrace(uint32_t nowUs) {
    if (traceCount_ < TraceCapacity)
        trace_[traceCount_++] = {nowUs - strikeStartUs_, angle(), speed(), rawSpeed(),
                                 appliedPwm_, dropDetector_.speed(), dropDetector_.baseline()};
}
void MotionController::endStrike(const char* reason, bool candidate) {
    motor_.stop(); appliedPwm_ = 0; state_ = State::Idle; autoFire_ = false;
    traceReason_ = reason; traceContact_ = candidate; lastReason_=reason;
    // Normal completion keeps the absolute encoder zero; STOP/aborts invalidate it.
    recordTrace(micros());
    hostPrintf("EVT SHOT reason=%s contact_candidate=%u shot=%u target_speed=%.2f "
               "release_angle=%.2f release_speed=%.2f samples=%u zeroed=%u\n",
               reason, unsigned(candidate), shotId_, traceTarget_, angle(), speed(),
               traceCount_, unsigned(zeroed_));
}
void MotionController::abortMotion(const char* reason) {
    const bool wasStrike = state_ == State::Strike; lastReason_=reason;
    motor_.brake(); appliedPwm_ = 0; state_ = State::Idle;
    zeroed_ = false; autoFire_ = false; homing_ = false; jogging_ = false;
    if (wasStrike) { traceReason_ = reason; traceContact_ = false; recordTrace(micros()); }
    hostPrintf("EVT ABORT reason=%s angle=%.2f speed=%.2f shot=%u samples=%u\n",
               reason, angle(), speed(), shotId_, traceCount_);
}
void MotionController::stop() { abortMotion("USER_STOP"); }
void MotionController::update() {
    const uint32_t nowUs = micros(), elapsed = nowUs - lastUpdateUs_;
    #ifdef ARDUINO_ARCH_ESP32
    // Timer wakeups have microsecond jitter; do not skip a nominal 1ms tick
    // merely because it arrived at 999us. Always use the measured dt below.
    if (elapsed < ControlPeriodUs / 2) return;
#else
    if (elapsed < ControlPeriodUs) return;
#endif
    lastUpdateUs_ = nowUs;
    const float dt = elapsed * 1e-6f;
    encoder_.updateSpeed(dt);
    const uint32_t nowMs = millis();
    const float pos = angle(), measured = speed();
    if (state_ == State::Idle) return;
    if (elapsed > MaxControlGapUs || !isfinite(pos) || !isfinite(measured)) {
        abortMotion("CONTROL_OR_SENSOR"); return;
    }
    if (jogging_) {
        // Press-and-hold command lease. No heartbeat means release, not a fault state.
        if (uint32_t(nowMs-jogRefreshMs_)>=350) {
            motor_.stop(); appliedPwm_=0; jogging_=false; state_=State::Idle; lastReason_="JOG_RELEASE";
            hostPrintf("EVT JOG_END reason=RELEASE_TIMEOUT zeroed=0\n"); return;
        }
        const float reference=jogTarget_*fminf(1.0f,float(nowMs-phaseStartMs_)/HomeRampMs);
        const float err=reference-measured;
        const float raw=HomeKp*err+homeIntegral_;
        if (fabsf(raw)<=HomeForwardPwm || raw*err<0)
            homeIntegral_=constrain(homeIntegral_+HomeKi*err*dt,-HomeIntegralLimit,HomeIntegralLimit);
        drive(constrain(HomeKp*err+homeIntegral_,-HomeForwardPwm,HomeForwardPwm));
        return;
    }
    if (homing_) {
        if (nowMs - phaseStartMs_ >= HomeTimeoutMs) { abortMotion("HOME_TIMEOUT"); return; }
        const int32_t count = encoder_.getCount();
        if (count != homeLastCount_) {
            homeLastCount_ = count; homeStillSinceMs_ = nowMs;
        }
        homePeakSpeed_ = fmaxf(homePeakSpeed_, fabsf(measured));
        // A sustained stop while gently pushing forward is the target candidate.
        // Also works when HOME starts already touching the target. Never auto-FIRE.
        if (nowMs-phaseStartMs_ >= HomeMinMs && nowMs-homeStillSinceMs_ >= HomeStillMs
            && fabsf(measured) < 5 && appliedPwm_ >= HomeContactPwm) {
            motor_.stop(); appliedPwm_ = 0; state_ = State::Idle; homing_ = false;
            const float travel = pos-homeStartAngle_;
            encoder_.reset(); zeroed_ = true; speedController_.reset(); holdIntegral_ = 0; lastReason_="HOME_DONE";
            hostPrintf("EVT HOME contact_candidate=1 angle_before=%.2f travel=%.2f peak_speed=%.2f zeroed=1\n",
                       pos, travel, homePeakSpeed_);
            return;
        }
        const float reference = HomeSpeed*fminf(1.0f,float(nowMs-phaseStartMs_)/HomeRampMs);
        const float err = reference-measured;
        const float raw = HomeBias + HomeKp*err + homeIntegral_;
        if ((raw >= -HomeReversePwm && raw <= HomeForwardPwm) || raw*err < 0)
            homeIntegral_ = constrain(homeIntegral_+HomeKi*err*dt,-HomeIntegralLimit,HomeIntegralLimit);
        drive(constrain(HomeBias+HomeKp*err+homeIntegral_,-HomeReversePwm,HomeForwardPwm));
        return;
    }
    if (state_ == State::Retract) {
        if (nowMs - phaseStartMs_ >= RetractTimeoutMs) { abortMotion("RETRACT_TIMEOUT"); return; }
        const float err = targetAngle_ - pos;
        const int32_t count = encoder_.getCount();
        if (abs(count - lastProgressCount_) >= 2) {
            lastProgressCount_ = count; lastProgressMs_ = nowMs;
        } else if (fabsf(err) > 3 && nowMs - lastProgressMs_ > 750) {
            abortMotion("RETRACT_NO_PROGRESS"); return;
        }
        // Control never changes when the arrival predicate changes.
        drive(holdControl(pos, measured, dt));
        if (fabsf(err) <= AngleTolerance) {
            if (fabsf(measured) <= StillSpeed) {
                if (!stableSinceMs_) stableSinceMs_ = nowMs;
                if (nowMs - stableSinceMs_ >= ReadyStableMs) {
                    state_ = State::Ready; lastReason_="READY";
                    hostPrintf("EVT READY target=%.2f actual=%.2f\n", targetAngle_, pos);
                    if (autoFire_) fire();
                }
            } else stableSinceMs_ = 0;
            return;
        }
        stableSinceMs_ = 0;
        return;
    }
    if (state_ == State::Ready) {
        drive(holdControl(pos, measured, dt));
        // A quadrature edge can arrive between READY qualification and fire().
        // Keep the original START intent if that one attempt was rejected.
        if (autoFire_) fire();
        return;
    }
    if (state_ == State::Strike) {
        if (nowMs - phaseStartMs_ >= StrikeTimeoutMs) { endStrike("TIMEOUT_NO_DROP", false); return; }
        // Cap the recent peak to 115% of target: correcting an overshoot down to
        // target is not a contact. Do not disable early-contact detection.
        const float ceiling = fmaxf(DropMinSpeed, settings_.strikeSpeedDps * 1.15f);
        // During a recent reverse correction, a moderate speed decrease is
        // expected. Still release on a near-stop even while braking; no angle
        // gate or requirement to have reached target speed is introduced.
        const bool recentReverse = reversedThisStrike_ && uint32_t(nowUs-lastReverseUs_) <= DropWindowUs;
        const float triggerCeiling = recentReverse ? fmaxf(DropMinSpeed, settings_.strikeSpeedDps*.25f) : INFINITY;
        if (dropDetector_.update(pos, nowUs, true, true, ceiling, triggerCeiling, measured)) {
            endStrike("SPEED_DROP", true); return;
        }
        drive(speedController_.update(settings_.strikeSpeedDps, measured, pos, dt, tuning_));
        if (appliedPwm_ < 0) { lastReverseUs_ = nowUs; reversedThisStrike_ = true; }
        recordTrace(nowUs);
    }
}

#pragma once
#include <stdint.h>

// Hardware / experiment switches and tunable defaults live here.
namespace HammerConfig {
constexpr uint8_t MotorIn1 = 41, MotorIn2 = 42, EncoderA = 39, EncoderB = 40;
constexpr uint32_t ControlPeriodUs = 1000;
constexpr uint32_t MaxControlGapUs = 50000; // existing abnormal-gap check
constexpr uint32_t RetractTimeoutMs = 8000, StrikeTimeoutMs = 1800, ReadyStableMs = 130;
constexpr float MaxPwm = 1000, AngleTolerance = 1.2f, StillSpeed = 20;
constexpr float DefaultRetractAngle = 60, DefaultStrikeSpeed = 200;
// One continuous position law for both approach and READY; no tolerance-boundary switch.
constexpr float PositionKp = 12, PositionKi = 15, PositionStaticKi = 40, PositionKd = 2.5f;
constexpr float PositionFrictionPwm = 105, PositionRetractPerDeg = 2;
constexpr float PositionFrictionWidthDeg = 1, PositionIntegralZoneDeg = 20;
constexpr float PositionIntegralSpeed = 40, PositionIntegralLimit = 300;
constexpr float SpeedFilterTauSec = .002f, CountFilterTauSec = .005f;
// First steady-data fit on the current bench. KP/KI are then verified per speed.
// KS is a signed constant bias (load/friction), not strictly Coulomb friction.
constexpr float SpeedKp = 3, SpeedKi = 20, FeedforwardKs = -96.7f, FeedforwardKv = .4477f;
constexpr float FeedforwardKg = 3.25f; // signed PWM * sin(output shaft angle)
constexpr float SpeedIntegralLimit = 1000;
constexpr uint32_t DropWindowUs = 6000, DropPeakAgeUs = 20000, DropArmUs = 4000;
constexpr float DropMinSpeed = 40, DropAbsolute = 35, DropRelative = .70f;
constexpr float StableRelativeError = .10f;
constexpr uint32_t StableSpeedUs = 10000;
constexpr unsigned TraceCapacity = 2048, TraceChunk = 8;
// Low-speed, bounded-effort approach to the fixed bench target. Encoder-only
// standstill is a contact candidate, not an independent limit/force sensor.
constexpr float HomeSpeed = 40, HomeForwardPwm = 120, HomeReversePwm = 600;
constexpr float HomeContactPwm = 60;
constexpr float HomeKp = 8, HomeKi = 30, HomeBias = 0, HomeIntegralLimit = 120;
constexpr uint32_t HomeTimeoutMs = 5000, HomeStillMs = 200, HomeMinMs = 300, HomeRampMs = 200;
constexpr char BleName[] = "Rotary_Hammer_BLE";
}

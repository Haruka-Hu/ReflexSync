#include "hall_encoder.h"
#include "encoder_speed_estimator.h"
#include "control_config.h"
#include "pcnt_count.h"
#ifdef ARDUINO_ARCH_ESP32
#include <driver/pcnt.h>
#include <soc/gpio_struct.h>
#endif

HallEncoder *HallEncoder::instance_ = nullptr;

/**
 * @brief 正交解码 4 倍频状态转移表
 *
 * 索引由 [前一次(A B)电平 << 2 | 当前(A B)电平] 组成，共 16 种组合：
 * - 结果为  1：代表正转（顺时针/下击）
 * - 结果为 -1：代表反转（逆时针/蓄力）
 * - 结果为  0：代表状态未变或非法跳变（如高频电磁杂波导致两相同时翻转），自动滤除
 */
static DRAM_ATTR const int8_t QUAD_TABLE[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0
};

HallEncoder::HallEncoder(const Config &config) : config_(config) {
    // 计算每转动 1 度对应的脉冲数系数
    countsPerDegree_ = static_cast<float>(config_.ppr * 4 * config_.gearRatio) / 360.0f;
    instance_ = this; // 设置静态单例指针，供中断 ISR 路由访问
}

bool HallEncoder::begin() {
    pinMode(config_.pinA, INPUT_PULLUP);
    pinMode(config_.pinB, INPUT_PULLUP);
#ifdef ARDUINO_ARCH_ESP32
    // 00 -> 01 -> 11 -> 10 -> 00 is positive, matching QUAD_TABLE.
    // A counts up on a rising edge with B high; B counts up with A low.
    pcnt_config_t a{};
    a.unit=PCNT_UNIT_0; a.channel=PCNT_CHANNEL_0;
    a.pulse_gpio_num=config_.pinA; a.ctrl_gpio_num=config_.pinB;
    a.pos_mode=PCNT_COUNT_INC; a.neg_mode=PCNT_COUNT_DEC;
    a.hctrl_mode=PCNT_MODE_KEEP; a.lctrl_mode=PCNT_MODE_REVERSE;
    a.counter_h_lim=EncoderPcntLimit; a.counter_l_lim=-EncoderPcntLimit;
    auto b=a;
    b.channel=PCNT_CHANNEL_1; b.pulse_gpio_num=config_.pinB; b.ctrl_gpio_num=config_.pinA;
    b.hctrl_mode=PCNT_MODE_REVERSE; b.lctrl_mode=PCNT_MODE_KEEP;
    if (pcnt_unit_config(&a)!=ESP_OK || pcnt_unit_config(&b)!=ESP_OK ||
        pcnt_counter_pause(PCNT_UNIT_0)!=ESP_OK ||
        pcnt_set_filter_value(PCNT_UNIT_0,1023)!=ESP_OK || // 12.8us; valid 900deg/s edges are about 231us apart
        pcnt_filter_enable(PCNT_UNIT_0)!=ESP_OK ||
        pcnt_counter_clear(PCNT_UNIT_0)!=ESP_OK) return false;
#endif
    lastEncodedState_=readState();
    attachInterrupt(digitalPinToInterrupt(config_.pinA), isrHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.pinB), isrHandler, CHANGE);
#ifdef ARDUINO_ARCH_ESP32
    ready_=pcnt_counter_resume(PCNT_UNIT_0)==ESP_OK;
#else
    ready_=true;
#endif
    return ready_;
}

uint8_t IRAM_ATTR HallEncoder::readState() const {
#ifdef ARDUINO_ARCH_ESP32
    // GPIO39/40 share one bank: capture both phases in the same register read.
    const uint32_t low = (config_.pinA<32 || config_.pinB<32) ? GPIO.in : 0;
    const uint32_t high = (config_.pinA>=32 || config_.pinB>=32) ? GPIO.in1.val : 0;
    const uint8_t a=((config_.pinA<32 ? low : high) >> (config_.pinA%32)) & 1U;
    const uint8_t b=((config_.pinB<32 ? low : high) >> (config_.pinB%32)) & 1U;
#else
    const uint8_t a=digitalRead(config_.pinA), b=digitalRead(config_.pinB);
#endif
    return (a<<1) | b;
}

void IRAM_ATTR HallEncoder::isrHandler() {
    if (instance_ == nullptr) {
        return; // 如果实例未初始化，直接返回
    }

    portENTER_CRITICAL_ISR(&instance_->mux_);
    const uint8_t currentEncodedState=instance_->readState();
    const uint8_t previous=instance_->lastEncodedState_;
    const int8_t step=QUAD_TABLE[(previous<<2) | currentEncodedState];
    if ((previous ^ currentEncodedState)==3) ++instance_->invalidTransitions_;
    instance_->lastEncodedState_=currentEncodedState;
    instance_->isrCount_+=step;
#ifndef ARDUINO_ARCH_ESP32
    instance_->pulseCount_+=step;
#endif
    if (step) {
        const uint32_t now = micros();
        if (step != instance_->edgeDirection_) {
            instance_->sameDirectionEdges_ = 0;
            instance_->fourEdgeUs_ = 0;
            instance_->edgeHead_ = 0;
        }
        const uint8_t head = instance_->edgeHead_;
        if (instance_->sameDirectionEdges_ >= 4)
            instance_->fourEdgeUs_ = now - instance_->edgeTimes_[head];
        instance_->edgeTimes_[head] = now;
        instance_->edgeHead_ = (head + 1) % 4;
        if (instance_->sameDirectionEdges_ < 5) ++instance_->sameDirectionEdges_;
        instance_->lastEdgeUs_ = now;
        instance_->edgeDirection_ = step;
    }
    portEXIT_CRITICAL_ISR(&instance_->mux_);

}

int32_t HallEncoder::isrCount() const {
    portENTER_CRITICAL(&mux_);
    const int32_t count=isrCount_;
    portEXIT_CRITICAL(&mux_);
    return count;
}
uint32_t HallEncoder::invalidTransitions() const {
    portENTER_CRITICAL(&mux_);
    const uint32_t count=invalidTransitions_;
    portEXIT_CRITICAL(&mux_);
    return count;
}

int32_t HallEncoder::getCount() const {
    return pulseCount_;
}

float HallEncoder::getAngleDeg() const {
    return static_cast<float>(pulseCount_) / countsPerDegree_;
}

float HallEncoder::getSpeedDegPerSec() const {
    // One feedback signal for the controller, STATUS and recorded trajectories.
    return currentSpeedDeg_;
}

void HallEncoder::updateSpeed(float dtSec) {
    if (dtSec <= 0.0f) {
        return; // 避免除以零或负时间间隔
    }

#ifdef ARDUINO_ARCH_ESP32
    int16_t hardwareCount=0;
    if (!ready_ || pcnt_get_counter_value(PCNT_UNIT_0,&hardwareCount)!=ESP_OK) {
        ready_=false; return;
    }
    pulseCount_+=pcntDelta(hardwareCount,lastHardwareCount_);
    lastHardwareCount_=hardwareCount;
#endif
    portENTER_CRITICAL(&mux_);
    int32_t currentCount = pulseCount_;
    // A missed GPIO interrupt must not corrupt the reciprocal speed estimate.
    // Position and count-speed continue to use the hardware counter.
    if (currentCount-lastSpeedCount_ != isrCount_-lastIsrSpeedCount_) {
        sameDirectionEdges_=0; fourEdgeUs_=0;
    }
    lastIsrSpeedCount_=isrCount_;
    const uint32_t fourEdgeUs = fourEdgeUs_;
    const uint32_t lastEdgeUs = lastEdgeUs_;
    const int direction = edgeDirection_;
    const bool valid = sameDirectionEdges_ >= 5;
    const uint32_t now = micros();
    portEXIT_CRITICAL(&mux_);
    int32_t deltaCount = currentCount - lastSpeedCount_;
    lastSpeedCount_ = currentCount;

    // 计算瞬时角速度 (°/s)
    float rawSpeed = (static_cast<float>(deltaCount) / countsPerDegree_) / dtSec;
    rawSpeedDeg_ = rawSpeed;
    const float countAlpha = dtSec / (HammerConfig::CountFilterTauSec + dtSec);
    currentCountSpeedDeg_ += countAlpha * (rawSpeed - currentCountSpeedDeg_);
    const float estimated = reciprocalEncoderSpeed(currentCountSpeedDeg_, countsPerDegree_,
        fourEdgeUs, now - lastEdgeUs, direction, valid);
    const float alpha = dtSec / (HammerConfig::SpeedFilterTauSec + dtSec);
    currentSpeedDeg_ += alpha * (estimated - currentSpeedDeg_);
}

void HallEncoder::reset(float newAngleDeg) {
  portENTER_CRITICAL(&mux_);
#ifdef ARDUINO_ARCH_ESP32
  // Rebase in software without pausing/clearing the running hardware counter.
  int16_t hardwareCount=0;
  if (pcnt_get_counter_value(PCNT_UNIT_0,&hardwareCount)!=ESP_OK) ready_=false;
  lastHardwareCount_=hardwareCount;
#endif
  lastEncodedState_=readState();
  // 换算目标角度对应的脉冲偏置
  pulseCount_ = static_cast<int32_t>(newAngleDeg * countsPerDegree_);
  lastSpeedCount_ = pulseCount_;
  isrCount_=pulseCount_; lastIsrSpeedCount_=pulseCount_;
  currentSpeedDeg_ = 0.0f;
  rawSpeedDeg_ = 0.0f;
  currentCountSpeedDeg_ = 0.0f;
  sameDirectionEdges_ = 0;
  fourEdgeUs_ = 0;
  lastEdgeUs_ = 0;
  edgeHead_ = 0;
  edgeDirection_ = 0;
  portEXIT_CRITICAL(&mux_);
}

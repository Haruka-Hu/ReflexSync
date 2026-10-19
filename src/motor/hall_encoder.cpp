#include "hall_encoder.h"
#include "encoder_speed_estimator.h"
#include "control_config.h"

HallEncoder *HallEncoder::instance_ = nullptr;

/**
 * @brief 正交解码 4 倍频状态转移表
 *
 * 索引由 [前一次(A B)电平 << 2 | 当前(A B)电平] 组成，共 16 种组合：
 * - 结果为  1：代表正转（顺时针/下击）
 * - 结果为 -1：代表反转（逆时针/蓄力）
 * - 结果为  0：代表状态未变或非法跳变（如高频电磁杂波导致两相同时翻转），自动滤除
 */
static const int8_t QUAD_TABLE[16] = {
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

void HallEncoder::begin() {
    // 配置 A 相和 B 相 GPIO 引脚为输入模式
    pinMode(config_.pinA, INPUT);
    pinMode(config_.pinB, INPUT);

    // 读取初始的 A/B 相电平状态
    uint8_t aState = digitalRead(config_.pinA);
    uint8_t bState = digitalRead(config_.pinB);
    lastEncodedState_ = (aState << 1) | bState;

    // 注册 GPIO 中断服务例程 (ISR)
    attachInterrupt(digitalPinToInterrupt(config_.pinA), isrHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.pinB), isrHandler, CHANGE);
}

void IRAM_ATTR HallEncoder::isrHandler() {
    if (instance_ == nullptr) {
        return; // 如果实例未初始化，直接返回
    }

    // 读取当前 A/B 相电平状态
    uint8_t aState = digitalRead(instance_->config_.pinA);
    uint8_t bState = digitalRead(instance_->config_.pinB);
    uint8_t currentEncodedState = (aState << 1) | bState;

    // 根据状态转移表计算脉冲增量
    uint8_t idx = (instance_->lastEncodedState_ << 2) | currentEncodedState;
    const int8_t step = QUAD_TABLE[idx];
    portENTER_CRITICAL_ISR(&instance_->mux_);
    instance_->pulseCount_ += step;
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

    // 更新上一次的编码器状态
    instance_->lastEncodedState_ = currentEncodedState;
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

    portENTER_CRITICAL(&mux_);
    int32_t currentCount = pulseCount_;
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
  // 换算目标角度对应的脉冲偏置
  pulseCount_ = static_cast<int32_t>(newAngleDeg * countsPerDegree_);
  lastSpeedCount_ = pulseCount_;
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

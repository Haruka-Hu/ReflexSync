#include "drv8871_motor.h"

Drv8871Motor::Drv8871Motor(const Config &config) : config_(config) {}

void Drv8871Motor::begin() {
    // 配置 IN1 和 IN2 引脚为输出模式
    pinMode(config_.pinIn1, OUTPUT);
    pinMode(config_.pinIn2, OUTPUT);

    // 配置 LEDC PWM 通道
    ledcSetup(config_.pwmChannel1, config_.pwmFreq, config_.pwmResolutionBits);
    ledcSetup(config_.pwmChannel2, config_.pwmFreq, config_.pwmResolutionBits);

    // 将 PWM 通道绑定到 GPIO 引脚
    ledcAttachPin(config_.pinIn1, config_.pwmChannel1);
    ledcAttachPin(config_.pinIn2, config_.pwmChannel2);

    // 初始化电机为安全停止状态
    stop();
}

void Drv8871Motor::stop() {
    // 自由停机：IN1 和 IN2 同时置 0
    writeDuty(0, 0);
}

void Drv8871Motor::brake() {
    // 主动制动：将两路输入全部输出最大占空比（100% 高电平）
    const int fullDuty = maxPwm();
    writeDuty(fullDuty, fullDuty);
}

void Drv8871Motor::driveForward(int pwm) {
    writeDuty(pwm, 0);
}

void Drv8871Motor::driveReverse(int pwm) {
    writeDuty(0, pwm);
}

void Drv8871Motor::writeDuty(int dutyIn1, int dutyIn2) {
    duty1_=clampPwm(dutyIn1); duty2_=clampPwm(dutyIn2);
    ledcWrite(config_.pwmChannel1,duty1_);
    ledcWrite(config_.pwmChannel2,duty2_);
}

int Drv8871Motor::maxPwm() const {
    return (1 << config_.pwmResolutionBits) - 1;
}

int Drv8871Motor::clampPwm(int value) const {
    if (value < 0) {
        return 0;
    } else if (value > maxPwm()) {
        return maxPwm();
    } else {
        return value;
    }
}

#pragma once

#include <Arduino.h>


/**
 * @brief DRV8871 单 H 桥直流电机驱动类
 *
 * DRV8871 采用双输入逻辑控制内部 H 桥功率 MOS 管：
 * - IN1=PWM, IN2=0   : 正转调速（用于敲击下砸）
 * - IN1=0,   IN2=PWM : 反转调速（用于后撤蓄力）
 * - IN1=1,   IN2=1   : 主动电制动/刹车（全桥短路电机电枢，利用反电动势快速制动）
 * - IN1=0,   IN2=0   : 自由滑行（MOS 全部关断，断电惯性滑行）
 */
class Drv8871Motor {
    public:
        /**
         * @brief 配置结构体
         */
        struct Config {
            uint8_t pinIn1;             // 连接 DRV8871 IN1 的 GPIO 引脚编号（如 GPIO 41）
            uint8_t pinIn2;             // 连接 DRV8871 IN2 的 GPIO 引脚编号（如 GPIO 42）
            uint8_t pwmChannel1;        // ESP32 LEDC 硬件通道编号（0 ~ 7）
            uint8_t pwmChannel2;        // ESP32 LEDC 硬件通道编号（0 ~ 7，不可与通道 1 重复）
            uint32_t pwmFreq;           // PWM 开关频率（推荐 20000 Hz，避开人耳听觉噪音）
            uint8_t pwmResolutionBits;  // PWM 占空比分辨率位数（推荐 10 位，对应取值范围 0 ~ 1023）
        };

        /**
         * @brief 构造函数
         * @param config 配置结构体
         */
        explicit Drv8871Motor(const Config &config);

        /**
         * @brief 初始化底层的 ESP32 LEDC 硬件定时器通道并绑定 GPIO
         * 必须在 setup() 中调用一次，调用完成后电机默认进入安全停止状态
         */
        void begin();

        /**
         * @brief 自由停机
         * IN1 和 IN2 同时置 0，关断 H 桥功率管，电机线圈悬空，靠自然机械摩擦力停下
         */
        void stop();

        /**
         * @brief 主动制动
         * IN1 和 IN2 同时置 1，短路电机线圈，利用反电动势快速制动
         */
        void brake();

        /**
         * @brief 以指定 PWM 占空比正向驱动
         * @param pwm 目标占空比（0 ~ maxPwm()），内部会自动做上下限截断保护
         */
        void driveForward(int pwm);

        /**
         * @brief 以指定 PWM 占空比反向驱动
         * @param pwm 目标占空比（0 ~ maxPwm()），内部会自动做上下限截断保护
         */
        void driveReverse(int pwm);

        /**
         * @brief 获取最大 PWM 占空比值
         * @return 最大占空比值（根据 pwmResolutionBits 计算得出）
         */
        int maxPwm() const;
        const char* outputMode() const {
            if (duty1_ && duty2_) return "BRAKE";
            if (duty1_) return "FORWARD";
            if (duty2_) return "REVERSE";
            return "COAST";
        }
        int in1Duty() const { return duty1_; }
        int in2Duty() const { return duty2_; }

    private:
        /**
         * @brief 向两个LEDC通道写入PWM占空比
         * @param dutyIn1 IN1通道占空比（0 ~ maxPwm()）
         * @param dutyIn2 IN2通道占空比（0 ~ maxPwm()）
         */
        void writeDuty(int dutyIn1, int dutyIn2);

        /**
         * @brief 数值截断保护函数：将输入数值限制在合法范围内（0 ~ maxPwm()）
         * @param value 待检查的数值
         * @return 安全合法的占空比数值
         */
        int clampPwm(int value) const;

        int duty1_=0, duty2_=0;
        Config config_;  // 配置结构体实例
};

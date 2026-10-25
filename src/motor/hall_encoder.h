#pragma once
#include <Arduino.h>

/**
 * @brief 霍尔正交编码器：PCNT 硬件四倍频位置，GPIO 边沿周期辅助测速
 *
 * 原理说明：
 * - 电机转子每转一圈输出 16 个脉冲周期 (16 PPR)
 * - PCNT 对 A/B 双边沿计数；GPIO ISR 仅提供边沿周期和计数对照诊断
 * - 配合 1:27 减速比，叩诊锤出轴旋转 1 圈的总脉冲数为：
 *   16 * 4 * 27 = 1728 个脉冲
 * - 角度分辨率为：360° / 1728 ≈ 0.208° (即每度 4.8 个脉冲)
 */
class HallEncoder {
    public:
        /**
         * @brief 结构体：编码器配置参数
         */
        struct Config {
            uint8_t pinA;  // A 相 GPIO 引脚编号
            uint8_t pinB;  // B 相 GPIO 引脚编号
            uint8_t ppr;   // 编码器每转一圈的脉冲数（Pulses Per Revolution）
            uint8_t gearRatio; // 减速比（出轴一圈对应的电机轴圈数）
        };

        /**
         * @brief 构造函数
         * @param config 编码器配置参数
         */
        explicit HallEncoder(const Config &config);

        /**
         * @brief 初始化编码器 GPIO 引脚与中断
         * 必须在 setup() 中调用一次，调用完成后编码器开始计数
         */
        bool begin();
        bool ready() const { return ready_; }
        int32_t isrCount() const;
        uint32_t invalidTransitions() const;

        /**
         * @brief 获取累积的四倍频脉冲总计数值
         * @return 正向累加为正，反向累加为负
         */
        int32_t getCount() const;

        /**
         * @brief 获取电机出轴的旋转角度（单位：度）
         * @return 累计角度值，可负；不对360取模
         */
        float getAngleDeg() const;

        /**
         * @brief 获取电机出轴的瞬时角速度（单位：度/秒）
         * @return 角速度值，单位为度/秒
         */
        float getSpeedDegPerSec() const;
        float getCountSpeedDegPerSec() const { return currentCountSpeedDeg_; }
        float getPeriodSpeedDegPerSec() const { return currentSpeedDeg_; }
        float getRawSpeedDegPerSec() const { return rawSpeedDeg_; }

        /**
         * @brief 周期性测速刷新接口
         * 必须由上层以 1ms 周期调用，累计硬件计数并刷新速度、位置快照
         * @param dtSec 距离上一次调用的时间间隔（单位：秒）
         */
        void updateSpeed(float dtSec);

        /**
         * @brief 软件归零/设定当前物理角度
         * 通常在叩诊锤靠在膝腱待机点时调用，将该位置标定为 0°
         * @param newAngleDeg 重新标定的目标角度值（默认为 0.0°）
         */
        void reset(float newAngleDeg = 0.0f);

        private:
        float rawSpeedDeg_ = 0;
        /**
         * @brief GPIO 硬件中断服务例程 (ISR)
         * 必须放置于 IRAM 中以获得微秒级的极速中断响应
         */
        static void IRAM_ATTR isrHandler();

        float currentCountSpeedDeg_{0.0f};
        mutable portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
        volatile uint32_t edgeTimes_[4]{};
        volatile uint32_t lastEdgeUs_ = 0;
        volatile uint32_t fourEdgeUs_ = 0;
        volatile uint8_t edgeHead_ = 0;
        volatile uint8_t sameDirectionEdges_ = 0;
        volatile int8_t edgeDirection_ = 0;
        Config config_;
        bool ready_ = false;
        int16_t lastHardwareCount_ = 0;
        int32_t lastIsrSpeedCount_ = 0;
        volatile int32_t isrCount_ = 0;
        volatile uint32_t invalidTransitions_ = 0;
        uint8_t IRAM_ATTR readState() const;
        static HallEncoder *instance_;            // 静态单例指针，供中断 ISR 路由访问

        volatile int32_t pulseCount_{0};          // 易失变量：累计脉冲计数值 （volatile表示该变量可能在程序的任何地方被改变，所以每次访问时都从内存中重新读取）
        volatile uint8_t lastEncodedState_{0};    // 易失变量：上一次 A/B 相的二进制电平状态

        float countsPerDegree_{4.8f};             // 每转动 1 度对应的脉冲数系数
        int32_t lastSpeedCount_{0};               // 上一次测速周期的计数值快照
        float currentSpeedDeg_{0.0f};             // 当前解算出的出轴角速度 (°/s)
};

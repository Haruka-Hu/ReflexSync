#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <stdint.h>

#include "ads1298_48ch_driver.h"
#include "capture_common.h"

// MasterApp 是采集链路中心节点：
// 1. 采集本地 ADS1298 EMG；
// 2. 广播 Beacon 给 Slave 做时间同步；
// 3. 接收 IMU batch、去重、回 ACK；
// 4. 把 EMG/IMU/Diag 统一发到 PC。
class MasterApp {
 public:
  // 板级配置只保留硬件引脚和串口速率，协议参数在 capture_common 中统一定义。
  struct Config {
    Ads1298_48chDriver::Pins adsPins;
    uint32_t serialBaud;
  };

  // 构造 Master 应用并保存板级配置。
  explicit MasterApp(const Config &config);

  // 初始化硬件和后台任务。
  void begin();

 private:
  // ESP-NOW 回调只负责把原始包塞进队列，后续解码在无线任务中做。
  struct SlaveRxItem {
    uint8_t mac[6];
    uint8_t len;
    uint32_t rxTimeUs;
    uint8_t bytes[capture::IMU_BATCH_WIRE_SIZE];
  };

  // 用于诊断在线状态，不参与可靠传输确认窗口。
  struct SlaveTracker {
    bool used;
    uint8_t mac[6];
    uint8_t source;
    uint32_t lastSeenUs;
    uint32_t totalSamples;
  };

  // Master 为每个 Slave 维护一个 32 位接收窗口。
  // ackBaseSeq 表示连续收到的最后一个 batch；recvBitmap 表示后续乱序到达的 batch。
  struct SlaveRxState {
    bool used;
    bool initialized;
    uint8_t mac[6];
    uint8_t source;
    uint16_t ackBaseSeq;
    uint32_t ackSampleSeq;
    uint32_t recvBitmap;
    bool ackPending;
    uint32_t lastAckSentUs;
    uint32_t duplicatePackets;
    uint32_t retransmitPackets;
    uint32_t missingPackets;
  };

  static constexpr size_t PC_CONTROL_FRAME_MAX_SIZE = 32;
  struct PcSerialTxFrame {
    uint16_t size;
    uint8_t bytes[PC_CONTROL_FRAME_MAX_SIZE];
  };

  struct PcRetxCacheEntry {
    bool valid;
    uint8_t frameType;
    uint16_t sequence;
    uint16_t size;
    uint8_t bytes[capture::PC_SERIAL_MAX_PACKET_SIZE];
  };

  static constexpr size_t MAX_TRACKED_SLAVES = 16;
  static constexpr size_t SLAVE_RX_QUEUE_SIZE = 128;
  static constexpr size_t OLD_SERIAL_QUEUE_SIZE = 128;
  static constexpr size_t EMG48_QUEUE_SIZE = 256;
  static constexpr size_t IMU_PC_QUEUE_SIZE = 256;
  static constexpr size_t PC_SERIAL_FRAME_QUEUE_SIZE = 16;
  static constexpr size_t PC_RETX_CACHE_SIZE = 16;
  static constexpr uint8_t EMG_BATCHES_PER_IMU_SERVICE = 4;
  static constexpr uint32_t SLAVE_OFFLINE_TIMEOUT_US = 3000000;
  static constexpr uint32_t ACK_MIN_INTERVAL_US = 8000;

  // Arduino/ESP-NOW/FreeRTOS 需要 C 风格回调，用静态函数转发到当前 MasterApp 实例。
  static void onAdsDrdyStatic();
  static void onEspNowRecvStatic(const uint8_t *mac, const uint8_t *data, int len);
  static void serialTxTaskStatic(void *arg);
  static void wirelessTaskStatic(void *arg);
  static void sensorTaskStatic(void *arg);
  static void serialCommandTaskStatic(void *arg);

  // ADS1298 DRDY ISR 的实例处理函数。
  void onAdsDrdyFromIsr();
  // ESP-NOW 收包的实例处理函数。
  void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);
  // USB CDC 批量发送任务主体。
  void serialTxTask();
  // Beacon、IMU 解码、ACK 和诊断任务主体。
  void wirelessTask();
  void retryStateCommand();
  // ADS1298 采样任务主体。
  void sensorTask();
  void serialCommandTask();

  // 无线和 ACK 窗口管理。
  // 初始化 ESP-NOW 通道和广播 peer。
  bool initEspNow();
  // 确保 Slave MAC 已被登记为可单播 peer。
  bool ensureEspNowPeer(const uint8_t mac[6]);
  // 更新 Slave 在线诊断状态。
  void trackSlave(const uint8_t mac[6], uint8_t source, uint32_t nowUs, uint32_t sampleCount);
  // 统计当前在线 Slave 数量。
  int32_t countOnlineSlaves(uint32_t nowUs);
  // 获取或创建 Slave 的 ACK 接收窗口。
  SlaveRxState *getSlaveRxState(const uint8_t mac[6], uint8_t source);
  // 推进连续 ACK 基准。
  void advanceAckWindow(SlaveRxState &state);
  // 标记 batch 已收到，并判断是否重复。
  void markBatchReceived(SlaveRxState &state, uint16_t batchSeq, uint32_t sampleStartSeq, uint8_t flags, bool &outDuplicate);
  // 发送当前接收窗口对应的 ACK。
  bool sendAck(SlaveRxState &state, uint32_t nowUs);
  // 合并并限速发送所有 Slave 的待发 ACK。
  void sendPendingAcks(uint32_t nowUs);
  // 回复 Slave 发起的双向同步探针。
  void handleSyncProbe(const SlaveRxItem &item);

  // 串口输出统一入口。所有来源最终都先进入 serialQueue_，再由 serialTxTask 合批写出。
  void writeSerialSample(uint8_t source,
                         uint32_t timestampUs,
                         uint32_t sampleSeq,
                         uint16_t batchSeq,
                         uint8_t flags,
                         int32_t x,
                         int32_t y,
                         int32_t z);
  void writeSerialEmg48Sample(capture::Emg48Sample sample);
  void writeSerialImuWirePacket(const capture::ImuBatchWirePacket &wirePacket);
  // 将指定字节全部写入 USB CDC。
  void writeSerialBytesAll(const uint8_t *data, size_t size);
  void enqueuePcSerialFrame(const uint8_t *data, size_t size);
  void cachePcDataFrame(uint8_t frameType, uint16_t sequence, const uint8_t *data, size_t size);
  bool resendPcDataFrame(uint8_t frameType, uint16_t sequence);
  void handlePcRetransmitCommand(const String &raw);
  void handlePcRadioCommand(const String &raw);
  // 写入一条诊断帧。
  void writeDiagFrame(uint32_t timestampUs,
                      int32_t forwardedPerSec,
                      int32_t hardErrorsPerSec,
                      int32_t onlineSlaveCount,
                      uint32_t linkEventsPerSec,
                      uint16_t queueDropsPerSec,
                      uint8_t ackFailsPerSec);
  void writePcSyncDiag(const capture::SyncDiagPacket &diag);
  void writePcStateEvent(const capture::StateAckPacket &event);
  void handlePcCommand(String raw);
  void transitionTo(uint8_t newState, uint32_t effectiveMasterTimeUs);
  void broadcastCommand(uint8_t targetState, uint32_t effectiveMasterTimeUs);
  void broadcastRadioConfig(uint8_t channel, uint8_t rateCode);
  bool applyRadioConfig(uint8_t channel, uint8_t rateCode);
  void broadcastBeacon(uint32_t nowUs);
  void resetStreamingState();
  uint8_t currentState() const;
  uint32_t beaconIntervalMs(uint8_t state) const;

  Config config_;
  Ads1298_48chDriver ads_;
  TaskHandle_t sensorTaskHandle_ = nullptr;
  QueueHandle_t slaveRxQueue_ = nullptr;
  QueueHandle_t serialQueue_ = nullptr;
  QueueHandle_t emg48Queue_ = nullptr;
  QueueHandle_t imuPcQueue_ = nullptr;
  QueueHandle_t serialFrameQueue_ = nullptr;
  SemaphoreHandle_t serialWriteMux_ = nullptr;
  SemaphoreHandle_t pcRetxMux_ = nullptr;
  uint8_t broadcastMac_[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // 下面这些计数器每秒汇总为 SOURCE_DIAG，方便 PC 端观察链路质量。
  uint32_t serialQueueDrops_ = 0;
  uint32_t serialFrameQueueDrops_ = 0;
  uint32_t slaveQueueDrops_ = 0;
  uint32_t slaveDecodeFails_ = 0;
  uint32_t slaveCrcFails_ = 0;
  uint32_t slaveForwardedSamples_ = 0;
  uint32_t serialWriteShorts_ = 0;
  uint32_t slaveRegistryDrops_ = 0;
  uint32_t slaveDuplicatePackets_ = 0;
  uint32_t slaveRetransmitPackets_ = 0;
  uint32_t slaveMissingPackets_ = 0;
  uint32_t ackSendFails_ = 0;
  uint32_t emg48QueueDrops_ = 0;
  uint32_t emg48Overruns_ = 0;
  uint32_t emg48ConfigVerifyFailures_ = 0;
  uint32_t emg48ForwardedSamples_ = 0;
  uint32_t imuPcQueueDrops_ = 0;
  uint32_t startupFaults_ = 0;
  uint8_t pcSerialSequence_ = 0;
  uint8_t pcImuSequence_ = 0;
  uint16_t pcEmg48Sequence_ = 0;
  uint8_t pcEventSequence_ = 0;
  uint16_t commandSeq_ = 0;
  SemaphoreHandle_t commandTxMux_ = nullptr;
  uint8_t retryCommand_[capture::COMMAND_WIRE_SIZE] = {};
  bool retryCommandActive_ = false;
  uint32_t retryCommandNextMs_ = 0;
  uint32_t retryCommandUntilMs_ = 0;
  uint16_t radioConfigSeq_ = 0;
  uint16_t beaconSeq_ = 0;
  uint8_t currentChannel_ = capture::ESPNOW_CHANNEL;
  uint8_t currentRateCode_ = capture::ESPNOW_DEFAULT_RATE;
  volatile uint8_t state_ = capture::STATE_IDLE;
  uint32_t pendingStreamStartUs_ = 0;
  uint32_t emgSampleSeq_ = 0;
  volatile uint32_t emgDrdyCount_ = 0;
  bool emg48QueueDropFlagPending_ = false;
  bool espNowReady_ = false;
  uint8_t pcRetxWriteIndex_ = 0;

  SlaveTracker slaveTrackers_[MAX_TRACKED_SLAVES] = {};
  SlaveRxState slaveRxStates_[MAX_TRACKED_SLAVES] = {};
  PcRetxCacheEntry *pcRetxCache_ = nullptr;
};

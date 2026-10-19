#include "imu_slave_node.h"
#include "adxl375_math.h"

#include <algorithm>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>
#include <driver/temp_sensor.h>

#include "sample_schedule.h"

namespace {

ImuSlaveApp *g_activeApp = nullptr;

wifi_phy_rate_t radioRateFromCode(uint8_t rateCode) {
  return rateCode == capture::ESPNOW_RATE_1M ? WIFI_PHY_RATE_1M_L : WIFI_PHY_RATE_2M_L;
}

}  // namespace

// 保存节点配置；两种传感器驱动只初始化当前板级入口选中的一种。
ImuSlaveApp::ImuSlaveApp(const Config &config)
    : config_(config), mpu_(config.mpuConfig), adxl_(config.adxlPins) {}

// 初始化 CPU 频率、串口、传感器、ESP-NOW 和采样/发送任务。
void ImuSlaveApp::begin() {
  g_activeApp = this;
  // Both SuperMini nodes default to 80 MHz.
  setCpuFrequencyMhz(config_.cpuFreqMhz);
  Serial.begin(config_.serialBaud);
  delay(300);

  Serial.printf("NODE_BOOT source=0x%02X sensor=%s build=%s %s cpu_mhz=%lu\n",
                config_.nodeSource, config_.sensorMode == SensorMode::Accel ? "ADXL375" : "MPU6500",
                __DATE__, __TIME__, static_cast<unsigned long>(getCpuFrequencyMhz()));
  temp_sensor_config_t temperatureConfig = TSENS_CONFIG_DEFAULT();
  temperatureConfig.dac_offset = TSENS_DAC_L1; // Die temperature, 20..100 C.
  temperatureSensorReady_ = temp_sensor_set_config(temperatureConfig) == ESP_OK &&
                            temp_sensor_start() == ESP_OK;

  sensorReady_ = config_.sensorMode == SensorMode::Accel ? adxl_.begin() : mpu_.begin();
  if (sensorReady_ && config_.sensorMode == SensorMode::Accel) {
    Serial.printf("ADXL375_INIT_OK DEVID=0x%02X\n", adxl_.deviceId());
  } else if (!sensorReady_) {
    if (config_.sensorMode == SensorMode::Accel) {
      Serial.printf("ERR: ADXL375_INIT_FAILED DEVID=0x%02X; retrying\n", adxl_.deviceId());
    } else {
      Serial.println("ERR: SENSOR_INIT_FAILED");
    }
  }
  dataReadyInterruptEnabled_ = sensorReady_ && config_.sensorMode == SensorMode::Gyro && mpu_.hasDataReadyInterrupt();
  if (dataReadyInterruptEnabled_) {
    attachInterrupt(digitalPinToInterrupt(mpu_.dataReadyInterruptPin()), onMpuDataReadyStatic, RISING);
  }
  sendDoneSem_ = xSemaphoreCreateBinary();
  downlinkQueue_ = xQueueCreate(32, sizeof(Downlink));
  if (!initEspNow()) Serial.println("ERR: ESPNOW_INIT_FAILED");

  // 采样任务只写本地 batch ring；无线任务独立按 slot 发送和重传。
  xTaskCreatePinnedToCore(wirelessTaskStatic, "slaveWireless", 6144, this, 4, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTaskStatic, "slaveSensor", 4096, this, 5, nullptr, 1);
}

// ESP-NOW 发送完成回调的静态转发入口。
void ImuSlaveApp::onEspNowSentStatic(const uint8_t *, esp_now_send_status_t status) {
  if (g_activeApp != nullptr) {
    g_activeApp->onEspNowSent(status);
  }
}

// ESP-NOW 接收回调的静态转发入口。
void ImuSlaveApp::onEspNowRecvStatic(const uint8_t *mac, const uint8_t *data, int len) {
  if (g_activeApp != nullptr) {
    g_activeApp->onEspNowRecv(mac, data, len);
  }
}

// MPU6500 DATA_RDY 中断的静态转发入口。
void IRAM_ATTR ImuSlaveApp::onMpuDataReadyStatic() {
  if (g_activeApp != nullptr) {
    g_activeApp->onMpuDataReadyFromIsr();
  }
}

// FreeRTOS 无线发送任务的静态转发入口。
void ImuSlaveApp::wirelessTaskStatic(void *arg) {
  static_cast<ImuSlaveApp *>(arg)->wirelessTask();
}

// FreeRTOS 传感器采样任务的静态转发入口。
void ImuSlaveApp::sensorTaskStatic(void *arg) {
  static_cast<ImuSlaveApp *>(arg)->sensorTask();
}

void IRAM_ATTR ImuSlaveApp::onSampleTimer() {
  if (g_activeApp == nullptr || g_activeApp->sensorTaskHandle_ == nullptr) return;
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(g_activeApp->sensorTaskHandle_, &higherPriorityTaskWoken);
  if (higherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void ImuSlaveApp::printDiagnostics(uint32_t sampleRate, uint32_t sensorBusyUs) {
  float temperatureC = NAN;
  if (temperatureSensorReady_) temp_sensor_read_celsius(&temperatureC);
  uint8_t channel = 0;
  wifi_second_chan_t secondary;
  int8_t txPower = 0;
  esp_wifi_get_channel(&channel, &secondary);
  esp_wifi_get_max_tx_power(&txPower);
  portENTER_CRITICAL(&offsetMux_);
  const uint32_t syncAgeMs = lastTwoWaySyncMs_ == 0 ? UINT32_MAX : millis() - lastTwoWaySyncMs_;
  const uint32_t syncReplies = syncProbeReplies_;
  const uint32_t syncRttUs = lastSyncProbeRttUs_;
  const int32_t residualUs = syncResidualUs_;
  const int32_t driftPpm = syncDriftPpm_;
  portEXIT_CRITICAL(&offsetMux_);
  Serial.printf("NODE_DIAG state=%u cpu_mhz=%lu temp_c=%.1f samples/s=%lu sensor_us=%lu "
                "sample_drops=%lu send_fails=%lu ring_overflows=%lu radio_ch=%u tx_qdbm=%d "
                "radio_on=%u rx_ps=%u sensor_ready=%u devid=0x%02X tx_timeouts=%lu downlink_drops=%lu "
                "sync_age_ms=%lu sync_replies=%lu sync_rtt_us=%lu sync_residual_us=%ld sync_ppm=%ld\n",
                currentState(), static_cast<unsigned long>(getCpuFrequencyMhz()), temperatureC,
                static_cast<unsigned long>(sampleRate), static_cast<unsigned long>(sensorBusyUs),
                static_cast<unsigned long>(sampleDrops_), static_cast<unsigned long>(sendFails_),
                static_cast<unsigned long>(ringOverflows_), channel, txPower,
                diagnosticRadioEnabled_, radioPowerSavingActive_, sensorReady_, config_.sensorMode == SensorMode::Accel ? adxl_.deviceId() : mpu_.deviceId(),
                static_cast<unsigned long>(sendTimeouts_), static_cast<unsigned long>(downlinkDrops_),
                static_cast<unsigned long>(syncAgeMs), static_cast<unsigned long>(syncReplies),
                static_cast<unsigned long>(syncRttUs), static_cast<long>(residualUs), static_cast<long>(driftPpm));
}

void ImuSlaveApp::handleDiagnosticCommand() {
  // USB-only, volatile diagnostic override. Keep radio protocol and persisted settings intact.
  for (uint8_t n = 0; n < 48 && Serial.available() > 0; ++n) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c != '\n') {
      if (diagnosticCommandLength_ < sizeof(diagnosticCommand_) - 1)
        diagnosticCommand_[diagnosticCommandLength_++] = c;
      continue;
    }
    diagnosticCommand_[diagnosticCommandLength_] = '\0';
    const String command(diagnosticCommand_);
    diagnosticCommandLength_ = 0;
    if (command == "STATUS") {
      printDiagnostics(lastDiagnosticRate_, lastDiagnosticBusyUs_);
    } else if (command == "DIAG_IDLE_PS OFF" || command == "DIAG_IDLE_PS ON" ||
               command == "DIAG_STREAM_PS OFF" || command == "DIAG_STREAM_PS ON") {
      if (currentState() != capture::STATE_IDLE) {
        Serial.println("ERR DIAG_PS requires IDLE; send STOP to Master first");
      } else {
        if (command.startsWith("DIAG_IDLE_PS")) idlePowerSavingEnabled_ = command.endsWith(" ON");
        else streamPowerSavingEnabled_ = command.endsWith(" ON");
        updateRadioPower();
        Serial.printf("ACK DIAG_PS idle=%u stream=%u\n", idlePowerSavingEnabled_, streamPowerSavingEnabled_);
      }
    } else if (command == "DIAG_RADIO OFF" || command == "DIAG_RADIO ON") {
      if (currentState() != capture::STATE_IDLE) {
        Serial.println("ERR DIAG_RADIO requires IDLE; send STOP to Master first");
      } else if (command == "DIAG_RADIO OFF") {
        releaseSyncRadio();
        esp_now_deinit();
        WiFi.mode(WIFI_OFF);
        diagnosticRadioEnabled_ = false;
        radioPowerSavingActive_ = false;
        radioPowerModeKnown_ = false;
        Serial.println("ACK DIAG_RADIO OFF (until reboot)");
      } else {
        if (!diagnosticRadioEnabled_) diagnosticRadioEnabled_ = initEspNow();
        Serial.println(diagnosticRadioEnabled_ ? "ACK DIAG_RADIO ON" : "ERR DIAG_RADIO start failed");
      }
    } else if (command.startsWith("DIAG_CPU ")) {
      const String value = command.substring(9);
      if (currentState() != capture::STATE_IDLE) {
        Serial.println("ERR DIAG_CPU requires IDLE; send STOP to Master first");
      } else if (value != "80" && value != "160" && value != "240") {
        Serial.println("ERR DIAG_CPU accepts 80, 160 or 240");
      } else if (setCpuFrequencyMhz(value.toInt())) {
        Serial.printf("ACK DIAG_CPU %lu (until reboot)\n", static_cast<unsigned long>(getCpuFrequencyMhz()));
      } else {
        Serial.println("ERR DIAG_CPU failed");
      }
    }
  }
}

// 记录最近一次 ESP-NOW 发送结果，并唤醒等待发送完成的无线任务。
void ImuSlaveApp::onEspNowSent(esp_now_send_status_t status) {
  lastSendOk_ = (status == ESP_NOW_SEND_SUCCESS);
  txBusy_ = false;
  // ESP-NOW invokes this callback from the Wi-Fi task, not an ISR.
  if (sendDoneSem_ != nullptr) xSemaphoreGive(sendDoneSem_);
}

bool ImuSlaveApp::sendPacket(const uint8_t *mac, const uint8_t *data, size_t size, uint32_t timeoutMs) {
  if (sendDoneSem_ == nullptr || txBusy_) return false;
  xSemaphoreTake(sendDoneSem_, 0);
  lastSendOk_ = false;
  txBusy_ = true;
  if (esp_now_send(mac, data, size) != ESP_OK) {
    txBusy_ = false;
    ++sendFails_;
    return false;
  }
  if (xSemaphoreTake(sendDoneSem_, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) {
    // Do not submit another packet until this one's late callback has arrived.
    ++sendTimeouts_;
    return false;
  }
  if (!lastSendOk_) ++sendFails_;
  return lastSendOk_;
}

void ImuSlaveApp::onMpuDataReadyFromIsr() {
  lastDataReadyLocalUs_ = micros();
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  if (sensorTaskHandle_ != nullptr) {
    vTaskNotifyGiveFromISR(sensorTaskHandle_, &higherPriorityTaskWoken);
  }
  if (higherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// 根据下行包类型分发到 ACK、双向同步或 Beacon 时间同步处理。
void ImuSlaveApp::onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (mac == nullptr || data == nullptr || len <= 0 || len > 64 || downlinkQueue_ == nullptr) return;
  Downlink item{};
  memcpy(item.mac, mac, 6);
  item.size = static_cast<uint8_t>(len);
  item.receivedUs = micros();
  memcpy(item.bytes, data, len);
  if (xQueueSend(downlinkQueue_, &item, 0) != pdTRUE) ++downlinkDrops_;
}

void ImuSlaveApp::processDownlink(const Downlink &item) {
  // Slave 接收 COMMAND/无线配置/ACK/SYNC_REPLY/Beacon 等下行包。
  const uint32_t localRecvUs = item.receivedUs;
  const auto *mac = item.mac;
  const auto *data = item.bytes;
  const int len = item.size;
  if (data == nullptr) {
    return;
  }
  if (mac != nullptr) {
    portENTER_CRITICAL(&offsetMux_);
    memcpy(masterMac_, mac, 6);
    hasMasterMac_ = true;
    portEXIT_CRITICAL(&offsetMux_);
  }

  if (len == static_cast<int>(capture::COMMAND_WIRE_SIZE) && data[0] == capture::MSG_TYPE_COMMAND) {
    handleCommandPacket(data, len);
    return;
  }

  if (len == static_cast<int>(capture::RADIO_CONFIG_WIRE_SIZE) && data[0] == capture::MSG_TYPE_RADIO_CONFIG) {
    handleRadioConfigPacket(data, len);
    return;
  }

  if (len == static_cast<int>(capture::IMU_ACK_WIRE_SIZE) && data[0] == capture::MSG_TYPE_IMU_ACK) {
    handleAckPacket(data, len);
    return;
  }

  if (len == static_cast<int>(capture::SYNC_REPLY_WIRE_SIZE) && data[0] == capture::MSG_TYPE_SYNC_REPLY) {
    handleSyncReply(data, len, localRecvUs);
    return;
  }

  handleBeacon(mac, data, len, localRecvUs);
}

// 初始化 Slave 的 ESP-NOW STA 模式、固定信道和发送功率。
bool ImuSlaveApp::initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_channel(currentChannel_, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(config_.espnowTxPowerQdbm);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_wifi_config_espnow_rate(WIFI_IF_STA, radioRateFromCode(currentRateCode_));

  esp_now_register_send_cb(onEspNowSentStatic);
  esp_now_register_recv_cb(onEspNowRecvStatic);
  txBusy_ = false;
  radioPowerModeKnown_ = false;
  return true;
}

void ImuSlaveApp::updateRadioPower() {
#if CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE
  if (!diagnosticRadioEnabled_) return;
  const uint8_t state = currentState();
  const bool saving = (idlePowerSavingEnabled_ && state == capture::STATE_IDLE) ||
                    (streamPowerSavingEnabled_ && state == capture::STATE_STREAM);
  if (radioPowerModeKnown_ && saving == radioPowerSavingActive_) return;
  const esp_err_t intervalResult = esp_wifi_set_connectionless_wake_interval(POWER_SAVE_RX_INTERVAL_MS);
  const esp_err_t windowResult = esp_now_set_wake_window(saving ? POWER_SAVE_RX_WINDOW_MS : UINT16_MAX);
  const esp_err_t psResult = esp_wifi_set_ps(saving ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE);
  if (intervalResult == ESP_OK && windowResult == ESP_OK && psResult == ESP_OK) {
    radioPowerSavingActive_ = saving;
    radioPowerModeKnown_ = true;
  } else {
    // A firmware/SDK mismatch must preserve reception instead of silently sleeping.
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_now_set_wake_window(UINT16_MAX);
    radioPowerSavingActive_ = false;
    idlePowerSavingEnabled_ = false;
    streamPowerSavingEnabled_ = false;
    radioPowerModeKnown_ = true;
    Serial.printf("ERR radio power configuration interval=%d window=%d ps=%d\n",
                  intervalResult, windowResult, psResult);
  }
#endif
}

void ImuSlaveApp::releaseSyncRadio() {
  if (syncRadioHeld_) {
    esp_wifi_force_wakeup_release();
    syncRadioHeld_ = false;
  }
}

bool ImuSlaveApp::loadClockModel(double &slope, double &intercept) {
  bool valid = false;
  portENTER_CRITICAL(&offsetMux_);
  valid = clockModelValid_;
  slope = clockSlope_;
  intercept = clockIntercept_;
  portEXIT_CRITICAL(&offsetMux_);
  return valid;
}

// 兼容旧接口；采样时间戳实际使用动态平滑时钟模型。
bool ImuSlaveApp::loadSampleClockModel(double &slope, double &intercept) {
  bool valid = false;
  portENTER_CRITICAL(&offsetMux_);
  valid = sampleClockModelValid_;
  slope = sampleClockSlope_;
  intercept = sampleClockIntercept_;
  portEXIT_CRITICAL(&offsetMux_);
  return valid;
}

// 线程安全读取最近收到 Beacon 的 Master MAC，用于后续单播发送。
bool ImuSlaveApp::loadMasterMac(uint8_t outMac[6]) {
  bool valid = false;
  portENTER_CRITICAL(&offsetMux_);
  valid = hasMasterMac_;
  if (valid) {
    memcpy(outMac, masterMac_, 6);
  }
  portEXIT_CRITICAL(&offsetMux_);
  return valid;
}

// 处理 Master Beacon，更新时间偏移和 Master MAC。
void ImuSlaveApp::handleBeacon(const uint8_t *mac, const uint8_t *data, int len, uint32_t localRecvUs) {
  if (len != static_cast<int>(sizeof(capture::BeaconPacket))) {
    return;
  }
  capture::BeaconPacket packet{};
  memcpy(&packet, data, sizeof(packet));
  if (packet.type != capture::MSG_TYPE_BEACON) {
    return;
  }

  const uint32_t localNowUs = localRecvUs;
  const uint32_t nowMs = millis();
  portENTER_CRITICAL(&offsetMux_);
  if ((state_ == capture::STATE_SYNC || state_ == capture::STATE_STREAM_PENDING || state_ == capture::STATE_STREAM) &&
      (lastTwoWaySyncMs_ == 0 || static_cast<uint32_t>(nowMs - lastTwoWaySyncMs_) > TWO_WAY_BEACON_SUPPRESS_MS)) {
    recordBeaconLocked(localNowUs, packet.masterTimeUs, packet.beaconSeq);
  }
  if (mac != nullptr) {
    memcpy(masterMac_, mac, 6);
    hasMasterMac_ = true;
  }
  portEXIT_CRITICAL(&offsetMux_);
}

void ImuSlaveApp::handleCommandPacket(const uint8_t *data, int len) {
  capture::CommandPacket command{};
  if (!capture::decodeCommandPacket(data, len, command)) {
    return;
  }
  applyState(command.targetState, command.commandSeq, command.effectiveMasterTimeUs);
}

void ImuSlaveApp::handleRadioConfigPacket(const uint8_t *data, int len) {
  capture::RadioConfigPacket config{};
  if (!capture::decodeRadioConfigPacket(data, len, config)) {
    return;
  }

  portENTER_CRITICAL(&radioMux_);
  if (config.configSeq == lastRadioConfigSeq_ && radioConfigPending_) {
    portEXIT_CRITICAL(&radioMux_);
    return;
  }
  lastRadioConfigSeq_ = config.configSeq;
  pendingRadioChannel_ = config.channel;
  pendingRadioRateCode_ = config.rateCode;
  pendingRadioApplyMs_ = millis() + config.applyDelayMs;
  radioConfigPending_ = true;
  portEXIT_CRITICAL(&radioMux_);
}

bool ImuSlaveApp::applyRadioConfig(uint8_t channel, uint8_t rateCode) {
  if (!capture::isValidRadioChannel(channel) || !capture::isValidRadioRate(rateCode)) {
    return false;
  }

  portENTER_CRITICAL(&offsetMux_);
  ++syncFitGeneration_;
  state_ = capture::STATE_IDLE;
  streamStarted_ = false;
  pendingStreamStartUs_ = 0;
  clockModelValid_ = false;
  sampleClockModelValid_ = false;
  portEXIT_CRITICAL(&offsetMux_);

  portENTER_CRITICAL(&ringMux_);
  resetRingLocked();
  portEXIT_CRITICAL(&ringMux_);

  currentChannel_ = channel;
  currentRateCode_ = rateCode;
  esp_wifi_set_channel(currentChannel_, WIFI_SECOND_CHAN_NONE);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, radioRateFromCode(currentRateCode_));
  return true;
}

void ImuSlaveApp::applyPendingRadioConfig() {
  uint8_t channel = 0;
  uint8_t rateCode = 0;
  bool shouldApply = false;

  portENTER_CRITICAL(&radioMux_);
  if (radioConfigPending_ && static_cast<int32_t>(millis() - pendingRadioApplyMs_) >= 0) {
    channel = pendingRadioChannel_;
    rateCode = pendingRadioRateCode_;
    radioConfigPending_ = false;
    shouldApply = true;
  }
  portEXIT_CRITICAL(&radioMux_);

  if (shouldApply) {
    applyRadioConfig(channel, rateCode);
  }
}

void ImuSlaveApp::handleSyncReply(const uint8_t *data, int len, uint32_t localRecvUs) {
  capture::SyncReplyPacket reply{};
  if (!capture::decodeSyncReplyPacket(data, len, reply) || reply.source != config_.nodeSource) {
    return;
  }
  if (syncRadioHeld_ && reply.probeSeq == syncRadioProbeSeq_) releaseSyncRadio();

  const uint32_t rttUs = static_cast<uint32_t>(localRecvUs - reply.slaveT1LocalUs);
  if (rttUs == 0 || rttUs > TWO_WAY_SYNC_RTT_MAX_US) {
    portENTER_CRITICAL(&offsetMux_);
    ++syncProbeDrops_;
    portEXIT_CRITICAL(&offsetMux_);
    return;
  }

  const int64_t t2MinusT1 = static_cast<int32_t>(reply.masterT2RecvUs - reply.slaveT1LocalUs);
  const int64_t t3MinusT4 = static_cast<int32_t>(reply.masterT3SendUs - localRecvUs);
  const int64_t offsetUs = (t2MinusT1 + t3MinusT4) / 2;
  const uint32_t localMidUs = reply.slaveT1LocalUs + rttUs / 2U;
  const uint32_t masterMidUs = static_cast<uint32_t>(static_cast<int64_t>(localMidUs) + offsetUs);

  portENTER_CRITICAL(&offsetMux_);
  if (state_ == capture::STATE_SYNC || state_ == capture::STATE_STREAM_PENDING || state_ == capture::STATE_STREAM) {
    recordSyncPointLocked(localMidUs, masterMidUs, reply.probeSeq);
    lastTwoWaySyncMs_ = millis();
    lastSyncProbeRttUs_ = rttUs;
    ++syncProbeReplies_;
  }
  portEXIT_CRITICAL(&offsetMux_);
}

uint8_t ImuSlaveApp::currentState() const {
  return state_;
}

void ImuSlaveApp::resetRingLocked() {
  for (size_t i = 0; i < BATCH_RING_CAPACITY; ++i) {
    batchRing_[i] = {};
  }
  nextBatchSeq_ = 0;
  oldestUnackedSeq_ = 0;
  nextSampleSeq_ = 0;
  ringOverflows_ = 0;
  lastSampleTimestampUs_ = 0;
  hasLastSampleTimestamp_ = false;
  linkFaultPending_ = false;
  ackPackets_ = 0;
  ackedBatches_ = 0;
  sendAttempts_ = 0;
  sendFails_ = 0;
}

void ImuSlaveApp::resetSyncFitLocked() {
  clockSlope_ = 1.0;
  clockIntercept_ = 0.0;
  sampleClockSlope_ = 1.0;
  sampleClockIntercept_ = 0.0;
  syncOffsetUs_ = 0;
  syncDriftPpm_ = 0;
  syncResidualUs_ = 0;
  clockModelValid_ = false;
  sampleClockModelValid_ = false;
  syncPointCount_ = 0;
  syncPointHead_ = 0;
  lastBeaconSeq_ = 0;
  syncProbeSeq_ = 0;
  lastSyncDiagMs_ = 0;
  lastSyncProbeMs_ = 0;
  lastTwoWaySyncMs_ = 0;
  syncProbeReplies_ = 0;
  syncProbeDrops_ = 0;
  lastSyncProbeRttUs_ = 0;
}

void ImuSlaveApp::applyState(uint8_t newState, uint16_t commandSeq, uint32_t effectiveMasterTimeUs) {
  if (commandSeq == lastCommandSeq_) {
    sendStateAck(commandSeq, effectiveMasterTimeUs);
    return;
  }
  lastCommandSeq_ = commandSeq;

  portENTER_CRITICAL(&ringMux_);
  if (newState == capture::STATE_IDLE || newState == capture::STATE_SYNC || newState == capture::STATE_STREAM_PENDING) {
    resetRingLocked();
  }
  portEXIT_CRITICAL(&ringMux_);

  portENTER_CRITICAL(&offsetMux_);
  ++syncFitGeneration_;
  state_ = newState;
  pendingStreamStartUs_ = effectiveMasterTimeUs;
  streamStarted_ = false;
  if (newState == capture::STATE_SYNC) {
    resetSyncFitLocked();
  }
  if (newState == capture::STATE_IDLE) {
    streamStarted_ = false;
    sampleClockModelValid_ = false;
  }
  portEXIT_CRITICAL(&offsetMux_);

  sendStateAck(commandSeq, effectiveMasterTimeUs);
}

void ImuSlaveApp::sendStateAck(uint16_t commandSeq, uint32_t effectiveMasterTimeUs) {
  uint8_t masterMac[6] = {0};
  if (!loadMasterMac(masterMac)) {
    return;
  }
  if (!esp_now_is_peer_exist(masterMac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
  }
  uint8_t packet[capture::STATE_ACK_WIRE_SIZE] = {};
  capture::buildStateAckPacket(config_.nodeSource, currentState(), commandSeq, effectiveMasterTimeUs, packet);
  stateAckPending_ = !sendPacket(masterMac, packet, sizeof(packet), 3);
  pendingStateAckSeq_ = commandSeq;
  pendingStateAckTimeUs_ = effectiveMasterTimeUs;
}

void ImuSlaveApp::recordSyncPointLocked(uint32_t localUs, uint32_t masterUs, uint16_t beaconSeq) {
  syncPoints_[syncPointHead_] = {localUs, masterUs};
  syncPointHead_ = static_cast<uint8_t>((syncPointHead_ + 1U) % SYNC_WINDOW_CAPACITY);
  if (syncPointCount_ < SYNC_WINDOW_CAPACITY) {
    ++syncPointCount_;
  }
  lastBeaconSeq_ = beaconSeq;
}

void ImuSlaveApp::recordBeaconLocked(uint32_t localRecvUs, uint32_t masterTimeUs, uint16_t beaconSeq) {
  recordSyncPointLocked(localRecvUs, masterTimeUs, beaconSeq);
}

// Copy the small window under the lock; double-precision fitting and sorting must
// run with interrupts enabled so the other core can keep its 1 kHz deadline.
void ImuSlaveApp::updateClockFit() {
  SyncPoint points[SYNC_WINDOW_CAPACITY];
  uint8_t count;
  uint32_t generation;
  uint8_t state;
  imu_clock::Model previous;
  portENTER_CRITICAL(&offsetMux_);
  count = syncPointCount_;
  generation = syncFitGeneration_;
  state = state_;
  previous.valid = clockModelValid_;
  previous.slope = clockSlope_;
  const uint8_t oldest = static_cast<uint8_t>((syncPointHead_ + SYNC_WINDOW_CAPACITY - count) % SYNC_WINDOW_CAPACITY);
  for (uint8_t i = 0; i < count; ++i) points[i] = syncPoints_[(oldest + i) % SYNC_WINDOW_CAPACITY];
  portEXIT_CRITICAL(&offsetMux_);

  const auto fitted = imu_clock::robust(points, count, previous);
  if (!imu_clock::plausible(fitted)) return; // Keep the last valid fit across transient bad windows.
  const uint32_t nowUs = micros();
  const int32_t offsetUs = static_cast<int32_t>(fitted.slope * nowUs + fitted.intercept - nowUs);
  const int32_t driftPpm = static_cast<int32_t>((fitted.slope - 1.0) * 1000000.0);
  portENTER_CRITICAL(&offsetMux_);
  // A STOP/START_SYNC received during the fit invalidates this snapshot.
  if (generation == syncFitGeneration_ && state == state_) {
    clockModelValid_ = fitted.valid;
    if (fitted.valid) {
      clockSlope_ = fitted.slope;
      clockIntercept_ = fitted.intercept;
      syncOffsetUs_ = offsetUs;
      syncDriftPpm_ = driftPpm;
      syncResidualUs_ = static_cast<int32_t>(fitted.residualUs);
    }
  }
  portEXIT_CRITICAL(&offsetMux_);
}

// 采样时间戳使用动态平滑模型；保留该函数只兼容 STREAM 进入流程。
void ImuSlaveApp::freezeSampleClockLocked() {
  sampleClockModelValid_ = false;
}

uint32_t ImuSlaveApp::localToMasterTimeUs(uint32_t localUs) {
  double slope = 1.0;
  double intercept = 0.0;
  if (!loadClockModel(slope, intercept)) {
    return localUs;
  }
  return static_cast<uint32_t>(slope * static_cast<double>(localUs) + intercept);
}

uint32_t ImuSlaveApp::localToSampleTimeUs(uint32_t localUs) {
  return localToMasterTimeUs(localUs);
}

void ImuSlaveApp::sendSyncProbe() {
  if (txBusy_ || syncRadioHeld_) return;
  uint8_t masterMac[6] = {0};
  if (!loadMasterMac(masterMac)) {
    return;
  }
  if (!esp_now_is_peer_exist(masterMac)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      return;
    }
  }

  uint16_t probeSeq = 0;
  portENTER_CRITICAL(&offsetMux_);
  probeSeq = syncProbeSeq_++;
  portEXIT_CRITICAL(&offsetMux_);

  if (radioPowerSavingActive_) {
    // Wake RF before taking T1 and keep it on until T4. A sleep-delayed reply
    // would otherwise bias the two-way clock offset by half that extra delay.
    if (esp_wifi_force_wakeup_acquire() == ESP_OK) {
      syncRadioHeld_ = true;
      syncRadioProbeSeq_ = probeSeq;
      syncRadioUntilUs_ = micros() + TWO_WAY_SYNC_RTT_MAX_US;
    } else {
      streamPowerSavingEnabled_ = false;
      updateRadioPower();
      Serial.println("ERR sync RF wake failed; stream power saving disabled");
    }
  }

  uint8_t packet[capture::SYNC_PROBE_WIRE_SIZE] = {};
  capture::buildSyncProbePacket(config_.nodeSource, probeSeq, micros(), packet);
  sendPacket(masterMac, packet, sizeof(packet), 3);
}

void ImuSlaveApp::sendSyncDiag() {
  uint8_t masterMac[6] = {0};
  if (!loadMasterMac(masterMac)) {
    return;
  }

  capture::SyncDiagPacket diag{};
  updateClockFit();
  portENTER_CRITICAL(&offsetMux_);
  diag.source = config_.nodeSource;
  diag.state = state_;
  diag.beaconSeq = lastBeaconSeq_;
  diag.offsetUs = syncOffsetUs_;
  diag.driftPpm = syncDriftPpm_;
  diag.residualUs = syncResidualUs_;
  diag.beaconCount = syncPointCount_;
  portEXIT_CRITICAL(&offsetMux_);

  uint8_t packet[capture::SYNC_DIAG_WIRE_SIZE] = {};
  capture::buildSyncDiagPacket(diag, packet);
  sendPacket(masterMac, packet, sizeof(packet), 3);
}

// 处理 Master ACK，释放本地 ring 中已经被确认的 batch。
void ImuSlaveApp::handleAckPacket(const uint8_t *data, int len) {
  // ACK 到达后遍历本地 ring，释放所有已被 base/bitmap 覆盖的 batch。
  uint16_t ackBaseSeq = 0;
  uint32_t recvBitmap = 0;
  if (!capture::decodeAckPacket(data, len, config_.nodeSource, ackBaseSeq, recvBitmap)) {
    return;
  }

  portENTER_CRITICAL(&ringMux_);
  for (size_t i = 0; i < BATCH_RING_CAPACITY; ++i) {
    BatchSlot &slot = batchRing_[i];
    if (!slot.used) {
      continue;
    }
    if (capture::ackCoversBatchSeq(slot.batchSeq, ackBaseSeq, recvBitmap)) {
      slot.used = false;
      ++ackedBatches_;
    }
  }
  ++ackPackets_;
  advanceOldestUnackedLocked();
  portEXIT_CRITICAL(&ringMux_);
}

// 在持有 ringMux_ 时推进最早未确认 batch 序号。
void ImuSlaveApp::advanceOldestUnackedLocked() {
  // oldestUnackedSeq_ 总是指向最早仍需发送/重传的 batch，发送任务从这里开始找。
  while (oldestUnackedSeq_ != nextBatchSeq_) {
    BatchSlot &slot = batchRing_[oldestUnackedSeq_ % BATCH_RING_CAPACITY];
    if (slot.used && slot.batchSeq == oldestUnackedSeq_) {
      break;
    }
    ++oldestUnackedSeq_;
  }
}

// 记录无法保存或无法读取的样本数量，并标记下一包 link fault。
void ImuSlaveApp::noteDroppedSamples(uint32_t count) {
  // 不能伪造连续样本；一旦本地无法保存，就推进 sampleSeq 并给下一包打 link fault。
  if (count == 0) {
    return;
  }

  portENTER_CRITICAL(&ringMux_);
  nextSampleSeq_ += count;
  linkFaultPending_ = true;
  portEXIT_CRITICAL(&ringMux_);
  sampleDrops_ += count;
}

// 将采样任务组好的 batch 写入可靠重传 ring buffer。
bool ImuSlaveApp::storeBatch(const capture::ImuRawSample *samples, uint8_t count) {
  // 采样任务只把 batch 放进 ring，不等待无线发送；这是可靠链路改造的核心。
  const uint16_t batchSeq = nextBatchSeq_;
  const uint32_t sampleStartSeq = nextSampleSeq_;
  capture::ImuBatchWirePacket packet = capture::makeImuBatchPacket(config_.nodeSource, samples, count, batchSeq, sampleStartSeq);
  bool stored = false;

  portENTER_CRITICAL(&ringMux_);
  BatchSlot &slot = batchRing_[batchSeq % BATCH_RING_CAPACITY];
  if (!slot.used) {
    if (linkFaultPending_) {
      packet.bytes[9] |= capture::IMU_FLAG_LINK_FAULT;
      capture::writeImuBatchCrc(packet);
      linkFaultPending_ = false;
    }
    slot.used = true;
    slot.sendAttempts = 0;
    slot.batchSeq = batchSeq;
    slot.sampleStartSeq = sampleStartSeq;
    // ✅ 优化：新包入队时，将下一次发送时间设为 0，代表“立即发送”
    slot.nextSendUs = 0;
    slot.packet = packet;
    ++nextBatchSeq_;
    nextSampleSeq_ += count;
    stored = true;
  } else {
    // 256 个 batch 都没被 ACK 时才会走到这里，说明无线中断已经超过可缓存范围。
    ++ringOverflows_;
    linkFaultPending_ = true;
    nextSampleSeq_ += count;
  }
  portEXIT_CRITICAL(&ringMux_);
  return stored;
}

// 从 ring 中取出当前最应该发送或重传的 batch。
bool ImuSlaveApp::takeSendCandidate(capture::ImuBatchWirePacket &outPacket, uint32_t nowUs) {
  bool found = false;

  portENTER_CRITICAL(&ringMux_);
  advanceOldestUnackedLocked();
  uint16_t seq = oldestUnackedSeq_;
  const uint16_t pendingCount = static_cast<uint16_t>(nextBatchSeq_ - oldestUnackedSeq_);

  // ✅ 核心优化 1：发送窗口限制 (In-Flight Window Limit)
  // 无论本地积压了多少包，我们绝不发送超越 Master 32 位接收窗口的包
  const uint16_t windowLimit = std::min(pendingCount, static_cast<uint16_t>(32));

  for (uint16_t i = 0; i < windowLimit; ++i, ++seq) {
    BatchSlot &slot = batchRing_[seq % BATCH_RING_CAPACITY];
    if (!slot.used || slot.batchSeq != seq) {
      continue;
    }

    // ✅ 核心优化 2 & 4：判断是否到了该发的时间
    // 如果 nextSendUs != 0 且当前时间还未到达 nextSendUs，跳过不发
    if (slot.nextSendUs != 0 && static_cast<int32_t>(nowUs - slot.nextSendUs) < 0) {
      continue;
    }

    outPacket = slot.packet;
    if (slot.sendAttempts > 0) {
      outPacket.bytes[9] |= capture::IMU_FLAG_RETRANSMIT;
      capture::writeImuBatchCrc(outPacket);
    }

    slot.nextSendUs = nowUs + RETRANSMIT_INTERVAL_US + random(RETRANSMIT_JITTER_US);

    ++slot.sendAttempts;
    ++sendAttempts_;
    found = true;
    break;
  }
  portEXIT_CRITICAL(&ringMux_);
  return found;
}

// Both sensors return one latest three-axis register sample per read.
bool ImuSlaveApp::readSensor(capture::ImuRawSample &sample) {
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  const bool readOk = config_.sensorMode == SensorMode::Accel
                          ? adxl_.readSample(x, y, z)
                          : mpu_.readGyro(x, y, z);
  if (!readOk) {
    return false;
  }
  sample.x = x;
  sample.y = y;
  sample.z = z;
  return true;
}

// 等待时间同步后，按错峰发送窗口向 Master 发送和重传 batch。
void ImuSlaveApp::wirelessTask() {
  // 发送 slot 使用 Master 时间轴计算，两个 Slave 因配置的 offset 不同而错峰。
  capture::ImuBatchWirePacket packet{};
  uint8_t masterMac[6] = {0};
  uint64_t lastWindowKey = UINT64_MAX;
  uint32_t lastSensorDiagMs = millis();
  uint32_t lastSensorSamples = 0;
  uint32_t lastSensorBusyUs = 0;

  for (;;) {
    if (syncRadioHeld_ && (currentState() != capture::STATE_STREAM ||
        static_cast<int32_t>(micros() - syncRadioUntilUs_) >= 0)) releaseSyncRadio();
    handleDiagnosticCommand();
    if (static_cast<uint32_t>(millis() - lastSensorDiagMs) >= 1000) {
      const uint32_t diagNowMs = millis();
      const uint32_t elapsedMs = diagNowMs - lastSensorDiagMs;
      const uint32_t sampleCount = sensorSamples_;
      const uint32_t busyUs = sensorBusyUs_;
      lastDiagnosticRate_ = static_cast<uint64_t>(sampleCount - lastSensorSamples) * 1000U / elapsedMs;
      lastDiagnosticBusyUs_ = static_cast<uint64_t>(busyUs - lastSensorBusyUs) * 1000U / elapsedMs;
      if (Serial) printDiagnostics(lastDiagnosticRate_, lastDiagnosticBusyUs_);
      lastSensorSamples = sampleCount;
      lastSensorBusyUs = busyUs;
      lastSensorDiagMs = diagNowMs;
    }
    if (!diagnosticRadioEnabled_) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    updateRadioPower();
    applyPendingRadioConfig();

    Downlink downlink{};
    for (uint8_t received = 0; received < 16 && downlinkQueue_ != nullptr &&
         xQueueReceive(downlinkQueue_, &downlink, 0) == pdTRUE; ++received) {
      processDownlink(downlink);
    }
    if (stateAckPending_ && !txBusy_) sendStateAck(pendingStateAckSeq_, pendingStateAckTimeUs_);

    if (!loadMasterMac(masterMac)) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (!esp_now_is_peer_exist(masterMac)) {
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, masterMac, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      peerInfo.ifidx = WIFI_IF_STA;
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(20)); // Do not spin if peer creation repeatedly fails.
        continue;
      }
    }

    const uint8_t state = currentState();
    const uint32_t nowMs = millis();
    if ((state == capture::STATE_SYNC || state == capture::STATE_STREAM_PENDING || state == capture::STATE_STREAM) &&
        static_cast<uint32_t>(nowMs - lastSyncProbeMs_) >= SYNC_PROBE_INTERVAL_MS) {
      sendSyncProbe();
      lastSyncProbeMs_ = nowMs;
    }

    if (static_cast<uint32_t>(nowMs - lastSyncDiagMs_) >= SYNC_DIAG_INTERVAL_MS) {
      if (state == capture::STATE_SYNC || state == capture::STATE_STREAM_PENDING) {
        sendSyncDiag();
      } else if (state == capture::STATE_STREAM) {
        updateClockFit();
      }
      lastSyncDiagMs_ = millis();
    }

    if (state == capture::STATE_STREAM_PENDING &&
        static_cast<int32_t>(localToMasterTimeUs(micros()) - pendingStreamStartUs_) >= 0) {
      portENTER_CRITICAL(&offsetMux_);
      state_ = capture::STATE_STREAM;
      streamStarted_ = true;
      freezeSampleClockLocked();
      portEXIT_CRITICAL(&offsetMux_);
      sendStateAck(lastCommandSeq_, pendingStreamStartUs_);
    }

    if (currentState() != capture::STATE_STREAM || !streamStarted_) {
      vTaskDelay(pdMS_TO_TICKS(currentState() == capture::STATE_IDLE ? 20 : 2));
      continue;
    }

    const uint32_t nowUs = micros();
    const uint32_t masterNowUs = localToMasterTimeUs(nowUs);
    const uint32_t slotIndex = masterNowUs / SEND_SUPERFRAME_US;
    const uint32_t phaseUs = masterNowUs % SEND_SUPERFRAME_US;
    const bool accel = config_.sensorMode == SensorMode::Accel;
    const uint8_t window = accel && phaseUs >= config_.sendSlotOffsetUs + 5000U ? 1 : 0;
    const uint32_t windowStart = config_.sendSlotOffsetUs + static_cast<uint32_t>(window) * 5000U;
    const uint64_t windowKey = (static_cast<uint64_t>(slotIndex) << 1) | window;
    if (windowKey == lastWindowKey || phaseUs < windowStart || phaseUs >= windowStart + 5000U) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    lastWindowKey = windowKey;

    const uint8_t sendLimit = accel && window == 1 ? 1 : MAX_SENDS_PER_SLOT;
    for (uint8_t sendCount = 0; sendCount < sendLimit; ++sendCount) {
      const uint32_t currentPhase = localToMasterTimeUs(micros()) % SEND_SUPERFRAME_US;
      if (currentPhase < windowStart || currentPhase >= windowStart + 5000U) break;
      // 每个 slot 最多追发 2 个 batch，避免重发挤占下一批新数据的 ACK 时间。
      if (txBusy_ || !takeSendCandidate(packet, micros())) {
        break;
      }

      const uint32_t sendTimeoutMs = accel ? 3U : 8U;
      if (!sendPacket(masterMac, packet.bytes, sizeof(packet.bytes), sendTimeoutMs)) break;
    }
  }
}

// 以 1kHz 采样 IMU，按 IMU_BATCH_SIZE 组包后写入本地 ring。
void ImuSlaveApp::sensorTask() {
  // Both unwired sensors use the timer ISR; a wired gyro may use DATA_RDY.
  sensorTaskHandle_ = xTaskGetCurrentTaskHandle();
  uint32_t nextSampleUs = micros();
  const bool timedSampling = !dataReadyInterruptEnabled_;
  bool timerRunning = false;
  if (timedSampling) {
    // Allocate the interrupt on the sensor core, independently of Wi-Fi task timing.
    sampleTimer_ = timerBegin(0, 80, true); // 80 MHz APB / 80 = 1 MHz.
    if (sampleTimer_ == nullptr) {
      Serial.println("ERR IMU sample timer creation failed");
      vTaskDelete(nullptr);
      return;
    }
    timerAttachInterrupt(sampleTimer_, onSampleTimer, false);
    timerAlarmWrite(sampleTimer_, SAMPLE_PERIOD_US, true);
  }

  capture::ImuRawSample batchSamples[capture::IMU_BATCH_SIZE] = {};
  uint8_t batchCount = 0;
  bool wasStreaming = false;
  bool useDataReadyInterrupt = dataReadyInterruptEnabled_;
  uint8_t dataReadyTimeouts = 0;

  for (;;) {
    if (!sensorReady_) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      sensorReady_ = config_.sensorMode == SensorMode::Accel ? adxl_.begin() : mpu_.begin();
      if (sensorReady_) {
        Serial.printf("SENSOR_INIT_OK DEVID=0x%02X\n", config_.sensorMode == SensorMode::Accel ? adxl_.deviceId() : mpu_.deviceId());
        nextSampleUs = micros();
      } else {
        Serial.printf("ERR: SENSOR_INIT_FAILED DEVID=0x%02X; retrying\n", config_.sensorMode == SensorMode::Accel ? adxl_.deviceId() : mpu_.deviceId());
      }
      continue;
    }
    if (currentState() != capture::STATE_STREAM || !streamStarted_) {
      if (timerRunning) {
        timerAlarmDisable(sampleTimer_);
        timerRunning = false;
        ulTaskNotifyTake(pdTRUE, 0);
      }
      if (batchCount > 0) {
        batchCount = 0;
      }
      wasStreaming = false;
      useDataReadyInterrupt = dataReadyInterruptEnabled_;
      dataReadyTimeouts = 0;
      if (dataReadyInterruptEnabled_) {
        ulTaskNotifyTake(pdTRUE, 0);
      }
      vTaskDelay(pdMS_TO_TICKS(timedSampling && currentState() == capture::STATE_IDLE ? 20 : 2));
      nextSampleUs = micros();
      continue;
    }

    if (!wasStreaming) {
      nextSampleUs = micros();
      batchCount = 0;
      useDataReadyInterrupt = dataReadyInterruptEnabled_;
      dataReadyTimeouts = 0;
      wasStreaming = true;
      if (timedSampling) {
        ulTaskNotifyTake(pdTRUE, 0);
        nextSampleUs += SAMPLE_PERIOD_US;
        timerWrite(sampleTimer_, 0);
        timerAlarmEnable(sampleTimer_);
        timerRunning = true;
      }
    }

    uint32_t localTimeUs = 0;
    if (timedSampling) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) continue;
      if (currentState() != capture::STATE_STREAM || !streamStarted_) continue;
      localTimeUs = micros();
      const auto tick = imu_sampling::advance(localTimeUs, nextSampleUs, SAMPLE_PERIOD_US);
      if (!tick.due) continue;
      nextSampleUs = tick.nextUs;
      if (tick.missed > 0) {
        noteDroppedSamples(static_cast<uint32_t>(batchCount) + tick.missed);
        batchCount = 0;
      }
    } else if (useDataReadyInterrupt) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0) {
        if (++dataReadyTimeouts >= 3) {
          useDataReadyInterrupt = false;
          nextSampleUs = micros();
        }
        batchCount = 0;
        continue;
      }
      dataReadyTimeouts = 0;
      localTimeUs = lastDataReadyLocalUs_;
    } else {
      while (static_cast<int32_t>(micros() - nextSampleUs) < 0) {
        taskYIELD();
      }
      nextSampleUs += SAMPLE_PERIOD_US;
      localTimeUs = micros();
    }

    uint32_t sampleTimeUs = localToSampleTimeUs(localTimeUs);
    portENTER_CRITICAL(&ringMux_);
    if (hasLastSampleTimestamp_ && static_cast<int32_t>(sampleTimeUs - lastSampleTimestampUs_) <= 0) {
      sampleTimeUs = lastSampleTimestampUs_ + 1U;
    }
    lastSampleTimestampUs_ = sampleTimeUs;
    hasLastSampleTimestamp_ = true;
    portEXIT_CRITICAL(&ringMux_);

    capture::ImuRawSample sample{};
    sample.timestampUs = sampleTimeUs;
    if (timedSampling && batchCount > 0 &&
        static_cast<uint32_t>(sampleTimeUs - batchSamples[0].timestampUs) > UINT16_MAX) {
      // Never silently wrap the wire format's 16-bit relative timestamp.
      noteDroppedSamples(batchCount);
      batchCount = 0;
    }

    const uint32_t readStartUs = micros();
    if (!readSensor(sample)) {
      noteDroppedSamples(static_cast<uint32_t>(batchCount) + 1U);
      batchCount = 0;
      continue;
    }
    ++sensorSamples_;
    sensorBusyUs_ += micros() - readStartUs;

    batchSamples[batchCount++] = sample;

    if (batchCount >= capture::IMU_BATCH_SIZE) {
      storeBatch(batchSamples, batchCount);
      batchCount = 0;
    }
  }
}

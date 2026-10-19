#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stdint.h>

#include "capture_common.h"

class Ads1298_48chDriver {
 public:
  struct Pins {
    uint8_t cs[6];
    uint8_t mosi;
    uint8_t miso;
    uint8_t sck;
    uint8_t drdy;
    uint8_t reset;
    uint8_t start;
  };

  explicit Ads1298_48chDriver(const Pins &pins);

  void begin(void (*drdyIsr)());
  bool verifyAndRepair();
  bool readChannels(int32_t channels[capture::EMG48_CHANNEL_COUNT]);

  uint32_t configVerifyFailureCount() const { return configVerifyFailures_; }
  uint8_t lastId(size_t adsIndex) const { return adsIndex < ADS_COUNT ? lastId_[adsIndex] : 0; }
  bool lastVerifyOk(size_t adsIndex) const { return adsIndex < ADS_COUNT ? lastVerifyOk_[adsIndex] : false; }

 private:
  static constexpr size_t ADS_COUNT = 6;
  static constexpr size_t ADS_CHANNELS = 8;
  static constexpr size_t ADS_FRAME_BYTES = 27;
  static constexpr uint8_t CMD_WAKEUP = 0x02;
  static constexpr uint8_t CMD_SDATAC = 0x11;
  static constexpr uint8_t CMD_RDATAC = 0x10;
  static constexpr uint8_t REG_ID = 0x00;
  static constexpr uint8_t REG_CONFIG1 = 0x01;
  static constexpr uint8_t REG_CONFIG2 = 0x02;
  static constexpr uint8_t REG_CONFIG3 = 0x03;
  static constexpr uint8_t REG_CH1SET = 0x05;
  static constexpr uint8_t REG_RLD_SENSP = 0x0D;
  static constexpr uint8_t REG_RLD_SENSN = 0x0E;

  void deselectAll();
  void select(size_t adsIndex);
  void release();
  void sendCmd(size_t adsIndex, uint8_t cmd);
  void sendCmdAll(uint8_t cmd);
  void writeReg(size_t adsIndex, uint8_t addr, uint8_t val);
  uint8_t readReg(size_t adsIndex, uint8_t addr);
  void writeConfigOnce(size_t adsIndex);
  bool verifyConfig(size_t adsIndex);
  bool configureOne(size_t adsIndex);
  uint8_t expectedRldSenseP(size_t adsIndex) const;
  uint8_t expectedRldSenseN(size_t adsIndex) const;
  int32_t signExtend24(uint32_t data) const;

  Pins pins_;
  int selectedCsPin_ = -1;
  uint32_t configVerifyFailures_ = 0;
  uint8_t lastId_[ADS_COUNT] = {};
  bool lastVerifyOk_[ADS_COUNT] = {};
};

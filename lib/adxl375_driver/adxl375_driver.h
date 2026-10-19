#pragma once

#include <Arduino.h>
#include <SPI.h>

class Adxl375Driver {
 public:
  struct Pins { uint8_t cs, mosi, miso, sck; };
  explicit Adxl375Driver(Pins pins) : pins_(pins) {}
  bool begin();
  uint8_t deviceId() const { return deviceId_; }
  bool readSample(int16_t &x, int16_t &y, int16_t &z);

 private:
  void writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);
  Pins pins_;
  uint8_t deviceId_ = 0;
};

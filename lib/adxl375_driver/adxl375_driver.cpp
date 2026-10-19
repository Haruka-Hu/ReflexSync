#include "adxl375_driver.h"
#include "adxl375_math.h"

namespace {
constexpr uint8_t REG_DEVID = 0x00;
constexpr uint8_t REG_BW_RATE = 0x2C;
constexpr uint8_t REG_POWER_CTL = 0x2D;
constexpr uint8_t REG_DATA_FORMAT = 0x31;
constexpr uint8_t REG_DATAX0 = 0x32;
constexpr uint8_t REG_FIFO_CTL = 0x38;
const SPISettings SPI_CONFIG(2000000, MSBFIRST, SPI_MODE3);
}  // namespace

void Adxl375Driver::writeReg(uint8_t reg, uint8_t value) {
  SPI.beginTransaction(SPI_CONFIG);
  digitalWrite(pins_.cs, LOW);
  SPI.transfer(reg & 0x3f);
  SPI.transfer(value);
  digitalWrite(pins_.cs, HIGH);
  SPI.endTransaction();
}

uint8_t Adxl375Driver::readReg(uint8_t reg) {
  SPI.beginTransaction(SPI_CONFIG);
  digitalWrite(pins_.cs, LOW);
  SPI.transfer(adxl375::kReadBit | (reg & 0x3f));
  const uint8_t value = SPI.transfer(0);
  digitalWrite(pins_.cs, HIGH);
  SPI.endTransaction();
  return value;
}

bool Adxl375Driver::begin() {
  SPI.begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs);
  pinMode(pins_.cs, OUTPUT);
  digitalWrite(pins_.cs, HIGH);
  deviceId_ = readReg(REG_DEVID);
  if (deviceId_ != adxl375::kDeviceId) return false;
  writeReg(REG_POWER_CTL, 0x00);   // Configure while in standby.
  writeReg(REG_DATA_FORMAT, 0x0B); // 4-wire, active-high, right-justified.
  writeReg(REG_BW_RATE, adxl375::kRate1600); // 1600 samples/s.
  writeReg(REG_FIFO_CTL, 0x00);    // Bypass FIFO; read the latest sample directly.
  writeReg(REG_POWER_CTL, 0x08);   // Measurement mode.
  return readReg(REG_BW_RATE) == adxl375::kRate1600 &&
         readReg(REG_DATA_FORMAT) == 0x0B && readReg(REG_FIFO_CTL) == 0x00;
}

bool Adxl375Driver::readSample(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t raw[6];
  SPI.beginTransaction(SPI_CONFIG);
  digitalWrite(pins_.cs, LOW);
  SPI.transfer(adxl375::kReadBit | adxl375::kMultiBit | REG_DATAX0);
  for (uint8_t &byte : raw) byte = SPI.transfer(0);
  digitalWrite(pins_.cs, HIGH);
  SPI.endTransaction();
  x = adxl375::decodeAxis(raw[0], raw[1]);
  y = adxl375::decodeAxis(raw[2], raw[3]);
  z = adxl375::decodeAxis(raw[4], raw[5]);
  return true;
}

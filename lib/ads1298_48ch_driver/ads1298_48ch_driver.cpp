#include "ads1298_48ch_driver.h"

namespace {

// ======================== USER CONFIG ========================
// Channel count is controlled in complete 8-channel ADS blocks: 8, 16, ... 48.
constexpr size_t ENABLED_CHANNEL_COUNT = 48;

// true: route the ADS1298 internal square wave to all enabled channels.
// false: use the normal external electrode inputs configured below.
constexpr bool ENABLE_INTERNAL_TEST_SIGNAL = false;

// RLD configuration. The source channels start at logical CH01 and are selected
// in complete 8-channel ADS blocks.
constexpr bool ENABLE_RLD = true;
constexpr size_t RLD_SOURCE_CHANNEL_COUNT = 8;
constexpr bool RLD_USE_P_INPUTS = true;
constexpr bool RLD_USE_N_INPUTS = true;
// Block 0 is CH01..CH08 / CS5, whose RLDOUT is connected to the RLD electrode.
constexpr size_t RLD_OUTPUT_CHANNEL_BLOCK = 0;

// CHnSET gain bits for normal input: 0x10=1, 0x20=2, 0x30=3, 0x40=4,
// 0x00=6, 0x50=8, 0x60=12.
constexpr uint8_t NORMAL_CH_CONFIG = 0x00;

constexpr uint32_t DATA_SPI_HZ = 8000000;
constexpr uint32_t REGISTER_SPI_HZ = 1000000;
constexpr uint8_t CONFIG_RETRIES = 3;
constexpr uint32_t POWER_STABLE_DELAY_MS = 500;
// ====================== END USER CONFIG ======================

static_assert(ENABLED_CHANNEL_COUNT >= 8 && ENABLED_CHANNEL_COUNT <= 48 &&
                  ENABLED_CHANNEL_COUNT % 8 == 0,
              "ENABLED_CHANNEL_COUNT must be 8, 16, 24, 32, 40, or 48");
static_assert(RLD_SOURCE_CHANNEL_COUNT <= ENABLED_CHANNEL_COUNT &&
                  RLD_SOURCE_CHANNEL_COUNT % 8 == 0,
              "RLD source channels must be enabled and use complete ADS blocks");

const SPISettings ADS_READ_SPI_SETTINGS(DATA_SPI_HZ, MSBFIRST, SPI_MODE1);
const SPISettings ADS_REG_SPI_SETTINGS(REGISTER_SPI_HZ, MSBFIRST, SPI_MODE1);

// Physical CS order is not the logical channel order on the three-board stack.
constexpr size_t ADS_TO_CHANNEL_BLOCK[] = {4, 5, 2, 3, 0, 1};
constexpr uint8_t NORMAL_CONFIG2 = 0x40;   // Internal test source disabled.
constexpr uint8_t TEST_CONFIG2 = 0x55;     // Internal test source enabled.
constexpr uint8_t TEST_CH_CONFIG = 0x65;   // Gain 12, internal test signal MUX.
constexpr uint8_t DISABLED_CH_CONFIG = 0x81; // Channel power-down, internal input short.
constexpr uint8_t CONFIG3_RLD_ENABLED = 0xCC;
constexpr uint8_t CONFIG3_RLD_DISABLED = 0xC8;

constexpr bool isAdsEnabled(size_t adsIndex) {
  return ADS_TO_CHANNEL_BLOCK[adsIndex] * 8 < ENABLED_CHANNEL_COUNT;
}

constexpr uint8_t expectedChannelConfig(size_t adsIndex) {
  return !isAdsEnabled(adsIndex)
             ? DISABLED_CH_CONFIG
             : (ENABLE_INTERNAL_TEST_SIGNAL ? TEST_CH_CONFIG : NORMAL_CH_CONFIG);
}

constexpr uint8_t expectedConfig2() {
  return ENABLE_INTERNAL_TEST_SIGNAL ? TEST_CONFIG2 : NORMAL_CONFIG2;
}

constexpr bool isRldSourceAds(size_t adsIndex) {
  return !ENABLE_INTERNAL_TEST_SIGNAL && ENABLE_RLD &&
         ADS_TO_CHANNEL_BLOCK[adsIndex] * 8 < RLD_SOURCE_CHANNEL_COUNT;
}

constexpr bool isRldOutputAds(size_t adsIndex) {
  return !ENABLE_INTERNAL_TEST_SIGNAL && ENABLE_RLD &&
         ADS_TO_CHANNEL_BLOCK[adsIndex] == RLD_OUTPUT_CHANNEL_BLOCK;
}

constexpr uint8_t expectedConfig3(size_t adsIndex) {
  return isRldOutputAds(adsIndex) ? CONFIG3_RLD_ENABLED : CONFIG3_RLD_DISABLED;
}

}  // namespace

Ads1298_48chDriver::Ads1298_48chDriver(const Pins &pins) : pins_(pins) {}

void Ads1298_48chDriver::begin(void (*drdyIsr)()) {
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    pinMode(pins_.cs[i], OUTPUT);
    digitalWrite(pins_.cs[i], HIGH);
  }
  selectedCsPin_ = -1;

  pinMode(pins_.reset, OUTPUT);
  pinMode(pins_.start, OUTPUT);
  pinMode(pins_.drdy, INPUT_PULLUP);
  digitalWrite(pins_.reset, HIGH);
  digitalWrite(pins_.start, LOW);

  delay(POWER_STABLE_DELAY_MS);
  SPI.begin(pins_.sck, pins_.miso, pins_.mosi, pins_.cs[0]);

  digitalWrite(pins_.start, LOW);
  digitalWrite(pins_.reset, LOW);
  delay(50);
  digitalWrite(pins_.reset, HIGH);
  delay(300);

  for (size_t i = 0; i < ADS_COUNT; ++i) {
    configureOne(i);
  }

  sendCmdAll(CMD_RDATAC);
  delay(5);
  attachInterrupt(digitalPinToInterrupt(pins_.drdy), drdyIsr, FALLING);
  digitalWrite(pins_.start, HIGH);
}

bool Ads1298_48chDriver::verifyAndRepair() {
  digitalWrite(pins_.start, LOW);
  delay(5);
  sendCmdAll(CMD_SDATAC);
  delay(20);

  bool allOk = true;
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    bool ok = verifyConfig(i);
    for (uint8_t attempt = 1; !ok && attempt <= CONFIG_RETRIES; ++attempt) {
      ++configVerifyFailures_;
      writeConfigOnce(i);
      delay(10);
      ok = verifyConfig(i);
    }
    lastVerifyOk_[i] = ok;
    allOk = allOk && ok;
  }

  sendCmdAll(CMD_RDATAC);
  delay(5);
  digitalWrite(pins_.start, HIGH);
  delay(2);
  return allOk;
}

bool Ads1298_48chDriver::readChannels(int32_t channels[capture::EMG48_CHANNEL_COUNT]) {
  uint8_t rx[ADS_FRAME_BYTES] = {};

  SPI.beginTransaction(ADS_READ_SPI_SETTINGS);
  deselectAll();
  delayMicroseconds(1);

  for (size_t ads = 0; ads < ADS_COUNT; ++ads) {
    // Keep the shared DRDY/read sequence; disabled channel slots remain zero.
    digitalWrite(pins_.cs[ads], LOW);
    selectedCsPin_ = pins_.cs[ads];
    SPI.transferBytes(nullptr, rx, ADS_FRAME_BYTES);
    digitalWrite(pins_.cs[ads], HIGH);
    selectedCsPin_ = -1;
    delayMicroseconds(1);

    for (size_t ch = 0; ch < ADS_CHANNELS; ++ch) {
      const size_t offset = 3 + ch * 3;
      const uint32_t raw = (static_cast<uint32_t>(rx[offset]) << 16) |
                           (static_cast<uint32_t>(rx[offset + 1]) << 8) |
                           static_cast<uint32_t>(rx[offset + 2]);
      const size_t channelBlock = ADS_TO_CHANNEL_BLOCK[ads];
      channels[channelBlock * ADS_CHANNELS + ch] = isAdsEnabled(ads) ? signExtend24(raw) : 0;
    }
  }

  SPI.endTransaction();
  return true;
}

void Ads1298_48chDriver::deselectAll() {
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    digitalWrite(pins_.cs[i], HIGH);
  }
  selectedCsPin_ = -1;
}

void Ads1298_48chDriver::select(size_t adsIndex) {
  deselectAll();
  delayMicroseconds(2);
  digitalWrite(pins_.cs[adsIndex], LOW);
  selectedCsPin_ = pins_.cs[adsIndex];
}

void Ads1298_48chDriver::release() {
  if (selectedCsPin_ >= 0) {
    digitalWrite(selectedCsPin_, HIGH);
    selectedCsPin_ = -1;
  }
  delayMicroseconds(2);
}

void Ads1298_48chDriver::sendCmd(size_t adsIndex, uint8_t cmd) {
  SPI.beginTransaction(ADS_REG_SPI_SETTINGS);
  select(adsIndex);
  SPI.transfer(cmd);
  release();
  SPI.endTransaction();
  delayMicroseconds(8);
}

void Ads1298_48chDriver::sendCmdAll(uint8_t cmd) {
  for (size_t i = 0; i < ADS_COUNT; ++i) {
    sendCmd(i, cmd);
  }
}

void Ads1298_48chDriver::writeReg(size_t adsIndex, uint8_t addr, uint8_t val) {
  SPI.beginTransaction(ADS_REG_SPI_SETTINGS);
  select(adsIndex);
  SPI.transfer(0x40U | (addr & 0x1FU));
  SPI.transfer(0x00);
  SPI.transfer(val);
  release();
  SPI.endTransaction();
  delay(2);
}

uint8_t Ads1298_48chDriver::readReg(size_t adsIndex, uint8_t addr) {
  SPI.beginTransaction(ADS_REG_SPI_SETTINGS);
  select(adsIndex);
  SPI.transfer(0x20U | (addr & 0x1FU));
  SPI.transfer(0x00);
  const uint8_t value = SPI.transfer(0x00);
  release();
  SPI.endTransaction();
  delayMicroseconds(8);
  return value;
}

void Ads1298_48chDriver::writeConfigOnce(size_t adsIndex) {
  writeReg(adsIndex, REG_CONFIG3, expectedConfig3(adsIndex));
  writeReg(adsIndex, REG_CONFIG1, 0x84);
  writeReg(adsIndex, REG_CONFIG2, expectedConfig2());
  const uint8_t channelConfig = expectedChannelConfig(adsIndex);
  for (uint8_t ch = 0; ch < ADS_CHANNELS; ++ch) {
    writeReg(adsIndex, REG_CH1SET + ch, channelConfig);
  }
  writeReg(adsIndex, REG_RLD_SENSP, expectedRldSenseP(adsIndex));
  writeReg(adsIndex, REG_RLD_SENSN, expectedRldSenseN(adsIndex));
}

bool Ads1298_48chDriver::verifyConfig(size_t adsIndex) {
  bool ok = true;
  ok = ok && (readReg(adsIndex, REG_CONFIG1) == 0x84);
  ok = ok && (readReg(adsIndex, REG_CONFIG2) == expectedConfig2());
  ok = ok && (readReg(adsIndex, REG_CONFIG3) == expectedConfig3(adsIndex));
  const uint8_t channelConfig = expectedChannelConfig(adsIndex);
  for (uint8_t ch = 0; ch < ADS_CHANNELS; ++ch) {
    ok = ok && (readReg(adsIndex, REG_CH1SET + ch) == channelConfig);
  }
  ok = ok && (readReg(adsIndex, REG_RLD_SENSP) == expectedRldSenseP(adsIndex));
  ok = ok && (readReg(adsIndex, REG_RLD_SENSN) == expectedRldSenseN(adsIndex));
  lastVerifyOk_[adsIndex] = ok;
  return ok;
}

bool Ads1298_48chDriver::configureOne(size_t adsIndex) {
  sendCmd(adsIndex, CMD_WAKEUP);
  delay(10);
  sendCmd(adsIndex, CMD_SDATAC);
  delay(20);
  lastId_[adsIndex] = readReg(adsIndex, REG_ID);

  bool ok = false;
  for (uint8_t attempt = 1; attempt <= CONFIG_RETRIES; ++attempt) {
    writeConfigOnce(adsIndex);
    delay(10);
    ok = verifyConfig(adsIndex);
    if (ok) {
      break;
    }
    ++configVerifyFailures_;
  }
  lastVerifyOk_[adsIndex] = ok;
  return ok;
}

uint8_t Ads1298_48chDriver::expectedRldSenseP(size_t adsIndex) const {
  return RLD_USE_P_INPUTS && isRldSourceAds(adsIndex) ? 0xFF : 0x00;
}

uint8_t Ads1298_48chDriver::expectedRldSenseN(size_t adsIndex) const {
  return RLD_USE_N_INPUTS && isRldSourceAds(adsIndex) ? 0xFF : 0x00;
}

int32_t Ads1298_48chDriver::signExtend24(uint32_t data) const {
  if ((data & 0x800000UL) != 0) {
    return static_cast<int32_t>(data | 0xFF000000UL);
  }
  return static_cast<int32_t>(data);
}

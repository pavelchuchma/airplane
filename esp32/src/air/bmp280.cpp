#include "bmp280.h"

#include <Arduino.h>
#include <math.h>

#include "i2cbus.h"

namespace bmp280 {
namespace {

constexpr uint8_t kExpectedChipId = 0x58;  // 0x60 would be a BME280

constexpr uint8_t kRegCalibStart = 0x88;  // dig_T1..dig_P9, 24 bytes
constexpr uint8_t kRegChipId = 0xD0;
constexpr uint8_t kRegReset = 0xE0;
constexpr uint8_t kRegCtrlMeas = 0xF4;
constexpr uint8_t kRegConfig = 0xF5;
constexpr uint8_t kRegDataStart = 0xF7;  // press msb/lsb/xlsb, temp msb/lsb/xlsb

constexpr uint8_t kResetCommand = 0xB6;
// t_standby 0.5 ms | IIR filter coefficient 16 | SPI 3-wire off
constexpr uint8_t kConfigValue = (0 << 5) | (4 << 2) | 0;
// temperature oversampling x2 | pressure oversampling x16 | normal mode
constexpr uint8_t kCtrlMeasValue = (2 << 5) | (5 << 2) | 3;

uint8_t deviceAddress = 0x76;

// Factory calibration (datasheet 3.11.2), loaded once in begin().
uint16_t digT1;
int16_t digT2, digT3;
uint16_t digP1;
int16_t digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;

int32_t tFine;  // shared between the T and P compensation formulas

bool writeRegister(uint8_t reg, uint8_t value) {
  return i2cbus::writeRegs(deviceAddress, reg, &value, 1) == i2cbus::Status::Ok;
}

bool readRegisters(uint8_t reg, uint8_t *buf, uint8_t len) {
  return i2cbus::readRegs(deviceAddress, reg, buf, len) == i2cbus::Status::Ok;
}

bool readCalibration() {
  uint8_t c[24];
  if (!readRegisters(kRegCalibStart, c, sizeof(c))) {
    return false;
  }
  digT1 = (uint16_t)(c[1] << 8 | c[0]);
  digT2 = (int16_t)(c[3] << 8 | c[2]);
  digT3 = (int16_t)(c[5] << 8 | c[4]);
  digP1 = (uint16_t)(c[7] << 8 | c[6]);
  digP2 = (int16_t)(c[9] << 8 | c[8]);
  digP3 = (int16_t)(c[11] << 8 | c[10]);
  digP4 = (int16_t)(c[13] << 8 | c[12]);
  digP5 = (int16_t)(c[15] << 8 | c[14]);
  digP6 = (int16_t)(c[17] << 8 | c[16]);
  digP7 = (int16_t)(c[19] << 8 | c[18]);
  digP8 = (int16_t)(c[21] << 8 | c[20]);
  digP9 = (int16_t)(c[23] << 8 | c[22]);
  return true;
}

// Datasheet 3.11.3, returns temperature in 0.01 degC and sets tFine.
int32_t compensateTemperature(int32_t adcT) {
  int32_t var1 = ((((adcT >> 3) - ((int32_t)digT1 << 1))) * ((int32_t)digT2)) >> 11;
  int32_t var2 = (((((adcT >> 4) - ((int32_t)digT1)) * ((adcT >> 4) - ((int32_t)digT1))) >> 12) *
                  ((int32_t)digT3)) >>
                 14;
  tFine = var1 + var2;
  return (tFine * 5 + 128) >> 8;
}

// Datasheet 3.11.3 (64-bit variant), returns pressure in Pa as Q24.8.
uint32_t compensatePressure(int32_t adcP) {
  int64_t var1 = ((int64_t)tFine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)digP6;
  var2 = var2 + ((var1 * (int64_t)digP5) << 17);
  var2 = var2 + (((int64_t)digP4) << 35);
  var1 = ((var1 * var1 * (int64_t)digP3) >> 8) + ((var1 * (int64_t)digP2) << 12);
  var1 = ((((int64_t)1) << 47) + var1) * ((int64_t)digP1) >> 33;
  if (var1 == 0) {
    return 0;  // avoid division by zero
  }
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)digP9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)digP8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)digP7) << 4);
  return (uint32_t)p;
}

}  // namespace

bool begin(uint8_t address) {
  deviceAddress = address;

  uint8_t chipId = 0;
  if (!readChipId(chipId) || chipId != kExpectedChipId) {
    return false;
  }

  writeRegister(kRegReset, kResetCommand);
  delay(10);  // startup after reset takes max 2 ms

  if (!readCalibration()) {
    return false;
  }
  // Order matters: config is ignored in normal mode, so write it first.
  writeRegister(kRegConfig, kConfigValue);
  writeRegister(kRegCtrlMeas, kCtrlMeasValue);
  delay(50);  // first conversion at x16/x2 oversampling takes ~44 ms
  return true;
}

bool readChipId(uint8_t &chipId) {
  return readRegisters(kRegChipId, &chipId, 1);
}

bool read(float &pressurePa, float &temperatureC) {
  uint8_t d[6];
  if (!readRegisters(kRegDataStart, d, sizeof(d))) {
    return false;
  }
  int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);

  temperatureC = compensateTemperature(adcT) / 100.0f;
  pressurePa = compensatePressure(adcP) / 256.0f;
  return true;
}

float altitudeFromPressure(float pressurePa, float referencePa) {
  return 44330.0f * (1.0f - powf(pressurePa / referencePa, 0.190295f));
}

}  // namespace bmp280

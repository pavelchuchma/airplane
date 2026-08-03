#include "ms4525do.h"

#include <Arduino.h>
#include <Wire.h>

namespace ms4525do {
namespace {

// Output type A transfer function: 10-90 % of 16383 counts over the
// +-1 psi span, zero differential pressure at the 50 % midpoint.
constexpr float kPressureRangePsi = 1.0f;
constexpr float kPsiToPa = 6894.757f;
constexpr int kZeroCounts = 8192;
constexpr float kCountsPerPsi = 0.8f * 16383.0f / (2.0f * kPressureRangePsi);
// Resting counts further than this from the midpoint mean a bad clone or a
// blocked port (doc/parts/pitot-ms4525do.md); ~1500 counts ~ 1580 Pa.
constexpr int kSanityCountTolerance = 1500;
// Output type A only ever produces 10-90 % of the count range; anything
// outside is a bus artifact (an all-zero read decodes as status 0 and
// would poison the filter with -8.6 kPa) or a broken sensor.
constexpr uint16_t kMinValidCounts = 1638;   // 10 % of 16383
constexpr uint16_t kMaxValidCounts = 14745;  // 90 % of 16383

uint8_t deviceAddress = 0x28;

// Status bits (b0 >> 6): 0 = fresh, 2 = stale (conversion not finished
// since the last read - impossible to hit at our 50 Hz vs its 2 kHz),
// 3 = fault. Anything non-fresh fails the read.
bool readRaw(uint16_t &pressureCounts, uint16_t &temperatureCounts) {
  if (Wire.requestFrom(deviceAddress, (uint8_t)4) != 4) {
    return false;
  }
  uint8_t b0 = Wire.read();
  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint8_t b3 = Wire.read();
  if ((b0 >> 6) != 0) {
    return false;
  }
  pressureCounts = ((b0 & 0x3F) << 8) | b1;
  temperatureCounts = (uint16_t)(((b2 << 8) | b3) >> 5);
  return pressureCounts >= kMinValidCounts && pressureCounts <= kMaxValidCounts;
}

}  // namespace

bool begin(uint8_t address) {
  deviceAddress = address;

  Wire.beginTransmission(deviceAddress);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delay(10);  // first data is available 8.4 ms after power-on

  uint16_t pressureCounts, temperatureCounts;
  if (!readRaw(pressureCounts, temperatureCounts)) {
    return false;
  }
  return abs((int)pressureCounts - kZeroCounts) <= kSanityCountTolerance;
}

bool read(float &diffPressPa, float &temperatureC) {
  uint16_t pressureCounts, temperatureCounts;
  if (!readRaw(pressureCounts, temperatureCounts)) {
    return false;
  }
  // Negated: airflow into the dynamic port drives counts BELOW the midpoint
  // on this breakout (same convention as the ArduPilot/PX4 MS4525 drivers),
  // so positive dP = positive airspeed. Un-negated, every real airflow went
  // negative and the IAS clamp pinned the HUD to 0.
  diffPressPa = (kZeroCounts - (int)pressureCounts) * (kPsiToPa / kCountsPerPsi);
  temperatureC = temperatureCounts * (200.0f / 2047.0f) - 50.0f;
  return true;
}

}  // namespace ms4525do

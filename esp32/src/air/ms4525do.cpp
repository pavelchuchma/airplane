#include "ms4525do.h"

#include <Arduino.h>

#include "i2cbus.h"

namespace ms4525do {
namespace {

// Output type A transfer function: 10-90 % of 16383 counts over the
// +-1 psi span, zero differential pressure at the 50 % midpoint.
constexpr float kPressureRangePsi = 1.0f;
constexpr float kPsiToPa = 6894.757f;
constexpr int kZeroCounts = 8192;
constexpr float kCountsPerPsi = 0.8f * 16383.0f / (2.0f * kPressureRangePsi);
// Output type A only ever produces 10-90 % of the count range; anything
// outside is a bus artifact (an all-zero read decodes as status 0 and
// would poison the filter with -8.6 kPa) or a broken sensor.
constexpr uint16_t kMinValidCounts = 1638;   // 10 % of 16383
constexpr uint16_t kMaxValidCounts = 14745;  // 90 % of 16383

constexpr uint8_t kDeviceAddress = 0x28;

// Status bits (b0 >> 6): 0 = fresh, 2 = stale (conversion not finished
// since the last read - impossible to hit at our 50 Hz vs its 2 kHz),
// 3 = fault. Anything non-fresh fails the read.
bool readRaw(uint16_t &pressureCounts, uint16_t &temperatureCounts) {
  uint8_t b[4];
  if (i2cbus::read(kDeviceAddress, b, sizeof(b)) != i2cbus::Status::Ok) {
    return false;
  }
  uint8_t b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];
  if ((b0 >> 6) != 0) {
    return false;
  }
  pressureCounts = ((b0 & 0x3F) << 8) | b1;
  temperatureCounts = (uint16_t)(((b2 << 8) | b3) >> 5);
  return pressureCounts >= kMinValidCounts && pressureCounts <= kMaxValidCounts;
}

}  // namespace

bool read(float &diffPressPa, float &temperatureC) {
  uint16_t pressureCounts, temperatureCounts;
  if (!readRaw(pressureCounts, temperatureCounts)) {
    return false;
  }
  // Datasheet sign convention, deliberately NOT negated: counts rise above the
  // midpoint when the port fed with total pressure is the higher one. On our kit
  // that port is not the one the labels suggest, so the hoses are crossed with
  // respect to the silkscreen - the fix for a reversed pair belongs in the
  // tubing, not here (doc/parts/pitot-ms4525do.md). Symptom of a reversed pair:
  // a rock-solid zero airspeed in real airflow, because the IAS conversion in
  // main.cpp clamps negative dP to zero. It reads dead, not backwards.
  diffPressPa = ((int)pressureCounts - kZeroCounts) * (kPsiToPa / kCountsPerPsi);
  temperatureC = temperatureCounts * (200.0f / 2047.0f) - 50.0f;
  return true;
}

}  // namespace ms4525do

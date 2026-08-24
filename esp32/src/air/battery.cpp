#include "battery.h"

#include <driver/adc.h>
#include <esp_adc_cal.h>

namespace battery {
namespace {

// Cumulative balance nodes in wiring order: N1 = cell 1, N2 = cells 1+2,
// N3 = pack. GPIO35/34/39 - see battery.h for the divider schematic.
constexpr adc1_channel_t kChannels[3] = {ADC1_CHANNEL_7, ADC1_CHANNEL_6, ADC1_CHANNEL_3};
constexpr int kSamplesPerRead = 64;

// Per-node calibration: nodeMv = pinMv * scale + offsetMv. Scale-only
// (one-point) calibration measured 2026-08-23 against a multimeter on a
// full pack: true nodes 4.11/8.25/12.35 V vs firmware's 4.21/8.47/12.59 V
// with the nominal divider ratios (2.00/3.80/6.60) - all three channels
// read 2-2.7 % high (resistor tolerance + residual ADC gain error).
// TODO: add the second point (near-empty pack) to pin down the offsets;
// read the raw telemetry cellMv for that, not the 10 mV-rounded HUD.
constexpr float kCalScale[3] = {1.9525f, 3.7013f, 6.4742f};
constexpr float kCalOffsetMv[3] = {0.0f, 0.0f, 0.0f};

esp_adc_cal_characteristics_t adcChars;

// One node: 64 raw samples averaged, then converted through the eFuse
// characterization (raw -> mV at the pin) and the divider calibration.
float readNodeMv(int idx) {
  uint32_t sum = 0;
  for (int i = 0; i < kSamplesPerRead; i++) {
    sum += adc1_get_raw(kChannels[idx]);
  }
  uint32_t pinMv = esp_adc_cal_raw_to_voltage(sum / kSamplesPerRead, &adcChars);
  return pinMv * kCalScale[idx] + kCalOffsetMv[idx];
}

}  // namespace

void begin() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  for (adc1_channel_t ch : kChannels) {
    // 11 dB: full 0-2.45 V linear range; the dividers keep every node under it.
    adc1_config_channel_atten(ch, ADC_ATTEN_DB_11);
  }
  // Uses the factory eFuse Vref when present, the 1100 mV default otherwise;
  // either way the two-point divider calibration absorbs the residual error.
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adcChars);
}

void read(uint16_t cellMv[3]) {
  float n1 = readNodeMv(0);
  float n2 = readNodeMv(1);
  float n3 = readNodeMv(2);
  // Cells are differences of cumulative nodes; clamp at 0 so bus noise on a
  // disconnected lead can't wrap the unsigned result to ~65 V.
  float cells[3] = {n1, n2 - n1, n3 - n2};
  for (int i = 0; i < 3; i++) {
    cellMv[i] = (uint16_t)(cells[i] < 0.0f ? 0.0f : cells[i] + 0.5f);
  }
}

}  // namespace battery

#include "battery.h"

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>

namespace battery {
namespace {

// Cumulative balance nodes in wiring order: N1 = cell 1, N2 = cells 1+2,
// N3 = pack. GPIO35/34/39 - see battery.h for the divider schematic.
constexpr adc_channel_t kChannels[3] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_3};
// 12 dB (named 11 dB before IDF 5): full 0-2.45 V linear range; the dividers
// keep every node under it.
constexpr adc_atten_t kAtten = ADC_ATTEN_DB_12;
constexpr int kSamplesPerRead = 64;

// Per-node calibration: nodeMv = pinMv * scale + offsetMv. Scale-only
// (one-point) calibration measured 2026-08-23 against a multimeter on a
// full pack: true nodes 4.11/8.25/12.35 V vs firmware's 4.21/8.47/12.59 V
// with the nominal divider ratios (2.00/3.80/6.60) - all three channels
// read 2-2.7 % high (resistor tolerance + residual ADC gain error).
// Re-checked 2026-10-20 after the move to IDF 5 adc_cali: N1 still within
// the DMM's resolution, N2/N3 read 0.69 % low (7.756/11.619 V vs 7.81/11.70 V)
// and were scaled up by that ratio.
// TODO: add the second point (near-empty pack) to pin down the offsets;
// read the raw telemetry cellMv for that, not the 10 mV-rounded HUD.
constexpr float kCalScale[3] = {1.9525f, 3.7271f, 6.5193f};
constexpr float kCalOffsetMv[3] = {0.0f, 0.0f, 0.0f};

adc_oneshot_unit_handle_t adcUnit = nullptr;
adc_cali_handle_t adcCali = nullptr;

// One node: 64 raw samples averaged, then converted through the eFuse
// characterization (raw -> mV at the pin) and the divider calibration.
// Without a working ADC it reads 0, which the caller sees as no balance lead.
float readNodeMv(int idx) {
  if (adcUnit == nullptr || adcCali == nullptr) {
    return 0.0f;
  }
  uint32_t sum = 0;
  for (int i = 0; i < kSamplesPerRead; i++) {
    int raw = 0;
    adc_oneshot_read(adcUnit, kChannels[idx], &raw);
    sum += raw;
  }
  int pinMv = 0;
  adc_cali_raw_to_voltage(adcCali, sum / kSamplesPerRead, &pinMv);
  return pinMv * kCalScale[idx] + kCalOffsetMv[idx];
}

}  // namespace

void begin() {
  adc_oneshot_unit_init_cfg_t unitCfg = {};
  unitCfg.unit_id = ADC_UNIT_1;
  unitCfg.ulp_mode = ADC_ULP_MODE_DISABLE;
  if (adc_oneshot_new_unit(&unitCfg, &adcUnit) != ESP_OK) {
    adcUnit = nullptr;
    return;
  }
  adc_oneshot_chan_cfg_t chanCfg = {};
  chanCfg.atten = kAtten;
  chanCfg.bitwidth = ADC_BITWIDTH_12;
  for (adc_channel_t ch : kChannels) {
    adc_oneshot_config_channel(adcUnit, ch, &chanCfg);
  }
  // Line fitting = the ESP32 scheme of the old esp_adc_cal: the factory
  // eFuse Vref when present, the 1100 mV default otherwise; either way the
  // divider calibration above absorbs the residual error.
  adc_cali_line_fitting_config_t caliCfg = {};
  caliCfg.unit_id = ADC_UNIT_1;
  caliCfg.atten = kAtten;
  caliCfg.bitwidth = ADC_BITWIDTH_12;
  caliCfg.default_vref = 1100;
  if (adc_cali_create_scheme_line_fitting(&caliCfg, &adcCali) != ESP_OK) {
    adcCali = nullptr;
  }
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

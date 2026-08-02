/*
 * Air side: sensors, servos, autopilot.
 *
 * Reads the GY-91 sensor board over I2C (Wire):
 *   - accelerometer/gyroscope (MPU-6500 at 0x68; the board is sold as
 *     MPU-9250 but ours is a 6-axis clone without the AK8963 magnetometer).
 *     Roll/pitch come from a complementary filter; yaw is gyro-integration
 *     only, so it drifts (planned fix: GPS course-over-ground correction,
 *     not a magnetometer).
 *   - BMP280 barometer (0x76): altitude relative to the boot reference
 *     pressure plus a low-pass-filtered climb rate (vario).
 *
 * Reads the MS4525DO pitot sensor (0x28, same I2C bus): differential
 * pressure zeroed at boot (stationary airplane) and low-pass filtered,
 * plus indicated airspeed at fixed sea-level density.
 *
 * Reads the Quescan G10A-F30 GPS (u-blox M10) over UART2 (see gps.h).
 *
 * Telemetry goes out over ESP-NOW as one TelemetryMessage per sample
 * (50 Hz, lib/protocol/messages.h) to the ground bridge, which forwards it
 * framed over USB serial to cocpitView. Commands (CommandMessage, 50 Hz
 * from cocpitView) arrive the same way in reverse; no command for
 * kFailsafeTimeoutMs raises the failsafe flag in telemetry — the only
 * failsafe action for now, there are no servos yet. Telemetry also carries
 * link stats: RSSI of the peer's frames (via a promiscuous-mode callback —
 * the IDF 4.4 ESP-NOW rx callback has no RSSI), command-loss and
 * telemetry-NACK percentages over 1 s windows.
 *
 * USB serial (115200 baud) carries only '#'-prefixed boot/status
 * diagnostics — the CSV telemetry output is gone.
 *
 * The peer (ground board) MAC comes from -DPEER_MAC_STR, injected from
 * [env:ground] custom_board_mac by tools/peer_mac.py.
 *
 * Wiring GY-91 -> ESP32:
 *   VIN -> 3V3, GND -> GND, SCL -> GPIO22, SDA -> GPIO21,
 *   SAO/SDO -> GND (MPU at 0x68), NCS + CSB unconnected (I2C mode)
 * Wiring MS4525DO -> ESP32:
 *   VCC -> 5V, GND -> GND, SCL -> GPIO22, SDA -> GPIO21 (shared bus, 3.3 V
 *   pull-ups; the sensor is a 5 V part with open-drain I2C - safe)
 * Wiring GPS -> ESP32:
 *   VCC -> 5V, GND -> GND, TX -> GPIO16, RX -> GPIO17, PPS + EN unconnected
 *
 * Assumes the module lies flat with its X axis pointing forward and Z axis
 * pointing up - depending on how it's actually mounted on the airframe you
 * may need to swap/invert axes below.
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "bmp280.h"
#include "espnow_link.h"
#include "gps.h"
#include "messages.h"
#include "ms4525do.h"

namespace {

constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kRegPowerManagement1 = 0x6B;
constexpr uint8_t kRegAccelStart = 0x3B;

constexpr float kAccelScale = 1.0f / 16384.0f;        // LSB -> g, +-2g range
constexpr float kGyroScale = (PI / 180.0f) / 131.0f;  // LSB -> rad/s, +-250 deg/s range
constexpr float kComplementaryAlpha = 0.98f;          // gyro vs. accelerometer weight
constexpr uint32_t kSampleIntervalUs = 20000;         // 50 Hz
constexpr int kGyroCalibrationSamples = 200;

constexpr uint32_t kBaroIntervalUs = 40000;   // 25 Hz, just under the BMP280's ~26 Hz output
constexpr int kBaroReferenceSamples = 25;     // 1 s average defining altitude zero
constexpr float kVarioTimeConstant = 0.5f;    // [s] climb-rate low-pass

constexpr int kPitotZeroSamples = 50;             // 1 s average defining pressure zero
constexpr float kAirspeedTimeConstant = 0.3f;     // [s] differential-pressure low-pass
constexpr int kPitotMaxConsecutiveFailures = 25;  // ~0.5 s of failed reads = sensor dead
constexpr float kAirDensity = 1.225f;  // [kg/m3] fixed sea-level rho - the result is IAS

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t lastSampleUs = 0;

bool baroOk = false;
float baroReferencePa = 0.0f;
float altitudeM = NAN;
float climbRate = 0.0f;
uint32_t lastBaroUs = 0;

bool pitotOk = false;
float pitotZeroPa = 0.0f;
float diffPressPa = NAN;  // zero-corrected + low-pass filtered
float iasMps = NAN;
int pitotFailures = 0;
uint32_t lastPitotUs = 0;

// ------------------------------------------------------------ ESP-NOW link

constexpr uint32_t kFailsafeTimeoutMs = 500;  // 25 consecutive lost command frames
constexpr uint32_t kStatsWindowMs = 1000;
constexpr uint32_t kExpectedCmdPerWindow = 50;  // command rate is fixed 50 Hz

uint8_t peerMac[6];
bool linkOk = false;

// Shared with the ESP-NOW callbacks (WiFi task context).
portMUX_TYPE linkMux = portMUX_INITIALIZER_UNLOCKED;
CommandMessage lastCommand = {};  // last valid command (future servo input)
uint32_t lastCmdMs = 0;           // 0 = no valid command since boot
uint32_t cmdRxCount = 0;          // valid commands in the current stats window
uint8_t lastCmdVersion = 0;       // version byte of the last command, 0 = none
bool cmdVersionMismatch = false;
uint32_t telemSendOk = 0;  // esp_now_send delivery reports in the window
uint32_t telemSendFail = 0;
volatile int8_t peerRssiDbm = 0;  // last frame from the peer, 0 = none yet

// Stats window results, published into every TelemetryMessage.
uint8_t cmdLossPct = 100;
uint8_t txFailPct = 0;
uint32_t statsWindowStartMs = 0;

void onCommandReceived(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != (int)sizeof(CommandMessage) || data[0] != MSG_COMMAND) {
    return;
  }
  CommandMessage cmd;
  memcpy(&cmd, data, sizeof(cmd));
  portENTER_CRITICAL(&linkMux);
  lastCmdVersion = cmd.version;
  if (cmd.version == PROTOCOL_VERSION) {
    lastCommand = cmd;
    lastCmdMs = millis();
    cmdRxCount++;
    cmdVersionMismatch = false;
  } else {
    // Unusable command: report the mismatch but let the failsafe trip.
    cmdVersionMismatch = true;
  }
  portEXIT_CRITICAL(&linkMux);
}

void onTelemetrySent(const uint8_t *mac, esp_now_send_status_t status) {
  portENTER_CRITICAL(&linkMux);
  if (status == ESP_NOW_SEND_SUCCESS) {
    telemSendOk++;
  } else {
    telemSendFail++;
  }
  portEXIT_CRITICAL(&linkMux);
}

// The IDF 4.4 ESP-NOW rx callback carries no RSSI, so it is sniffed from a
// promiscuous-mode callback instead: ESP-NOW frames are 802.11 action
// frames (management type), the transmitter address sits at payload+10.
void onPromiscuousRx(void *buf, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) {
    return;
  }
  auto *pkt = (wifi_promiscuous_pkt_t *)buf;
  if (memcmp(pkt->payload + 10, peerMac, 6) == 0) {
    peerRssiDbm = (int8_t)pkt->rx_ctrl.rssi;
  }
}

// Recomputes the 1 s window stats feeding cmdLossPct/txFailPct.
void updateLinkStats(uint32_t nowMs) {
  if (nowMs - statsWindowStartMs < kStatsWindowMs) {
    return;
  }
  statsWindowStartMs = nowMs;
  portENTER_CRITICAL(&linkMux);
  uint32_t rx = cmdRxCount;
  uint32_t ok = telemSendOk;
  uint32_t fail = telemSendFail;
  cmdRxCount = 0;
  telemSendOk = 0;
  telemSendFail = 0;
  portEXIT_CRITICAL(&linkMux);
  uint32_t rxPct = rx * 100 / kExpectedCmdPerWindow;
  cmdLossPct = (uint8_t)(100 - min(rxPct, (uint32_t)100));
  txFailPct = (ok + fail) ? (uint8_t)(fail * 100 / (ok + fail)) : 0;
}

void mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void mpuReadRaw(int16_t *accel, int16_t *gyro) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(kRegAccelStart);
  Wire.endTransmission(false);
  Wire.requestFrom(kMpuAddress, (uint8_t)14, (uint8_t) true);

  accel[0] = (Wire.read() << 8) | Wire.read();
  accel[1] = (Wire.read() << 8) | Wire.read();
  accel[2] = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();  // temperature, unused
  gyro[0] = (Wire.read() << 8) | Wire.read();
  gyro[1] = (Wire.read() << 8) | Wire.read();
  gyro[2] = (Wire.read() << 8) | Wire.read();
}

// The ESP32 must be held still during this, otherwise the gyro bias will be wrong.
void calibrateGyro() {
  int16_t accel[3];
  int16_t gyro[3];
  long sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < kGyroCalibrationSamples; i++) {
    mpuReadRaw(accel, gyro);
    sumX += gyro[0];
    sumY += gyro[1];
    sumZ += gyro[2];
    delay(3);
  }
  gyroBiasX = (sumX / (float)kGyroCalibrationSamples) * kGyroScale;
  gyroBiasY = (sumY / (float)kGyroCalibrationSamples) * kGyroScale;
  gyroBiasZ = (sumZ / (float)kGyroCalibrationSamples) * kGyroScale;
}

// Averages 1 s of pressure to define altitude zero at the boot location.
// The first few samples are discarded while the IIR filter settles.
void calibrateBaroReference() {
  constexpr int kDiscard = 5;
  float sum = 0.0f;
  int count = 0;
  for (int i = 0; i < kBaroReferenceSamples + kDiscard; i++) {
    float pressure, temperature;
    if (bmp280::read(pressure, temperature) && i >= kDiscard) {
      sum += pressure;
      count++;
    }
    delay(40);
  }
  if (count > 0) {
    baroReferencePa = sum / count;
  } else {
    baroOk = false;
  }
}

// Averages ~1 s of readings at zero airflow to define the pressure zero.
// The MS4525DO zero offset is tens of Pa and drifts with temperature, so it
// is re-measured at every boot; the airplane must be stationary and the
// pitot shielded from wind. The first few samples are discarded like in
// calibrateBaroReference().
void calibratePitotZero() {
  constexpr int kDiscard = 5;
  float sum = 0.0f;
  int count = 0;
  for (int i = 0; i < kPitotZeroSamples + kDiscard; i++) {
    float pressure, temperature;
    if (ms4525do::read(pressure, temperature) && i >= kDiscard) {
      sum += pressure;
      count++;
    }
    delay(20);
  }
  if (count > 0) {
    pitotZeroPa = sum / count;
  } else {
    pitotOk = false;
  }
}

// Reads the barometer at 25 Hz; altitude is relative to the boot reference,
// climb rate is a low-pass-filtered derivative (raw would be +-1 m/s noise).
void updateBaro(uint32_t now) {
  if (!baroOk || now - lastBaroUs < kBaroIntervalUs) {
    return;
  }
  float dt = (now - lastBaroUs) / 1000000.0f;
  lastBaroUs = now;

  float pressure, temperature;
  if (!bmp280::read(pressure, temperature)) {
    return;
  }
  float alt = bmp280::altitudeFromPressure(pressure, baroReferencePa);
  if (!isnan(altitudeM)) {
    float rate = (alt - altitudeM) / dt;
    float alpha = dt / (kVarioTimeConstant + dt);
    climbRate += alpha * (rate - climbRate);
  }
  altitudeM = alt;
}

// Reads the pitot at the 50 Hz sample rate. The differential pressure is
// zero-corrected and low-pass filtered before the IAS conversion - the
// sqrt blows raw sensor noise up at low speeds. Negative (filtered) dP
// clamps to zero airspeed. A sensor that stops answering for
// kPitotMaxConsecutiveFailures reads is declared dead: fields go NAN and
// the TELEM_FLAG_PITOT_OK flag drops.
void updatePitot(uint32_t now) {
  if (!pitotOk) {
    return;
  }
  float dt = (now - lastPitotUs) / 1000000.0f;
  lastPitotUs = now;

  float pressure, temperature;
  if (!ms4525do::read(pressure, temperature)) {
    if (++pitotFailures >= kPitotMaxConsecutiveFailures) {
      pitotOk = false;
      diffPressPa = NAN;
      iasMps = NAN;
      Serial.println("# MS4525DO stopped responding - airspeed is nan");
    }
    return;
  }
  pitotFailures = 0;

  float dp = pressure - pitotZeroPa;
  if (isnan(diffPressPa)) {
    diffPressPa = dp;
  } else {
    float alpha = dt / (kAirspeedTimeConstant + dt);
    diffPressPa += alpha * (dp - diffPressPa);
  }
  iasMps = sqrtf(2.0f * max(diffPressPa, 0.0f) / kAirDensity);
}

}  // namespace

void setup() {
  Serial.begin(115200);

  // Radio first: the command receiver and stats windows start counting
  // while the blocking sensor calibrations below (~2 s) run.
  if (!espnowlink::parseMac(PEER_MAC_STR, peerMac)) {
    Serial.println("# bad PEER_MAC_STR - ESP-NOW disabled");
  } else if (!espnowlink::begin(peerMac)) {
    Serial.print("# ESP-NOW init failed: ");
    Serial.println(espnowlink::lastError());
  } else {
    linkOk = true;
    esp_now_register_recv_cb(onCommandReceived);
    esp_now_register_send_cb(onTelemetrySent);
    esp_wifi_set_promiscuous(true);  // RSSI sniffing, see onPromiscuousRx
    wifi_promiscuous_filter_t filter = {.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT};
    esp_wifi_set_promiscuous_filter(&filter);
    esp_wifi_set_promiscuous_rx_cb(onPromiscuousRx);
  }
  statsWindowStartMs = millis();

  Wire.begin();
  Wire.setClock(400000);

  mpuWriteRegister(kRegPowerManagement1, 0x00);  // wake up from sleep mode
  delay(50);

  baroOk = bmp280::begin();
  if (!baroOk) {
    // '#' lines are ignored by the cocpitView parser
    Serial.println("# BMP280 not found - alt/climb will be nan");
  }
  pitotOk = ms4525do::begin();
  if (!pitotOk) {
    Serial.println("# MS4525DO not found - airspeed will be nan");
  }

  calibrateGyro();
  if (baroOk) {
    calibrateBaroReference();
  }
  if (pitotOk) {
    calibratePitotZero();
  }

  // After the blocking calibrations (~2 s) the GPS module is certainly out
  // of reset, so the fire-and-forget config frames in begin() are not lost.
  gps::begin();

  lastSampleUs = micros();
  lastBaroUs = lastSampleUs;
  lastPitotUs = lastSampleUs;
}

void loop() {
  // Drained every pass (the rate gate below early-returns, so this runs far
  // above the 38400 baud byte rate - no UART overflow between samples).
  gps::poll();

  uint32_t now = micros();
  if (now - lastSampleUs < kSampleIntervalUs) {
    return;
  }

  static bool gpsChecked = false;
  if (!gpsChecked && now > 5000000) {
    gpsChecked = true;
    if (!gps::alive()) {
      Serial.println("# GPS not responding on UART2 - GPS fields will be nan");
    }
  }
  float dt = (now - lastSampleUs) / 1000000.0f;
  lastSampleUs = now;

  updateBaro(now);
  updatePitot(now);

  int16_t accelRaw[3];
  int16_t gyroRaw[3];
  mpuReadRaw(accelRaw, gyroRaw);

  float ax = accelRaw[0] * kAccelScale;
  float ay = accelRaw[1] * kAccelScale;
  float az = accelRaw[2] * kAccelScale;
  float gx = gyroRaw[0] * kGyroScale - gyroBiasX;
  float gy = gyroRaw[1] * kGyroScale - gyroBiasY;
  float gz = gyroRaw[2] * kGyroScale - gyroBiasZ;

  float rollAccel = atan2f(ay, az);
  float pitchAccel = atan2f(-ax, sqrtf(ay * ay + az * az));

  roll = kComplementaryAlpha * (roll + gx * dt) + (1.0f - kComplementaryAlpha) * rollAccel;
  pitch = kComplementaryAlpha * (pitch + gy * dt) + (1.0f - kComplementaryAlpha) * pitchAccel;
  yaw += gz * dt;  // no magnetometer - gyro integration only, drifts over time

  if (!linkOk) {
    return;
  }

  uint32_t nowMs = millis();
  updateLinkStats(nowMs);

  portENTER_CRITICAL(&linkMux);
  uint32_t cmdMs = lastCmdMs;
  bool mismatch = cmdVersionMismatch;
  uint8_t cmdVersion = lastCmdVersion;
  portEXIT_CRITICAL(&linkMux);

  bool failsafe = cmdMs == 0 || nowMs - cmdMs > kFailsafeTimeoutMs;
  static bool wasFailsafe = true;  // failsafe until the first valid command
  if (failsafe != wasFailsafe) {
    wasFailsafe = failsafe;
    // No servos yet - the flag (+ this diagnostic) is the whole action.
    Serial.println(failsafe ? "# FAILSAFE entered - command link lost"
                            : "# FAILSAFE cleared");
  }

  gps::Fix f = gps::fix();

  TelemetryMessage t = {};
  t.msgId = MSG_TELEMETRY;
  t.version = PROTOCOL_VERSION;
  t.flags = (f.valid ? TELEM_FLAG_GPS_FIX : 0) | (failsafe ? TELEM_FLAG_FAILSAFE : 0) |
            (mismatch ? TELEM_FLAG_VERSION_MISMATCH : 0) | (baroOk ? TELEM_FLAG_BARO_OK : 0) |
            (pitotOk ? TELEM_FLAG_PITOT_OK : 0);
  t.sats = f.sats;
  t.timeBootMs = nowMs;
  t.rollRad = roll;
  t.pitchRad = pitch;
  t.yawRad = yaw;
  t.rollRate = gx;
  t.pitchRate = gy;
  t.yawRate = gz;
  t.baroAltM = altitudeM;
  t.climbMps = climbRate;
  t.diffPressPa = diffPressPa;
  t.iasMps = iasMps;
  if (f.valid) {
    t.lat1e7 = (int32_t)llround(f.latDeg * 1e7);
    t.lon1e7 = (int32_t)llround(f.lonDeg * 1e7);
  }
  t.altMslM = f.altMslM;
  t.speedMps = f.speedMps;
  t.courseDeg = f.courseDeg;
  t.hdop = f.hdop;
  t.hAccM = f.hAccM;
  t.vAccM = f.vAccM;
  t.rssiDbm = peerRssiDbm;
  t.cmdLossPct = cmdLossPct;
  t.txFailPct = txFailPct;
  t.lastCmdVersion = cmdVersion;

  esp_now_send(peerMac, (const uint8_t *)&t, sizeof(t));
}

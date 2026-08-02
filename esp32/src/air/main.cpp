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
 * All I2C sensor access runs in a dedicated FreeRTOS task: the Arduino
 * I2C stack can stall ~1 s per transaction when a device drops off the
 * bus (Wire.setTimeOut does not bound that path — the driver keeps waiting
 * while the HW FSM claims progress), and a stall like that must never
 * touch the safety-critical 50 Hz link loop. loop() only consumes the
 * latest published sensor snapshot.
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
constexpr int kPinSda = 21;
constexpr int kPinScl = 22;

constexpr float kAccelScale = 1.0f / 16384.0f;        // LSB -> g, +-2g range
constexpr float kGyroScale = (PI / 180.0f) / 131.0f;  // LSB -> rad/s, +-250 deg/s range
constexpr float kComplementaryAlpha = 0.98f;          // gyro vs. accelerometer weight
constexpr uint32_t kSampleIntervalUs = 20000;         // 50 Hz
constexpr int kGyroCalibrationSamples = 200;
constexpr int kImuMaxConsecutiveFailures = 25;  // ~0.5 s of failed reads = IMU out
constexpr int kImuRecoveryStreak = 10;  // consecutive good samples to revive a dead IMU

constexpr uint32_t kBaroIntervalUs = 40000;   // 25 Hz, just under the BMP280's ~26 Hz output
constexpr int kBaroReferenceSamples = 25;     // 1 s average defining altitude zero
constexpr float kVarioTimeConstant = 0.5f;    // [s] climb-rate low-pass
constexpr uint32_t kBaroDeadAfterUs = 1000000;      // 1 s without a good read = baro dead
constexpr uint32_t kBaroRecoveryIntervalMs = 2000;  // re-init cadence for a dead baro

constexpr int kPitotZeroSamples = 50;             // 1 s average defining pressure zero
constexpr float kAirspeedTimeConstant = 0.3f;     // [s] differential-pressure low-pass
constexpr uint32_t kPitotDeadAfterUs = 500000;    // 0.5 s without a good read = sensor dead
constexpr uint32_t kPitotRecoveryIntervalMs = 2000;  // re-probe cadence for a dead sensor
// A transaction against an absent/wedged device must fail fast: the Arduino
// default lets it stall the loop for ~1 s, collapsing the 50 Hz telemetry
// to ~0.5 Hz whenever a sensor drops off the bus.
constexpr uint16_t kWireTimeoutMs = 20;
constexpr float kAirDensity = 1.225f;  // [kg/m3] fixed sea-level rho - the result is IAS

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t lastSampleUs = 0;

bool baroOk = false;
bool baroRefValid = false;  // boot reference exists - required for recovery
float baroReferencePa = 0.0f;
float altitudeM = NAN;
float climbRate = 0.0f;
uint32_t lastBaroUs = 0;
uint32_t lastBaroGoodUs = 0;
uint32_t lastBaroRecoveryMs = 0;

bool pitotOk = false;
bool pitotZeroValid = false;  // boot zero exists - required for recovery
float pitotZeroPa = 0.0f;
float diffPressPa = NAN;  // zero-corrected + low-pass filtered
float iasMps = NAN;
uint32_t lastPitotUs = 0;
uint32_t lastPitotGoodUs = 0;
uint32_t lastPitotRecoveryMs = 0;

// The sensor task publishes here, loop() copies under sensorMux. Defaults
// are what telemetry reports until the first sample lands (the link is
// already up during the ~2 s of boot calibration).
struct SensorSnapshot {
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  float rollRate = 0.0f, pitchRate = 0.0f, yawRate = 0.0f;
  float baroAltM = NAN;
  float climbMps = 0.0f;
  float diffPressPa = NAN;
  float iasMps = NAN;
  bool baroOk = false;
  bool pitotOk = false;
  bool imuOk = false;  // clear = roll/pitch/yaw frozen at the last good sample
};
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;
SensorSnapshot sensorShared;

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

// An MCU reset in the middle of an I2C read leaves the addressed slave
// holding SDA low forever (the sensors keep their power across ESP32
// resets), which kills the whole bus - every device reads as "not found".
// The standard cure before Wire.begin(): clock SCL until the slave
// releases SDA, then issue a STOP.
void clearI2cBus() {
  pinMode(kPinSda, INPUT_PULLUP);
  pinMode(kPinScl, INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(kPinSda) == HIGH) {
    return;  // bus is free
  }
  Serial.println("# I2C bus stuck (SDA low) - clocking it free");
  pinMode(kPinScl, OUTPUT_OPEN_DRAIN);
  digitalWrite(kPinScl, HIGH);
  for (int i = 0; i < 9 && digitalRead(kPinSda) == LOW; i++) {
    digitalWrite(kPinScl, LOW);
    delayMicroseconds(5);
    digitalWrite(kPinScl, HIGH);
    delayMicroseconds(5);
  }
  // STOP condition: SDA low -> high while SCL is high.
  pinMode(kPinSda, OUTPUT_OPEN_DRAIN);
  digitalWrite(kPinSda, LOW);
  delayMicroseconds(5);
  digitalWrite(kPinSda, HIGH);
  delayMicroseconds(5);
  if (digitalRead(kPinSda) == LOW) {
    Serial.println("# I2C bus clear failed - check wiring/power");
  }
}

bool mpuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// False on I2C errors - without the check a dead bus reads as all 0xFF,
// i.e. an accel vector (-1,-1,-1), and the complementary filter converges
// to a bogus roll -135 / pitch +35 deg instead of freezing.
bool mpuReadRaw(int16_t *accel, int16_t *gyro) {
  Wire.beginTransmission(kMpuAddress);
  Wire.write(kRegAccelStart);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(kMpuAddress, (uint8_t)14, (uint8_t) true) != 14) {
    return false;
  }

  accel[0] = (Wire.read() << 8) | Wire.read();
  accel[1] = (Wire.read() << 8) | Wire.read();
  accel[2] = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();  // temperature, unused
  gyro[0] = (Wire.read() << 8) | Wire.read();
  gyro[1] = (Wire.read() << 8) | Wire.read();
  gyro[2] = (Wire.read() << 8) | Wire.read();
  return true;
}

// The ESP32 must be held still during this, otherwise the gyro bias will be wrong.
void calibrateGyro() {
  int16_t accel[3];
  int16_t gyro[3];
  long sumX = 0, sumY = 0, sumZ = 0;
  int count = 0;
  for (int i = 0; i < kGyroCalibrationSamples; i++) {
    if (mpuReadRaw(accel, gyro)) {
      sumX += gyro[0];
      sumY += gyro[1];
      sumZ += gyro[2];
      count++;
    }
    delay(3);
  }
  if (count == 0) {
    Serial.println("# MPU-6500 not responding - gyro bias not calibrated");
    return;
  }
  gyroBiasX = (sumX / (float)count) * kGyroScale;
  gyroBiasY = (sumY / (float)count) * kGyroScale;
  gyroBiasZ = (sumZ / (float)count) * kGyroScale;
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
    baroRefValid = true;
  } else {
    baroOk = false;
  }
}

// Same shape as tryPitotRecovery below. A power-cycled BMP280 loses its
// config registers (it ACKs but the data registers read as reset values),
// so recovery re-runs the full begin(); the boot reference pressure is
// kept - altitude stays relative to the boot location.
void tryBaroRecovery() {
  if (!baroRefValid) {
    return;
  }
  uint32_t nowMs = millis();
  if (nowMs - lastBaroRecoveryMs < kBaroRecoveryIntervalMs) {
    return;
  }
  lastBaroRecoveryMs = nowMs;
  if (!bmp280::begin()) {
    return;
  }
  baroOk = true;
  lastBaroUs = micros();
  lastBaroGoodUs = lastBaroUs;
  Serial.println("# BMP280 recovered");
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
    pitotZeroValid = true;
  } else {
    pitotOk = false;
  }
}

// A transient I2C dropout (vibration, loose wire) must not kill airspeed
// for the rest of the flight: a dead sensor is re-probed every
// kPitotRecoveryIntervalMs and on success resumes with the boot zero
// offset (re-zeroing in flight is impossible - it needs still air).
// A sensor with no boot zero stays dead: measuring against zero = 0 would
// show a false airspeed of several m/s.
void tryPitotRecovery(uint32_t now) {
  if (!pitotZeroValid) {
    return;
  }
  uint32_t nowMs = millis();
  if (nowMs - lastPitotRecoveryMs < kPitotRecoveryIntervalMs) {
    return;
  }
  lastPitotRecoveryMs = nowMs;
  if (!ms4525do::begin()) {
    return;
  }
  pitotOk = true;
  lastPitotUs = now;
  lastPitotGoodUs = now;
  Serial.println("# MS4525DO recovered");
}

// Reads the barometer at 25 Hz; altitude is relative to the boot reference,
// climb rate is a low-pass-filtered derivative (raw would be +-1 m/s noise).
// 1 s without a plausible reading = baro dead: alt/climb go NAN and the
// TELEM_FLAG_BARO_OK flag drops until tryBaroRecovery() brings it back.
void updateBaro(uint32_t now) {
  if (!baroOk) {
    tryBaroRecovery();
    return;
  }
  if (now - lastBaroUs < kBaroIntervalUs) {
    return;
  }
  float dt = (now - lastBaroUs) / 1000000.0f;
  lastBaroUs = now;

  float pressure, temperature;
  // The plausibility band (300-1100 hPa) catches a power-cycled chip: it
  // ACKs, but with the config lost the data registers hold reset values
  // that would otherwise decode as a believable-looking constant altitude.
  bool good = bmp280::read(pressure, temperature) && pressure > 30000.0f && pressure < 110000.0f;
  if (!good) {
    if (now - lastBaroGoodUs >= kBaroDeadAfterUs) {
      baroOk = false;
      altitudeM = NAN;
      climbRate = NAN;
      Serial.println("# BMP280 stopped responding - alt/climb nan until it recovers");
    }
    return;
  }
  lastBaroGoodUs = now;

  float alt = bmp280::altitudeFromPressure(pressure, baroReferencePa);
  if (isnan(altitudeM)) {
    climbRate = 0.0f;  // fresh start after boot/recovery - no derivative yet
  } else {
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
// kPitotMaxConsecutiveFailures reads is declared dead - fields go NAN and
// the TELEM_FLAG_PITOT_OK flag drops - until tryPitotRecovery() brings it
// back.
void updatePitot(uint32_t now) {
  if (!pitotOk) {
    tryPitotRecovery(now);
    return;
  }
  float dt = (now - lastPitotUs) / 1000000.0f;
  lastPitotUs = now;

  float pressure, temperature;
  if (!ms4525do::read(pressure, temperature)) {
    // Time-based, not attempt-based: failed transactions can be slow, and
    // the sensor must read as dead (NAN) within a fixed real-time budget.
    if (now - lastPitotGoodUs >= kPitotDeadAfterUs) {
      pitotOk = false;
      diffPressPa = NAN;
      iasMps = NAN;
      Serial.println("# MS4525DO stopped responding - airspeed nan until it recovers");
    }
    return;
  }
  lastPitotGoodUs = now;

  float dp = pressure - pitotZeroPa;
  if (isnan(diffPressPa)) {
    diffPressPa = dp;
  } else {
    float alpha = dt / (kAirspeedTimeConstant + dt);
    diffPressPa += alpha * (dp - diffPressPa);
  }
  iasMps = sqrtf(2.0f * max(diffPressPa, 0.0f) / kAirDensity);
}

// Owns the I2C bus and every sensor on it - see the file header for why
// this is a separate task. Bring-up, calibration and the 50 Hz sampling
// all happen here; a stalled transaction only delays the next sample.
void sensorTask(void *) {
  clearI2cBus();
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(kWireTimeoutMs);

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

  uint32_t lastAttitudeUs = micros();
  lastBaroUs = lastAttitudeUs;
  lastBaroGoodUs = lastAttitudeUs;
  lastPitotUs = lastAttitudeUs;
  lastPitotGoodUs = lastAttitudeUs;
  int imuFailures = 0;
  int imuGoodStreak = 0;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));  // 50 Hz
    uint32_t now = micros();
    float dt = (now - lastAttitudeUs) / 1000000.0f;
    lastAttitudeUs = now;

    updateBaro(now);
    updatePitot(now);

    int16_t accelRaw[3];
    int16_t gyroRaw[3];
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;  // body rates, zero while the IMU is out
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    bool imuSample = mpuReadRaw(accelRaw, gyroRaw);
    if (imuSample) {
      ax = accelRaw[0] * kAccelScale;
      ay = accelRaw[1] * kAccelScale;
      az = accelRaw[2] * kAccelScale;
      // An MPU that lost power comes back in sleep mode: it ACKs but the
      // data registers read all-zero. |a| this low is aerodynamically
      // impossible, so treat it as a failed sample.
      imuSample = sqrtf(ax * ax + ay * ay + az * az) >= 0.05f;
    }
    if (imuSample && imuFailures >= kImuMaxConsecutiveFailures) {
      // Recovery hysteresis: garbage bytes off a floating bus occasionally
      // pass the plausibility check and a single such "sample" must not
      // revive the IMU (it used to blink the frozen horizon/compass back
      // for 0.5 s). Only an unbroken streak of good samples counts; the
      // streak samples themselves are discarded, the filter resumes after.
      if (++imuGoodStreak >= kImuRecoveryStreak) {
        imuFailures = 0;
        imuGoodStreak = 0;
        Serial.println("# MPU-6500 recovered");
      }
    } else if (imuSample) {
      imuFailures = 0;
      imuGoodStreak = 0;

      gx = gyroRaw[0] * kGyroScale - gyroBiasX;
      gy = gyroRaw[1] * kGyroScale - gyroBiasY;
      gz = gyroRaw[2] * kGyroScale - gyroBiasZ;

      float rollAccel = atan2f(ay, az);
      float pitchAccel = atan2f(-ax, sqrtf(ay * ay + az * az));

      roll = kComplementaryAlpha * (roll + gx * dt) + (1.0f - kComplementaryAlpha) * rollAccel;
      pitch = kComplementaryAlpha * (pitch + gy * dt) + (1.0f - kComplementaryAlpha) * pitchAccel;
      yaw += gz * dt;  // no magnetometer - gyro integration only, drifts over time
    } else {
      imuFailures++;
      imuGoodStreak = 0;
      if (imuFailures == kImuMaxConsecutiveFailures) {
        // Attitude freezes at the last good sample - much better than the
        // fake roll/pitch the filter would converge to on garbage reads.
        // TELEM_FLAG_IMU_OK drops, so the HUD hides horizon and compass.
        Serial.println("# MPU-6500 not responding - attitude frozen");
      }
      if (imuFailures % kImuMaxConsecutiveFailures == 0) {
        // Periodic recovery: wake the MPU again in case it power-cycled
        // back into sleep mode (it comes back asleep, ACKing but reading
        // all-zero).
        mpuWriteRegister(kRegPowerManagement1, 0x00);
      }
    }

    portENTER_CRITICAL(&sensorMux);
    sensorShared.roll = roll;
    sensorShared.pitch = pitch;
    sensorShared.yaw = yaw;
    sensorShared.rollRate = gx;
    sensorShared.pitchRate = gy;
    sensorShared.yawRate = gz;
    sensorShared.baroAltM = altitudeM;
    sensorShared.climbMps = climbRate;
    sensorShared.diffPressPa = diffPressPa;
    sensorShared.iasMps = iasMps;
    sensorShared.baroOk = baroOk;
    sensorShared.pitotOk = pitotOk;
    sensorShared.imuOk = imuFailures < kImuMaxConsecutiveFailures;
    portEXIT_CRITICAL(&sensorMux);
  }
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

  // Core 0 with the WiFi stack; loop() keeps core 1. Priority 1 = same as
  // loop, it is blocked (not busy) while a transaction stalls.
  xTaskCreatePinnedToCore(sensorTask, "sensors", 6144, nullptr, 1, nullptr, 0);

  lastSampleUs = micros();
}

void loop() {
  // The GPS config frames are fire-and-forget, so they are sent once the
  // module is certainly out of reset (~2.5 s after power-on; the sensor
  // task no longer blocks setup() to provide that delay for free).
  static bool gpsStarted = false;
  if (!gpsStarted && millis() > 2500) {
    gpsStarted = true;
    gps::begin();
  }
  if (gpsStarted) {
    // Drained every pass (the rate gate below early-returns, so this runs
    // far above the 38400 baud byte rate - no UART overflow between samples).
    gps::poll();
  }

  uint32_t now = micros();
  if (now - lastSampleUs < kSampleIntervalUs) {
    return;
  }
  lastSampleUs = now;

  static bool gpsChecked = false;
  if (!gpsChecked && now > 5000000) {
    gpsChecked = true;
    if (!gps::alive()) {
      Serial.println("# GPS not responding on UART2 - GPS fields will be nan");
    }
  }

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

  SensorSnapshot s;
  portENTER_CRITICAL(&sensorMux);
  s = sensorShared;
  portEXIT_CRITICAL(&sensorMux);

  gps::Fix f = gps::fix();

  TelemetryMessage t = {};
  t.msgId = MSG_TELEMETRY;
  t.version = PROTOCOL_VERSION;
  t.flags = (f.valid ? TELEM_FLAG_GPS_FIX : 0) | (failsafe ? TELEM_FLAG_FAILSAFE : 0) |
            (mismatch ? TELEM_FLAG_VERSION_MISMATCH : 0) | (s.baroOk ? TELEM_FLAG_BARO_OK : 0) |
            (s.pitotOk ? TELEM_FLAG_PITOT_OK : 0) | (s.imuOk ? TELEM_FLAG_IMU_OK : 0);
  t.sats = f.sats;
  t.timeBootMs = nowMs;
  t.rollRad = s.roll;
  t.pitchRad = s.pitch;
  t.yawRad = s.yaw;
  t.rollRate = s.rollRate;
  t.pitchRate = s.pitchRate;
  t.yawRate = s.yawRate;
  t.baroAltM = s.baroAltM;
  t.climbMps = s.climbMps;
  t.diffPressPa = s.diffPressPa;
  t.iasMps = s.iasMps;
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

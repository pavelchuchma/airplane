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
 * Wiring battery balance lead -> ESP32 (dividers, see battery.h):
 *   C1 -> 10k -> GPIO35, C2 -> 28k -> GPIO34, C3 -> 56k -> GPIO39,
 *   10k + 100 nF from each pin to GND, B- -> GND (star ground)
 *
 * The attitude math works in a body frame with X forward, Y right and Z up.
 * The GY-91 is mounted upside down in the airframe (180 deg about body X),
 * so its readings are remapped in mpuReadRaw() per kAxisSign - see there.
 */

#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "battery.h"
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

// Mounted orientation. The GY-91 sits in the airframe rotated 180 deg about
// the body X axis (upside down, sensor X still forward), so Y and Z readings
// (accel and gyro alike) are negated into the body frame the attitude math
// below assumes. Applied inside mpuReadRaw(), i.e. everything downstream -
// including the gyro bias calibration - already works in body frame. The
// signs must stay a proper rotation (determinant +1): a mirror (e.g. Z
// alone) puts gyro and accel in opposite frames on one axis and the
// complementary filter degrades to its slow accel path - seconds of rubbery
// lag, verified on the airframe 2026-08-25.
//
// That body frame is X-forward, Y-LEFT, Z-up (FLU). It cannot be Y-right:
// the sensor's own frame is right-handed and a determinant +1 rotation
// keeps it that way, and with X forward and Z up, right-handedness forces
// Y = Z x X = left. An earlier comment here claimed Y-right, which is what
// hid the consequence for a whole flight: roll and pitch come out with the
// usual aerospace signs in FLU (positive = right wing down / nose up), but
// `yaw += gz*dt` counts positive counter-clockwise, i.e. a LEFT turn. The
// reported heading therefore ran backwards, all the way through the HUD,
// QGC and the wind-triangle fit in yawcal.py, which silently absorbed the
// mirrored heading into a bogus 17 m/s wind (2026-08-29_123339.cvlog, the
// flight that ended in a spin). The flip to a clockwise heading is applied
// once, where telemetry is filled - not here, so that the frame stays a
// consistent FLU triad for the filter.
constexpr int8_t kAxisSign[3] = {1, -1, -1};  // X, Y, Z: sensor -> body frame (FLU)

// Level trim: mounting misalignment of the board vs. the airframe, subtracted
// from the accel-derived angles so the filter converges to airframe-relative
// zero. Pitch zero is a flight reference (the wing cruises nose-up relative
// to any bench level), so it is read in straight-and-level cruise, not on the
// bench. A re-measurement reads the residual the current trim leaves, so the
// measured value is *added* to the constant, not written in its place.
constexpr float kRollTrimRad = -0.522f * PI / 180.0f;
constexpr float kPitchTrimRad = 7.562f * PI / 180.0f;
constexpr uint32_t kSampleIntervalUs = 20000;         // 50 Hz
constexpr int kGyroCalibrationSamples = 200;
constexpr uint32_t kImuDeadAfterUs = 500000;  // 0.5 s without a good read = IMU out

constexpr uint32_t kBaroIntervalUs = 40000;   // 25 Hz, just under the BMP280's ~26 Hz output
constexpr int kBaroReferenceSamples = 25;     // 1 s average defining altitude zero
constexpr float kVarioTimeConstant = 0.5f;    // [s] climb-rate low-pass
constexpr uint32_t kBaroDeadAfterUs = 1000000;  // 1 s without a good read = baro dead

constexpr int kPitotZeroSamples = 50;           // 1 s average defining pressure zero
constexpr float kAirspeedTimeConstant = 0.3f;   // [s] differential-pressure low-pass
constexpr uint32_t kPitotDeadAfterUs = 500000;  // 0.5 s without a good read = sensor dead

constexpr uint32_t kBattIntervalUs = 100000;  // 10 Hz - plenty for an alarm signal
constexpr uint32_t kBattDeadAfterUs = 1000000;  // 1 s of implausible reads = lead gone
// Plausibility band per cell: below = disconnected balance lead (all nodes
// read near 0) or a cell too dead to measure; above = not a Li-ion cell.
constexpr uint16_t kCellMinMv = 2500;
constexpr uint16_t kCellMaxMv = 4350;

constexpr int kRecoveryStreak = 10;          // consecutive good reads reviving a dead sensor
constexpr uint32_t kProbeIntervalMs = 2000;  // dead-sensor probe cadence (failed probes are slow)
constexpr uint32_t kBusRescueAfterUs = 2000000;  // nothing good on the bus this long -> rescue
constexpr uint32_t kBusRescueIntervalMs = 5000;  // bus-rescue attempt cadence
// 100 kHz, not 400: 400 kHz is the MS4525DO's spec maximum, and that one
// device hangs on a wire harness off the well-pulled-up GY-91 board (5 V
// part judging a 3.3 V bus, ~10 k pull-ups). Bus traffic is ~30 B per
// 20 ms tick, so a quarter of the bandwidth is still plenty.
constexpr uint32_t kI2cClockHz = 100000;
constexpr uint32_t kSnapshotStaleUs = 200000;  // loop() distrusts a snapshot older than this
// Shortens SOME failure paths but does NOT bound the worst ones: the driver
// can still stall ~1 s per transaction on a disturbed bus (measured 1006 ms
// with this set to 20) - which is why all I2C access lives in sensorTask.
constexpr uint16_t kWireTimeoutMs = 20;
constexpr float kAirDensity = 1.225f;  // [kg/m3] fixed sea-level rho - the result is IAS

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t lastSampleUs = 0;

float baroReferencePa = 0.0f;
float altitudeM = NAN;
float climbRate = NAN;

float pitotZeroPa = 0.0f;
float diffPressPa = NAN;  // zero-corrected + low-pass filtered
float iasMps = NAN;

uint16_t cellMv[3] = {0, 0, 0};  // per-cell [mV], 0 = unknown

// One lifecycle for all three I2C sensors. The only failure mode designed
// for is a temporary bus disturbance: transactions fail (slowly - see the
// file header) or ACK corrupted data for a while, then the bus recovers.
// The sensors keep their power - and therefore their config registers -
// throughout, so recovery never re-initializes anything: a plausible data
// read IS the proof of health. Death is time-based (failed transactions
// can take ~1 s each), revival needs an unbroken streak of good reads
// (garbage off a floating bus occasionally passes the plausibility checks,
// and a single such sample must not blink the frozen HUD back on; the
// streak samples themselves are discarded). A dead sensor is probed at
// most every kProbeIntervalMs so its slow failures don't starve the
// healthy sensors; a successful probe does not re-arm the throttle, so the
// streak then builds at the full sample rate.
struct SensorLifecycle {
  SensorLifecycle(const char *name, uint32_t deadAfterUs)
      : name(name), deadAfterUs(deadAfterUs) {}

  const char *name;  // for the '#' diagnostics
  uint32_t deadAfterUs;
  bool present = false;  // decided once at boot: sensors never lose power,
                         // so one absent at boot cannot appear later
  bool ok = false;
  uint32_t lastGoodUs = 0;
  int goodStreak = 0;
  uint32_t nextProbeMs = 0;

  bool shouldAttempt(uint32_t nowMs) const {
    return present && (ok || (int32_t)(nowMs - nextProbeMs) >= 0);
  }

  // Feeds one read attempt's verdict; true = process the sample. The
  // caller detects the death transition (ok dropping) to NAN its outputs.
  bool onResult(bool good, uint32_t nowUs, uint32_t nowMs) {
    if (good) {
      lastGoodUs = nowUs;
      if (ok) {
        return true;
      }
      if (++goodStreak >= kRecoveryStreak) {
        ok = true;
        goodStreak = 0;
        Serial.printf("# %s recovered\n", name);
      }
      return false;
    }
    goodStreak = 0;
    if (ok && nowUs - lastGoodUs >= deadAfterUs) {
      ok = false;
      Serial.printf("# %s stopped responding - data nan until it recovers\n", name);
    }
    if (!ok) {
      nextProbeMs = nowMs + kProbeIntervalMs;
    }
    return false;
  }
};

SensorLifecycle imuLc("MPU-6500", kImuDeadAfterUs);
SensorLifecycle baroLc("BMP280", kBaroDeadAfterUs);
SensorLifecycle pitotLc("MS4525DO", kPitotDeadAfterUs);
// Not an I2C device (and deliberately absent from tryBusRescue's list): the
// on-chip ADC can't die, so present/ok track the *balance lead* being
// connected and the readings staying inside the plausibility band.
SensorLifecycle battLc("battery", kBattDeadAfterUs);

// The sensor task publishes here, loop() copies under sensorMux. Defaults
// are what telemetry reports until the first sample lands (the link is
// already up during the ~2 s of boot calibration); snapshotUs = 0 means
// nothing was ever published, which loop()'s staleness check treats the
// same as a wedged task - all OK flags read false.
struct SensorSnapshot {
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
  float rollRate = 0.0f, pitchRate = 0.0f, yawRate = 0.0f;
  float baroAltM = NAN;
  float climbMps = NAN;
  float diffPressPa = NAN;
  float iasMps = NAN;
  uint16_t cellMv[3] = {0, 0, 0};
  bool baroOk = false;
  bool pitotOk = false;
  bool imuOk = false;  // clear = roll/pitch/yaw frozen at the last good sample
  bool battOk = false;
  uint32_t snapshotUs = 0;  // micros() at publish time
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
  // ESP-NOW hands frames from ANY sender to this callback - the PMK/LMK
  // encryption only covers traffic with the registered peer. Without this
  // check a stray transmitter on the channel could keep refreshing
  // lastCmdMs (suppressing the failsafe) and, once servos exist, fly the
  // airplane.
  if (memcmp(mac, peerMac, 6) != 0) {
    return;
  }
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

// Sensor -> body frame per kAxisSign. Negation is safe for every count the
// MPU produces except INT16_MIN, whose negative does not fit in an int16_t;
// that one is clamped (it is the saturated end of the range anyway).
int16_t toBodyFrame(int16_t raw, int axis) {
  if (kAxisSign[axis] > 0) {
    return raw;
  }
  return raw == INT16_MIN ? INT16_MAX : (int16_t)-raw;
}

// Returns body-frame counts. False on I2C errors - without the check a dead
// bus reads as all 0xFF, i.e. an accel vector (-1,-1,-1), and the
// complementary filter converges to a bogus roll -135 / pitch +35 deg
// instead of freezing.
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

  for (int i = 0; i < 3; i++) {
    accel[i] = toBodyFrame(accel[i], i);
    gyro[i] = toBodyFrame(gyro[i], i);
  }
  return true;
}

// The ESP32 must be held still during this, otherwise the gyro bias will be
// wrong. Requires at least half the window: a bias averaged from a handful
// of noisy reads (or silently left at 0.0, as the old code did on a fully
// failed window) would drift the yaw all flight while looking calibrated.
bool calibrateGyro() {
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
  if (count < kGyroCalibrationSamples / 2) {
    return false;
  }
  gyroBiasX = (sumX / (float)count) * kGyroScale;
  gyroBiasY = (sumY / (float)count) * kGyroScale;
  gyroBiasZ = (sumZ / (float)count) * kGyroScale;
  return true;
}

// Averages 1 s of pressure to define altitude zero at the boot location.
// The first few samples are discarded while the IIR filter settles.
// Requires at least half the window, like calibrateGyro().
bool calibrateBaroReference() {
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
  if (count < kBaroReferenceSamples / 2) {
    return false;
  }
  baroReferencePa = sum / count;
  return true;
}

// Averages ~1 s of readings at zero airflow to define the pressure zero.
// The MS4525DO zero offset is tens of Pa and drifts with temperature, so it
// is re-measured at every boot; the airplane must be stationary and the
// pitot shielded from wind. The first few samples are discarded like in
// calibrateBaroReference(); at least half the window is required (a zero
// from one noisy sample reads as several m/s of phantom airspeed).
bool calibratePitotZero() {
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
  if (count < kPitotZeroSamples / 2) {
    return false;
  }
  pitotZeroPa = sum / count;
  return true;
}

// Reads the barometer at 25 Hz; altitude is relative to the boot reference,
// climb rate is a low-pass-filtered derivative (raw would be +-1 m/s noise).
// kBaroDeadAfterUs without a plausible reading = baro dead: alt/climb go
// NAN and TELEM_FLAG_BARO_OK drops until the lifecycle revives it.
void updateBaro(uint32_t nowUs, uint32_t nowMs) {
  static uint32_t lastAttemptUs = 0;  // 25 Hz gate only; dt comes from lastGoodUs
  if (!baroLc.shouldAttempt(nowMs) || nowUs - lastAttemptUs < kBaroIntervalUs) {
    return;
  }
  lastAttemptUs = nowUs;

  float pressure, temperature;
  // The plausibility band (300-1100 hPa) rejects corrupted-but-ACKed data:
  // I2C acknowledges per byte but carries no integrity check, so a
  // disturbed bus can deliver garbage that would otherwise decode as a
  // believable-looking altitude.
  bool good = bmp280::read(pressure, temperature) && pressure > 30000.0f && pressure < 110000.0f;
  bool wasOk = baroLc.ok;
  uint32_t prevGoodUs = baroLc.lastGoodUs;
  if (!baroLc.onResult(good, nowUs, nowMs)) {
    if (wasOk && !baroLc.ok) {
      altitudeM = NAN;
      climbRate = NAN;
    }
    return;
  }

  // dt spans back to the last good sample (capped at the death threshold),
  // so a sub-threshold dropout can't inflate the derivative - the altitude
  // delta and the time it took stay consistent.
  float dt = min(nowUs - prevGoodUs, kBaroDeadAfterUs) / 1000000.0f;
  float alt = bmp280::altitudeFromPressure(pressure, baroReferencePa);
  if (isnan(altitudeM)) {
    climbRate = 0.0f;  // fresh start after boot/revival - no derivative yet
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
// clamps to zero airspeed. kPitotDeadAfterUs without a good read = sensor
// dead: fields go NAN and TELEM_FLAG_PITOT_OK drops until the lifecycle
// revives it, resuming with the boot zero offset (re-zeroing in flight is
// impossible - it needs still air).
void updatePitot(uint32_t nowUs, uint32_t nowMs) {
  if (!pitotLc.shouldAttempt(nowMs)) {
    return;
  }
  float pressure, temperature;
  bool good = ms4525do::read(pressure, temperature);
  bool wasOk = pitotLc.ok;
  uint32_t prevGoodUs = pitotLc.lastGoodUs;
  if (!pitotLc.onResult(good, nowUs, nowMs)) {
    if (wasOk && !pitotLc.ok) {
      diffPressPa = NAN;
      iasMps = NAN;
    }
    return;
  }

  float dt = min(nowUs - prevGoodUs, kPitotDeadAfterUs) / 1000000.0f;
  float dp = pressure - pitotZeroPa;
  if (isnan(diffPressPa)) {
    diffPressPa = dp;
  } else {
    float alpha = dt / (kAirspeedTimeConstant + dt);
    diffPressPa += alpha * (dp - diffPressPa);
  }
  iasMps = sqrtf(2.0f * max(diffPressPa, 0.0f) / kAirDensity);
}

// Reads the three balance nodes at 10 Hz and publishes the subtracted cell
// voltages. "good" = every cell inside the plausibility band - a pulled
// balance lead reads ~0 mV on all nodes and kills all three at once, a
// single corrupted conversion can't (the 64-sample average already ate it).
// kBattDeadAfterUs of implausible reads = fields to 0 and TELEM_FLAG_BATT_OK
// drops; the lifecycle revives it (with the same boot calibration constants)
// if the lead comes back.
void updateBattery(uint32_t nowUs, uint32_t nowMs) {
  static uint32_t lastAttemptUs = 0;
  if (!battLc.shouldAttempt(nowMs) || nowUs - lastAttemptUs < kBattIntervalUs) {
    return;
  }
  lastAttemptUs = nowUs;

  uint16_t mv[3];
  battery::read(mv);
  bool good = true;
  for (uint16_t v : mv) {
    good = good && v >= kCellMinMv && v <= kCellMaxMv;
  }
  bool wasOk = battLc.ok;
  if (!battLc.onResult(good, nowUs, nowMs)) {
    if (wasOk && !battLc.ok) {
      cellMv[0] = cellMv[1] = cellMv[2] = 0;
    }
    return;
  }
  memcpy(cellMv, mv, sizeof(cellMv));
}

// Boot presence probes: a few normal data reads - cheap, retried against a
// bus that may still be settling, and the only operation every sensor is
// guaranteed to support (the MS4525DO has no registers at all; the old
// write-mode probe rejected clones that NACK address+W).
bool probeImu() {
  mpuWriteRegister(kRegPowerManagement1, 0x00);  // wake up from sleep mode
  delay(50);
  int16_t accel[3];
  int16_t gyro[3];
  for (int i = 0; i < 3; i++) {
    if (mpuReadRaw(accel, gyro)) {
      return true;
    }
    delay(10);
  }
  return false;
}

bool probePitot() {
  float pressure, temperature;
  for (int i = 0; i < 3; i++) {
    if (ms4525do::read(pressure, temperature)) {
      return true;
    }
    delay(10);
  }
  return false;
}

bool probeBaro() {
  // begin() is the probe: the BMP280 is the only sensor with real init
  // (chip-id check, calibration load, mode config) - and it runs only
  // here, at boot. The config survives any bus disturbance.
  for (int i = 0; i < 3; i++) {
    if (bmp280::begin()) {
      return true;
    }
    delay(10);
  }
  return false;
}

// The ADC itself always answers - this probes the *balance lead*. In flight
// the pack powers the ESP32, so the lead is there from boot; on the USB
// bench there is no pack and battery reports absent, like any other missing
// sensor. Same present-is-decided-once model: the lead connects together
// with the pack, never later.
bool probeBattery() {
  for (int i = 0; i < 3; i++) {
    uint16_t mv[3];
    battery::read(mv);
    if (mv[0] >= kCellMinMv && mv[0] <= kCellMaxMv && mv[1] >= kCellMinMv &&
        mv[1] <= kCellMaxMv && mv[2] >= kCellMinMv && mv[2] <= kCellMaxMv) {
      return true;
    }
    delay(10);
  }
  return false;
}

// A disturbance can abort a transaction mid-byte and leave a slave holding
// SDA low - a bus-level latch-up no amount of per-sensor probing clears.
// The signature is every present sensor going silent at once; the cure is
// the same SCL clocking that frees the bus at boot.
void tryBusRescue(uint32_t nowUs, uint32_t nowMs) {
  static uint32_t nextRescueMs = 0;
  if ((int32_t)(nowMs - nextRescueMs) < 0) {
    return;
  }
  bool anyPresent = false;
  for (const SensorLifecycle *lc : {&imuLc, &baroLc, &pitotLc}) {
    if (!lc->present) {
      continue;
    }
    anyPresent = true;
    if (lc->ok || nowUs - lc->lastGoodUs < kBusRescueAfterUs) {
      return;  // something on the bus is still alive (or reviving)
    }
  }
  if (!anyPresent) {
    return;
  }
  nextRescueMs = nowMs + kBusRescueIntervalMs;
  Serial.println("# whole I2C bus dead - re-clocking it free");
  Wire.end();
  clearI2cBus();
  Wire.begin();
  Wire.setClock(kI2cClockHz);  // begin() resets the clock to its default
  Wire.setTimeOut(kWireTimeoutMs);
  // Probe immediately instead of waiting out the per-sensor throttles.
  imuLc.nextProbeMs = baroLc.nextProbeMs = pitotLc.nextProbeMs = nowMs;
}

// Owns the I2C bus and every sensor on it - see the file header for why
// this is a separate task. Bring-up, calibration and the 50 Hz sampling
// all happen here; a stalled transaction only delays the next sample.
void sensorTask(void *) {
  clearI2cBus();
  Wire.begin();
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(kWireTimeoutMs);

  // Presence is decided once, here: the sensors never lose power in
  // flight, so one absent now cannot appear later - and probing an absent
  // device costs ~1 s per failed transaction, forever. '#' lines are
  // ignored by the cocpitView parser.
  imuLc.present = probeImu();
  baroLc.present = probeBaro();
  pitotLc.present = probePitot();
  battery::begin();
  battLc.present = probeBattery();

  // Boot calibrations, one retry each: a transient bus glitch during the
  // window must not cost the whole flight's data. A sensor whose
  // calibration fails twice is treated as absent - resuming later with a
  // half-made zero/bias/reference would look healthy and lie.
  if (imuLc.present && !(calibrateGyro() || calibrateGyro())) {
    imuLc.present = false;
  }
  if (baroLc.present && !(calibrateBaroReference() || calibrateBaroReference())) {
    baroLc.present = false;
  }
  if (pitotLc.present && !(calibratePitotZero() || calibratePitotZero())) {
    pitotLc.present = false;
  }

  uint32_t bootUs = micros();
  for (SensorLifecycle *lc : {&imuLc, &baroLc, &pitotLc, &battLc}) {
    if (lc->present) {
      lc->ok = true;
      lc->lastGoodUs = bootUs;
    } else {
      Serial.printf("# %s absent - data nan for this flight\n", lc->name);
    }
  }

  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));  // 50 Hz
    uint32_t now = micros();
    uint32_t nowMs = millis();

    updateBaro(now, nowMs);
    updatePitot(now, nowMs);
    updateBattery(now, nowMs);

    float gx = 0.0f, gy = 0.0f, gz = 0.0f;  // body rates, zero while the IMU is out
    if (imuLc.shouldAttempt(nowMs)) {
      int16_t accelRaw[3];
      int16_t gyroRaw[3];
      float ax = 0.0f, ay = 0.0f, az = 0.0f;
      bool good = mpuReadRaw(accelRaw, gyroRaw);
      if (good) {
        ax = accelRaw[0] * kAccelScale;
        ay = accelRaw[1] * kAccelScale;
        az = accelRaw[2] * kAccelScale;
        // |a| this low is aerodynamically impossible - a disturbed bus can
        // ACK corrupted (e.g. all-zero) data, and the accel magnitude is
        // the cheapest plausibility check the sample offers.
        good = sqrtf(ax * ax + ay * ay + az * az) >= 0.05f;
      }
      uint32_t prevGoodUs = imuLc.lastGoodUs;
      if (imuLc.onResult(good, now, nowMs)) {
        gx = gyroRaw[0] * kGyroScale - gyroBiasX;
        gy = gyroRaw[1] * kGyroScale - gyroBiasY;
        gz = gyroRaw[2] * kGyroScale - gyroBiasZ;

        float rollAccel = atan2f(ay, az) - kRollTrimRad;
        float pitchAccel = atan2f(-ax, sqrtf(ay * ay + az * az)) - kPitchTrimRad;

        // dt spans back to the last good sample (capped), so the rotation
        // during a sub-threshold dropout is approximated by this sample's
        // rates instead of silently dropped - yaw has no other reference
        // to pull it back. When the IMU is dead longer, attitude freezes
        // at the last good sample (far better than the fake roll/pitch the
        // filter would converge to on garbage) and revival resumes with
        // dt = one tick: the discarded streak reads refreshed lastGoodUs.
        float dt = min(now - prevGoodUs, kImuDeadAfterUs) / 1000000.0f;
        roll = kComplementaryAlpha * (roll + gx * dt) + (1.0f - kComplementaryAlpha) * rollAccel;
        pitch = kComplementaryAlpha * (pitch + gy * dt) + (1.0f - kComplementaryAlpha) * pitchAccel;
        yaw += gz * dt;  // no magnetometer - gyro integration only, drifts over time
      }
    }

    tryBusRescue(now, nowMs);

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
    memcpy(sensorShared.cellMv, cellMv, sizeof(cellMv));
    sensorShared.baroOk = baroLc.ok;
    sensorShared.pitotOk = pitotLc.ok;
    sensorShared.imuOk = imuLc.ok;
    sensorShared.battOk = battLc.ok;
    sensorShared.snapshotUs = now;
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
  // A wedged sensor task (a transaction the driver never times out of)
  // must not keep publishing pre-failure data as healthy: age out the OK
  // flags. A single ~1 s stalled transaction blanks all three for its
  // duration - honest, the data in the snapshot really is that old.
  if (now - s.snapshotUs > kSnapshotStaleUs) {
    s.baroOk = s.pitotOk = s.imuOk = s.battOk = false;
  }

  gps::Fix f = gps::fix();

  TelemetryMessage t = {};
  t.msgId = MSG_TELEMETRY;
  t.version = PROTOCOL_VERSION;
  t.flags = (f.valid ? TELEM_FLAG_GPS_FIX : 0) | (failsafe ? TELEM_FLAG_FAILSAFE : 0) |
            (mismatch ? TELEM_FLAG_VERSION_MISMATCH : 0) | (s.baroOk ? TELEM_FLAG_BARO_OK : 0) |
            (s.pitotOk ? TELEM_FLAG_PITOT_OK : 0) | (s.imuOk ? TELEM_FLAG_IMU_OK : 0) |
            (s.battOk ? TELEM_FLAG_BATT_OK : 0);
  t.sats = f.sats;
  t.timeBootMs = nowMs;
  t.rollRad = s.roll;
  t.pitchRad = s.pitch;
  // Body frame is X-forward/Y-left/Z-up, so a positive rotation about Z is a
  // LEFT turn; the wire (and MAVLink behind it) wants a clockwise heading.
  // Roll and pitch need no such flip - see the frame note above.
  t.yawRad = -s.yaw;
  t.rollRate = s.rollRate;
  t.pitchRate = s.pitchRate;
  t.yawRate = -s.yawRate;
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
  memcpy(t.cellMv, s.cellMv, sizeof(t.cellMv));

  esp_now_send(peerMac, (const uint8_t *)&t, sizeof(t));
}

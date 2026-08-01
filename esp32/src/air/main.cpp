/*
 * Air side: sensors, servos, autopilot.
 *
 * Current state: a 1:1 port of the arduinoNano MPU-6050 firmware — the ESP32
 * is connected directly to the PC over USB serial, no ESP-NOW link yet.
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
 * Reads the Quescan G10A-F30 GPS (u-blox M10) over UART2 (see gps.h).
 *
 * Prints one CSV line over USB serial (115200 baud, 50 Hz):
 *
 *   roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,alt,climb,
 *   lat,lon,alt_msl,gs,cog,sats,hdop,hacc,vacc\n
 *     (angles in radians, *speed in radians/s, alt in m above the boot
 *      location, climb in m/s; lat/lon in degrees, alt_msl in m, gs in
 *      m/s, cog in degrees, hacc/vacc are the receiver's 1-sigma
 *      horizontal/vertical error estimates in m. alt/climb are "nan" when
 *      the BMP280 is missing; lat..cog and hacc/vacc are "nan" without a
 *      valid GPS fix - sats/hdop stay live so fix acquisition is visible)
 *
 * The lines are read by cocpitView (cockpitview/serial_source.py), which
 * forwards them to QGroundControl as MAVLink ATTITUDE + VFR_HUD + ALTITUDE
 * + GPS_RAW_INT + GLOBAL_POSITION_INT over UDP. Lines starting with '#'
 * are boot diagnostics, ignored by the parser.
 *
 * Wiring GY-91 -> ESP32:
 *   VIN -> 3V3, GND -> GND, SCL -> GPIO22, SDA -> GPIO21,
 *   SAO/SDO -> GND (MPU at 0x68), NCS + CSB unconnected (I2C mode)
 * Wiring GPS -> ESP32:
 *   VCC -> 5V, GND -> GND, TX -> GPIO16, RX -> GPIO17, PPS + EN unconnected
 *
 * Assumes the module lies flat with its X axis pointing forward and Z axis
 * pointing up - depending on how it's actually mounted on the airframe you
 * may need to swap/invert axes below.
 */

#include <Arduino.h>
#include <Wire.h>

#include "bmp280.h"
#include "gps.h"

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

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t lastSampleUs = 0;

bool baroOk = false;
float baroReferencePa = 0.0f;
float altitudeM = NAN;
float climbRate = 0.0f;
uint32_t lastBaroUs = 0;

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

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpuWriteRegister(kRegPowerManagement1, 0x00);  // wake up from sleep mode
  delay(50);

  baroOk = bmp280::begin();
  if (!baroOk) {
    // '#' lines are ignored by the cocpitView parser
    Serial.println("# BMP280 not found - alt/climb will be nan");
  }

  calibrateGyro();
  if (baroOk) {
    calibrateBaroReference();
  }

  // After the blocking calibrations (~2 s) the GPS module is certainly out
  // of reset, so the fire-and-forget config frames in begin() are not lost.
  gps::begin();

  lastSampleUs = micros();
  lastBaroUs = lastSampleUs;
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

  Serial.print(roll, 5);
  Serial.print(',');
  Serial.print(pitch, 5);
  Serial.print(',');
  Serial.print(yaw, 5);
  Serial.print(',');
  Serial.print(gx, 5);
  Serial.print(',');
  Serial.print(gy, 5);
  Serial.print(',');
  Serial.print(gz, 5);
  Serial.print(',');
  Serial.print(altitudeM, 2);
  Serial.print(',');
  Serial.print(climbRate, 2);

  gps::Fix f = gps::fix();
  Serial.print(',');
  Serial.print(f.latDeg, 7);  // 7 decimals ~ 1 cm; invalid fields print "nan"
  Serial.print(',');
  Serial.print(f.lonDeg, 7);
  Serial.print(',');
  Serial.print(f.altMslM, 1);
  Serial.print(',');
  Serial.print(f.speedMps, 2);
  Serial.print(',');
  Serial.print(f.courseDeg, 1);
  Serial.print(',');
  Serial.print(f.sats);
  Serial.print(',');
  Serial.print(f.hdop, 2);
  Serial.print(',');
  Serial.print(f.hAccM, 2);
  Serial.print(',');
  Serial.println(f.vAccM, 2);
}

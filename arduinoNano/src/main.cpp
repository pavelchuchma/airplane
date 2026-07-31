/*
 * arduinoNano - physical counterpart of emulator/mpu6050_emulator.py.
 *
 * Reads the accelerometer/gyroscope from an MPU-6050 over I2C (Wire), computes
 * roll/pitch with a complementary filter (yaw is gyro-integration only - no
 * magnetometer, so it drifts) and prints one CSV line over USB serial
 * (115200 baud, 50 Hz):
 *
 *   roll,pitch,yaw,rollspeed,pitchspeed,yawspeed\n   (roll/pitch/yaw in radians,
 *                                                       *speed in radians/s)
 *
 * The lines are read by emulator/mpu6050_hw_bridge.py, which forwards them to
 * QGroundControl as MAVLink ATTITUDE over UDP.
 *
 * Wiring MPU-6050 -> Arduino Nano:
 *   VCC -> 5V, GND -> GND, SCL -> A5, SDA -> A4, AD0 -> GND (I2C address 0x68)
 *
 * Assumes the module lies flat with its X axis pointing forward and Z axis
 * pointing up - depending on how it's actually mounted on the airframe you
 * may need to swap/invert axes below.
 */

#include <Arduino.h>
#include <Wire.h>

namespace {

constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kRegPowerManagement1 = 0x6B;
constexpr uint8_t kRegAccelStart = 0x3B;

constexpr float kAccelScale = 1.0f / 16384.0f;        // LSB -> g, +-2g range
constexpr float kGyroScale = (PI / 180.0f) / 131.0f;  // LSB -> rad/s, +-250 deg/s range
constexpr float kComplementaryAlpha = 0.98f;          // gyro vs. accelerometer weight
constexpr uint32_t kSampleIntervalUs = 20000;         // 50 Hz
constexpr int kGyroCalibrationSamples = 200;

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
uint32_t lastSampleUs = 0;

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

// The Arduino must be held still during this, otherwise the gyro bias will be wrong.
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

}  // namespace

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpuWriteRegister(kRegPowerManagement1, 0x00);  // wake up from sleep mode
  delay(50);

  calibrateGyro();

  lastSampleUs = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - lastSampleUs < kSampleIntervalUs) {
    return;
  }
  float dt = (now - lastSampleUs) / 1000000.0f;
  lastSampleUs = now;

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
  Serial.println(gz, 5);
}

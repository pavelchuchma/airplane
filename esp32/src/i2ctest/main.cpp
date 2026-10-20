/*
 * I2C driver failure-timing test (doc/plans/2026-10-20-arduino3-native-i2c.md).
 *
 * The ~1 s sensor stalls of doc/todo/002 were single glitched transactions
 * that the Wire driver turned into a ~1 s wait. A read from an address
 * nobody answers is the reproducible stand-in for such a glitch: it needs
 * nothing on the bus. Each case is timed through Wire (the driver under
 * test) and through i2cbus (the bit-banged reference), and the results are
 * printed once on USB serial (115200), then repeated every 10 s.
 */

#include <Arduino.h>
#include <Wire.h>

#include "../air/i2cbus.h"

namespace {

constexpr uint8_t kAbsentAddress = 0x29;  // nothing on a bare devkit answers anywhere
constexpr uint8_t kRegister = 0x3B;
constexpr int kRepeats = 5;
constexpr uint32_t kClockHz = 100000;
constexpr uint16_t kTimeoutMs = 20;  // what the air firmware set

struct Result {
  uint32_t minUs = UINT32_MAX, maxUs = 0, sumUs = 0;
  int lastCode = 0;
};

template <typename F>
Result timeIt(F op) {
  Result r;
  for (int i = 0; i < kRepeats; i++) {
    uint32_t startUs = micros();
    r.lastCode = op();
    uint32_t tookUs = micros() - startUs;
    r.minUs = min(r.minUs, tookUs);
    r.maxUs = max(r.maxUs, tookUs);
    r.sumUs += tookUs;
  }
  return r;
}

void report(const char *name, const Result &r) {
  Serial.printf("%-28s min %8.2f  avg %8.2f  max %8.2f ms  (last code %d)\n", name, r.minUs / 1000.0f,
                r.sumUs / 1000.0f / kRepeats, r.maxUs / 1000.0f, r.lastCode);
}

void runWire() {
  Wire.begin(21, 22, kClockHz);
  Wire.setTimeOut(kTimeoutMs);
  report("Wire plain read (pitot)", timeIt([] { return (int)Wire.requestFrom(kAbsentAddress, (uint8_t)4); }));
  report("Wire register read (MPU)", timeIt([] {
           Wire.beginTransmission(kAbsentAddress);
           Wire.write(kRegister);
           int err = Wire.endTransmission(false);
           return err != 0 ? -err : (int)Wire.requestFrom(kAbsentAddress, (uint8_t)14);
         }));
  report("Wire probe (empty write)", timeIt([] {
           Wire.beginTransmission(kAbsentAddress);
           return (int)Wire.endTransmission();
         }));
  Wire.end();
}

void runBitBang() {
  i2cbus::begin();
  i2cbus::clear();
  uint8_t buf[14];
  report("i2cbus plain read", timeIt([&] { return (int)i2cbus::read(kAbsentAddress, buf, 4); }));
  report("i2cbus register read", timeIt([&] { return (int)i2cbus::readRegs(kAbsentAddress, kRegister, buf, 14); }));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1500);
}

void loop() {
  Serial.printf("\n# i2ctest: arduino-esp32 %d.%d.%d, IDF %s, Wire timeout %u ms, %u Hz, address 0x%02X x%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
                esp_get_idf_version(), kTimeoutMs, (unsigned)kClockHz, kAbsentAddress, kRepeats);
  runWire();
  runBitBang();
  Serial.println("# i2ctest: done");
  delay(10000);
}

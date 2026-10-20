/*
 * I2C driver failure-timing test (doc/plans/2026-10-20-arduino3-native-i2c.md).
 *
 * The ~1 s sensor stalls of doc/todo/002 were single glitched transactions
 * that the Wire driver turned into a ~1 s wait. Two stand-ins that need no
 * sensor and no disturbance:
 *
 *   - NACK: a read from an address nobody answers.
 *   - Held SDA: a jumper from GPIO21 (SDA) to GPIO4, which pulls SDA low in
 *     open-drain mode like a slave that lost track mid-byte (seen once in
 *     2026-10-20_181656). Measured while held, when released in the middle
 *     of a transaction, and after the release (recovered = back to the fast
 *     NACK). Without the jumper these cases just repeat the NACK numbers.
 *   - Held SCL: a jumper from GPIO22 (SCL) to GPIO5 - held throughout, and
 *     clock stretches of 1 and 5 ms started inside the address byte.
 *
 * Each case runs through Wire (the driver under test) and through i2cbus
 * (the bit-banged reference); results go to USB serial (115200), repeated
 * every 10 s.
 */

#include <Arduino.h>
#include <Wire.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "../air/i2cbus.h"

namespace {

constexpr uint8_t kAbsentAddress = 0x29;  // nothing on a bare devkit answers anywhere
constexpr uint8_t kRegister = 0x3B;
constexpr int kRepeats = 5;
constexpr int kHeldRepeats = 10;
constexpr uint32_t kSpacingMs = 20;  // the air firmware's sample period
constexpr uint64_t kMidReleaseUs = 30000;
constexpr uint32_t kClockHz = 100000;
constexpr uint16_t kTimeoutMs = 20;  // what the air firmware set
constexpr gpio_num_t kHoldPin = GPIO_NUM_4;     // jumpered to SDA
constexpr gpio_num_t kSclHoldPin = GPIO_NUM_5;  // jumpered to SCL
constexpr uint64_t kStretchStartUs = 50;  // inside the address byte (~90 us at 100 kHz)

esp_timer_handle_t releaseTimer;
esp_timer_handle_t sclHoldTimer;
esp_timer_handle_t sclReleaseTimer;
uint64_t stretchUs = 0;

void holdSda(bool hold) {
  gpio_set_level(kHoldPin, hold ? 0 : 1);
}

void holdScl(bool hold) {
  gpio_set_level(kSclHoldPin, hold ? 0 : 1);
}

esp_timer_handle_t makeTimer(esp_timer_cb_t cb, const char *name) {
  esp_timer_create_args_t args = {};
  args.callback = cb;
  args.name = name;
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  return timer;
}

void beginHoldPin() {
  gpio_config_t conf = {};
  conf.pin_bit_mask = (1ULL << kHoldPin) | (1ULL << kSclHoldPin);
  conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  gpio_config(&conf);
  holdSda(false);
  holdScl(false);
  releaseTimer = makeTimer([](void *) { holdSda(false); }, "release");
  sclReleaseTimer = makeTimer([](void *) { holdScl(false); }, "scl-release");
  sclHoldTimer = makeTimer([](void *) {
    holdScl(true);
    esp_timer_start_once(sclReleaseTimer, stretchUs);
  }, "scl-hold");
}

struct Result {
  uint32_t minUs = UINT32_MAX, maxUs = 0, sumUs = 0;
  int n = 0;
  int lastCode = 0;
};

template <typename F>
Result timeIt(F op, int repeats, uint32_t spacingMs = 0) {
  Result r;
  for (int i = 0; i < repeats; i++) {
    uint32_t startUs = micros();
    r.lastCode = op();
    uint32_t tookUs = micros() - startUs;
    r.minUs = min(r.minUs, tookUs);
    r.maxUs = max(r.maxUs, tookUs);
    r.sumUs += tookUs;
    r.n++;
    if (spacingMs) {
      delay(spacingMs);
    }
  }
  return r;
}

void report(const char *name, const Result &r) {
  Serial.printf("%-34s min %8.2f  avg %8.2f  max %8.2f ms  (last code %d)\n", name, r.minUs / 1000.0f,
                r.sumUs / 1000.0f / r.n, r.maxUs / 1000.0f, r.lastCode);
}

// The held-SDA sequence for one driver: held, released mid-transaction,
// recovered.
template <typename F>
void heldCases(const char *driver, F op) {
  char name[48];
  holdSda(true);
  delay(2);
  snprintf(name, sizeof(name), "%s SDA held", driver);
  report(name, timeIt(op, kHeldRepeats, kSpacingMs));

  esp_timer_start_once(releaseTimer, kMidReleaseUs);
  snprintf(name, sizeof(name), "%s released after 30 ms", driver);
  report(name, timeIt(op, 1));
  delay(kSpacingMs);

  holdSda(false);
  snprintf(name, sizeof(name), "%s after release", driver);
  report(name, timeIt(op, kRepeats, kSpacingMs));
}

// The held-SCL sequence: held throughout, then stretches inside the address
// byte, then recovered.
template <typename F>
void sclCases(const char *driver, F op) {
  char name[48];
  holdScl(true);
  delay(2);
  snprintf(name, sizeof(name), "%s SCL held", driver);
  report(name, timeIt(op, kRepeats, kSpacingMs));
  holdScl(false);
  delay(kSpacingMs);
  for (uint64_t us : {1000ULL, 5000ULL}) {
    stretchUs = us;
    snprintf(name, sizeof(name), "%s SCL stretch %u ms", driver, (unsigned)(us / 1000));
    // spaced so the stretch is over before the next one starts
    report(name, timeIt([&] {
             esp_timer_start_once(sclHoldTimer, kStretchStartUs);
             return op();
           }, kRepeats, us / 1000 + kSpacingMs));
  }
  snprintf(name, sizeof(name), "%s after SCL", driver);
  report(name, timeIt(op, kRepeats, kSpacingMs));
}

void runWire() {
  Wire.begin(21, 22, kClockHz);
  Wire.setTimeOut(kTimeoutMs);
  auto plainRead = [] { return (int)Wire.requestFrom(kAbsentAddress, (uint8_t)4); };
  report("Wire plain read (pitot)", timeIt(plainRead, kRepeats));
  report("Wire register read (MPU)", timeIt([] {
           Wire.beginTransmission(kAbsentAddress);
           Wire.write(kRegister);
           int err = Wire.endTransmission(false);
           return err != 0 ? -err : (int)Wire.requestFrom(kAbsentAddress, (uint8_t)14);
         }, kRepeats));
  report("Wire probe (empty write)", timeIt([] {
           Wire.beginTransmission(kAbsentAddress);
           return (int)Wire.endTransmission();
         }, kRepeats));
  heldCases("Wire plain read", plainRead);
  sclCases("Wire plain read", plainRead);
  Wire.end();
}

void runBitBang() {
  i2cbus::begin();
  i2cbus::clear();
  static uint8_t buf[14];
  // code = status * 10 + 1 when the transaction had to free the bus
  auto plainRead = [] { return (int)i2cbus::read(kAbsentAddress, buf, 4) * 10 + i2cbus::lastCleared(); };
  report("i2cbus plain read", timeIt(plainRead, kRepeats));
  report("i2cbus register read", timeIt([] { return (int)i2cbus::readRegs(kAbsentAddress, kRegister, buf, 14); }, kRepeats));
  heldCases("i2cbus plain read", plainRead);
  sclCases("i2cbus plain read", plainRead);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  beginHoldPin();
  delay(1500);
}

void loop() {
  Serial.printf("\n# i2ctest: arduino-esp32 %d.%d.%d, IDF %s, Wire timeout %u ms, %u Hz, address 0x%02X\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
                esp_get_idf_version(), kTimeoutMs, (unsigned)kClockHz, kAbsentAddress);
  runWire();
  runBitBang();
  Serial.println("# i2ctest: done");
  delay(10000);
}

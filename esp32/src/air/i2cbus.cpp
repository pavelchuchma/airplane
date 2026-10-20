#include "i2cbus.h"

#include <Arduino.h>
#include <driver/gpio.h>

namespace i2cbus {
namespace {

constexpr gpio_num_t kSda = GPIO_NUM_21;
constexpr gpio_num_t kScl = GPIO_NUM_22;
constexpr uint32_t kHalfPeriodUs = 5;        // ~90 kHz with the call overhead
constexpr uint32_t kStretchTimeoutUs = 2000;  // no sensor here stretches anywhere near this

Status status = Status::Ok;
uint32_t maxStretchUs = 0;
bool cleared = false;

// Open drain: "high" releases the line to the pull-ups, "low" drives it.
inline void sdaRelease() { gpio_set_level(kSda, 1); }
inline void sdaLow() { gpio_set_level(kSda, 0); }
inline void sclLow() { gpio_set_level(kScl, 0); }
inline bool sdaRead() { return gpio_get_level(kSda); }
inline bool sclRead() { return gpio_get_level(kScl); }
inline void halfPeriod() { delayMicroseconds(kHalfPeriodUs); }

// Releases SCL and waits for it to rise - a slave may hold it low (clock
// stretching). Records the longest wait; false = timeout.
bool sclRelease() {
  gpio_set_level(kScl, 1);
  if (sclRead()) {
    return true;
  }
  uint32_t startUs = micros();
  while (!sclRead()) {
    uint32_t waitedUs = micros() - startUs;
    if (waitedUs > kStretchTimeoutUs) {
      maxStretchUs = max(maxStretchUs, waitedUs);
      return false;
    }
  }
  maxStretchUs = max(maxStretchUs, (uint32_t)(micros() - startUs));
  return true;
}

// Both lines must be idle-high; a repeated START comes from SCL low.
Status start(bool repeated) {
  if (repeated) {
    sdaRelease();
    halfPeriod();
    if (!sclRelease()) {
      return Status::StretchTimeout;
    }
    halfPeriod();
  }
  if (!sdaRead() || !sclRead()) {
    return Status::BusBusy;
  }
  sdaLow();
  halfPeriod();
  sclLow();
  halfPeriod();
  return Status::Ok;
}

void stop() {
  sdaLow();
  halfPeriod();
  sclRelease();  // best effort - a stuck SCL is the next START's BusBusy
  halfPeriod();
  sdaRelease();
  halfPeriod();
}

// One bit with SCL starting and ending low. A released SDA must read
// high while SCL is high, otherwise somebody else is driving it.
Status writeBit(bool bit) {
  if (bit) {
    sdaRelease();
  } else {
    sdaLow();
  }
  halfPeriod();
  if (!sclRelease()) {
    return Status::StretchTimeout;
  }
  bool conflict = bit && !sdaRead();
  halfPeriod();
  sclLow();
  return conflict ? Status::SdaConflict : Status::Ok;
}

Status readBit(bool &bit) {
  sdaRelease();
  halfPeriod();
  if (!sclRelease()) {
    return Status::StretchTimeout;
  }
  bit = sdaRead();
  halfPeriod();
  sclLow();
  return Status::Ok;
}

// Sends a byte; nack = the slave did not pull SDA low in the ACK slot.
Status writeByte(uint8_t value, Status nackStatus) {
  for (int i = 7; i >= 0; i--) {
    Status s = writeBit(value & (1 << i));
    if (s != Status::Ok) {
      return s;
    }
  }
  bool nack;
  Status s = readBit(nack);
  if (s != Status::Ok) {
    return s;
  }
  return nack ? nackStatus : Status::Ok;
}

Status readByte(uint8_t &value, bool ack) {
  value = 0;
  for (int i = 0; i < 8; i++) {
    bool bit;
    Status s = readBit(bit);
    if (s != Status::Ok) {
      return s;
    }
    value = (value << 1) | bit;
  }
  return writeBit(!ack);  // ACK = SDA low, NACK after the last byte
}

Status readBytes(uint8_t addr, uint8_t *buf, size_t n, bool repeated) {
  Status s = start(repeated);
  if (s == Status::Ok) {
    s = writeByte((addr << 1) | 1, Status::AddrNack);
  }
  for (size_t i = 0; i < n && s == Status::Ok; i++) {
    s = readByte(buf[i], i + 1 < n);
  }
  return s;
}

Status finish(Status s) {
  if (s != Status::BusBusy) {
    stop();  // after BusBusy we never drove the bus
  }
  // A slave that lost track mid-byte keeps SDA low and no STOP gets
  // through - free it now, not after the seconds of whole-bus silence the
  // rescue in main.cpp waits for.
  if (!sdaRead() || !sclRead()) {
    clear();
    cleared = true;
  }
  status = s;
  return s;
}

void beginTransaction() {
  maxStretchUs = 0;
  cleared = false;
}

}  // namespace

const char *statusName(Status s) {
  switch (s) {
    case Status::Ok: return "ok";
    case Status::BusBusy: return "bus-busy";
    case Status::AddrNack: return "addr-nack";
    case Status::DataNack: return "data-nack";
    case Status::StretchTimeout: return "stretch-timeout";
    case Status::SdaConflict: return "sda-conflict";
  }
  return "?";
}

void begin() {
  gpio_config_t conf = {};
  conf.pin_bit_mask = (1ULL << kSda) | (1ULL << kScl);
  conf.mode = GPIO_MODE_INPUT_OUTPUT_OD;
  conf.pull_up_en = GPIO_PULLUP_ENABLE;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&conf);
  sdaRelease();
  gpio_set_level(kScl, 1);
  delayMicroseconds(10);
}

bool clear(bool *wasStuck) {
  sdaRelease();
  gpio_set_level(kScl, 1);
  halfPeriod();
  bool stuck = !sdaRead();
  if (wasStuck) {
    *wasStuck = stuck;
  }
  if (!stuck) {
    return sclRead();
  }
  for (int i = 0; i < 9 && !sdaRead(); i++) {
    sclLow();
    halfPeriod();
    gpio_set_level(kScl, 1);
    halfPeriod();
  }
  // STOP: SDA low -> high while SCL is high.
  sdaLow();
  halfPeriod();
  sdaRelease();
  halfPeriod();
  return sdaRead() && sclRead();
}

Status writeRegs(uint8_t addr, uint8_t reg, const uint8_t *data, size_t n) {
  beginTransaction();
  Status s = start(false);
  if (s == Status::Ok) {
    s = writeByte(addr << 1, Status::AddrNack);
  }
  if (s == Status::Ok) {
    s = writeByte(reg, Status::DataNack);
  }
  for (size_t i = 0; i < n && s == Status::Ok; i++) {
    s = writeByte(data[i], Status::DataNack);
  }
  return finish(s);
}

Status readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  beginTransaction();
  Status s = start(false);
  if (s == Status::Ok) {
    s = writeByte(addr << 1, Status::AddrNack);
  }
  if (s == Status::Ok) {
    s = writeByte(reg, Status::DataNack);
  }
  if (s == Status::Ok) {
    s = readBytes(addr, buf, n, true);
  }
  return finish(s);
}

Status read(uint8_t addr, uint8_t *buf, size_t n) {
  beginTransaction();
  return finish(readBytes(addr, buf, n, false));
}

Status lastStatus() {
  return status;
}

uint32_t lastMaxStretchUs() {
  return maxStretchUs;
}

bool lastCleared() {
  return cleared;
}

}  // namespace i2cbus

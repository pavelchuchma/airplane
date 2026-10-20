/*
 * Bit-banged single-master I2C on the sensor bus (GPIO21 SDA / GPIO22 SCL)
 * - doc/plans/2026-10-20-soft-i2c.md.
 *
 * Replaces the Arduino/IDF driver, which turned a single glitched
 * transaction into a ~1 s wait (measured 1006-1008 ms regardless of
 * Wire.setTimeOut, doc/todo/002; where in the driver stack is unknown).
 * Here every wait is bounded, and each transaction reports how it failed,
 * which the driver could not.
 *
 * Not thread-safe by design: sensorTask is the only user of the bus.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace i2cbus {

enum class Status : uint8_t {
  Ok,
  BusBusy,         // SDA or SCL low before START - a slave holds the bus
  AddrNack,        // nobody acknowledged the address
  DataNack,        // a written byte was not acknowledged
  StretchTimeout,  // SCL held low by a slave longer than the stretch timeout
  SdaConflict,     // SDA read low while released with SCL high - a glitch or a driving slave
};
constexpr int kStatusCount = 6;

const char *statusName(Status s);

// Configures the pins (open-drain, internal pull-ups, both released).
// Call clear() next: a slave may still hold SDA from before an MCU reset.
void begin();

// Frees a slave left holding SDA low (an MCU reset mid-read keeps the
// sensors powered): clocks SCL up to 9 times, then issues a STOP. True =
// bus idle afterwards. *wasStuck tells whether there was anything to free.
bool clear(bool *wasStuck = nullptr);

// START addr+W reg data[0..n) STOP.
Status writeRegs(uint8_t addr, uint8_t reg, const uint8_t *data, size_t n);

// START addr+W reg, repeated START addr+R buf[0..n) STOP.
Status readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

// START addr+R buf[0..n) STOP - for register-less devices (MS4525DO).
Status read(uint8_t addr, uint8_t *buf, size_t n);

// Result and longest SCL clock stretch [us] of the last transaction, and
// whether it ended with a line held low that clear() then had to free
// (checked after every transaction, so a stuck slave costs one sample).
Status lastStatus();
uint32_t lastMaxStretchUs();
bool lastCleared();

}  // namespace i2cbus

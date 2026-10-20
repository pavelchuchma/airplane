/*
 * The sensor I2C bus (GPIO21 SDA / GPIO22 SCL) on the ESP-IDF 5 master
 * driver (`driver/i2c_master.h`) - doc/plans/2026-10-20-arduino3-native-i2c.md.
 *
 * Every transaction is bounded by a few-ms timeout; the arduino-esp32 2.x
 * driver turned a single glitch into a ~1 s wait (doc/todo/002). Both lines
 * are checked around every transaction: a slave holding SDA is clocked free
 * at once (an MCU reset mid-read or a glitch mid-byte leaves it so), and a
 * bus held low before the START is reported as such instead of attempted.
 *
 * Not thread-safe by design: sensorTask is the only user of the bus.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace i2cbus {

enum class Status : uint8_t {
  Ok,
  BusBusy,  // SDA or SCL still low before START after a bus clear
  Failed,   // NACK or another bus error - the IDF driver does not tell them apart
  Timeout,  // the transaction exceeded its timeout (SCL held, hardware stuck)
};
constexpr int kStatusCount = 4;

const char *statusName(Status s);

// Creates the bus on the calling core's interrupt (sensorTask: core 0).
// Call clear() next: a slave may still hold SDA from before an MCU reset.
void begin();

// Frees a slave left holding SDA low (9 SCL clocks + STOP). True = bus
// idle afterwards. *wasStuck tells whether there was anything to free.
bool clear(bool *wasStuck = nullptr);

// START addr+W reg data[0..n) STOP.
Status writeRegs(uint8_t addr, uint8_t reg, const uint8_t *data, size_t n);

// START addr+W reg, repeated START addr+R buf[0..n) STOP.
Status readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

// START addr+R buf[0..n) STOP - for register-less devices (MS4525DO).
Status read(uint8_t addr, uint8_t *buf, size_t n);

// Result of the last transaction, and whether it ended with a line held
// low that clear() then had to free.
Status lastStatus();
bool lastCleared();

}  // namespace i2cbus

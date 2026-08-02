/*
 * Minimal I2C driver for the TE MS4525DO differential pressure sensor
 * (pitot airspeed, address 0x28). Register-less: the sensor converts
 * continuously and a plain 4-byte read returns the latest sample plus
 * status bits. Our part is the DS5AI001DP variant: +-1 psi, output type A
 * (10-90 % of 2^14 counts). See doc/parts/pitot-ms4525do.md.
 */

#pragma once

#include <stdint.h>

namespace ms4525do {

// Probes the sensor. It has no chip-id register, so presence is an ACK at
// the address plus a sanity check that the resting counts sit near the
// zero-pressure midpoint (catches clones/faults). Returns false when the
// sensor is missing or the reading is implausible.
bool begin(uint8_t address = 0x28);

// Reads the latest conversion. Differential pressure in Pa (raw, NOT
// zero-offset corrected - the caller subtracts its boot-time zero),
// temperature in degrees C. Returns false on I2C errors, stale data, a
// sensor fault, or counts outside the sensor's legal 10-90 % output band
// (bus artifacts decode as status 0 with counts 0).
bool read(float &diffPressPa, float &temperatureC);

}  // namespace ms4525do

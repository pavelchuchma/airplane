/*
 * Minimal I2C driver for the TE MS4525DO differential pressure sensor
 * (pitot airspeed, address 0x28). Register-less: the sensor converts
 * continuously and a plain 4-byte read returns the latest sample plus
 * status bits. Our part is the DS5AI001DP variant: +-1 psi, output type A
 * (10-90 % of 2^14 counts) - the range is not guesswork from the seller's
 * listing, it was measured against a 981 Pa water column (a +-2 psi die, the
 * usual clone surprise, would read low by exactly sqrt(2) in speed).
 * See doc/parts/pitot-ms4525do.md.
 *
 * There is no begin(): the sensor has no chip-id or config registers, so
 * presence is simply read() succeeding — main.cpp proves it with the boot
 * zero calibration reads.
 */

#pragma once

namespace ms4525do {

// Reads the latest conversion. Differential pressure in Pa (raw, NOT
// zero-offset corrected - the caller subtracts its boot-time zero),
// temperature in degrees C. Returns false on I2C errors, stale data, a
// sensor fault, or counts outside the sensor's legal 10-90 % output band
// (bus artifacts decode as status 0 with counts 0).
bool read(float &diffPressPa, float &temperatureC);

}  // namespace ms4525do

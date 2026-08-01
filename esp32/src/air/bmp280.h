/*
 * Minimal I2C driver for the Bosch BMP280 barometer (on the GY-91 board,
 * address 0x76). Register-level, same style as the MPU code in main.cpp -
 * no library dependencies.
 *
 * Configured per the datasheet's "indoor navigation" preset: normal mode,
 * pressure oversampling x16, temperature x2, IIR filter coefficient 16.
 * Gives ~26 Hz output and ~2 cm RMS pressure noise.
 */

#pragma once

#include <stdint.h>

namespace bmp280 {

// Probes the chip, loads factory calibration and starts continuous
// measurement. Returns false when the chip is missing or the chip-id
// does not match a BMP280.
bool begin(uint8_t address = 0x76);

// Reads the latest compensated measurement. Pressure in Pa, temperature
// in degrees C. Returns false on I2C errors.
bool read(float &pressurePa, float &temperatureC);

// Barometric altitude [m] relative to the reference pressure [Pa].
float altitudeFromPressure(float pressurePa, float referencePa);

}  // namespace bmp280

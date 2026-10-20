/*
 * Per-cell battery voltage monitoring via the ESP32's internal ADC1
 * (doc/plans/2026-08-10-battery-cell-monitoring.md).
 *
 * The 3S pack's balance connector feeds three resistor dividers into ADC1
 * input-only pins; the cumulative node voltages are measured against B-
 * (= star ground) and the individual cells come from subtraction. ADC1 is
 * the WiFi-compatible unit (ADC2 is not), and every node stays under the
 * ~2.45 V ceiling of the 11 dB attenuation's linear range.
 *
 * Wiring (100 nF from each pin to GND, lower divider resistor 10k):
 *   C1 (cell 1+) --[10k]-- GPIO35 / ADC1_CH7   (max 4.25 V -> 2.13 V)
 *   C2           --[28k]-- GPIO34 / ADC1_CH6   (max 8.50 V -> 2.24 V)
 *   C3 (pack +)  --[56k]-- GPIO39 / ADC1_CH3   (max 12.75 V -> 1.93 V)
 */

#pragma once

#include <stdint.h>

namespace battery {

// Configures the ADC channels and loads the eFuse Vref characterization.
// Reports nothing - presence of the *pack* is the caller's boot probe:
// read() returning implausible cells means no balance lead (or, should the
// on-chip ADC fail to initialize, zeros that look the same).
void begin();

// Samples all three nodes (64 raw samples each, averaged), converts via the
// eFuse characterization + per-channel two-point calibration, and returns
// the three cell voltages in mV. Always returns raw results - plausibility
// (2.5-4.35 V per cell) is the caller's check, like the other sensors.
// A disconnected balance lead reads near 0 on all nodes.
void read(uint16_t cellMv[3]);

}  // namespace battery

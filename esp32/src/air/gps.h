/*
 * Quescan G10A-F30 GPS module (u-blox M10) on UART2, 38400 baud NMEA,
 * parsed by TinyGPSPlus.
 *
 * The chip is flash-less: configuration survives only in RAM/BBR, so
 * begin() re-applies it on every boot via raw UBX-CFG-VALSET frames
 * (5 Hz nav rate, "airborne <=4g" dynamic model, and only the RMC + GGA
 * + GST NMEA sentences at the nav rate - together they carry everything
 * Fix needs and keep 5 Hz comfortably inside 38400 baud; GST adds the
 * navigation filter's 1-sigma error estimates in meters - plus GSV once
 * a second for the satellites-in-view count while acquiring).
 */

#pragma once

#include <stdint.h>

namespace gps {

struct Fix {
  bool alive;       // an NMEA sentence passed its checksum within the last 2 s
  bool valid;       // fresh 2D/3D fix; when false the fields below are NAN
  double latDeg;
  double lonDeg;
  float altMslM;    // altitude above mean sea level [m]
  float speedMps;   // groundspeed [m/s]
  float courseDeg;  // course over ground [deg]; NAN also when stationary
  float hdop;       // NAN until reported (sats/hdop stay live without a fix)
  float hAccM;      // 1-sigma horizontal error [m] from GST, NAN when unknown
  float vAccM;      // 1-sigma vertical error [m] from GST, NAN when unknown
  uint8_t sats;     // satellites used with a fix, in view (GSV) while acquiring
};

// Opens UART2 (GPS TX -> GPIO16, GPS RX -> GPIO17) and sends the boot
// configuration. Fire-and-forget: call it once the module is certainly
// out of reset (a second or two after power-on).
void begin();

// Drains pending UART bytes into the NMEA parser. Call every loop() pass.
void poll();

// True while NMEA sentences with a valid checksum keep arriving (last one
// within 2 s); false for an unplugged or dead receiver.
bool alive();

// Latest fix state; invalid fields are NAN so they print as "nan" in CSV.
Fix fix();

}  // namespace gps

/*
 * ULTRA 1600 video transmitter over IRC Tramp (doc/plans/2026-10-25-vtx-
 * tramp-control.md). Single-wire half-duplex UART, 9600 8N1, 16-byte
 * frames; the VTX's IRC pin is wired to UART1 RX (GPIO26, via 10 kOhm) and
 * TX (GPIO27, via 1 kOhm), so RX hears every byte TX sends and the driver
 * drops its own echo.
 *
 * The driver reconciles a desired state (from the 50 Hz CommandMessage)
 * with the VTX's readback: a set command (F/P/I) goes out only when the
 * readback differs, and the follow-up 'v' is the confirmation - Tramp has
 * no acks. Nothing blocks: poll() steps a state machine, one request in
 * flight, 100 ms reply timeout, >= 200 ms between requests.
 */

#pragma once

#include <stdint.h>

namespace vtx {

struct Status {
  bool present;            // an 'r' reply arrived since boot (range known)
  bool alive;              // a valid reply within the last 2 s
  uint16_t freqMhz;        // last 'v' readback, 0 = none yet
  uint16_t powerMw;        // configured power from 'v', 0 = none yet
  uint16_t actualPowerMw;  // actual power from 'v' (differs in pit mode / throttling)
  bool pitMode;            // readback
  bool raceLock;           // control mode bit 0 set: the VTX refuses sets
  bool refused;            // the current desired value was not taken (gave up)
  int8_t tempC;            // last 's' readback, VTX_TEMP_UNKNOWN = none yet
};

// Opens UART1 on the IRC pin. The VTX may be unpowered (USB-only bench):
// it keeps being probed once a second and shows up when the pack comes.
void begin();

// Drains RX, steps the state machine. Call every loop() pass.
void poll(uint32_t nowMs);

// Desired state from the operator. freqMhz 0 = leave as is; powerMw 0 =
// pit mode, 0xFFFF = leave as is. A change resets the attempt counter and
// the refused flag. Called with (0, 0xFFFF) during failsafe: no VTX action
// while the command link is down.
void setDesired(uint16_t freqMhz, uint16_t powerMw);

Status status();

}  // namespace vtx

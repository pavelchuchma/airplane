// Message definitions shared by the air and ground firmware.
// The Python mirror of these definitions lives in cocpitView (protocol.py).
// Bump PROTOCOL_VERSION on every change to the wire format.

#pragma once

#include <stdint.h>

constexpr uint8_t PROTOCOL_VERSION = 4;

// Both radios must sit on the same channel; set explicitly at bring-up.
constexpr uint8_t WIFI_CHANNEL = 1;

constexpr uint8_t MSG_TELEMETRY = 0x01;
constexpr uint8_t MSG_COMMAND = 0x02;

// TelemetryMessage.flags
constexpr uint8_t TELEM_FLAG_GPS_FIX = 1 << 0;           // lat1e7/lon1e7 valid
constexpr uint8_t TELEM_FLAG_FAILSAFE = 1 << 1;          // no valid command for kFailsafeTimeoutMs
constexpr uint8_t TELEM_FLAG_VERSION_MISMATCH = 1 << 2;  // last command had wrong version
constexpr uint8_t TELEM_FLAG_BARO_OK = 1 << 3;
constexpr uint8_t TELEM_FLAG_PITOT_OK = 1 << 4;
constexpr uint8_t TELEM_FLAG_IMU_OK = 1 << 5;  // clear = attitude fields are frozen/stale
constexpr uint8_t TELEM_FLAG_BATT_OK = 1 << 6;  // cellMv valid (balance lead present + plausible)

// Air -> PC, 50 Hz. GPS fields repeat between the 5 Hz GPS updates; floats
// keep the NAN-as-invalid convention of gps::Fix (survives struct.unpack).
struct __attribute__((packed)) TelemetryMessage {
  uint8_t msgId;    // MSG_TELEMETRY
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t flags;    // TELEM_FLAG_*
  uint8_t sats;
  uint32_t timeBootMs;
  float rollRad, pitchRad, yawRad;     // attitude [rad]
  float rollRate, pitchRate, yawRate;  // body rates [rad/s]
  float baroAltM;                      // m relative to boot location, NAN = no baro
  float climbMps;                      // low-pass-filtered vario [m/s], NAN = no baro
  float diffPressPa;                   // pitot dP, zeroed at boot + filtered, NAN = no pitot
  float iasMps;                        // indicated airspeed [m/s], NAN = no pitot
  int32_t lat1e7, lon1e7;              // deg * 1e7, valid iff TELEM_FLAG_GPS_FIX
  float altMslM, speedMps, courseDeg;  // NAN = unknown
  float hdop, hAccM, vAccM;            // NAN = unknown (hAcc/vAcc = 1-sigma [m])
  int8_t rssiDbm;                      // RSSI of last command frame seen at air, 0 = none yet
  uint8_t cmdLossPct;                  // % of expected command frames missed, last 1 s window
  uint8_t txFailPct;                   // % of telemetry esp_now_send without ACK, last 1 s
  uint8_t lastCmdVersion;              // version byte of the last received command, 0 = none
  uint16_t cellMv[3];                  // per-cell voltage [mV], valid iff TELEM_FLAG_BATT_OK
};
static_assert(sizeof(TelemetryMessage) == 90, "wire format changed - bump PROTOCOL_VERSION");

constexpr uint8_t COMMAND_CHANNELS = 8;

// PC -> air, fixed 50 Hz regardless of change (silence = link loss).
// Channels: AETR + 4 aux; control surfaces -10000..+10000 (0 = neutral),
// throttle 0..10000 (0 = idle).
struct __attribute__((packed)) CommandMessage {
  uint8_t msgId;    // MSG_COMMAND
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t flags;    // reserved (bit0 = ARMED later), 0 for now
  uint8_t seq;      // wrapping counter for duplicate/loss diagnostics
  int16_t channels[COMMAND_CHANNELS];
};
static_assert(sizeof(CommandMessage) == 20, "wire format changed - bump PROTOCOL_VERSION");

// ESP-NOW payload limit
static_assert(sizeof(TelemetryMessage) <= 250 && sizeof(CommandMessage) <= 250,
              "message exceeds the 250 B ESP-NOW payload limit");

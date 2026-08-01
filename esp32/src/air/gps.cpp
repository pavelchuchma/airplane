#include "gps.h"

#include <Arduino.h>
#include <TinyGPSPlus.h>

namespace gps {
namespace {

constexpr int kRxPin = 16;  // <- GPS TX
constexpr int kTxPin = 17;  // -> GPS RX
constexpr uint32_t kBaud = 38400;      // Quescan board preset (bare chip: 9600)
constexpr uint32_t kMaxFixAgeMs = 2000;

// UBX-CFG-VALSET key IDs (u-blox M10 interface description UBX-21035062).
constexpr uint32_t kKeyRateMeas = 0x30210001;    // U2, ms between fixes
constexpr uint32_t kKeyDynModel = 0x20110021;    // E1, dynamic platform model
constexpr uint32_t kKeyMsgoutGsaUart1 = 0x209100c0;  // U1, sentences per fix
constexpr uint32_t kKeyMsgoutGsvUart1 = 0x209100c5;
constexpr uint32_t kKeyMsgoutGllUart1 = 0x209100ca;
constexpr uint32_t kKeyMsgoutVtgUart1 = 0x209100b1;
constexpr uint32_t kKeyMsgoutGstUart1 = 0x209100d4;

constexpr uint16_t kMeasIntervalMs = 200;  // 5 Hz (module may clamp to ~4 Hz
                                           // with all 4 constellations enabled)
constexpr uint8_t kDynModelAirborne4g = 8;

TinyGPSPlus parser;

// GST error statistics (not parsed natively by TinyGPSPlus): terms 6/7/8 are
// the 1-sigma lat/lon/alt errors in meters. The talker ID depends on the
// solution ("GN" with multiple constellations), so both variants are watched.
TinyGPSCustom gstLatErrGn(parser, "GNGST", 6);
TinyGPSCustom gstLonErrGn(parser, "GNGST", 7);
TinyGPSCustom gstAltErrGn(parser, "GNGST", 8);
TinyGPSCustom gstLatErrGp(parser, "GPGST", 6);
TinyGPSCustom gstLonErrGp(parser, "GPGST", 7);
TinyGPSCustom gstAltErrGp(parser, "GPGST", 8);

// Fresh GST sigma in meters, NAN when missing (empty fields parse to 0).
float gstMeters(TinyGPSCustom &gn, TinyGPSCustom &gp) {
  TinyGPSCustom &src = gn.isValid() ? gn : gp;
  if (!src.isValid() || src.age() > kMaxFixAgeMs) {
    return NAN;
  }
  float v = atof(src.value());
  return v > 0.0f ? v : NAN;
}

// Frame: 0xB5 0x62, class, id, len (LE), payload, Fletcher-8 over class..payload.
void sendUbx(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t len) {
  uint8_t header[4] = {msgClass, msgId, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 0; i < sizeof(header); i++) {
    ckA += header[i];
    ckB += ckA;
  }
  for (uint16_t i = 0; i < len; i++) {
    ckA += payload[i];
    ckB += ckA;
  }
  Serial2.write(0xB5);
  Serial2.write(0x62);
  Serial2.write(header, sizeof(header));
  Serial2.write(payload, len);
  Serial2.write(ckA);
  Serial2.write(ckB);
}

// Appends key + value little-endian to a VALSET payload, returns the new offset.
size_t putCfg(uint8_t *buf, size_t off, uint32_t key, uint32_t value, uint8_t valueBytes) {
  for (int i = 0; i < 4; i++) {
    buf[off++] = (key >> (8 * i)) & 0xFF;
  }
  for (int i = 0; i < valueBytes; i++) {
    buf[off++] = (value >> (8 * i)) & 0xFF;
  }
  return off;
}

void sendBootConfig() {
  // One UBX-CFG-VALSET (0x06 0x8A) with all keys; fire-and-forget (the
  // module defaults are usable, the config just improves rate/filtering).
  uint8_t payload[4 + 6 + 6 * 5];
  payload[0] = 0x00;  // message version
  payload[1] = 0x03;  // layers: RAM | BBR (no flash on this chip)
  payload[2] = 0x00;  // reserved
  payload[3] = 0x00;
  size_t off = 4;
  off = putCfg(payload, off, kKeyRateMeas, kMeasIntervalMs, 2);
  off = putCfg(payload, off, kKeyDynModel, kDynModelAirborne4g, 1);
  off = putCfg(payload, off, kKeyMsgoutGsaUart1, 0, 1);
  off = putCfg(payload, off, kKeyMsgoutGsvUart1, 0, 1);
  off = putCfg(payload, off, kKeyMsgoutGllUart1, 0, 1);
  off = putCfg(payload, off, kKeyMsgoutVtgUart1, 0, 1);
  off = putCfg(payload, off, kKeyMsgoutGstUart1, 1, 1);  // error estimates
  sendUbx(0x06, 0x8A, payload, off);
}

}  // namespace

void begin() {
  Serial2.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
  sendBootConfig();
}

void poll() {
  while (Serial2.available()) {
    parser.encode(Serial2.read());
  }
}

bool alive() {
  return parser.passedChecksum() > 0;
}

Fix fix() {
  Fix f{};
  f.valid = parser.location.isValid() && parser.location.age() <= kMaxFixAgeMs;
  f.latDeg = f.valid ? parser.location.lat() : NAN;
  f.lonDeg = f.valid ? parser.location.lng() : NAN;
  f.altMslM = (f.valid && parser.altitude.isValid()) ? parser.altitude.meters() : NAN;
  f.speedMps = (f.valid && parser.speed.isValid()) ? parser.speed.mps() : NAN;
  // RMC leaves the course empty when (nearly) stationary
  f.courseDeg = (f.valid && parser.course.isValid()) ? parser.course.deg() : NAN;
  f.hdop = parser.hdop.isValid() ? parser.hdop.hdop() : NAN;
  float sigmaLat = gstMeters(gstLatErrGn, gstLatErrGp);
  float sigmaLon = gstMeters(gstLonErrGn, gstLonErrGp);
  f.hAccM = (f.valid && !isnan(sigmaLat) && !isnan(sigmaLon))
                ? sqrtf(sigmaLat * sigmaLat + sigmaLon * sigmaLon)
                : NAN;
  f.vAccM = f.valid ? gstMeters(gstAltErrGn, gstAltErrGp) : NAN;
  f.sats = parser.satellites.isValid() ? parser.satellites.value() : 0;
  return f;
}

}  // namespace gps

// Byte framing for the ground bridge's USB serial link (which, unlike
// ESP-NOW, does not preserve packet boundaries): COBS encoding with a 0x00
// frame delimiter plus a CRC-16/CCITT-FALSE trailer.
//
//   frame = COBS(payload || crc16_be(payload)) || 0x00
//
// Deliberately Arduino-free (<stdint.h> only) so the logic mirrors 1:1 to
// the Python implementation in cocpitView (protocol.py).

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace framing {

// Longest payload we ever frame (ESP-NOW payload limit).
constexpr size_t kMaxPayload = 250;

// Worst-case encoded size for a payload of `len` bytes: payload + CRC,
// plus one COBS overhead byte per started 254-byte block, plus the delimiter.
constexpr size_t encodedBound(size_t len) { return (len + 2) + (len + 2 + 253) / 254 + 1; }

constexpr size_t kMaxFrame = encodedBound(kMaxPayload);

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
uint16_t crc16(const uint8_t *data, size_t len);

// Appends the CRC, COBS-encodes and terminates with 0x00. Returns the frame
// length written to `out`, or 0 when the payload is empty/oversized or
// `outCap` is too small (use encodedBound()).
size_t frameEncode(const uint8_t *payload, size_t len, uint8_t *out, size_t outCap);

// Byte-at-a-time deframer: feed raw serial bytes, get back CRC-checked
// payloads. Oversized, malformed and CRC-failing frames are dropped
// (counted in dropped()) and the decoder resyncs on the next 0x00.
class FrameDecoder {
 public:
  // Returns the payload length when `byte` completed a valid frame (the
  // payload is then in payload()), 0 otherwise.
  size_t feed(uint8_t byte);

  const uint8_t *payload() const { return buf_; }
  uint32_t dropped() const { return dropped_; }

 private:
  uint8_t buf_[kMaxFrame];
  size_t len_ = 0;
  bool overflow_ = false;
  uint32_t dropped_ = 0;
};

}  // namespace framing

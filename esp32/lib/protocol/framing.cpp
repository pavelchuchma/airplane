#include "framing.h"

namespace framing {

namespace {

// COBS-encodes `len` bytes from `in` into `out` (needs encodedBound space,
// excluding the delimiter). Returns the encoded length.
size_t cobsEncode(const uint8_t *in, size_t len, uint8_t *out) {
  size_t codeIdx = 0;
  size_t writeIdx = 1;
  uint8_t code = 1;
  for (size_t readIdx = 0; readIdx < len; readIdx++) {
    if (in[readIdx] == 0) {
      out[codeIdx] = code;
      codeIdx = writeIdx++;
      code = 1;
    } else {
      out[writeIdx++] = in[readIdx];
      if (++code == 0xFF) {  // block full: restart before the next byte
        out[codeIdx] = code;
        codeIdx = writeIdx++;
        code = 1;
      }
    }
  }
  out[codeIdx] = code;
  return writeIdx;
}

// In-place COBS decode; returns the decoded length or SIZE_MAX when malformed.
size_t cobsDecode(uint8_t *buf, size_t len) {
  size_t readIdx = 0;
  size_t writeIdx = 0;
  while (readIdx < len) {
    uint8_t code = buf[readIdx];
    if (code == 0 || readIdx + code > len) {
      return SIZE_MAX;
    }
    readIdx++;
    for (uint8_t i = 1; i < code; i++) {
      buf[writeIdx++] = buf[readIdx++];
    }
    if (code != 0xFF && readIdx < len) {
      buf[writeIdx++] = 0;
    }
  }
  return writeIdx;
}

}  // namespace

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

size_t frameEncode(const uint8_t *payload, size_t len, uint8_t *out, size_t outCap) {
  if (len == 0 || len > kMaxPayload || outCap < encodedBound(len)) {
    return 0;
  }
  uint8_t raw[kMaxPayload + 2];
  for (size_t i = 0; i < len; i++) {
    raw[i] = payload[i];
  }
  uint16_t crc = crc16(payload, len);
  raw[len] = crc >> 8;  // big-endian trailer
  raw[len + 1] = crc & 0xFF;

  size_t encoded = cobsEncode(raw, len + 2, out);
  out[encoded++] = 0x00;
  return encoded;
}

size_t FrameDecoder::feed(uint8_t byte) {
  if (byte != 0) {
    if (len_ < sizeof(buf_)) {
      buf_[len_++] = byte;
    } else {
      overflow_ = true;
    }
    return 0;
  }

  size_t encodedLen = len_;
  bool overflow = overflow_;
  len_ = 0;
  overflow_ = false;

  if (encodedLen == 0) {
    return 0;  // idle delimiter / resync after garbage, not an error
  }
  if (overflow) {
    dropped_++;
    return 0;
  }
  size_t decoded = cobsDecode(buf_, encodedLen);
  if (decoded == SIZE_MAX || decoded < 3) {  // at least 1 payload byte + CRC
    dropped_++;
    return 0;
  }
  size_t payloadLen = decoded - 2;
  uint16_t expected = ((uint16_t)buf_[payloadLen] << 8) | buf_[payloadLen + 1];
  if (crc16(buf_, payloadLen) != expected) {
    dropped_++;
    return 0;
  }
  return payloadLen;
}

}  // namespace framing

#include "diag.h"

#include <Arduino.h>
#include <stdarg.h>

#include "messages.h"

namespace diag {
namespace {

// Holds the boot burst until cocpitView connects (lines are only sent while
// commands arrive), with room for a stall burst on top.
constexpr size_t kCapacity = 32;

struct Line {
  uint16_t seq;
  uint32_t timeBootMs;
  uint8_t len;
  char text[LOG_TEXT_MAX];
};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Line ring[kCapacity];
size_t head = 0;  // oldest queued line
size_t count = 0;
uint16_t nextSeq = 0;
uint16_t dropped = 0;

}  // namespace

void print(const char *fmt, ...) {
  char text[160];  // USB gets the whole line, the radio the first LOG_TEXT_MAX bytes
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(text, sizeof(text), fmt, args);
  va_end(args);
  if (n < 0) {
    return;
  }
  Serial.printf("# %s\n", text);

  uint8_t len = (uint8_t)min((size_t)n, min(sizeof(text) - 1, (size_t)LOG_TEXT_MAX));
  uint32_t nowMs = millis();
  portENTER_CRITICAL(&mux);
  if (count == kCapacity) {
    head = (head + 1) % kCapacity;
    count--;
    dropped++;
  }
  Line &line = ring[(head + count) % kCapacity];
  line.seq = nextSeq++;
  line.timeBootMs = nowMs;
  line.len = len;
  memcpy(line.text, text, len);
  count++;
  portEXIT_CRITICAL(&mux);
}

size_t peek(uint8_t *out, uint16_t &seq) {
  LogMessageHeader hdr = {};
  hdr.msgId = MSG_LOG;
  hdr.version = PROTOCOL_VERSION;
  size_t len = 0;
  portENTER_CRITICAL(&mux);
  if (count > 0) {
    const Line &line = ring[head];
    hdr.seq = line.seq;
    hdr.timeBootMs = line.timeBootMs;
    hdr.dropped = dropped;
    memcpy(out + sizeof(hdr), line.text, line.len);
    len = sizeof(hdr) + line.len;
  }
  portEXIT_CRITICAL(&mux);
  if (len > 0) {
    memcpy(out, &hdr, sizeof(hdr));
    seq = hdr.seq;
  }
  return len;
}

void commit(uint16_t seq) {
  portENTER_CRITICAL(&mux);
  if (count > 0 && ring[head].seq == seq) {
    head = (head + 1) % kCapacity;
    count--;
  }
  portEXIT_CRITICAL(&mux);
}

}  // namespace diag

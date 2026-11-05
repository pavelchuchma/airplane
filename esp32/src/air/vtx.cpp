#include "vtx.h"

#include <Arduino.h>
#include <string.h>

#include "diag.h"
#include "messages.h"

namespace vtx {
namespace {

constexpr int kRxPin = 26;  // <- IRC via 10 kOhm
constexpr int kTxPin = 27;  // -> IRC via 1 kOhm
constexpr uint32_t kBaud = 9600;

constexpr size_t kFrameLen = 16;
constexpr uint8_t kFrameStart = 0x0F;

// Tramp commands (ASCII).
constexpr uint8_t kCmdRange = 'r';
constexpr uint8_t kCmdStatus = 'v';
constexpr uint8_t kCmdTemp = 's';
constexpr uint8_t kCmdSetFreq = 'F';
constexpr uint8_t kCmdSetPower = 'P';
constexpr uint8_t kCmdSetPit = 'I';

// 'I' payload: Betaflight sends 0 to enter pit mode, 1 to leave it; the 'v'
// readback byte 7 reads 1 while in pit mode. Confirmed on the ULTRA at
// bring-up step 3 of the plan.
constexpr uint8_t kPitEnter = 0;
constexpr uint8_t kPitLeave = 1;
constexpr uint8_t kControlRaceLock = 0x01;  // 'v' byte 6

constexpr uint32_t kReplyTimeoutMs = 100;
constexpr uint32_t kMinRequestGapMs = 200;  // Betaflight's TRAMP_MIN_REQUEST_PERIOD
constexpr uint32_t kProbeIntervalMs = 1000;
constexpr uint32_t kStatusIntervalMs = 500;
constexpr uint32_t kTempIntervalMs = 2000;
constexpr uint32_t kSetIntervalMs = 500;
constexpr uint32_t kSetFollowUpMs = 200;  // 'v' after a set
constexpr uint32_t kAliveTimeoutMs = 2000;
constexpr uint32_t kBadFrameReportMs = 5000;
constexpr uint8_t kMaxSetAttempts = 10;  // answered-but-disagreeing tries

// Frame assembler.
uint8_t rxBuf[kFrameLen];
size_t rxLen = 0;
uint32_t lastBadFrameMs = 0;
bool badFrameSeen = false;

// Request in flight.
uint8_t txFrame[kFrameLen];
uint8_t pendingCmd = 0;  // 0 = none
bool echoPending = false;
uint32_t pendingSentMs = 0;
uint32_t lastRequestMs = 0;

// Schedule.
uint32_t nextProbeMs = 0;
uint32_t nextStatusMs = 0;
uint32_t nextTempMs = 0;
uint32_t nextSetAllowedMs = 0;
uint32_t followUpDueMs = 0;
bool followUpPending = false;

// Readback.
bool present = false;
bool haveStatus = false;  // a valid 'v' since boot
bool alive = false;
bool everAlive = false;
uint32_t lastReplyMs = 0;
uint16_t freqMinMhz = 0, freqMaxMhz = 0, powerMaxMw = 0;
uint16_t rbFreq = 0, rbPower = 0, rbActual = 0;
uint8_t rbControl = 0;
bool rbPit = false;
int8_t rbTemp = VTX_TEMP_UNKNOWN;
bool raceLockReported = false;
bool dumped[3] = {false, false, false};  // first r / v / s reply per boot

// Desired state and the attempt bookkeeping for the current desired value.
uint16_t desiredFreq = CMD_VTX_FREQ_LEAVE;
uint16_t desiredPower = CMD_VTX_POWER_LEAVE;
bool refused = false;
uint8_t lastSetCmd = 0;
uint16_t lastSetValue = 0;
bool lastSetAnswered = false;  // a 'v' arrived after the last set
uint8_t setAttempts = 0;

struct SetRequest {
  uint8_t cmd;     // 0 = nothing to do
  uint16_t value;  // MHz, mW or the 'I' byte
};

uint8_t checksum(const uint8_t *frame) {
  uint8_t sum = 0;
  for (size_t i = 1; i < 14; i++) {
    sum += frame[i];
  }
  return sum;
}

uint16_t u16At(const uint8_t *frame, size_t i) {
  return (uint16_t)(frame[i] | (frame[i + 1] << 8));
}

void hexLine(const char *label, const uint8_t *frame) {
  char hex[kFrameLen * 3 + 1];
  for (size_t i = 0; i < kFrameLen; i++) {
    snprintf(hex + i * 3, 4, "%02X ", frame[i]);
  }
  hex[kFrameLen * 3 - 1] = '\0';
  diag::print("VTX rx %s: %s", label, hex);
}

void send(uint8_t cmd, uint16_t param, uint32_t nowMs) {
  memset(txFrame, 0, sizeof(txFrame));
  txFrame[0] = kFrameStart;
  txFrame[1] = cmd;
  txFrame[2] = param & 0xFF;
  txFrame[3] = param >> 8;
  txFrame[14] = checksum(txFrame);
  Serial1.write(txFrame, sizeof(txFrame));
  pendingCmd = cmd;
  echoPending = true;
  pendingSentMs = nowMs;
  lastRequestMs = nowMs;
}

const char *setName(uint8_t cmd) {
  return cmd == kCmdSetFreq ? "freq" : cmd == kCmdSetPower ? "power" : "pit";
}

// What the readback still differs in, in the order the plan fixes it.
SetRequest chooseSet() {
  if (desiredFreq != CMD_VTX_FREQ_LEAVE && desiredFreq != rbFreq) {
    return {kCmdSetFreq, desiredFreq};
  }
  if (desiredPower == CMD_VTX_POWER_PIT) {
    return rbPit ? SetRequest{0, 0} : SetRequest{kCmdSetPit, kPitEnter};
  }
  if (desiredPower != CMD_VTX_POWER_LEAVE) {
    if (rbPit) {
      return {kCmdSetPit, kPitLeave};
    }
    if (desiredPower != rbPower) {
      return {kCmdSetPower, desiredPower};
    }
  }
  return {0, 0};
}

void reportReadback() {
  diag::print("VTX %u MHz %u mW (actual %u) pit=%d lock=%d %d C", rbFreq, rbPower, rbActual, rbPit,
              (rbControl & kControlRaceLock) != 0, rbTemp);
}

void handleRange(const uint8_t *f) {
  uint16_t fmin = u16At(f, 2);
  if (fmin == 0) {
    return;  // our own echo (zero payload), or garbage
  }
  freqMinMhz = fmin;
  freqMaxMhz = u16At(f, 4);
  powerMaxMw = u16At(f, 6);
  if (!present) {
    present = true;
    diag::print("VTX present: %u-%u MHz, max %u mW", freqMinMhz, freqMaxMhz, powerMaxMw);
  }
}

void handleStatus(const uint8_t *f) {
  uint16_t freq = u16At(f, 2);
  if (freq == 0) {
    return;
  }
  uint16_t power = u16At(f, 4);
  uint8_t control = f[6];
  bool pit = f[7] != 0;
  uint16_t actual = u16At(f, 8);
  bool changed = !haveStatus || freq != rbFreq || power != rbPower || actual != rbActual ||
                 pit != rbPit || control != rbControl;
  rbFreq = freq;
  rbPower = power;
  rbActual = actual;
  rbControl = control;
  rbPit = pit;
  haveStatus = true;
  if (lastSetCmd != 0) {
    lastSetAnswered = true;
  }
  if (changed) {
    reportReadback();
  }
  bool locked = (control & kControlRaceLock) != 0;
  if (locked != raceLockReported) {
    raceLockReported = locked;
    diag::print(locked ? "VTX race lock - sets disabled" : "VTX race lock released");
  }
}

void handleTemp(const uint8_t *f) {
  int16_t temp = (int16_t)u16At(f, 6);
  if (temp == 0) {
    return;
  }
  rbTemp = temp < -127 ? -127 : temp > 127 ? 127 : (int8_t)temp;
}

void handleFrame(const uint8_t *f, uint32_t nowMs) {
  if (echoPending && memcmp(f, txFrame, kFrameLen) == 0) {
    echoPending = false;
    if (pendingCmd == kCmdSetFreq || pendingCmd == kCmdSetPower || pendingCmd == kCmdSetPit) {
      pendingCmd = 0;  // sets are not answered; the echo is all there is
    }
    return;
  }
  uint8_t cmd = f[1];
  if (cmd != kCmdRange && cmd != kCmdStatus && cmd != kCmdTemp) {
    goto bad;
  }
  {
    int kind = cmd == kCmdRange ? 0 : cmd == kCmdStatus ? 1 : 2;
    if (!dumped[kind]) {
      dumped[kind] = true;
      hexLine(cmd == kCmdRange ? "r" : cmd == kCmdStatus ? "v" : "s", f);
    }
    if (cmd == kCmdRange) {
      handleRange(f);
    } else if (cmd == kCmdStatus) {
      handleStatus(f);
    } else {
      handleTemp(f);
    }
    lastReplyMs = nowMs;
    everAlive = true;
    if (pendingCmd == cmd) {
      pendingCmd = 0;
    }
    return;
  }
bad:
  if (!badFrameSeen || nowMs - lastBadFrameMs >= kBadFrameReportMs) {
    badFrameSeen = true;
    lastBadFrameMs = nowMs;
    hexLine("bad", f);
  }
}

void drainRx(uint32_t nowMs) {
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();
    if (rxLen == 0 && b != kFrameStart) {
      continue;  // resync on the start byte
    }
    rxBuf[rxLen++] = b;
    if (rxLen < kFrameLen) {
      continue;
    }
    rxLen = 0;
    if (rxBuf[14] == checksum(rxBuf) && rxBuf[15] == 0) {
      handleFrame(rxBuf, nowMs);
      continue;
    }
    if (!badFrameSeen || nowMs - lastBadFrameMs >= kBadFrameReportMs) {
      badFrameSeen = true;
      lastBadFrameMs = nowMs;
      hexLine("bad", rxBuf);
    }
    // Resync inside the bad frame: keep everything from the next start byte.
    for (size_t i = 1; i < kFrameLen; i++) {
      if (rxBuf[i] == kFrameStart) {
        rxLen = kFrameLen - i;
        memmove(rxBuf, rxBuf + i, rxLen);
        break;
      }
    }
  }
}

// Range checks against the 'r' reply; a request the VTX cannot do is
// refused here and never sent.
bool withinRange(const SetRequest &req) {
  if (req.cmd == kCmdSetFreq && (req.value < freqMinMhz || req.value > freqMaxMhz)) {
    diag::print("VTX rejected freq %u MHz - outside %u-%u", req.value, freqMinMhz, freqMaxMhz);
    return false;
  }
  if (req.cmd == kCmdSetPower && req.value > powerMaxMw) {
    diag::print("VTX rejected power %u mW - max %u", req.value, powerMaxMw);
    return false;
  }
  return true;
}

// Sends the set the readback still needs, if any; true when one went out.
bool trySet(uint32_t nowMs) {
  if (!haveStatus || refused || (rbControl & kControlRaceLock) || (int32_t)(nowMs - nextSetAllowedMs) < 0) {
    return false;
  }
  SetRequest req = chooseSet();
  if (req.cmd == 0) {
    setAttempts = 0;
    lastSetCmd = 0;
    return false;
  }
  if (!withinRange(req)) {
    refused = true;
    return false;
  }
  if (req.cmd == lastSetCmd && req.value == lastSetValue) {
    if (lastSetAnswered) {
      setAttempts++;  // a lost follow-up 'v' is not a refusal
    }
  } else {
    setAttempts = 1;
  }
  if (setAttempts > kMaxSetAttempts) {
    refused = true;
    if (req.cmd == kCmdSetPit) {
      diag::print("VTX refused pit %s (reads %d)", req.value == kPitEnter ? "on" : "off", rbPit);
    } else {
      diag::print("VTX refused %s %u (reads %u)", setName(req.cmd), req.value,
                  req.cmd == kCmdSetFreq ? rbFreq : rbPower);
    }
    return false;
  }
  lastSetCmd = req.cmd;
  lastSetValue = req.value;
  lastSetAnswered = false;
  if (req.cmd == kCmdSetPit) {
    diag::print("VTX set pit %s", req.value == kPitEnter ? "on" : "off");
  } else {
    diag::print("VTX set %s %u", setName(req.cmd), req.value);
  }
  send(req.cmd, req.value, nowMs);
  nextSetAllowedMs = nowMs + kSetIntervalMs;
  followUpDueMs = nowMs + kSetFollowUpMs;
  followUpPending = true;
  return true;
}

}  // namespace

void begin() {
  Serial1.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
}

void setDesired(uint16_t freqMhz, uint16_t powerMw) {
  if (freqMhz == desiredFreq && powerMw == desiredPower) {
    return;
  }
  desiredFreq = freqMhz;
  desiredPower = powerMw;
  refused = false;
  setAttempts = 0;
  lastSetCmd = 0;
}

void poll(uint32_t nowMs) {
  drainRx(nowMs);

  if (pendingCmd != 0 && nowMs - pendingSentMs > kReplyTimeoutMs) {
    pendingCmd = 0;
    echoPending = false;
  }

  bool nowAlive = everAlive && nowMs - lastReplyMs <= kAliveTimeoutMs;
  if (nowAlive != alive) {
    static bool wasSilent = false;  // first contact is "present", not "again"
    alive = nowAlive;
    if (!alive) {
      wasSilent = true;
      diag::print("VTX silent for 2 s");
    } else if (wasSilent) {
      diag::print("VTX talking again");
    }
  }

  if (pendingCmd != 0 || nowMs - lastRequestMs < kMinRequestGapMs) {
    return;
  }

  if (!present) {
    if ((int32_t)(nowMs - nextProbeMs) >= 0) {
      send(kCmdRange, 0, nowMs);
      nextProbeMs = nowMs + kProbeIntervalMs;
    }
    return;
  }

  if (followUpPending && (int32_t)(nowMs - followUpDueMs) >= 0) {
    followUpPending = false;
    send(kCmdStatus, 0, nowMs);
    nextStatusMs = nowMs + kStatusIntervalMs;
    return;
  }
  if (followUpPending) {
    return;  // the set is on the wire, give it the 200 ms before anything else
  }
  if (trySet(nowMs)) {
    return;
  }
  if ((int32_t)(nowMs - nextStatusMs) >= 0) {
    send(kCmdStatus, 0, nowMs);
    nextStatusMs = nowMs + kStatusIntervalMs;
    return;
  }
  if ((int32_t)(nowMs - nextTempMs) >= 0) {
    send(kCmdTemp, 0, nowMs);
    nextTempMs = nowMs + kTempIntervalMs;
  }
}

Status status() {
  Status s{};
  s.present = present;
  s.alive = alive;
  s.freqMhz = rbFreq;
  s.powerMw = rbPower;
  s.actualPowerMw = rbActual;
  s.pitMode = rbPit;
  s.raceLock = (rbControl & kControlRaceLock) != 0;
  s.refused = refused;
  s.tempC = rbTemp;
  return s;
}

}  // namespace vtx

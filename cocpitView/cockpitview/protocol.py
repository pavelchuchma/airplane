"""Python mirror of the ESP-NOW wire protocol (esp32/lib/protocol/).

Struct layouts mirror messages.h, the COBS + CRC-16/CCITT-FALSE framing
mirrors framing.cpp — change them together and bump PROTOCOL_VERSION on
every wire-format change (see CLAUDE.md). The module-level size asserts are
the Python-side counterpart of the static_asserts in messages.h.

Frame layout on the ground bridge's serial link:

    frame = COBS(payload || crc16_be(payload)) || 0x00

Attitude sign conventions on the wire (v5 onwards): roll positive right
wing down, pitch positive nose up, yaw a *clockwise* heading with an
arbitrary datum. The air side's internal frame is X-forward/Y-left/Z-up,
where yaw runs counter-clockwise, so it flips yaw and yawspeed before
sending. v4 and older shipped the unflipped (mirrored) heading -
flightlog.upgrade() corrects those logs on read.

Two invariants the flight log (flightlog.py) relies on:

  - msgId and version are the first two bytes of every message and never
    move, so a logged frame's version is known without knowing its layout.
  - Field names in Telemetry / Command are never renamed, only added. The
    wire order may change (the struct format handles it); the names are the
    API for analysis scripts and pandas columns across logs of all versions.
"""

import collections
import struct

PROTOCOL_VERSION = 6

MSG_TELEMETRY = 0x01
MSG_COMMAND = 0x02

# TelemetryMessage.flags
TELEM_FLAG_GPS_FIX = 1 << 0
TELEM_FLAG_FAILSAFE = 1 << 1
TELEM_FLAG_VERSION_MISMATCH = 1 << 2
TELEM_FLAG_BARO_OK = 1 << 3
TELEM_FLAG_PITOT_OK = 1 << 4
TELEM_FLAG_IMU_OK = 1 << 5  # clear = attitude fields are frozen/stale
TELEM_FLAG_BATT_OK = 1 << 6  # cell*_mv valid (balance lead present + plausible)
# boot gyro calibration waiting for stillness: attitude 0, rates raw (uncompensated)
TELEM_FLAG_IMU_CALIBRATING = 1 << 7

COMMAND_CHANNELS = 8
NEUTRAL_CHANNELS = (0,) * COMMAND_CHANNELS  # surfaces neutral, throttle idle

_TELEMETRY = struct.Struct("<BBBBI10f2i6fbBBB3H")
_COMMAND = struct.Struct("<BBBB8h")
assert _TELEMETRY.size == 90, "telemetry layout drifted from messages.h"
assert _COMMAND.size == 20, "command layout drifted from messages.h"

Telemetry = collections.namedtuple(
    "Telemetry",
    "msg_id version flags sats time_boot_ms "
    "roll pitch yaw rollspeed pitchspeed yawspeed baro_alt_m climb_mps "
    "diff_press_pa ias_mps "
    "lat1e7 lon1e7 alt_msl_m gs_mps cog_deg hdop hacc_m vacc_m "
    "rssi_dbm cmd_loss_pct tx_fail_pct last_cmd_version "
    "cell1_mv cell2_mv cell3_mv",
)

Command = collections.namedtuple(
    "Command", "msg_id version flags seq " + " ".join(f"ch{i}" for i in range(COMMAND_CHANNELS))
)

TELEMETRY_SIZE = _TELEMETRY.size


def decode_telemetry(payload):
    """Unpacks a MSG_TELEMETRY payload, or returns None when it isn't one."""
    if len(payload) != _TELEMETRY.size or payload[0] != MSG_TELEMETRY:
        return None
    return Telemetry._make(_TELEMETRY.unpack(payload))


def decode_command(payload):
    """Unpacks a MSG_COMMAND payload, or returns None when it isn't one."""
    if len(payload) != _COMMAND.size or payload[0] != MSG_COMMAND:
        return None
    return Command._make(_COMMAND.unpack(payload))


def encode_command(seq, channels=NEUTRAL_CHANNELS, flags=0):
    return _COMMAND.pack(MSG_COMMAND, PROTOCOL_VERSION, flags, seq & 0xFF, *channels)


def crc16(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), mirrors framing.cpp."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out = bytearray()
    block = bytearray()
    for byte in data:
        if byte == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
        else:
            block.append(byte)
            if len(block) == 254:
                out.append(255)
                out += block
                block.clear()
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data):
    """Decodes a COBS block (no 0x00 delimiter), or returns None when malformed."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            return None
        out += data[i + 1 : i + code]
        i += code
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(payload):
    crc = crc16(payload)
    return cobs_encode(payload + bytes((crc >> 8, crc & 0xFF))) + b"\x00"


class FrameSplitter:
    """Feed raw serial bytes, iterate CRC-checked payloads (mirrors
    framing::FrameDecoder; malformed and CRC-failing frames are dropped and
    counted, the stream resyncs on the next 0x00 delimiter)."""

    MAX_FRAME = 300

    def __init__(self):
        self._buf = bytearray()
        self.dropped = 0

    def feed(self, data):
        for byte in data:
            if byte != 0:
                self._buf.append(byte)
                continue
            encoded = bytes(self._buf)
            self._buf.clear()
            if not encoded:
                continue  # idle delimiter / resync, not an error
            payload = self._deframe(encoded)
            if payload is None:
                self.dropped += 1
            else:
                yield payload

    def _deframe(self, encoded):
        if len(encoded) > self.MAX_FRAME:
            return None
        decoded = cobs_decode(encoded)
        if decoded is None or len(decoded) < 3:  # at least 1 payload byte + CRC
            return None
        payload, trailer = decoded[:-2], decoded[-2:]
        if crc16(payload) != (trailer[0] << 8 | trailer[1]):
            return None
        return payload

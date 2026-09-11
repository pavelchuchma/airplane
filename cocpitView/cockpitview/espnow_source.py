"""ESP-NOW telemetry source - the ground bridge ESP32 over USB serial.

The ground board is a transparent ESP-NOW <-> USB bridge (esp32/src/ground);
its serial link carries COBS+CRC framed binary in both directions (460800
baud, see protocol.py):

  - down: TelemetryMessage from the air board, 50 Hz. Decoded and handed
    to publisher.TelemetryPublisher, which turns it into the MAVLink set
    for the hub (the log replay uses the same publisher). Every payload the
    bridge delivers is handed to the FlightLog before any filtering
    (flightlog.py), so the log records what arrived, not what we
    understood.
  - up: CommandMessage at a fixed 50 Hz - neutral channels for now (no
    joystick yet); the fixed rate is the air side's link-loss detector, so
    the sender runs whenever the port is open. Each sent payload goes to the
    FlightLog too.

By default the serial port is resolved by the ground board's factory MAC
address (the boards' clone CP2102s share one USB serial number, so port
names are ambiguous - see ../esp32/tools/board_id.py). The MAC and the
probe-order hint come from [env:ground] in ../esp32/platformio.ini. The
probe resets the board, which is harmless - the bridge is stateless and
reboots in under a second.

This never exits: while the board is missing (or the link drops) it keeps
retrying and the HUD shows NO TELEMETRY.
"""

import configparser
import datetime
import pathlib
import sys
import threading
import time

import serial

from . import protocol
from .flightlog import FlightLog
from .publisher import TelemetryPublisher

ESP32_DIR = pathlib.Path(__file__).resolve().parent.parent.parent / "esp32"
sys.path.insert(0, str(ESP32_DIR / "tools"))
import board_id  # noqa: E402

RETRY_PERIOD_S = 3.0
COMMAND_PERIOD_S = 0.02  # fixed 50 Hz, silence = link loss on the air side
STATS_WINDOW_S = 1.0
TELEMETRY_RATE_HZ = 50.0


def find_ground_port():
    """Resolve the ground ESP32's serial port by its factory MAC (resets the board)."""
    config = configparser.ConfigParser(inline_comment_prefixes=(";",))
    config.read(ESP32_DIR / "platformio.ini")
    mac = config.get("env:ground", "custom_board_mac", fallback="").strip()
    if not mac:
        raise LookupError(f"custom_board_mac not set for [env:ground] in {ESP32_DIR / 'platformio.ini'}")
    hint = config.get("env:ground", "custom_usb_location", fallback="").strip() or None
    return board_id.find_board(mac, hint)


class EspNowSource:
    """Background threads bridging the ground board's serial link to the hub."""

    def __init__(self, hub, port=None, baud=460800, log=None):
        self._hub = hub
        self._port = port
        self._baud = baud
        self._log = log if log is not None else FlightLog(None)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._publisher = TelemetryPublisher(hub)
        self._version_warned = False

    def start(self):
        self._thread.start()

    def _run(self):
        while True:
            try:
                port = self._port or find_ground_port()
            except (LookupError, RuntimeError) as err:
                print(f"ground board not found: {err} - retrying in {RETRY_PERIOD_S:.0f} s")
                time.sleep(RETRY_PERIOD_S)
                continue
            try:
                self._pump(port)
            except (serial.SerialException, OSError) as err:
                print(f"serial link lost ({err}) - reconnecting")
                self._log.event("serial_lost", port=str(port), error=str(err))
                time.sleep(RETRY_PERIOD_S)

    def _pump(self, port):
        with serial.Serial(port, self._baud, timeout=1.0) as ser:
            print(f"Reading ESP-NOW telemetry from the ground bridge at {port} @ {self._baud} baud")
            self._log.event("serial_open", port=str(port), baud=self._baud)
            stop = threading.Event()
            # pyserial is safe with one reading + one writing thread per port;
            # the sender dies with the connection and restarts on reconnect.
            sender = threading.Thread(target=self._send_commands, args=(ser, stop), daemon=True)
            sender.start()
            try:
                self._read(ser)
            finally:
                stop.set()

    def _send_commands(self, ser, stop):
        seq = 0
        next_send = time.monotonic()
        while True:
            # absolute schedule - a plain wait(period) would add the write
            # time + scheduler jitter to every cycle and drift below 50 Hz,
            # which the air side would (correctly) report as command loss
            next_send += COMMAND_PERIOD_S
            if stop.wait(max(0.0, next_send - time.monotonic())):
                return
            payload = protocol.encode_command(seq)
            try:
                ser.write(protocol.encode_frame(payload))
            except (serial.SerialException, OSError):
                return  # the reader handles the reconnect
            self._log.uplink(payload)
            seq = (seq + 1) & 0xFF

    def _read(self, ser):
        splitter = protocol.FrameSplitter()
        start = time.monotonic()
        window_start = start
        window_count = 0
        telem_loss_pct = 0

        while True:
            # return on the first byte, then drain: read(256) would block
            # until 256 B (~2.5 frames) arrive and quantize `now` - and the
            # logged timestamps - to ~50 ms batches
            data = ser.read(max(1, ser.in_waiting))
            now = time.monotonic()

            if now - window_start >= STATS_WINDOW_S:
                expected = (now - window_start) * TELEMETRY_RATE_HZ
                lost = max(0, int(round(expected)) - window_count)
                telem_loss_pct = min(100, int(round(lost * 100 / expected)))
                self._publisher.telem_lost_total += lost
                # 1 Hz liveness heartbeat for the log: proves the reader was
                # alive during a telemetry gap; utc pairs the monotonic
                # timestamps with video/GPS time even after a laptop sleep
                self._log.event(
                    "link_window",
                    telem_loss_pct=telem_loss_pct,
                    frames=window_count,
                    splitter_dropped=splitter.dropped,
                    utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds"),
                )
                window_start = now
                window_count = 0

            if not data:
                continue  # timeout with no data, try again

            for payload in splitter.feed(data):
                self._log.downlink(payload)  # before any filter: log what arrived
                if payload[0] != protocol.MSG_TELEMETRY:
                    continue
                if payload[1] != protocol.PROTOCOL_VERSION:
                    if not self._version_warned:
                        self._version_warned = True
                        print(
                            f"telemetry protocol v{payload[1]}, expected v{protocol.PROTOCOL_VERSION}"
                            " - reflash the air firmware / git pull; ignoring telemetry"
                        )
                    continue
                t = protocol.decode_telemetry(payload)
                if t is None:
                    continue
                window_count += 1
                self._publisher.publish(t, now - start, now, telem_loss_pct)

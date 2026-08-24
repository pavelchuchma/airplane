"""ESP-NOW telemetry source - the ground bridge ESP32 over USB serial.

The ground board is a transparent ESP-NOW <-> USB bridge (esp32/src/ground);
its serial link carries COBS+CRC framed binary in both directions (460800
baud, see protocol.py):

  - down: TelemetryMessage from the air board, 50 Hz. Published to the
    TelemetryHub as MAVLink ATTITUDE + VFR_HUD + ALTITUDE + GPS_RAW_INT +
    GLOBAL_POSITION_INT (same set and scaling the retired CSV reader used)
    plus a 1 Hz HEARTBEAT, a ~2 Hz RADIO_STATUS and a ~2 Hz BATTERY_STATUS
    (per-cell voltages; skipped while the air side reports the battery
    sense dead - the HUD hides its BAT line, QGC shows stale) for QGC. Link stats
    (air-side RSSI/command loss, PC-side telemetry loss, the failsafe flag)
    additionally go straight into the HUD snapshot via hub.update_link().
  - up: CommandMessage at a fixed 50 Hz - neutral channels for now (no
    joystick yet); the fixed rate is the air side's link-loss detector, so
    the sender runs whenever the port is open.

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
import math
import pathlib
import sys
import threading
import time

import serial
from pymavlink import mavutil

from . import protocol
from .telemetry import heartbeat

ESP32_DIR = pathlib.Path(__file__).resolve().parent.parent.parent / "esp32"
sys.path.insert(0, str(ESP32_DIR / "tools"))
import board_id  # noqa: E402

HEARTBEAT_PERIOD_S = 1.0
RETRY_PERIOD_S = 3.0
COMMAND_PERIOD_S = 0.02  # fixed 50 Hz, silence = link loss on the air side
RADIO_STATUS_PERIOD_S = 0.5
BATTERY_STATUS_PERIOD_S = 0.5
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

    def __init__(self, hub, port=None, baud=460800):
        self._hub = hub
        self._port = port
        self._baud = baud
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._version_warned = False
        self._mismatch_warned = False
        self._telem_lost_total = 0  # cumulative, for RADIO_STATUS.rxerrors
        self._failsafe = None  # last seen air-side failsafe state (None = unknown)
        self._last_battery_status = 0.0

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
                time.sleep(RETRY_PERIOD_S)

    def _pump(self, port):
        with serial.Serial(port, self._baud, timeout=1.0) as ser:
            print(f"Reading ESP-NOW telemetry from the ground bridge at {port} @ {self._baud} baud")
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
            try:
                ser.write(protocol.encode_frame(protocol.encode_command(seq)))
            except (serial.SerialException, OSError):
                return  # the reader handles the reconnect
            seq = (seq + 1) & 0xFF

    def _read(self, ser):
        splitter = protocol.FrameSplitter()
        start = time.monotonic()
        last_heartbeat = 0.0
        last_radio_status = 0.0
        window_start = start
        window_count = 0
        telem_loss_pct = 0

        while True:
            data = ser.read(256)
            now = time.monotonic()

            if now - window_start >= STATS_WINDOW_S:
                expected = (now - window_start) * TELEMETRY_RATE_HZ
                lost = max(0, int(round(expected)) - window_count)
                telem_loss_pct = min(100, int(round(lost * 100 / expected)))
                self._telem_lost_total += lost
                window_start = now
                window_count = 0

            if not data:
                continue  # timeout with no data, try again

            for payload in splitter.feed(data):
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

                if now - last_heartbeat >= HEARTBEAT_PERIOD_S:
                    self._hub.publish(heartbeat(self._hub.mav))
                    last_heartbeat = now

                self._publish_telemetry(t, now - start, telem_loss_pct)

                if now - last_radio_status >= RADIO_STATUS_PERIOD_S:
                    self._publish_radio_status(t)
                    last_radio_status = now

                if now - self._last_battery_status >= BATTERY_STATUS_PERIOD_S:
                    self._publish_battery(t)
                    self._last_battery_status = now

    def _publish_telemetry(self, t, elapsed, telem_loss_pct):
        time_boot_ms = int(elapsed * 1000) & 0xFFFFFFFF
        time_usec = int(elapsed * 1e6) & 0xFFFFFFFFFFFFFFFF
        fix = bool(t.flags & protocol.TELEM_FLAG_GPS_FIX)
        failsafe = bool(t.flags & protocol.TELEM_FLAG_FAILSAFE)

        if t.flags & protocol.TELEM_FLAG_VERSION_MISMATCH and not self._mismatch_warned:
            self._mismatch_warned = True
            print(
                f"air firmware rejects our commands: it speaks protocol"
                f" v{t.version}, our commands are v{t.last_cmd_version or protocol.PROTOCOL_VERSION}"
                " - reflash / git pull"
            )

        self._hub.update_link(
            rssi_dbm=t.rssi_dbm if t.rssi_dbm != 0 else None,
            cmd_loss_pct=t.cmd_loss_pct,
            telem_loss_pct=telem_loss_pct,
            failsafe=failsafe,
        )
        if failsafe != self._failsafe:
            self._failsafe = failsafe
            self._hub.publish(
                self._hub.mav.statustext_encode(
                    mavutil.mavlink.MAV_SEVERITY_CRITICAL if failsafe else mavutil.mavlink.MAV_SEVERITY_INFO,
                    b"FAILSAFE - command link lost" if failsafe else b"Failsafe cleared",
                )
            )

        # No ATTITUDE while the air side reports a dead IMU (the wire fields
        # are frozen at the last good sample): attitude_ts goes stale within
        # a second and the HUD hides the horizon, roll indicator and the
        # yaw-derived compass instead of showing stuck values.
        if t.flags & protocol.TELEM_FLAG_IMU_OK:
            self._hub.publish(
                self._hub.mav.attitude_encode(
                    time_boot_ms, t.roll, t.pitch, t.yaw, t.rollspeed, t.pitchspeed, t.yawspeed
                )
            )

        # VFR_HUD + ALTITUDE go out unconditionally: each float carries NaN
        # while its sensor is dead (pitot -> airspeed, baro -> alt/climb)
        # and the HUD hides just the affected readout - a dead baro must
        # not take a working airspeed down with it. No throttle sensor yet;
        # heading is the (drifting) integrated yaw. VFR_HUD.alt nominally
        # is AMSL but we only have the baro-relative altitude - the HUD
        # prefers ALTITUDE.altitude_relative anyway.
        self._hub.publish(
            self._hub.mav.vfr_hud_encode(
                t.ias_mps,
                0.0 if math.isnan(t.gs_mps) else t.gs_mps,
                int(math.degrees(t.yaw)) % 360,
                0,
                t.baro_alt_m,
                t.climb_mps,
            )
        )
        self._hub.publish(
            self._hub.mav.altitude_encode(
                time_usec,
                float("nan"),  # altitude_monotonic
                float("nan"),  # altitude_amsl - unknown (no QNH)
                t.baro_alt_m,  # altitude_local (relative to boot location)
                t.baro_alt_m,  # altitude_relative
                float("nan"),  # altitude_terrain
                float("nan"),  # bottom_clearance
            )
        )

        self._publish_gps(t, fix, time_usec, time_boot_ms)

    def _publish_gps(self, t, fix, time_usec, time_boot_ms):
        """GPS_RAW_INT always (sats visible while acquiring), GPI only with a fix.

        Scaling mirrors emulator_source.py: 1e7 deg, mm, cm/s, centidegrees;
        65535 / UINT16_MAX marks unknown eph/vel/cog per the MAVLink spec,
        0 marks unknown h_acc/v_acc (MAVLink 2 extension fields, in mm).
        """
        self._hub.publish(
            self._hub.mav.gps_raw_int_encode(
                time_usec,
                mavutil.mavlink.GPS_FIX_TYPE_3D_FIX if fix else mavutil.mavlink.GPS_FIX_TYPE_NO_FIX,
                t.lat1e7 if fix else 0,
                t.lon1e7 if fix else 0,
                0 if math.isnan(t.alt_msl_m) else int(t.alt_msl_m * 1000),
                65535 if math.isnan(t.hdop) else min(int(t.hdop * 100), 65535),
                65535,  # epv - the firmware does not report VDOP
                65535 if math.isnan(t.gs_mps) else int(t.gs_mps * 100),
                65535 if math.isnan(t.cog_deg) else int(t.cog_deg * 100) % 36000,
                t.sats,
                0,  # alt_ellipsoid - not reported
                0 if math.isnan(t.hacc_m) else int(t.hacc_m * 1000),
                0 if math.isnan(t.vacc_m) else int(t.vacc_m * 1000),
                0,  # vel_acc - not reported
                0,  # hdg_acc - not reported
            )
        )
        if not fix:
            return

        if math.isnan(t.cog_deg) or math.isnan(t.gs_mps):
            vx = vy = 0
        else:
            cog_rad = math.radians(t.cog_deg)
            vx = int(t.gs_mps * math.cos(cog_rad) * 100)
            vy = int(t.gs_mps * math.sin(cog_rad) * 100)
        # COG only counts as heading when actually moving; UINT16_MAX = unknown
        moving = not math.isnan(t.gs_mps) and t.gs_mps > 1.0 and not math.isnan(t.cog_deg)
        self._hub.publish(
            self._hub.mav.global_position_int_encode(
                time_boot_ms,
                t.lat1e7,
                t.lon1e7,
                0 if math.isnan(t.alt_msl_m) else int(t.alt_msl_m * 1000),
                0 if math.isnan(t.baro_alt_m) else int(t.baro_alt_m * 1000),
                vx,
                vy,
                0 if math.isnan(t.climb_mps) else int(-t.climb_mps * 100),
                int(t.cog_deg * 100) % 36000 if moving else 65535,
            )
        )

    def _publish_battery(self, t):
        """BATTERY_STATUS with the three per-cell voltages (mV; QGC renders
        them individually). Not published while the air side reports the
        battery sense dead/absent - the HUD's batt_ts goes stale and the BAT
        line disappears, mirroring how a dead IMU stops ATTITUDE."""
        if not t.flags & protocol.TELEM_FLAG_BATT_OK:
            return
        voltages = [t.cell1_mv, t.cell2_mv, t.cell3_mv] + [65535] * 7  # 65535 = cell unused
        self._hub.publish(
            self._hub.mav.battery_status_encode(
                0,  # id
                mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
                mavutil.mavlink.MAV_BATTERY_TYPE_LION,
                32767,  # temperature [cdegC], INT16_MAX = not measured
                voltages,
                -1,  # current_battery - not measured
                -1,  # current_consumed
                -1,  # energy_consumed
                -1,  # battery_remaining [%] - voltage is a poor fuel gauge, don't fake one
            )
        )

    def _publish_radio_status(self, t):
        """RADIO_STATUS so QGC shows link quality natively (SiK convention:
        rssi 0-254 mapped from dBm, 255 = unknown)."""
        if t.rssi_dbm != 0:
            rssi = max(0, min(254, int((t.rssi_dbm + 127) * 1.9)))
        else:
            rssi = 255
        self._hub.publish(
            self._hub.mav.radio_status_encode(
                rssi,
                rssi,  # remrssi - the air side is the only RSSI we have
                100,  # txbuf [%]
                0,  # noise - not measured
                0,  # remnoise
                min(65535, self._telem_lost_total),  # rxerrors
                0,  # fixed
            )
        )

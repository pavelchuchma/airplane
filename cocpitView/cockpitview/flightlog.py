"""Flight log - raw wire payloads plus PC-side events, one `.cvlog` per run.

cocpitView logs what crosses the ground bridge's serial link, not the
MAVLink it derives from it: every payload the bridge delivers (today the
90 B TelemetryMessage, 50 Hz) and every command payload sent back (20 B,
50 Hz). The MAVLink stream is lossy (NaN -> 0, integer scaling, messages
suppressed while a sensor is dead, the altitude rewritten by the baro
calibration) and fully derivable from these records; the reverse is not
true. Design notes: doc/plans/2026-08-25-telemetry-logging.md.

File format - a sequence of length-prefixed records:

    record = "<dBH" (t_mono [s], type, len) || payload[len]

    0x00 HEADER   JSON, first record: format, protocol version, the struct
                  formats + field names + flag map of the messages, UTC
                  start, git hash, CLI args. The file is self-describing:
                  readers decode by the header, not by the current
                  protocol.py, so old logs survive PROTOCOL_VERSION bumps.
    0x01 DOWN     raw downlink payload as delivered by the bridge
                  (TelemetryMessage and the air's diagnostic LogMessage)
    0x02 UP       raw uplink payload as sent
    0x03 EVENT    JSON {"ev": name, ...} - PC-side state that cannot be
                  recomputed from the wire: home_locked, baro_calibrated,
                  yaw_calibrated, the 1 Hz link_window liveness heartbeat,
                  serial_open/serial_lost
    0x04 MAVLINK  reserved, never written

Timestamps are time.monotonic() (records from the reader and sender threads
interleave, so they are not strictly monotonic across types). The 1 Hz
link_window event carries wall-clock UTC because macOS' monotonic clock
stops while the laptop sleeps.

Writing: producers only pack + queue.put (no I/O on the 50 Hz path); a
writer thread owns all filesystem access, opens the file lazily on the first
DOWN record (starting without the plane leaves no file) and writes the
accumulated buffer once per second. Crash resilience comes from the length
prefix: a reader consumes up to the last complete record and drops a
truncated tail. Logging can never take the app down - an OSError disables
it with one console line.

Reading: read() / header() / telemetry_frames() / commands() / events()
and to_dataframe() (pandas is an optional analysis dependency, not in
requirements.txt). CLI:

    python -m cockpitview.flightlog info|csv|events|diag|yawcal FILE.cvlog

`yawcal` replays the log through the GPS heading calibration (yawcal.py) -
that is where its two field-measured constants get their numbers.

The air board's diagnostic lines (LogMessage, the '#' lines of its USB
serial) are also written as text to `<stem>.air.log` next to the `.cvlog`,
flushed per line so it can be followed with `tail -f`. The text comes from
AirLogFormatter fed with exactly the records the `.cvlog` gets, so `diag`
regenerates the same text from any log - the `.air.log` is a derived view,
the `.cvlog` stays the source.
"""

import collections
import csv
import datetime
import json
import math
import pathlib
import queue
import signal
import struct
import subprocess
import sys
import threading
import time

from . import protocol

FORMAT = 1
RECORD = struct.Struct("<dBH")
TYPE_HEADER, TYPE_DOWN, TYPE_UP, TYPE_EVENT, TYPE_MAVLINK = range(5)
TYPE_NAMES = {TYPE_HEADER: "HEADER", TYPE_DOWN: "DOWN", TYPE_UP: "UP", TYPE_EVENT: "EVENT", TYPE_MAVLINK: "MAVLINK"}

FLUSH_PERIOD_S = 1.0
FLUSH_BYTES = 64 * 1024
# events kept while waiting for the first DOWN record; everything else that
# arrives before the file exists is dropped (an hour of neutral commands and
# link_window heartbeats with no plane must not end up behind the header)
PRE_OPEN_EVENT_PREFIX = "serial_"

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent

_TICK = object()  # writer wake-up without a record


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def pack_record(t, rtype, payload):
    return RECORD.pack(t, rtype, len(payload)) + payload


class FlightLog:
    """Queue + writer thread. `FlightLog(None)` is the null log: same API,
    every method a no-op, so call sites never test for logging."""

    def __init__(self, log_dir, args=None):
        self._dir = pathlib.Path(log_dir) if log_dir else None
        self._args = dict(args) if args else {}
        self._queue = queue.SimpleQueue()
        self._closed = False
        self.path = None  # set by the writer once the file exists
        self.air_log_path = None  # set by the writer with the first air diagnostic line
        self.bytes_written = 0
        self._thread = None
        if self._dir is not None:
            self._thread = threading.Thread(target=self._run, name="flightlog", daemon=True)
            self._thread.start()

    @property
    def enabled(self):
        return self._dir is not None

    def downlink(self, payload):
        self._put(TYPE_DOWN, payload)

    def uplink(self, payload):
        self._put(TYPE_UP, payload)

    def event(self, name, **fields):
        self._put(TYPE_EVENT, json.dumps({"ev": name, **fields}, separators=(",", ":")).encode(), name)

    def _put(self, rtype, payload, name=None):
        if self._dir is None or self._closed:
            return
        self._queue.put((time.monotonic(), rtype, bytes(payload), name))

    def close(self):
        if self._dir is None or self._closed:
            return
        self._closed = True
        self._queue.put(None)
        self._thread.join(timeout=2.0)

    # --- writer thread -----------------------------------------------------

    def _run(self):
        f = None
        air = None
        formatter = None
        buf = bytearray()
        pending_events = []
        next_flush = time.monotonic() + FLUSH_PERIOD_S
        try:
            while True:
                try:
                    item = self._queue.get(timeout=max(0.0, next_flush - time.monotonic()))
                except queue.Empty:
                    item = _TICK
                if item is None:
                    break
                if item is not _TICK:
                    t, rtype, payload, name = item
                    if f is None:
                        if rtype == TYPE_DOWN:
                            f, formatter = self._open(t, buf)
                            for pending in pending_events:
                                buf += pack_record(*pending[:3])
                        elif rtype == TYPE_EVENT and name.startswith(PRE_OPEN_EVENT_PREFIX):
                            pending_events.append(item)
                            continue
                        else:
                            continue
                    buf += pack_record(t, rtype, payload)
                    lines = formatter.feed(t, rtype, payload)
                    if lines:
                        if air is None:
                            air = self._open_air_log()
                        air.write("".join(line + "\n" for line in lines))
                        air.flush()
                now = time.monotonic()
                if f is not None and buf and (now >= next_flush or len(buf) >= FLUSH_BYTES):
                    self._write(f, buf)
                if now >= next_flush:
                    next_flush = now + FLUSH_PERIOD_S
            if f is not None:
                self._write(f, buf)
                f.close()
            if air is not None:
                air.close()
        except OSError as err:
            print(f"flight log disabled: {err}")
            self._closed = True  # producers stop queueing; the queue is garbage
            for handle in (f, air):
                if handle is not None:
                    try:
                        handle.close()
                    except OSError:
                        pass

    def _open(self, t_mono0, buf):
        self._dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        path = self._dir / f"{stamp}.cvlog"
        n = 1
        while path.exists():
            n += 1
            path = self._dir / f"{stamp}-{n}.cvlog"
        f = open(path, "wb")
        hdr = build_header(t_mono0, self._args)
        buf += pack_record(t_mono0, TYPE_HEADER, json.dumps(hdr, default=str).encode())
        self._write(f, buf)  # a crash in the first second still leaves a readable file
        self.path = path
        print(f"Logging to {path}")
        # through JSON, so the formatter sees what a reader of the file will
        return f, AirLogFormatter(json.loads(json.dumps(hdr, default=str)))

    def _open_air_log(self):
        path = air_log_path(self.path)
        air = open(path, "w", encoding="utf-8")
        self.air_log_path = path
        print(f"Air diagnostics to {path}")
        return air

    def _write(self, f, buf):
        f.write(buf)
        f.flush()
        self.bytes_written += len(buf)
        buf.clear()


def build_header(t_mono0, args):
    flags = {name[len("TELEM_FLAG_") :]: value for name, value in vars(protocol).items() if name.startswith("TELEM_FLAG_")}
    return {
        "format": FORMAT,
        "protocol_version": protocol.PROTOCOL_VERSION,
        "telemetry_struct": protocol._TELEMETRY.format,
        "telemetry_fields": list(protocol.Telemetry._fields),
        "telemetry_flags": flags,
        "command_struct": protocol._COMMAND.format,
        "command_fields": list(protocol.Command._fields),
        "log_struct": protocol._LOG_HEADER.format,
        "log_fields": [name for name in protocol.LogLine._fields if name != "text"],
        "started_utc": utc_now(),
        "t_mono0": t_mono0,
        **git_state(),
        "args": args,
    }


def git_state():
    """{"git": short hash or None, "git_dirty": bool} - identifies the era of
    the numbers together with protocol_version (semantic changes such as a
    sign fix leave the struct unchanged)."""
    try:
        run = lambda *cmd: subprocess.run(  # noqa: E731
            ["git", *cmd], cwd=REPO_ROOT, capture_output=True, text=True, timeout=5, check=True
        ).stdout.strip()
        return {"git": run("rev-parse", "--short", "HEAD"), "git_dirty": bool(run("status", "--porcelain", "--untracked-files=no"))}
    except (OSError, subprocess.SubprocessError):
        return {"git": None, "git_dirty": None}


# --- reading -----------------------------------------------------------------


def read(path):
    """Yields (t_mono, type, payload) for every complete record; a truncated
    tail (crash mid-write) is silently dropped."""
    data = pathlib.Path(path).read_bytes()
    view = memoryview(data)
    pos = 0
    while pos + RECORD.size <= len(data):
        t, rtype, length = RECORD.unpack_from(data, pos)
        pos += RECORD.size
        if pos + length > len(data):
            return
        yield t, rtype, bytes(view[pos : pos + length])
        pos += length


def header(path):
    """The header dict; raises ValueError for a non-cvlog or a newer format."""
    for t, rtype, payload in read(path):
        if rtype != TYPE_HEADER:
            raise ValueError(f"{path}: first record is not a header")
        hdr = json.loads(payload)
        if hdr.get("format", 0) > FORMAT:
            raise ValueError(f"{path}: log format {hdr['format']} is newer than this reader ({FORMAT})")
        return hdr
    raise ValueError(f"{path}: empty log")


# Up to v4 the air side sent its internal FLU yaw straight onto the wire, so
# the heading ran counter-clockwise - mirrored against every consumer of it
# (HUD, QGC, yawcal.py). v5 flips it at the source; here we flip the old logs
# so that one analysis path serves both. Only yaw is affected: roll and pitch
# carry the usual aerospace signs in FLU too.
_MIRRORED_YAW_MAX_VERSION = 4
_MIRRORED_YAW_FIELDS = ("yaw", "yawspeed")


def upgrade(frame, version):
    """Maps a frame decoded by a log's own layout onto the current Telemetry
    (or Command) namedtuple: missing fields become NaN, and this is the one
    place for version-dependent semantic fixes (re-scaled fields, sign
    flips...)."""
    target = protocol.Telemetry if frame.msg_id == protocol.MSG_TELEMETRY else protocol.Command
    if type(frame)._fields == target._fields:
        out = target._make(frame)
    else:
        out = target._make(getattr(frame, name, math.nan) for name in target._fields)
    if out.msg_id == protocol.MSG_TELEMETRY and version <= _MIRRORED_YAW_MAX_VERSION:
        out = out._replace(**{name: -getattr(out, name) for name in _MIRRORED_YAW_FIELDS})
    return out


class MessageDecoder:
    """Decodes one message type by the layout stored in a log header. Frames
    of another protocol version (or a size that doesn't match) are counted
    in `skipped` by version and returned as None - a mixed-version file
    comes from a bench session with a stale air firmware."""

    def __init__(self, hdr, msg_id, struct_key, fields_key, name):
        self.msg_id = msg_id
        self.version = hdr["protocol_version"]
        self._struct = struct.Struct(hdr[struct_key])
        self._tuple = collections.namedtuple(name, hdr[fields_key])
        self.skipped = collections.Counter()

    @classmethod
    def telemetry(cls, hdr):
        return cls(hdr, protocol.MSG_TELEMETRY, "telemetry_struct", "telemetry_fields", "LoggedTelemetry")

    @classmethod
    def command(cls, hdr):
        return cls(hdr, protocol.MSG_COMMAND, "command_struct", "command_fields", "LoggedCommand")

    def decode(self, payload):
        if len(payload) < 2 or payload[0] != self.msg_id:
            return None  # another message type, not ours to count
        if payload[1] != self.version or len(payload) != self._struct.size:
            self.skipped[payload[1]] += 1
            return None
        return upgrade(self._tuple._make(self._struct.unpack(payload)), payload[1])


def _decoded(path, rtype, make_decoder):
    decoder = make_decoder(header(path))
    for t, rec_type, payload in read(path):
        if rec_type == rtype:
            msg = decoder.decode(payload)
            if msg is not None:
                yield t, msg


def telemetry_frames(path):
    """Yields (t_mono, protocol.Telemetry) for every decodable downlink frame."""
    return _decoded(path, TYPE_DOWN, MessageDecoder.telemetry)


def commands(path):
    """Yields (t_mono, protocol.Command) for every logged command."""
    return _decoded(path, TYPE_UP, MessageDecoder.command)


def events(path):
    """Yields (t_mono, dict) for every EVENT record."""
    for t, rtype, payload in read(path):
        if rtype == TYPE_EVENT:
            yield t, json.loads(payload)


def air_log_path(cvlog_path):
    """`2026-10-20_153000.cvlog` -> `2026-10-20_153000.air.log`."""
    return pathlib.Path(cvlog_path).with_suffix(".air.log")


def _parse_utc(text):
    return datetime.datetime.fromisoformat(text.replace("Z", "+00:00"))


def format_log_time(seconds):
    """Position in the log as HH:MM:SS.s - the HUD's replay clock, accepted
    by --replay-start."""
    hours, rest = divmod(max(0.0, seconds), 3600)
    minutes, secs = divmod(rest, 60)
    return f"{int(hours):02d}:{int(minutes):02d}:{secs:04.1f}"


class AirLogFormatter:
    """Turns the records of a log, in file order, into `.air.log` text lines.

    Fed every record (feed() returns the lines it produced, usually none):
    DOWN LogMessages become lines, link_window events re-anchor the UTC
    column (monotonic time stops while macOS sleeps; before the first
    heartbeat the header's started_utc is the anchor). seq restarting with a
    smaller time_boot_ms marks an air reboot, a seq gap is reported as lost
    lines - split by the air's `dropped` counter into buffer overflow and
    radio loss - and a repeated seq is a duplicate and skipped."""

    def __init__(self, hdr):
        self._t0 = hdr["t_mono0"]
        self._anchor = (self._t0, _parse_utc(hdr["started_utc"]))
        self._version = hdr["protocol_version"]
        self._struct = struct.Struct(hdr["log_struct"]) if "log_struct" in hdr else None
        self._fields = hdr.get("log_fields")
        self._last = None  # previous LogLine of the current boot
        self.lines = 0  # air lines formatted (not counting the marker lines)

    def feed(self, t, rtype, payload):
        if rtype == TYPE_EVENT:
            ev = json.loads(payload)
            if ev.get("ev") == "link_window" and "utc" in ev:
                self._anchor = (t, _parse_utc(ev["utc"]))
            return []
        if rtype != TYPE_DOWN or self._struct is None or len(payload) < 2:
            return []
        if payload[0] != protocol.MSG_LOG or payload[1] != self._version:
            return []
        line = protocol.decode_log(payload, self._struct, self._fields)
        if line is None:
            return []

        prefix = self._prefix(t)
        out = []
        last = self._last
        if last is not None and line.seq == last.seq and line.time_boot_ms == last.time_boot_ms:
            return []  # duplicate delivery
        if last is None or line.time_boot_ms < last.time_boot_ms:
            if line.seq == 0:
                out.append(f"{prefix}--- air boot ---")
            else:
                out.append(f"{prefix}--- air boot already running, joined at #{line.seq} ---")
            missing, overflow = line.seq, line.dropped
            if last is None:
                missing = overflow = 0  # earlier lines may have gone to another session
        else:
            missing = (line.seq - last.seq - 1) & 0xFFFF
            overflow = (line.dropped - last.dropped) & 0xFFFF
        if missing:
            overflow = min(overflow, missing)
            out.append(f"{prefix}[lost {missing} lines ({overflow} buffer overflow on air, {missing - overflow} radio)]")
        out.append(f"{prefix}boot {line.time_boot_ms / 1000.0:9.3f}  #{line.seq:<5d} {line.text}")
        self._last = line
        self.lines += 1
        return out

    def _prefix(self, t):
        anchor_t, anchor_utc = self._anchor
        utc = anchor_utc + datetime.timedelta(seconds=t - anchor_t)
        stamp = utc.isoformat(timespec="milliseconds").replace("+00:00", "Z")
        return f"{stamp}  {format_log_time(t - self._t0)}  "


def air_log_lines(path):
    """Yields the `.air.log` lines of a log - what the writer produced live."""
    formatter = AirLogFormatter(header(path))
    for t, rtype, payload in read(path):
        yield from formatter.feed(t, rtype, payload)


def flag_columns(hdr, flags):
    """{'gps_fix': bool, ...} from a flags byte and the header's flag map."""
    return {name.lower(): bool(flags & bit) for name, bit in hdr["telemetry_flags"].items()}


def to_dataframe(path, kind="telemetry"):
    """pandas DataFrame of the telemetry (flags expanded to boolean columns)
    or the commands (`kind="commands"`), with a `t` column, sorted by it.
    Join the two with pandas.merge_asof."""
    import pandas  # optional analysis dependency

    hdr = header(path)
    if kind == "telemetry":
        rows = [(t, *frame, *flag_columns(hdr, frame.flags).values()) for t, frame in telemetry_frames(path)]
        columns = ["t", *protocol.Telemetry._fields, *(name.lower() for name in hdr["telemetry_flags"])]
    elif kind == "commands":
        rows = [(t, *cmd) for t, cmd in commands(path)]
        columns = ["t", *protocol.Command._fields]
    else:
        raise ValueError(f"unknown kind {kind!r}")
    return pandas.DataFrame(rows, columns=columns).sort_values("t", ignore_index=True)


# --- CLI ---------------------------------------------------------------------

GAP_REPORT_S = 0.5  # downlink silence worth listing in `info`
TELEMETRY_RATE_HZ = 50.0  # nominal, for the lost-frame estimate in `info`


def _info(path):
    hdr = header(path)
    print(f"{path}")
    print(f"  started {hdr['started_utc']}  protocol v{hdr['protocol_version']}  format {hdr['format']}")
    dirty = " (dirty)" if hdr.get("git_dirty") else ""
    print(f"  cocpitView git {hdr.get('git')}{dirty}")

    counts = collections.Counter()
    first = last = None
    telemetry = MessageDecoder.telemetry(hdr)
    cmds = MessageDecoder.command(hdr)
    frames = 0
    gaps = []
    reboots = 0
    prev_t = prev_boot_ms = None
    event_names = collections.Counter()
    air_log = AirLogFormatter(hdr)
    heartbeats = []  # t of link_window events, to tell "link down" from "PC dead"

    for t, rtype, payload in read(path):
        counts[rtype] += 1
        air_log.feed(t, rtype, payload)
        first = t if first is None else first
        last = t
        if rtype == TYPE_DOWN:
            frame = telemetry.decode(payload)
            if frame is None:
                continue
            frames += 1
            if prev_t is not None:
                if t - prev_t > GAP_REPORT_S:
                    gaps.append((prev_t, t))
                if frame.time_boot_ms < prev_boot_ms:
                    reboots += 1
            prev_t, prev_boot_ms = t, frame.time_boot_ms
        elif rtype == TYPE_UP:
            cmds.decode(payload)
        elif rtype == TYPE_EVENT:
            ev = json.loads(payload)
            event_names[ev["ev"]] += 1
            if ev["ev"] == "link_window":
                heartbeats.append(t)

    duration = (last - first) if first is not None else 0.0
    if prev_t is not None and last - prev_t > GAP_REPORT_S:
        gaps.append((prev_t, last))  # the plane went silent before the app was closed
    print(f"  duration {duration:.1f} s, {sum(counts.values())} records:", ", ".join(f"{TYPE_NAMES.get(k, k)} {v}" for k, v in sorted(counts.items())))
    print(f"  telemetry frames {frames}" + (f", air reboots {reboots}" if reboots else ""))
    if air_log.lines:
        print(f"  air diagnostic lines {air_log.lines} (`diag` prints them)")
    for version, n in sorted(telemetry.skipped.items()):
        print(f"  {n} telemetry frames v{version} skipped (header is v{hdr['protocol_version']})")
    for version, n in sorted(cmds.skipped.items()):
        print(f"  {n} commands v{version} skipped")
    if event_names:
        print("  events:", ", ".join(f"{k} {v}" for k, v in sorted(event_names.items())))
    for start, end in gaps:
        alive = any(start < hb < end for hb in heartbeats)
        lost = int(round((end - start) * TELEMETRY_RATE_HZ)) - 1
        until = "end of log" if end == last else f"{end - first:8.1f} s"
        print(f"  telemetry gap {start - first:8.1f} s -> {until} ({end - start:.1f} s, ~{lost} frames, PC {'alive' if alive else 'silent'})")


def _csv(path):
    hdr = header(path)
    flag_names = [name.lower() for name in hdr["telemetry_flags"]]
    out = csv.writer(sys.stdout)
    out.writerow(["t", *protocol.Telemetry._fields, *flag_names])
    for t, frame in telemetry_frames(path):
        # %.7g: the wire floats are float32, their float64 repr is noise
        values = (f"{v:.7g}" if isinstance(v, float) else v for v in frame)
        out.writerow([f"{t:.4f}", *values, *(int(v) for v in flag_columns(hdr, frame.flags).values())])


def _events(path):
    hdr = header(path)
    t0 = hdr["t_mono0"]
    for t, ev in events(path):
        rest = {k: v for k, v in ev.items() if k != "ev"}
        print(f"{t - t0:9.3f}  {ev['ev']:<16} {json.dumps(rest)}")


def _diag(path):
    for line in air_log_lines(path):
        print(line)


def _yawcal(path):
    """Replay the log through the GPS yaw calibration (yawcal.py) and print
    every window it solved.

    This is where the two constants yawcal.py cannot derive from first
    principles get measured. GPS_VELOCITY_LAG_S: re-run with --lag values
    around the default and take the one minimising `rms` (a wrong lag shows
    up as residual concentrated in the turns). DRIFT_PSD: watch how `offset`
    marches across a long flight - its slope is the gyro drift rate.

    `k` is worth a look on its own: it is the IAS scale, i.e. the pitot
    position error, measured in flight instead of on reciprocal legs.
    """
    from . import yawcal  # numpy is only needed for this subcommand

    hdr = header(path)
    print(f"{path}")
    print(f"  started {hdr['started_utc']}  protocol v{hdr['protocol_version']}")
    print(f"  lag {yawcal.GPS_VELOCITY_LAG_S:.3f} s  window {yawcal.WINDOW_S:.0f} s  "
          f"max cond {yawcal.MAX_CONDITION:.0f}")
    print()
    header_row = f"{'t':>8}  {'n':>4} {'cond':>6} {'conc':>5} {'rms':>5} {'delta':>7} {'k':>5} " \
                 f"{'wind':>11}  {'offset':>7} {'sigma':>6}  reject"
    print(header_row)
    print("-" * len(header_row))

    frames = telemetry_frames(path)
    t0 = hdr["t_mono0"]  # print log-relative times, like `info` and `events`
    rows = accepted = 0
    for fit, offset_deg, sigma_deg in yawcal.replay(frames, lambda f: flag_columns(hdr, f.flags)):
        rows += 1
        accepted += fit.reject is None
        wind_from = (math.degrees(math.atan2(fit.wind_e, fit.wind_n)) + 180.0) % 360.0
        wind = f"{wind_from:03.0f}/{math.hypot(fit.wind_n, fit.wind_e):4.1f}"
        cond = f"{fit.cond:6.1f}" if fit.cond < 1e4 else "  1e+4"
        print(
            f"{fit.t_mid - t0:8.1f}  {fit.n:4d} {cond} {fit.concentration:5.3f} {fit.residual_rms:5.2f} "
            f"{math.degrees(fit.delta):7.2f} {fit.k:5.3f} {wind:>11}  "
            f"{'   ----' if offset_deg is None else f'{offset_deg:7.2f}'} "
            f"{'  ----' if sigma_deg is None else f'{sigma_deg:6.2f}'}  {fit.reject or ''}"
        )
    if rows == 0:
        print("  (no window was ever solvable - the log has no stretch of fast,")
        print("   3D-fixed flight long enough; `info` shows what the session did contain)")
    else:
        print(f"\n  {accepted}/{rows} windows accepted")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # `... csv FILE | head` exits quietly
    commands_ = {"info": _info, "csv": _csv, "events": _events, "diag": _diag, "yawcal": _yawcal}
    if len(argv) != 2 or argv[0] not in commands_:
        print(f"usage: python -m cockpitview.flightlog {'|'.join(commands_)} FILE.cvlog", file=sys.stderr)
        return 2
    commands_[argv[0]](pathlib.Path(argv[1]))
    return 0


if __name__ == "__main__":
    sys.exit(main())

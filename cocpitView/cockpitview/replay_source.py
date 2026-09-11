"""Flight log replay - a .cvlog as a third telemetry source (--replay).

Plays the DOWN records of a log (the raw wire payloads the ground bridge
delivered) through publisher.TelemetryPublisher, i.e. through exactly the
code the live link uses. Everything downstream is today's: the HUD, the
home point, the baro calibration and the yaw calibration all re-run on the
old flight, so improving a calibrator improves the replay of a flight from
weeks ago. QGC gets the same MAVLink it would have got in the air.

Decoding goes through the log's own header (flightlog.MessageDecoder +
upgrade()), never through the current protocol.py: the first flight log is
protocol v4, whose yaw is mirrored, and upgrade() flips it so the replay
shows the heading a v5 firmware would have sent. Frames of a version the
header does not describe are counted and skipped.

Three clocks are kept apart:

  - wall time stays wall time for the HUD's staleness logic, so a replay
    that stops feeding greys the HUD out exactly like a lost link;
  - log time (record timestamp minus the header's t_mono0 - the same zero
    `flightlog info` prints its gap table against) paces the playback and
    goes on the wire as time_boot_ms/time_usec;
  - the calibrators run on log time via ReplayClock, or a 60 s yawcal
    window would mean 60 s of *watching* rather than of flying.

PC-side EVENT records (home_locked, baro_calibrated, yaw_calibrated) are
deliberately not injected - the replay recomputes them from the wire, which
is the whole point. They stay in the log as what that flight concluded.
"""

import threading
import time

from . import flightlog
from .publisher import TelemetryPublisher

TELEMETRY_RATE_HZ = 50.0  # nominal air-side rate, for the loss estimate
STATS_WINDOW_S = 1.0
DEFAULT_MAX_GAP_S = 3.0


def parse_time(spec):
    """'12', '3:15' or '1:02:03' -> seconds. Relative to the start of the
    log, i.e. the times `flightlog info` prints."""
    parts = str(spec).split(":")
    if len(parts) > 3:
        raise ValueError(f"cannot read '{spec}' as a time (use SS, MM:SS or HH:MM:SS)")
    seconds = 0.0
    for part in parts:
        seconds = seconds * 60 + float(part)
    return seconds


class ReplayClock:
    """The replay's position in log time [s], as a callable. Handed to the
    processors that measure time themselves (yawcal.YawCalibrator,
    home.HomePoint) so they see the flight's own clock - during the
    fast-forward warm-up too, where wall time barely advances."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class ReplaySource:
    """Background thread replaying a .cvlog into the hub.

    Everything before `start_s` is fast-forwarded (published as fast as it
    decodes, with the UDP forward suppressed) so the processors reach the
    state they had at that point in the flight; `cold=True` drops it
    instead, which reproduces "cocpitView was started mid-flight" - the one
    case where home and the altimeter stay off for good.
    """

    def __init__(self, hub, path, clock=None, speed=1.0, start_s=0.0, end_s=None,
                 cold=False, max_gap_s=DEFAULT_MAX_GAP_S):
        self._hub = hub
        self._path = path
        self._clock = clock if clock is not None else ReplayClock()
        self._speed = speed
        self._start_s = start_s
        self._end_s = end_s
        self._cold = cold
        self._max_gap_s = max_gap_s
        self._publisher = TelemetryPublisher(hub, warn=False)  # an old log is no reason to reflash
        self._thread = threading.Thread(target=self._run, name="replay", daemon=True)

    @property
    def clock(self):
        return self._clock

    def start(self):
        hdr = flightlog.header(self._path)  # raises before the HUD comes up if the file is not a log
        print(
            f"Replaying {self._path} (recorded {hdr['started_utc']}, protocol"
            f" v{hdr['protocol_version']}, cocpitView git {hdr.get('git')}) at {self._speed:g}x"
        )
        if self._start_s:
            print(f"  starting at {self._start_s:.1f} s"
                  + (" (cold - no warm-up)" if self._cold else " (fast-forwarding the run-up to it)"))
        self._thread.start()

    def _run(self):
        hdr = flightlog.header(self._path)
        decoder = flightlog.MessageDecoder.telemetry(hdr)
        t0 = hdr["t_mono0"]

        wall_ref = None  # wall time of the first real-time frame
        log_ref = 0.0  # its log time
        prev_t = None
        window_start = None
        window_count = 0
        telem_loss_pct = 0
        played = warmed = 0

        for t_rec, rtype, payload in flightlog.read(self._path):
            if rtype != flightlog.TYPE_DOWN:
                continue
            frame = decoder.decode(payload)
            if frame is None:
                continue
            t = t_rec - t0
            if self._end_s is not None and t > self._end_s:
                break
            self._clock.t = t

            if t < self._start_s:
                if not self._cold:
                    warmed += 1
                    self._publisher.publish(frame, t, t, 0, forward=False)
                continue

            if wall_ref is None:
                wall_ref, log_ref, window_start = time.monotonic(), t, t
            elif t > prev_t:
                # a long silence is faithful but unwatchable: this log holds
                # single gaps of 200 s. Only the *wait* is capped - log time
                # jumps by the full amount, so yawcal still sees the real dt
                # between the samples either side and rejects what it should.
                excess = (t - prev_t) / self._speed - self._max_gap_s
                if excess > 0:
                    wall_ref -= excess
                self._sleep_until(wall_ref + (t - log_ref) / self._speed)

            # same 1 Hz loss window the live source runs, in log time: the
            # HUD's LNK D% then reads what the link really lost that second
            window_count += 1
            if t - window_start >= STATS_WINDOW_S:
                expected = (t - window_start) * TELEMETRY_RATE_HZ
                lost = max(0, int(round(expected)) - window_count)
                telem_loss_pct = min(100, int(round(lost * 100 / expected)))
                self._publisher.telem_lost_total += lost
                window_start = t
                window_count = 0

            self._publisher.publish(frame, t, t, telem_loss_pct)
            played += 1
            prev_t = t

        self._report(played, warmed, decoder, hdr)

    def _sleep_until(self, target):
        delay = target - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    def _report(self, played, warmed, decoder, hdr):
        skipped = ", ".join(f"{n} frames v{v}" for v, n in sorted(decoder.skipped.items()))
        print(
            f"Replay finished: {played} frames played"
            + (f", {warmed} fast-forwarded" if warmed else "")
            + (f", skipped {skipped} (header is v{hdr['protocol_version']})" if skipped else "")
        )
        if played == 0:
            print("  (nothing to play - `flightlog info` shows what the file does contain)")
        # nothing resets the HUD: with no more frames its own staleness logic
        # takes over and it falls back to NO TELEMETRY, as on a lost link

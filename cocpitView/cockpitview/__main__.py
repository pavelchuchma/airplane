#!/usr/bin/env python3
"""Cockpit View - FPV video with an F-16 style HUD overlay.

Video comes directly from the USB camera (5.8G FPV receiver / grabber).
Telemetry arrives over the ESP-NOW link via the ground bridge ESP32 on USB
serial (default; commands go back out the same way at 50 Hz), is generated
by the built-in flight-profile emulator (--emulator) or is replayed from a
past flight's .cvlog (--replay, replay_source.py), and is forwarded to
QGroundControl as MAVLink over UDP (default 127.0.0.1:14550) - cocpitView is
the telemetry hub, QGC an optional downstream consumer. The raw wire traffic
is always logged to logs/<timestamp>.cvlog (see flightlog.py; --no-log to
disable; neither the emulator nor a replay is logged).

There is no magnetometer on board, so the heading arrives on an arbitrary
boot datum and is calibrated against the GPS in flight (yawcal.py).

Usage:
    ./run.sh
    ./.venv/bin/python -m cockpitview --camera 1 --fullscreen --fov 45
    ./.venv/bin/python -m cockpitview --emulator --no-camera
    ./run.sh --replay logs/2026-08-29_123339.cvlog --replay-start 3:15

Keys: Cmd+Q quits.
"""

import argparse
import os
import sys
import time

import pygame

from .altimeter import BaroCalibrator
from .camera import Camera
from .emulator_source import EmulatorSource
from .espnow_source import EspNowSource
from .flightlog import FlightLog
from .home import HomePoint
from .hud import Hud
from .replay_source import DEFAULT_MAX_GAP_S, ReplayClock, ReplaySource, parse_time
from .telemetry import TelemetryHub
from .terrain import Terrain
from .yawcal import YawCalibrator


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--emulator", action="store_true", help="replay the synthetic flight profile instead of reading the ground bridge")
    parser.add_argument("--replay", metavar="FILE.cvlog", default=None, help="replay a recorded flight instead of reading the ground bridge (never writes a log of its own)")
    parser.add_argument("--replay-speed", type=float, default=1.0, help="replay speed multiplier (default: 1.0)")
    parser.add_argument("--replay-start", metavar="T", default=None, help="skip to T (SS, MM:SS or HH:MM:SS) of the log - the times `flightlog info` prints")
    parser.add_argument("--replay-end", metavar="T", default=None, help="stop the replay at T")
    parser.add_argument("--replay-cold", action="store_true", help="drop the part before --replay-start instead of fast-forwarding it through the calibrators (reproduces a mid-flight start)")
    parser.add_argument("--replay-max-gap", type=float, default=DEFAULT_MAX_GAP_S, metavar="S", help=f"cap the wait across a telemetry gap at S seconds (default: {DEFAULT_MAX_GAP_S:g})")
    parser.add_argument("--serial-port", default=None, help="serial port of the ground bridge ESP32 (default: resolved by factory MAC from esp32/platformio.ini)")
    parser.add_argument("--baud", type=int, default=460800, help="serial baud rate (default: 460800)")
    parser.add_argument("--rate", type=float, default=20.0, help="emulator telemetry rate in Hz (default: 20)")
    parser.add_argument("--qgc-host", default="127.0.0.1", help="QGroundControl IP address for MAVLink forwarding (default: 127.0.0.1)")
    parser.add_argument("--qgc-port", type=int, default=14550, help="QGroundControl UDP port (default: 14550)")
    parser.add_argument("--system-id", type=int, default=1, help="MAVLink system id of the vehicle (default: 1)")
    parser.add_argument("--camera-name", default="AV TO USB2.0", help="camera device name to lock onto (default: AV TO USB2.0)")
    parser.add_argument("--camera", type=int, default=None, help="force a cv2 camera index instead of the by-name lookup")
    parser.add_argument("--width", type=int, default=960, help="window width (default: 960)")
    parser.add_argument("--height", type=int, default=640, help="window height (default: 640)")
    parser.add_argument("--fullscreen", action="store_true", help="start fullscreen")
    parser.add_argument("--fov", type=float, default=40.0, help="camera vertical FOV in degrees, sets pitch ladder scale (default: 40)")
    parser.add_argument("--no-camera", action="store_true", help="skip camera capture (HUD over test pattern)")
    parser.add_argument(
        "--terrain",
        default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data", "terrain.npz"),
        help="terrain grid from cockpitview/terrain_fetch.py (default: data/terrain.npz)",
    )
    parser.add_argument(
        "--log-dir",
        default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs"),
        help="directory for the raw telemetry/command logs (default: logs/ next to the package)",
    )
    parser.add_argument("--no-log", action="store_true", help="do not write a .cvlog flight log")
    parser.add_argument("--dump-frame", metavar="PATH", help="debug: render for 2 s, save a PNG screenshot to PATH and exit")
    args = parser.parse_args()

    if args.emulator and args.replay:
        parser.error("--emulator and --replay are two different sources - pick one")
    try:
        replay_start = parse_time(args.replay_start) if args.replay_start else 0.0
        replay_end = parse_time(args.replay_end) if args.replay_end else None
    except ValueError as err:
        parser.error(str(err))
    # a replay has no camera feed of its own, and looking for the grabber
    # would only print a "camera not found" line
    if args.replay and args.camera is None:
        args.no_camera = True

    if args.dump_frame:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    pygame.init()
    # SCALED keeps the drawing surface at the logical (width, height) resolution
    # regardless of the actual window size; SDL scales it to the window on the GPU.
    flags = (pygame.FULLSCREEN if args.fullscreen else pygame.RESIZABLE) | pygame.SCALED
    screen = pygame.display.set_mode((args.width, args.height), flags, vsync=1)
    pygame.display.set_caption("Cockpit View")

    terrain = Terrain.load(args.terrain)
    if terrain is None:
        print(f"No terrain grid at '{args.terrain}' - AMSL/AGL readouts disabled (run python -m cockpitview.terrain_fetch)")

    # the emulator has no wire frames to log (it publishes MAVLink directly),
    # and a replay would only copy the log it is reading
    log = FlightLog(None if args.no_log or args.emulator or args.replay else args.log_dir, args=vars(args))
    if log.enabled:
        print(f"Flight log directory {args.log_dir} (the file opens with the first telemetry frame)")

    # a replay runs the processors on the flight's own clock: yawcal's 60 s
    # window and home's baro freshness are flight time, not viewing time
    replay_clock = ReplayClock() if args.replay else None
    clock = replay_clock if replay_clock is not None else time.monotonic

    # the launch point drives the altimeter calibration too (same gate, same
    # samples - see home.py), so it exists even without a terrain grid
    home = HomePoint(log=log, clock=clock)
    calibrator = BaroCalibrator(terrain, home, log=log) if terrain is not None else None
    # last in the chain: it rewrites the heading in ATTITUDE/VFR_HUD, which
    # neither of the two above reads, so it observes their input unchanged
    yaw_cal = YawCalibrator(log=log, clock=clock)
    telemetry = TelemetryHub(
        qgc_host=args.qgc_host,
        qgc_port=args.qgc_port,
        system_id=args.system_id,
        processors=[p for p in (home, calibrator, yaw_cal) if p is not None],
    )
    if args.emulator:
        source = EmulatorSource(telemetry, rate=args.rate)
    elif args.replay:
        source = ReplaySource(
            telemetry,
            args.replay,
            clock=replay_clock,
            speed=args.replay_speed,
            start_s=replay_start,
            end_s=replay_end,
            cold=args.replay_cold,
            max_gap_s=args.replay_max_gap,
        )
    else:
        source = EspNowSource(telemetry, port=args.serial_port, baud=args.baud, log=log)
    source.start()
    print(f"Forwarding MAVLink to udp:{args.qgc_host}:{args.qgc_port}")

    camera = None
    if not args.no_camera and not args.dump_frame:
        camera = Camera(name=args.camera_name, index=args.camera)
        camera.start()
        if args.camera is not None:
            print(f"Opening camera index {args.camera}")
        else:
            print(f"Looking for camera '{args.camera_name}'")

    hud = Hud(screen.get_size(), fov_deg=args.fov)
    clock = pygame.time.Clock()
    start = time.monotonic()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q and event.mod & pygame.KMOD_META:
                        return

            size = screen.get_size()
            if size != (hud.w, hud.h):  # window resized or fullscreen toggled
                hud = Hud(size, fov_deg=args.fov)

            if camera is not None:
                screen.blit(camera.get_surface(size), (0, 0))
            else:
                screen.fill((28, 30, 32))

            dt = clock.tick(60) / 1000.0
            snap = telemetry.snapshot()
            # with a terrain grid the box shows calibrated AMSL ("---" until
            # the calibration locks); without one only the relative altitude
            # is available and AGL stays off
            snap.display_alt_m = snap.amsl_m if terrain is not None else snap.altitude_m
            if terrain is not None and snap.position is not None and snap.amsl_m is not None:
                snap.agl_m = terrain.agl_m(*snap.position, snap.amsl_m)
            snap.home_dist_m, snap.home_vert_m = home.offsets_from(snap)
            snap.yaw_cal = yaw_cal.state()
            if replay_clock is not None:
                snap.replay_time_s = replay_clock()
            hud.draw(screen, snap, dt)
            pygame.display.flip()

            if args.dump_frame and time.monotonic() - start > 2.0:
                pygame.image.save(screen, args.dump_frame)
                print(f"Frame saved to {args.dump_frame}")
                return
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.stop()
        log.close()
        pygame.quit()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Cockpit View - FPV video with an F-16 style HUD overlay.

Video comes directly from the USB camera (5.8G FPV receiver / grabber).
Telemetry arrives over the ESP-NOW link via the ground bridge ESP32 on USB
serial (default; commands go back out the same way at 50 Hz) or is generated
by the built-in flight-profile emulator (--emulator), and is forwarded to
QGroundControl as MAVLink over UDP (default 127.0.0.1:14550) - cocpitView is
the telemetry hub, QGC an optional downstream consumer. The raw wire traffic
is always logged to logs/<timestamp>.cvlog (see flightlog.py; --no-log to
disable; the emulator is never logged).

Usage:
    ./run.sh
    ./.venv/bin/python -m cockpitview --camera 1 --fullscreen --fov 45
    ./.venv/bin/python -m cockpitview --emulator --no-camera

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
from .telemetry import TelemetryHub
from .terrain import Terrain


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--emulator", action="store_true", help="replay the synthetic flight profile instead of reading the ground bridge")
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

    # the emulator has no wire frames to log (it publishes MAVLink directly)
    log = FlightLog(None if args.no_log or args.emulator else args.log_dir, args=vars(args))
    if log.enabled:
        print(f"Flight log directory {args.log_dir} (the file opens with the first telemetry frame)")

    # the launch point drives the altimeter calibration too (same gate, same
    # samples - see home.py), so it exists even without a terrain grid
    home = HomePoint(log=log)
    calibrator = BaroCalibrator(terrain, home, log=log) if terrain is not None else None
    telemetry = TelemetryHub(
        qgc_host=args.qgc_host,
        qgc_port=args.qgc_port,
        system_id=args.system_id,
        processors=[p for p in (home, calibrator) if p is not None],
    )
    if args.emulator:
        source = EmulatorSource(telemetry, rate=args.rate)
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

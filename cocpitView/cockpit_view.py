#!/usr/bin/env python3
"""Cockpit View - FPV video with an F-16 style HUD overlay.

Video comes directly from the USB camera (5.8G FPV receiver / grabber),
telemetry arrives as MAVLink forwarded by QGroundControl (Application
Settings -> MAVLink -> MAVLink Forwarding -> localhost:14445). For testing
without QGC, point ../emulator/mpu6050_emulator.py --port 14445 at it
directly.

Usage:
    ./.venv/bin/python cockpit_view.py
    ./.venv/bin/python cockpit_view.py --camera 1 --fullscreen --fov 45

Keys: ESC / Q quit, F toggles fullscreen.
"""

import argparse
import os
import sys
import time

import pygame

from camera import Camera
from hud import Hud
from telemetry import TelemetryReceiver


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mavlink-port", type=int, default=14445, help="UDP port QGC forwards MAVLink to (default: 14445)")
    parser.add_argument("--camera-name", default="AV TO USB2.0", help="camera device name to lock onto (default: AV TO USB2.0)")
    parser.add_argument("--camera", type=int, default=None, help="force a cv2 camera index instead of the by-name lookup")
    parser.add_argument("--width", type=int, default=960, help="window width (default: 960)")
    parser.add_argument("--height", type=int, default=640, help="window height (default: 640)")
    parser.add_argument("--fullscreen", action="store_true", help="start fullscreen")
    parser.add_argument("--fov", type=float, default=40.0, help="camera vertical FOV in degrees, sets pitch ladder scale (default: 40)")
    parser.add_argument("--no-camera", action="store_true", help="skip camera capture (HUD over test pattern)")
    parser.add_argument("--dump-frame", metavar="PATH", help="debug: render for 2 s, save a PNG screenshot to PATH and exit")
    args = parser.parse_args()

    if args.dump_frame:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    pygame.init()
    flags = pygame.FULLSCREEN if args.fullscreen else pygame.RESIZABLE
    screen = pygame.display.set_mode((args.width, args.height), flags)
    pygame.display.set_caption("Cockpit View")

    telemetry = TelemetryReceiver(port=args.mavlink_port)
    telemetry.start()
    print(f"Listening for MAVLink on udp:0.0.0.0:{args.mavlink_port}")

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
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        return
                    if event.key == pygame.K_f:
                        pygame.display.toggle_fullscreen()

            size = screen.get_size()
            if size != (hud.w, hud.h):  # window resized or fullscreen toggled
                hud = Hud(size, fov_deg=args.fov)

            if camera is not None:
                screen.blit(camera.get_surface(size), (0, 0))
            else:
                screen.fill((28, 30, 32))

            dt = clock.tick(60) / 1000.0
            hud.draw(screen, telemetry.snapshot(), dt)
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
        pygame.quit()


if __name__ == "__main__":
    main()

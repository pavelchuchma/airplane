# PlatformIO extra_script: pick the serial port by the board's factory MAC.
#
# Each environment sets `custom_board_mac` in platformio.ini (read once with
# `esptool.py --port <port> read_mac`). `custom_usb_location` is only a hint
# so the probe tries the most likely port first — probing resets the board,
# and the hint usually keeps the other board untouched. See tools/board_id.py.
#
# Only runs for upload/monitor targets; a plain build never touches the boards.

import os
import sys

Import("env")

sys.path.insert(0, os.path.join(env["PROJECT_DIR"], "tools"))
import board_id


def resolve_port():
    mac = env.GetProjectOption("custom_board_mac", "").strip()
    if not mac:
        return
    hint = env.GetProjectOption("custom_usb_location", "").strip() or None
    esptool_cmd = [
        env.subst("$PYTHONEXE"),
        os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py"),
    ]
    try:
        port = board_id.find_board(mac, hint, esptool_cmd)
    except (LookupError, RuntimeError) as err:
        print("select_port: WARNING: %s" % err)
        return
    env.Replace(UPLOAD_PORT=port, MONITOR_PORT=port)
    print("select_port: board %s -> %s" % (mac, port))


if {"upload", "monitor", "uploadfs"} & set(COMMAND_LINE_TARGETS):
    resolve_port()

"""Identify ESP32 devkits by their factory MAC address.

Both devkits use clone CP2102 USB bridges with an identical,
non-reprogrammable USB serial number, so port names and USB descriptors
cannot tell the boards apart. The ESP32's factory MAC (burned into eFuse,
unique per chip) can — it is read through the ROM bootloader with esptool,
which works regardless of what firmware is flashed.

Probing a port resets the board on it (bootloader entry + hard reset),
so callers pass a `location_hint` (physical USB location) to try the most
likely port first and usually leave the other board untouched.
"""

import importlib.util
import re
import shutil
import subprocess
import sys

from serial.tools import list_ports

CP210X_VID = 0x10C4
CP210X_PID = 0xEA60


def default_esptool_cmd():
    if importlib.util.find_spec("esptool"):
        return [sys.executable, "-m", "esptool"]
    exe = shutil.which("esptool.py") or shutil.which("esptool")
    if exe:
        return [exe]
    raise RuntimeError("esptool not found (neither importable nor in PATH)")


def read_mac(port, esptool_cmd=None):
    """Read the factory MAC via the ROM bootloader; (mac, None) or (None, reason)."""
    cmd = list(esptool_cmd or default_esptool_cmd()) + ["--port", port, "read_mac"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return None, "esptool timed out"
    match = re.search(r"^MAC:\s*([0-9a-fA-F:]{17})", result.stdout, re.MULTILINE)
    if match:
        return match.group(1).lower(), None
    lines = (result.stderr or result.stdout).strip().splitlines()
    return None, lines[-1] if lines else f"esptool exit code {result.returncode}"


def candidate_ports(location_hint=None):
    """CP210x serial ports, the one matching location_hint first."""
    ports = [p for p in list_ports.comports() if p.vid == CP210X_VID and p.pid == CP210X_PID]
    if location_hint:
        ports.sort(key=lambda p: 0 if p.location and p.location.startswith(location_hint) else 1)
    return ports


def find_board(mac, location_hint=None, esptool_cmd=None):
    """Return the serial port device of the board with the given factory MAC.

    Raises LookupError when no probed port matches, listing ports that
    could not be probed (typically busy — held by a monitor or bridge).
    """
    mac = mac.strip().lower()
    probed, unreadable = [], []
    for p in candidate_ports(location_hint):
        got, reason = read_mac(p.device, esptool_cmd)
        if got == mac:
            return p.device
        if got:
            probed.append(p.device)
        else:
            unreadable.append(f"{p.device} ({reason})")
    raise LookupError(
        f"no board with MAC {mac} found; other boards: {', '.join(probed) or 'none'}; "
        f"unreadable (port busy?): {', '.join(unreadable) or 'none'}"
    )

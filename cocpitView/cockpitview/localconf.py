"""Reader for esp32/local.ini - the git-ignored file with the ESP-NOW keys
and the flying site, shared with the firmware (see esp32/local.ini.example).
cocpitView only needs the [site] section; the keys stay firmware-only."""

import configparser
import pathlib

ESP32_DIR = pathlib.Path(__file__).resolve().parent.parent.parent / "esp32"
LOCAL_INI = ESP32_DIR / "local.ini"


def site_center():
    """(lat, lon) of the flying site from local.ini [site], or None when the
    file or the section is missing."""
    config = configparser.ConfigParser(inline_comment_prefixes=(";",))
    if not config.read(LOCAL_INI):
        return None
    try:
        return config.getfloat("site", "lat"), config.getfloat("site", "lon")
    except (configparser.Error, ValueError):
        return None

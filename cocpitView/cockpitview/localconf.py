"""Reader for esp32/local.ini - the git-ignored file with the ESP-NOW keys,
the flying site and the VTX channel, shared with the firmware (see
esp32/local.ini.example). cocpitView needs the [site] and [vtx] sections;
the keys stay firmware-only."""

import configparser
import pathlib

ESP32_DIR = pathlib.Path(__file__).resolve().parent.parent.parent / "esp32"
LOCAL_INI = ESP32_DIR / "local.ini"


def _read(path=LOCAL_INI):
    config = configparser.ConfigParser(inline_comment_prefixes=(";",))
    config.read(path)
    return config


def site_center():
    """(lat, lon) of the flying site from local.ini [site], or None when the
    file or the section is missing."""
    config = _read()
    try:
        return config.getfloat("site", "lat"), config.getfloat("site", "lon")
    except (configparser.Error, ValueError):
        return None


def vtx_frequency(path=LOCAL_INI):
    """The [vtx] freq setting as written ('5800' or 'R4'), or None when
    absent - the VTX is then left on whatever its button configured."""
    config = _read(path)
    value = config.get("vtx", "freq", fallback="").strip()
    return value or None

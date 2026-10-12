# PlatformIO extra_script: inject the ESP-NOW encryption keys from local.ini
# (merged by `extra_configs`) as -DLINK_PMK_HEX / -DLINK_LMK_HEX build flags.
#
# Hex strings rather than {0x..,...} initializer lists: SCons mangles commas
# in -D values, so the firmware parses the 32 digits once in
# espnowlink::begin(). A missing or malformed key aborts the build here with
# a pointer to local.ini.example instead of a cryptic compiler error.

import re
import sys

Import("env")

HEX16 = re.compile(r"^[0-9a-fA-F]{32}$")


def fail(msg):
    sys.stderr.write(
        "link_keys: ERROR: %s\n"
        "link_keys: copy local.ini.example to local.ini and generate the keys "
        "with `openssl rand -hex 16`\n" % msg
    )
    Exit(1)


def inject_keys():
    for option, macro in (("custom_link_pmk", "LINK_PMK_HEX"), ("custom_link_lmk", "LINK_LMK_HEX")):
        value = env.GetProjectOption(option, "").strip()
        if not value:
            fail("%s not set (local.ini missing?)" % option)
        if not HEX16.match(value):
            fail("%s must be exactly 32 hex digits" % option)
        env.Append(CPPDEFINES=[(macro, env.StringifyMacro(value))])


inject_keys()

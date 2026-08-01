# PlatformIO extra_script: compile the *other* environment's board MAC into
# the firmware as the ESP-NOW peer address, so platformio.ini stays the
# single source of truth for both boards' MACs.
#
# Injected as -DPEER_MAC_STR="68:09:47:..." (a quoted string parsed once in
# setup() via espnowlink::parseMac) — SCons mangles commas in -D values, so
# a {0x68,0x09,...} initializer list cannot be passed through CPPDEFINES.

Import("env")

PEER_ENV = {"air": "env:ground", "ground": "env:air"}


def inject_peer_mac():
    peer_section = PEER_ENV.get(env["PIOENV"])
    if peer_section is None:
        return
    mac = env.GetProjectConfig().get(peer_section, "custom_board_mac", "").strip()
    if not mac:
        print("peer_mac: WARNING: custom_board_mac not set in [%s]" % peer_section)
        return
    # StringifyMacro adds the (escaped) quotes itself - passing a pre-quoted
    # string would bake literal '"' characters into the C string.
    env.Append(CPPDEFINES=[("PEER_MAC_STR", env.StringifyMacro(mac))])


inject_peer_mac()

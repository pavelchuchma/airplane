"""Operator side of the VTX control (doc/plans/2026-10-25-vtx-tramp-control.md).

Power is stepped from the keyboard through the ULTRA 1600's table - pit,
25, 200, 400, 800, 1600 mW - never cycled, so the hot end is always several
deliberate presses away from the legal one. Frequency comes from
configuration only (esp32/local.ini [vtx] or --vtx-freq): changing it means
retuning the receiver, not a flight-time action. Only values from the
tables below are ever sent; what the VTX does with others is undocumented.
"""

from . import protocol

# Power steps of the ULTRA 1600 above pit mode, from the manual. Only the
# 25 mW step is EU-legal (doc/parts/fpv-58g-uvc-kit.md section 7).
POWER_STEPS_MW = (25, 200, 400, 800, 1600)

# 5.8 GHz channel tables, MHz, channel 1..8. The bands the receiver covers
# (5645-5945 MHz); the ULTRA's D, Low Race and X bands lie outside it.
BANDS = {
    "A": (5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725),
    "B": (5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866),
    "E": (5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945),
    "F": (5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880),
    "R": (5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917),
}
FREQUENCIES_MHZ = frozenset(f for band in BANDS.values() for f in band)


def parse_frequency(text):
    """'5800' or band+channel like 'R4' / 'f3' -> MHz; ValueError for
    anything not in the receiver's channel tables."""
    text = str(text).strip().upper()
    if len(text) == 2 and text[0] in BANDS and text[1] in "12345678":
        return BANDS[text[0]][int(text[1]) - 1]
    try:
        mhz = int(text)
    except ValueError:
        raise ValueError(f"VTX frequency '{text}' is neither MHz nor band+channel (e.g. R4)") from None
    if mhz not in FREQUENCIES_MHZ:
        raise ValueError(f"VTX frequency {mhz} MHz is not a channel of bands A/B/E/F/R")
    return mhz


def channel_name(mhz):
    """'F4' for 5800, or '' when the frequency is in no table."""
    for band, table in BANDS.items():
        if mhz in table:
            return f"{band}{table.index(mhz) + 1}"
    return ""


def _nearest_step_at_most(power_mw):
    below = [s for s in POWER_STEPS_MW if s <= power_mw]
    return below[-1] if below else POWER_STEPS_MW[0]


def step_power(power_mw, pit, direction):
    """The next desired power after one keypress: `power_mw`/`pit` is where
    the stepping starts (the last request, else the readback), `direction`
    -1 = down, +1 = up. Returns mW or protocol.CMD_VTX_POWER_PIT; pit is
    the bottom step, above 1600 mW stays 1600 mW. A readback that is not a
    listed step snaps to the listed one on the side of the move."""
    if pit:
        return protocol.CMD_VTX_POWER_PIT if direction < 0 else POWER_STEPS_MW[0]
    if direction < 0:
        below = [s for s in POWER_STEPS_MW if s < power_mw]
        return below[-1] if below else protocol.CMD_VTX_POWER_PIT
    above = [s for s in POWER_STEPS_MW if s > power_mw]
    return above[0] if above else POWER_STEPS_MW[-1]


def toggle_pit(power_mw, pit):
    """Pit on -> back to the (listed) configured power; pit off -> pit."""
    if pit:
        return _nearest_step_at_most(power_mw)
    return protocol.CMD_VTX_POWER_PIT


def describe_power(power_mw):
    return "pit" if power_mw == protocol.CMD_VTX_POWER_PIT else f"{power_mw} mW"


class VtxControl:
    """Turns keypresses into CommandState requests, stepping from the last
    request while one is outstanding, else from what the VTX reports."""

    def __init__(self, commands):
        self._commands = commands

    def _base(self, vtx):
        """(power_mw, pit) to step from, or None without a readback."""
        if vtx is None or not vtx.alive or vtx.power_mw == 0:
            return None
        _, requested, _ = self._commands.vtx_request()
        if requested == protocol.CMD_VTX_POWER_PIT:
            return vtx.power_mw, True
        if requested != protocol.CMD_VTX_POWER_LEAVE:
            return requested, False
        return vtx.power_mw, vtx.pit

    def step(self, vtx, direction):
        base = self._base(vtx)
        if base is None:
            print("VTX: no readback to step from")
            return None
        power = step_power(*base, direction)
        self._commands.set_vtx_power(power)
        print(f"VTX request: {describe_power(power)}")
        return power

    def toggle_pit(self, vtx):
        base = self._base(vtx)
        if base is None:
            print("VTX: no readback to step from")
            return None
        power = toggle_pit(*base)
        self._commands.set_vtx_power(power)
        print(f"VTX request: {describe_power(power)}")
        return power

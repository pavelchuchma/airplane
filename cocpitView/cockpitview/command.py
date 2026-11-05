"""The desired state the 50 Hz CommandMessage carries to the air side.

One thread-safe object between the operator's inputs (keys now, the
joystick later) and espnow_source's sender thread, which encodes it into
every frame. The air side reconciles what it reads here against reality
(vtx.cpp): the fields are *desired* values, not events, so a lost frame or a
link gap changes nothing - the next frame says the same thing.
"""

import threading
import time

from . import protocol


class CommandState:
    def __init__(self, vtx_freq_mhz=protocol.CMD_VTX_FREQ_LEAVE, clock=time.monotonic):
        self._lock = threading.Lock()
        self._clock = clock
        self._channels = protocol.NEUTRAL_CHANNELS
        self._vtx_freq_mhz = vtx_freq_mhz
        self._vtx_power_mw = protocol.CMD_VTX_POWER_LEAVE
        self._vtx_changed_at = -float("inf")

    def set_vtx_power(self, power_mw):
        """Desired VTX power in mW; protocol.CMD_VTX_POWER_PIT for pit mode."""
        with self._lock:
            if power_mw != self._vtx_power_mw:
                self._vtx_power_mw = power_mw
                self._vtx_changed_at = self._clock()

    def vtx_request(self):
        """(freq_mhz, power_mw, age_s): the desired VTX state (the protocol's
        "leave" sentinels when nothing was asked) and how long ago the power
        request last changed - the HUD blinks only after a pending request
        has had its time."""
        with self._lock:
            return self._vtx_freq_mhz, self._vtx_power_mw, self._clock() - self._vtx_changed_at

    def encode(self, seq):
        with self._lock:
            return protocol.encode_command(
                seq, channels=self._channels, vtx_freq_mhz=self._vtx_freq_mhz, vtx_power_mw=self._vtx_power_mw
            )

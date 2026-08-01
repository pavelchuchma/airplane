"""Cockpit View - FPV video with an F-16 style HUD overlay."""

import os

# Must be set before the first `from pymavlink import mavutil` anywhere:
# MAVLink 2 is needed for the GPS_RAW_INT extension fields (h_acc/v_acc).
os.environ.setdefault("MAVLINK20", "1")

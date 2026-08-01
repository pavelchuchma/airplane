# PlatformIO extra_script: pick the serial port by physical USB location.
#
# Both devkits use clone CP2102 bridges with an identical, non-reprogrammable
# serial number ("0001"), so when both are plugged in the port names are
# ambiguous. The USB location (hub port chain) is the only stable identifier,
# which means each board must stay in its assigned physical USB port.
#
# Each environment sets `custom_usb_location` in platformio.ini. Find the
# current value with `pio device list` (the LOCATION= field) after plugging
# the board in alone.

Import("env")

from serial.tools import list_ports


def resolve_port():
    wanted = env.GetProjectOption("custom_usb_location", "").strip()
    if not wanted:
        return
    matches = [p.device for p in list_ports.comports()
               if p.location and p.location.startswith(wanted)]
    if len(matches) == 1:
        env.Replace(UPLOAD_PORT=matches[0], MONITOR_PORT=matches[0])
        print("select_port: USB location %s -> %s" % (wanted, matches[0]))
    elif not matches:
        print("select_port: WARNING: no serial device at USB location %s "
              "(is the board plugged into its assigned port?)" % wanted)
    else:
        print("select_port: WARNING: multiple devices match location %s: %s"
              % (wanted, ", ".join(matches)))


resolve_port()

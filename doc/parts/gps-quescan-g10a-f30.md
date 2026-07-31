# Quescan G10A-F30 — u-blox M10 GPS/GNSS Module

![Quescan G10A-F30 GPS module](img/gps-quescan-g10a-f30.jpg)

- **Ordered:** 1×, AliExpress (Jul 19, 2026)
- **Product:** [Quescan G10A-F30 M10 GPS GNSS Module, UBX-M10050-KB](https://www.aliexpress.com/item/1005004131946090.html) — Quescan Store
- **What it is:** GNSS receiver on the u-blox M10 platform (UBX-M10050-KB, ROM SPG 5.10), ceramic patch antenna, 28×28×8 mm, 13 g

## Receiver specs (u-blox M10, from MAX-M10S datasheet R08)

| Parameter | Value |
|---|---|
| Constellations | concurrent 4-GNSS: GPS L1 C/A + Galileo E1 + GLONASS L1OF + BeiDou B1I/B1C, plus QZSS/SBAS |
| Position accuracy | **1.5 m CEP** |
| Update rate (default mode) | 10 Hz (GPS+GAL) … 4 Hz (4-GNSS); single GNSS 18 Hz |
| Update rate ("high performance" mode) | up to **25 Hz single GNSS / 10 Hz 4-GNSS** (needs config, SPG 5.10+) |
| Sensitivity | tracking −167 dBm; cold start −148 dBm |
| TTFF | cold 23–28 s; **hot 1 s**; aided ~2 s |
| Velocity / heading accuracy | 0.05 m/s / 0.3° |
| Operational limits | ≤4 g, 80 000 m, 500 m/s |
| Timepulse | default 1PPS, 30 ns RMS (configurable 0.25 Hz–10 MHz) |
| Protocols | **UBX binary + NMEA 0183 (default 4.10)** |
| Firmware | ROM-based (flash-less chip); config lives in RAM + BBR (+ optional flash — vendor claims onboard, verify) |

## Board pinout (6-pin, 1.25 mm PicoBlade-style; silkscreen "P V T R G E")

| Pin | Name | Function |
|---|---|---|
| 1 (P) | PPS | timepulse output, 1 Hz after fix (drives the PPS LED) |
| 2 (V) | VCC | **DC 3.3–5.0 V** (onboard LDO) |
| 3 (T) | TX | UART out of module, **3.3 V TTL** |
| 4 (R) | RX | UART into module, 3.3 V TTL |
| 5 (G) | GND | ground |
| 6 (E) | EN | enable — leave floating/high; low = module off |

- **Board default baud: 38400** (Quescan preset; bare u-blox chip default is 9600 — try both if silent)
- **No compass** on this model (no I2C pins) — for ArduPilot/iNav multirotor modes add a separate magnetometer (or Quescan's GPS+compass variant / Matek M10Q-5883)
- 2 LEDs: Power + PPS (blinks at 1 Hz once a fix is obtained)

## Connection — flight controller

```
GPS TX → FC UART RX,  GPS RX → FC UART TX,  VCC → 5V (or 3.3V),  GND → GND
PPS, EN — leave unconnected
```

- **ArduPilot:** `SERIALx_PROTOCOL = 5`, `GPS_TYPE = 2` (u-blox), `GPS_AUTO_CONFIG = 1` — auto-detects baud and configures via UBX
- **PX4:** GPS 1 port, stock u-blox driver — plug and play
- **Betaflight/iNav:** UART → GPS, protocol **UBLOX** (BF 4.4+ autobauds)

## Connection — ESP32

```cpp
// GPS TX → GPIO16 (RX2), GPS RX → GPIO17 (TX2), VCC → 5V or 3V3
Serial2.begin(38400, SERIAL_8N1, 16, 17);
```

- Libraries: **TinyGPSPlus** (parses the default NMEA immediately) or **SparkFun u-blox GNSS v3** (full UBX, M10-aware — needed for changing nav rate/dynamic model)
- 10 Hz: UBX-CFG-VALSET key `CFG-RATE-MEAS` (0x30210001) = 100 ms; raise baud first via `CFG-UART1-BAUDRATE` (0x40520001) if enabling more messages
- For airplane use set dynamic model: `CFG-NAVSPG-DYNMODEL` = 8 (airborne ≤4g)

## Configuration (u-center 2)

- M10 requires **u-center 2** — classic u-center supports only up to M9; connect at 38400
- Config is exclusively VALSET/VALGET/VALDEL with 32-bit keys and **layers RAM / BBR / Flash** (legacy CFG-MSG/CFG-RATE/CFG-PRT messages were removed on M10)
- **Always write to RAM + BBR (+ Flash)** — RAM-only config is lost at power-off; BBR survives only while backup power is present

## Gotchas

1. **3.3 V UART logic** — VCC up to 5 V goes through the LDO, but TX/RX are 3.3 V TTL
2. Default baud **38400**, not 9600 — module looks "dead" at the wrong rate
3. **Flash-less chip** — verify config persistence by power-cycling after saving; otherwise reconfigure at every boot from the MCU
4. 10 Hz with all 4 constellations needs "high performance mode"; default mode gives ~4–5 Hz
5. **EN pin**: don't wire to an MCU pin that boots low — module shuts down
6. Classic u-center won't talk to it — use u-center 2

## Sources

- [u-blox MAX-M10S datasheet UBX-20035208 R08](https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf)
- [u-blox M10 25 Hz info note UBX-23006557](https://content.u-blox.com/sites/default/files/documents/u-bloxM10-with-25Hz-Navigation-UpdateRate_IN_UBX-23006557.pdf)
- [u-blox M10 SPG 5.10 interface description UBX-21035062](https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf)
- [RCDrone product mirror (specs + pin image)](https://rcdrone.top/products/quescan-ubx-m10050-m10-gnss-module)
- [SparkFun u-blox GNSS Arduino Library v3](https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3)

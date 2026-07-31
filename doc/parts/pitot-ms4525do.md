# MS4525DO — Differential Airspeed Sensor + Pitot Tube

![MS4525DO airspeed sensor kit](img/pitot-ms4525do.jpg)

- **Ordered:** 1× variant "For Pixhawk 6C", AliExpress (Jul 19, 2026)
- **Product:** [Differential Airspeed Pitot Tube MS4525DO + Airspeed Sensor](https://www.aliexpress.com/item/1005009558214567.html) — HotRC Toy UAV Store
- **Kit contents:** sensor breakout PCB, aluminum pitot tube (~150–160 mm), silicone tubing, 4-pin **JST-GH 1.25 mm** cable (the "Pixhawk 6C" variant; older "2.4.8" variant uses DF13)

## Sensor specs (TE Connectivity MS4525DO-DS5AI001DP)

| Parameter | Value |
|---|---|
| Pressure range | ±1 psi differential (≈ ±6.89 kPa) → airspeed up to ~100 m/s |
| Resolution | 14-bit (≈ 0.84 Pa/count); zero ΔP ≈ count 8192 (0x2000) |
| Accuracy | ±0.25% span @ 25 °C; total error band ±1.0% span (−10…+85 °C) |
| Temperature sensor | 11-bit, −50…+150 °C (for true-airspeed correction) |
| Supply | 5.0 V nominal (chip abs. max 2.7–5.5 V), ~3.5 mA |
| Interface | **I2C, address 0x28 (fixed)**, SCL 100–400 kHz max |
| Data timing | update 0.5 ms; first data 8.4 ms after power-on |
| Ports | 1/8" barbed, for 3/32" (≈2.4 mm) ID tubing |

## Connector pinout (JST-GH 4-pin, Pixhawk standard, 1:1)

| Pin | Signal | Level |
|---|---|---|
| 1 | VCC | +5 V |
| 2 | SCL | 3.3 V logic |
| 3 | SDA | 3.3 V logic |
| 4 | GND | — |

On clone breakout headers the order may differ — always verify silkscreen.

## Tubing

- Pitot **straight/rear tube = total (dynamic) pressure → top port (Port 1)**
- Pitot **angled/side tube = static pressure → bottom port (Port 2)**
- Reversed tubes → negative/stuck readings: swap tubes, not code

## Connection — Pixhawk 6C

- Plug into the dedicated **I2C port** (I2C2; GPS2 port carries the same bus as alternative). No extra pull-ups/power needed.
- **PX4:** `SENS_EN_MS4525DO = 1`, reboot, run airspeed calibration in QGC (shield tube, blow gently when prompted). [PX4 docs](https://docs.px4.io/main/en/sensor/airspeed.html)
- **ArduPilot:** `ARSPD_TYPE = 1`, `ARSPD_BUS = 1` (try 0 if not detected), `ARSPD_USE = 1`; cover (don't seal) pitot during boot for zero-offset; tune `ARSPD_RATIO` via `ARSPD_AUTOCAL = 1`. [ArduPilot docs](https://ardupilot.org/plane/docs/airspeed.html)

## Connection — ESP32 / bare MCU

```
VCC → 5V, GND → GND, SDA → GPIO21, SCL → GPIO22 (Wire.begin(), 100–400 kHz)
```

- ESP32 3.3 V logic works directly (same as Pixhawk); if the clone board lacks pull-ups, add 2.2–4.7 kΩ **to 3.3 V** (never to 5 V — ESP32 pins are not 5V-tolerant)
- Read 4 bytes from 0x28:

```cpp
status   = b0 >> 6;                     // 0 = fresh data
pressure = ((b0 & 0x3F) << 8) | b1;     // 14-bit counts
temp     = ((b2 << 8) | b3) >> 5;       // 11-bit counts; T = counts*200/2047 - 50
// output A: 10–90 % of 16383 over ±1 psi
// dP_Pa = (counts - 8192) / (0.8 * 16383) * 2 * 6894.76
// airspeed = sqrt(2 * dP / 1.225)      // indicated, at sea-level density
```

- Zero-offset: average counts at zero airflow at startup and subtract

## Gotchas

1. **Zero-offset calibration is mandatory** — offset drifts with temperature; offset error reads as "too fast" and auto-throttle then flies dangerously slow
2. **Light-sensitive die** — shield the sensor board from direct light inside the fuselage
3. **I2C address 0x28 is fixed** — two MS4525 can't share one bus; watch for conflicts
4. **Pitot placement**: clean air, outside prop wash, ~1 chord ahead of leading edge; kinked/leaky tubing = reads low
5. **Clone quality varies** — after delivery: I2C scan for 0x28 and sanity-check ~8192 counts at rest
6. Max 400 kHz I2C — don't share a 1 MHz bus with it

## Sources

- [TE MS4525DO datasheet (PDF)](https://www.cdiweb.com/datasheets/te/ms4525do.pdf)
- [ArduPilot — airspeed sensor](https://ardupilot.org/plane/docs/airspeed.html) / [calibration](https://ardupilot.org/plane/docs/calibrating-an-airspeed-sensor.html)
- [PX4 — airspeed sensors](https://docs.px4.io/main/en/sensor/airspeed.html)
- [Holybro MS4525DO setup](https://docs.holybro.com/peripherals/ms4525do-and-ms5525dso-i2c-airspeed-sensor)
- [Pixhawk 6C ports](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports)

# MS4525DO — Differential Airspeed Sensor + Pitot Tube

![MS4525DO airspeed sensor kit](img/pitot-ms4525do.jpg)

- **Ordered:** 1× variant "For Pixhawk 6C" (AliExpress, Jul 19, 2026)
- **Product:** [Differential Airspeed Pitot Tube MS4525DO + Airspeed Sensor](https://www.aliexpress.com/item/1005009558214567.html) — HotRC Toy UAV Store
- **Kit contents:** sensor breakout PCB, aluminum pitot tube (~150–160 mm), silicone tubing, 4-pin **JST-GH 1.25 mm** cable (the "Pixhawk 6C" variant; older "2.4.8" variant uses DF13)

## Sensor specs (TE Connectivity MS4525DO-DS5AI001DP)

| Parameter | Value |
|---|---|
| Pressure range | ±1 psi differential (≈ ±6.89 kPa) → airspeed up to ~100 m/s |
| Resolution | 14-bit, **≈ 1.05 Pa/count** (2 psi spread over 0.8 · 16383 counts — output type A only uses 10–90 % of the range, not all 16383); zero ΔP ≈ count 8192 (0x2000) |
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

## Tubing — the port mapping on our unit is the opposite of the datasheet

**Verified the hard way (Aug 2026): the port assignment this document used to
give is inverted on our kit.** With the pitot's straight/rear (total) tube on the
port that looked like Port 1, airflow drove the counts *below* the 8192 midpoint
— i.e. the sensor reported negative dP for positive airspeed. Whether the clone's
silkscreen is mislabelled or the top/bottom identification was wrong we cannot
tell, and it does not matter. Working wiring:

- Pitot **straight/rear tube = total (dynamic) pressure → the port this doc used to call Port 2** (the "bottom" one)
- Pitot **angled/side tube = static pressure → the port this doc used to call Port 1**

**Identify the total port empirically, never from the label.** Five seconds:
disconnect both hoses from the pitot and blow gently into one of them. IAS rises
→ that is the total port. IAS stays at a rock-solid 0 → that is the static port,
use the other one.

Fix a reversed pair by swapping the hoses, **not** by negating in the driver —
otherwise the sign convention in the code stops matching the datasheet and the
next person to touch it (or the next sensor) is misled. Beware that `iasMps`
clamps negative dP to zero, so a reversed pair does not look wrong, it looks
*dead*: a perfectly steady 0 at any airflow.

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

## Verified on this unit (Aug 2026)

**The scale factor is correct — the die really is ±1 psi.** Checked against a
known pressure with a water column: a long hose from the total port, its open end
pushed 10 cm below the surface of a bucket of water. Water climbs ~1 cm into the
hose and stops (the trapped air only compresses by ~1 %), so the applied pressure
is ρgh ≈ **981 Pa** measured from the outer surface down to the meniscus inside
the hose. Expected IAS = √(2 · 981 / 1.225) = **40.0 m/s = 144 km/h**; measured
~140 km/h, inside the ±0.5 cm ruler error (±5 % of pressure = ±2.5 % of speed).
A ±2 psi die — the obvious suspect on a clone, and worth exactly √2 in speed —
would have read 102 km/h. Keep the sensor well above the water; water reaching
the die kills it.

**Leak-tight.** Pressure held for 10 s with no drift on one port and suction
likewise on the other. Pressurising the pitot's *tip hole* holds too, which rules
out an internal cross-leak between the total and static chambers inside the pitot
body — a real defect on cheap tubes, where air bleeds from the total chamber out
through the static holes and roughly halves dP.

**Do check the static holes are open** (not yet done): blow through the static
hose with the sensor disconnected, air must come out of the side holes freely.
Blocked holes are invisible on the ground — a sealed static chamber holds its boot
pressure and dP ≈ q comes out about right — but dangerous in flight: ambient
static falls as you climb while the sealed chamber does not, so
dP = q + (p_ambient − p_sealed) reads low in a climb and high in a descent.

## Mounting and position error

A correct sensor does not make the airspeed correct. Measured on a bicycle (pitot
on the handlebars aimed dead ahead, static holes ~11 cm ahead of the bars, GPS
with 12 satellites as reference): **IAS read ~30 % below GPS ground speed**,
repeatably over 5 runs at 0–40 km/h, and 0 at a standstill. With the scale factor
proven correct above and the plumbing proven tight, all of that is position error.

Two facts worth having before picking a spot:

- **The total hole is robust, the static holes are not.** Total pressure is
  conserved along a streamline, so the tip reads freestream total even inside a
  decelerated region. Position error lives almost entirely in the **static
  holes** — it is *their* position that matters, not the tip's and not the tube's
  length.
- **Ahead of a bluff body** (fuselage nose, cyclist) the static holes sit in a
  pressure rise, so the reading is low. Sphere model u/U = 1 − (a/r)³, distance
  measured from the nose, D = diameter: 0.5 D → ~12 %, 1 D → ~4 %, 1.5 D → ~1.6 %,
  2 D → ~0.8 %. A pessimistic bound; a pointed nose is better.
- **Ahead of a wing** the static rise is small (a thin section's leading-edge
  radius is only a few mm), and the dominant effect is flow **angularity**.

### A wing mount at c/4 ahead of the leading edge is the sweet spot

A probe bolted to the airframe sees the *difference* between angle of attack and
local upwash, and both grow with C_L:

    α − ε = α₀ + (C_L / 2π) · (1 − c / 2x)     x = distance ahead of the quarter chord

The bracket vanishes at **x = c/2**, i.e. **c/4 ahead of the leading edge**: there
the upwash grows exactly as fast as α does, and the probe's angle to the local
flow stops depending on C_L. Change in flow angle for ΔC_L = 0.5 (cruise → slow
flight) at c = 200 mm:

| static holes | Δ flow angle |
|---|---|
| **c/4 = 50 mm ahead of LE** | **0.0°** |
| 100 mm ahead of LE | 1.5° |
| 200 mm (1 chord) ahead of LE | 2.7° |
| nose boom / far ahead | 4.6° |

Counter-intuitively **further forward is worse** for angularity: any
airframe-fixed probe tracks Δα directly, and only the near-field upwash cancels
it. This is first-order thin-airfoil theory with the wing as a point vortex at the
quarter chord — a good place to start, then verify in flight.

Our kit tube has its static holes 50 mm ahead of the body/T-fitting, so clamping
the body at the leading edge of a 200 mm chord puts them exactly at the sweet
spot. **Do not add a boom**, it would make it worse.

### Placement on our airframe (pusher glider, prop on a pylon above mid-fuselage)

- **40–50 % semi-span.** Not at the root (fuselage interference, prop pylon), not
  at the tip (tip-vortex swirl, and the tip twists under load, which changes the
  tube's incidence in flight).
- **In the plane of the wing or slightly above it, never below.** The fuselage
  nose is further forward and takes a nose-over; nothing protrudes below, so a
  belly landing misses it too.
- **Clamp the T-fitting at the leading edge**, tube aligned with the chord line.
  The residual constant offset is α₀ (zero-lift angle, typically −2…−4°) and
  calibration absorbs it.
- **Not on the nose**, despite that being the cleanest air on a pusher: it is what
  hits the ground first on a botched landing; a pusher prop ~35 cm behind it sucks
  air forward, reading ~1.4 % high in cruise and up to ~4 % at full throttle — a
  *throttle-dependent* error, the nasty kind; and per the table above it carries
  full Δα sensitivity. At 40 % semi-span the prop's influence is ~0.3 %.

### Calibrating out what is left

Only a flight can produce the number, and it belongs to *this* airframe at *this*
position. **A factor measured on a bicycle is invalid on the plane**, and wrong in
the dangerous direction — it would make an airspeed-driven autothrottle fly slower
than it believes.

Fly two straight legs on **exactly reciprocal headings** at constant indicated
airspeed. TAS = mean of the two GPS ground speeds (wind cancels out), expected
IAS = TAS · √(ρ/1.225). The ratio against the actual indication is the position
error. Repeat at a fast and a slow speed: the same factor at both confirms the C_L
cancellation above and that a single constant suffices; a factor that moves with
speed means the tube's incidence needs a couple of degrees of adjustment. Fly it
once with the motor off and once on at the same speed too — the difference is the
prop's influence.

When that constant reaches the firmware, keep it **separate from
`kPressureRangePsi`**: the range is a property of the die (verified, correct), the
ratio is a property of the installation. Same split as ArduPilot's `ARSPD_RATIO`.

## Gotchas

1. **Zero-offset calibration is mandatory** — offset drifts with temperature; offset error reads as "too fast" and auto-throttle then flies dangerously slow. A zero captured *too high* (wind into the tube at boot) hides behind the negative-dP clamp: the HUD still shows a clean 0 while stationary, then stays at 0 until q exceeds the offset. Tell it apart from position error by comparing two speeds — position error is a constant *percentage*, a bad zero is a constant *pressure* (so its percentage explodes at low speed)
2. **Light-sensitive die** — shield the sensor board from direct light inside the fuselage
3. **I2C address 0x28 is fixed** — two MS4525 can't share one bus; watch for conflicts
4. **Pitot placement** — see [Mounting and position error](#mounting-and-position-error). Ignore the folklore "~1 chord ahead of the leading edge": it is a full-scale rule of thumb aimed at a different problem, and at our scale it is actively worse than c/4
5. **Leaky tubing reads low, kinked tubing does not.** At steady state no air flows in the lines, so a restriction or a kink only adds lag — a leak is the only plumbing fault that changes the *magnitude*. Long hoses are therefore free: 50 cm costs milliseconds against the 0.3 s filter. Do mount the sensor higher than the pitot so condensation cannot collect in the lines (a water slug does cause errors)
6. **Clone quality varies** — after delivery: I2C scan for 0x28 and sanity-check ~8192 counts at rest. Counts resting near 1638 instead would mean a unidirectional 0–1 psi (gage) die with a completely different scale factor
7. Max 400 kHz I2C — don't share a 1 MHz bus with it

## Sources

- [TE MS4525DO datasheet (PDF)](https://www.cdiweb.com/datasheets/te/ms4525do.pdf)
- [ArduPilot — airspeed sensor](https://ardupilot.org/plane/docs/airspeed.html) / [calibration](https://ardupilot.org/plane/docs/calibrating-an-airspeed-sensor.html)
- [PX4 — airspeed sensors](https://docs.px4.io/main/en/sensor/airspeed.html)
- [Holybro MS4525DO setup](https://docs.holybro.com/peripherals/ms4525do-and-ms5525dso-i2c-airspeed-sensor)
- [Pixhawk 6C ports](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports)

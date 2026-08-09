# RAY G2 R-20B — 20 A Brushless ESC with 5 V Linear BEC

![RAY G2 R-20B ESC](img/esc-ray-g2-r-20b.jpg)

- **Source:** [RAY G2 R-20B střídavý regulátor 20A s BEC](https://www.rcprofi.cz/ray-g2-r-20b-stridavy-regulator-20a-s-bec) — rcprofi.cz, 319 CZK (spare); the same unit comes installed in the [Beta V2 1400 donor airframe](airframe-beta-v2-1400.md)
- **What it is:** the airplane's motor controller — and, via its BEC, the 5 V
  power source for the servos (see the BEC section below for why *only* the
  servos)

## Specs

| Parameter | Value |
|---|---|
| Continuous / burst current | **20 A / 25 A** |
| Cells | 2–3S LiPo/Li-ion; 5–9 NiXX |
| BEC | **5.0 V / 2 A — linear** (not switching) |
| Weight | 19 g |
| Dimensions | 42 × 25 × 8 mm |

## Programmable options

| Option | Values |
|---|---|
| Timing (advance) | low 3.75° / medium 15° / high 26.25° |
| Soft start | normal 300 ms / soft 1.5 s / super soft 3 s |
| Brake | on / off |
| Battery type | LiPo / NiCd–NiMH |
| Voltage cutoff | adjustable |

## Protections

- **Startup:** motor disconnect if it doesn't spool within 2 s
- **Thermal:** throttle limiting at 110 °C
- **Signal loss:** throttle reduction after 1 s of missing PWM, full motor cut
  after 2 s — a useful backstop behind the project's own failsafe: if the
  air-side ESP32 (the PWM source) dies outright, the ESC kills the motor on
  its own

## The BEC — what it really is (measured; details in `architecture.md`)

The "5 V/2 A" rating is a *linear* regulator on a 19 g board: on 3S it drops
~12 V → 5 V and burns **7.6 W per amp**, so 2 A continuous is thermal fiction.
It works in this airplane because the real servo load is far below that:

- three servos holding trim + receiver ≈ **0.3 A** (~2.3 W — survivable in
  prop wash),
- all three servos moving at once peaks at ~0.95 A for fractions of a second
  (absorbed by thermal mass),
- the tail risk is a *sustained* overload (jammed linkage ≈ 1.2 A ≈ 9 W →
  BEC cooks, thermal shutdown takes the servos with it). Same risk as in the
  stock model; accepted.

Rules that follow (reasoning in `architecture.md`, Power architecture):

- **Servos run off this BEC. The ESP32 never does** — its 300–500 mA ESP-NOW
  bursts don't fit in the headroom, and a BEC thermal cutout must not kill
  the failsafe brain. Logic has its own 5 V buck from the pack.
- **Never parallel the BEC with another 5 V source.** If a dedicated servo
  BEC is ever added, pull the ESC's red wire out of the connector first —
  signal and ground only.

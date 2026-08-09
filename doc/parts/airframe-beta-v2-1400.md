# Beta V2 1400 M1 — EPP Trainer Airframe (RTF, Pelikan)

![Beta V2 1400 M1](img/airframe-beta-v2-1400.jpg)

- **Source:** [Beta V2 1400 M1 RTF 4k 2.4GHz brushless](https://www.rcprofi.cz/na-dily-beta-v2-1400-m1-rtf-4k-2.4ghz-brushless) — rcprofi.cz "na díly" listing, 4 090 CZK (regular 4 990 CZK); design 80 % (slightly bent fuselage), mechanically fully functional, product code `BM-4ST1479-M1`
- **What it is:** the airplane itself — a 1.42 m EPP high-wing trainer, sold
  RTF with RadioLink RC gear. This project replaces that RC gear with the
  ESP32 control link; motor, ESC, servos and airframe stay.

## Airframe

| Parameter | Value |
|---|---|
| Wingspan | **142 cm** |
| Length | 94 cm |
| Flying weight | 0.65–0.72 kg |
| Wing area | 24.5 dm² |
| Construction | EPP foam (fuselage, wing, tail); two-piece wing on a carbon tube; magnetic canopy |
| Classification | S0 build (RTF), P1–P2 piloting — stable trainer, thermals, basic aerobatics |

## Drive & power

| Parameter | Value |
|---|---|
| Motor | C2623-1950 brushless outrunner (1950 = KV per RAY naming) |
| Propeller | fixed 6×4" |
| ESC | **[RAY G2 R-20B](esc-ray-g2-r-20b.md)** — 20 A, 5 V/2 A linear BEC |
| Stock battery | LiPo 3S 11.1 V 1300 mAh 20C; bay fits up to ~2200 mAh 3S (this project flies a 3S Li-ion pack — see `architecture.md`) |

## Controls (stock RTF configuration)

| Parameter | Value |
|---|---|
| Servos | 4 — 2 in the fuselage (rudder, elevator), 2 in the wing (ailerons) |
| Functions | rudder, elevator, ailerons, throttle |
| Transmitter | RadioLink T8FB BT, 2.4 GHz FHSS, 8ch, Mode 1 — **replaced by this project's ESP-NOW link** |
| Receiver | RadioLink R8EF 8ch — **replaced by the air-side ESP32** |

## Project notes

- This is the "existing 3S model" referenced in `architecture.md`: the ESC's
  linear BEC demonstrably powers three servos + a receiver in flight, which
  is the measured basis for the servo-power decision there.
- The stock RadioLink set stays as reference/fallback gear while the ESP32
  link is being proven.

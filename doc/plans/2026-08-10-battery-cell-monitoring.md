# Plan: per-cell battery voltage monitoring (ESP32 internal ADC)

Status: **implemented + harness built** (2026-08-23, protocol v4). The
divider harness is wired and verified end-to-end (HUD shows the BAT line);
per-channel **scale calibration is done** against a multimeter on a full
pack (all channels read 2-2.7 % high on nominal ratios — constants in
`esp32/src/air/battery.cpp`). Remaining: the second calibration point on a
near-empty pack to pin down the offsets (use raw telemetry `cellMv`, not the
10 mV-rounded HUD).

## Context

`doc/parts/README.md` (Battery monitoring) currently sketches a single
pack-voltage divider (47k/15k → GPIO34). Goal is to watch the **individual
cells** of the 3S Li-ion pack in telemetry: a hard floor of ~3.0 V/cell and a
cell diverging from its siblings are the actionable signals (Li-ion discharge
curve is flat, so pack voltage alone hides a dying cell almost to the end).

The measurement uses the **ESP32's internal ADC1**. The job is an alarm on a
sagging cell (±50–80 mV per-cell accuracy is plenty), not mV-level balance
trending, and ADC1 does that fine once it is kept inside its linear range and
calibrated.

## Hardware

Signal source: the pack's **balance connector** (3S JST-XH: `B−, C1, C2, C3`).
Cells are not measured directly — three *cumulative* nodes are measured
against `B−` and the firmware subtracts:

```
cell1 = N1              (0–4.25 V)
cell2 = N2 − N1         (N2 = 0–8.5 V)
cell3 = N3 − N2         (N3 = 0–12.75 V = pack)
```

Three dividers from the balance leads to ADC1 input-only pins (GPIO35/34/39 —
free; the air firmware only claims 16/17 for the GPS UART and 21/22 for I2C.
ADC1 works with WiFi active, ADC2 does not):

```
JST-XH balance (3S)
 C3 (+pack) ──[56k]──┬── GPIO39 / ADC1_CH3   + 100 nF to GND
 C2 ────────[28k]──┬─│── GPIO34 / ADC1_CH6   + 100 nF to GND
 C1 ────────[10k]─┬│─│── GPIO35 / ADC1_CH7   + 100 nF to GND
 B− ──────────────┴┴─┴── GND (common star ground)
        lower resistor 10k from each GPIO to GND
```

| Node | Pin | ADC1 ch | Max | Divider | Ratio | At pin (max) | At pin (3.7 V/cell) | Branch current |
|---|---|---|---|---|---|---|---|---|
| C1 | GPIO35 | CH7 | 4.25 V | 10k / 10k | 2.00 | 2.13 V | 1.85 V | 0.21 mA |
| C2 | GPIO34 | CH6 | 8.50 V | 28k / 10k | 3.80 | 2.24 V | 1.95 V | 0.22 mA |
| C3 | GPIO39 | CH3 | 12.75 V | 56k / 10k | 6.60 | 1.93 V | 1.68 V | 0.19 mA |

Rules and gotchas:

- **Stay under ~2.45 V at the pin** (11 dB attenuation) — above that the ADC
  saturates and bends; this is where most of its bad reputation comes from.
  This is why the README's existing 47k/15k suggestion (→ 3.05 V) must be
  **replaced by 56k/10k**, not reused.
- The C3 node peaks at only 1.93 V, so it uses ~21 % less of the linear band
  than the other two. That costs nothing that matters: one ADC LSB is 5.3 mV
  at the C3 node (1.6 mV at C1, 3.1 mV at C2), an order of magnitude below
  the ±50–80 mV per-cell target.
- All three are **input-only pins with no internal pull-up/pull-down**, which
  is exactly what we want here — the divider sets the level and the 100 nF
  holds it. Watch the silkscreen when wiring: on most devkits GPIO39 is
  labelled `VN` (`SENSOR_VN`), not `39`.
- ESP32 errata 3.11 (GPIO36/39 get ~80 ns internal pull-down glitches when
  WiFi power save toggles the RTC domain) does not apply here: the link
  already runs `WIFI_PS_NONE` (`espnow_link.cpp`), and the 100 nF would eat
  the glitch anyway.
- **The 100 nF at each pin is mandatory** — it feeds the sample-and-hold cap
  (a high-impedance divider alone skews the reading) and filters ESP-NOW
  burst spikes.
- Divider ground = `B−` = the pack negative = the existing star ground. The
  balance black wire and the main minus are the same node inside the pack;
  never create a second ground path.
- No thin-balance-wire-as-power-wire risk: the only path is through 10–56 kΩ
  (hundreds of µA).
- Build the sense harness as a **Y-tap / balance extension** so the balance
  connector stays available for the charger.
- Total drain ≈ 0.63 mA while connected (unevenly: cell 1 carries all three
  branches ~0.63 mA, cell 2 ~0.42 mA, cell 3 only its own ~0.19 mA).
  Irrelevant in flight; **do not store the pack with the sense lead
  attached** — unplug it together with the pack.
- 1 % metal-film resistors; exact values don't matter (28k is E96 rather than
  E24 — irrelevant here), the per-channel two-point calibration absorbs
  tolerance.

## Firmware (air)

- Read in `sensorTask` — not because it's I2C (it isn't), but it's the same
  50→10 Hz sensor pattern and `SensorSnapshot` already exists. A battery
  entry under `SensorLifecycle` may be overkill (the ADC can't "die" like a
  bus device); decide during implementation — plausibility bounds
  2.5–4.35 V/cell at minimum.
- Per read: average ~64 raw samples, convert via
  `esp_adc_cal_characterize()` (eFuse Vref), apply per-channel **two-point
  calibration** (empty vs. full pack against a multimeter; constants in
  code) — this removes resistor tolerance and residual ADC error in one go.
- Expected accuracy after calibration + averaging: ±20–60 mV per node
  (scales with the divider ratio, so C3 at 6.60 is the worst of the three)
  → **~±50–80 mV per cell** after subtraction.
  Sufficient for "cell 3 sagging below 3.2 V" and "Δ between cells
  > 150 mV" alarms.
- Consider a min-hold window alongside the IIR filter: the interesting value
  is the sag under full throttle, which a plain low-pass hides.

## Wire protocol + ground

- `esp32/lib/protocol/messages.h`: `uint16_t cellMv[3]` (+6 B, 84 → 90 B —
  update the `static_assert(sizeof(TelemetryMessage) == 84)` too),
  a `TELEM_FLAG_BATT` health flag, **bump `PROTOCOL_VERSION`**; mirror in
  `cocpitView/cockpitview/protocol.py` atomically (CLAUDE.md rule).
- cocpitView → MAVLink `BATTERY_STATUS` (`voltages[]` in mV — QGC shows
  per-cell), plus a HUD line like `BAT 11.4  3.80/3.79/3.81  Δ20`.
- HUD placement (`hud.py`): `BAT` takes the **bottom-right corner** slot
  (`bottomright`, (w−8, h−6)) and the existing `LNK` status moves **one
  line up** above it; `GPS` stays bottom-left. Same small status font,
  outside the flight symbology. Hidden when `TELEM_FLAG_BATT` is down
  (same idiom as the GPS/LNK lines); a cell below 3.2 V or Δ > 150 mV
  blinks the whole line at 2 Hz (the established `FAILSAFE` idiom —
  the HUD is monochrome, blinking is the only emphasis).

## Out of scope / future

- **Current + consumed mAh (INA226 + shunt on the same I2C bus)** — voltage
  is a poor fuel gauge on a flat Li-ion curve; integrated mAh is the better
  flight-planning signal. If the power path is ever opened up for this, the
  per-cell voltages then serve mainly as the imbalance alarm and hard floor.
- Update `doc/parts/README.md` (Battery monitoring section — fix the 47k/15k
  divider) and `doc/architecture.md` (Power architecture) when this is
  implemented.

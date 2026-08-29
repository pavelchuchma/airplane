# Samsung INR18650-30Q — the 3S Li-ion flight pack

- **Source:** [Samsung INR18650-30Q 3000mAh 20A, vývody Z](https://www.abaterie.cz/baterie-samsung-inr18650-30q-3000mah-20a-vyvody-z/) — abaterie.cz, 125.60 CZK per cell incl. VAT
- **What it is:** the airplane's flight battery — three cells in series (3S1P).
  Sold as **single cells with welded Z-tabs and no protection circuit**, so the
  pack is self-assembled and nothing inside it enforces any limit.

## Cell specs (datasheet)

| Parameter | Value |
|---|---|
| Capacity | 3000 mAh typ. (2950 mAh min.) |
| Nominal voltage | 3.6–3.7 V |
| Charge voltage | **4.20 V** ±0.05 |
| **Discharge cut-off** | **2.50 V** |
| Discharge current | **15 A continuous**, 20 A short-term |
| Standard charge current | 1.5 A (0.5C); 4 A max |
| DC internal resistance | ~30 mΩ (estimate — see the caveat below) |
| Size / weight | 18.4 × 65 mm / 46 g |
| Protection | **none** |

As a 3S1P pack: **12.6 V** full, ~11.1 V nominal, 3000 mAh ≈ 33 Wh, ~140 g of
cells.

## How deep to discharge

| | Resting (OCV) | Roughly left |
|---|---|---|
| Routine landing target | **3.5 V/cell** (10.5 V) | 20–25 % |
| Occasional deep run | 3.2 V/cell (9.6 V) | 7–10 % |
| Never below | 3.0 V/cell (9.0 V) | ~3 % |
| Datasheet cut-off — **damage line, not a target** | 2.5 V/cell (7.5 V) | 0 % |

The trade is one-sided. An NMC discharge curve is flat, so between 3.5 and
3.0 V/cell there are only a few percent of capacity — but depth of discharge
dominates cycle life (roughly 300–500 cycles at ~100 % DoD versus about double
that at ~80 % DoD). The last few percent of a flight are bought with half the
pack's life, and they are also the only reserve for a go-around.

Below 2.5 V/cell the damage is real and permanent (copper dissolution at the
anode current collector, which shows up as an internal short on the next
charge). A pack taken below ~2.0 V/cell should be retired, not revived.

Approximate resting-voltage → state-of-charge mapping (generic NMC, **not
measured for this pack** — treat as a rule of thumb, and see the next section
before reading anything off a *loaded* pack):

| OCV/cell | 4.20 | 4.00 | 3.85 | 3.70 | 3.50 | 3.40 | 3.20 | 3.00 |
|---|---|---|---|---|---|---|---|---|
| SOC | 100 % | 85 % | 70 % | 50 % | 25 % | 15 % | 7 % | 3 % |

## Sag under load — measured on the airframe

Flight log `cocpitView/logs/2026-08-27_182713.cvlog` contains a series of motor
bursts from t ≈ 172 s (5–10 s each). Medians over clean windows of the first
burst:

| | Window | Pack | Per cell |
|---|---|---|---|
| At rest, before the burst | 162–171.5 s | 12 146 mV | 4049 mV |
| Loaded plateau | 175.0–177.2 s | 11 077 mV | 3692 mV |
| Recovery between bursts | 179.9–181.6 s | 11 935 mV | 3978 mV |

→ **sag of 286 mV/cell** (against the between-burst recovery) to **356 mV/cell**
(against the fully rested value). The truth sits between the two: a 2–3 s
recovery is incomplete relaxation, while the pre-burst rest predates a real
drop in SOC.

This number is solid, because **the balance taps carry no current** — the main
leads, the XT60 and the ESC contribute nothing to it. It is the drop across the
cells themselves.

### Deriving the current from it — and why the answer is soft

`I = ΔU_cell / R_i`, with `R_i` **assumed** at 30 mΩ. That figure is a typical
DC internal resistance for the 30Q from independent cell testing; it is *not*
from the datasheet (which quotes AC impedance at 1 kHz, a different and lower
quantity) and it was not measured here. The whole uncertainty lives in it:

| R_i | sag 286 mV | sag 356 mV |
|---|---|---|
| 25 mΩ | 11.4 A | 14.3 A |
| 30 mΩ | 9.5 A | 11.9 A |
| 40 mΩ | 7.2 A | 8.9 A |

So: **order of 10 A, band 7–14 A, one significant figure.** Two further
systematic errors both push the estimate down — the baseline ambiguity above,
and the fact that a multi-second load adds concentration polarisation on top of
the ohmic drop, so attributing all of the sag to `R_i` overestimates the
current.

**Do not use this to claim margin against the 15 A continuous rating.** The
real margin could be 40 % or zero. Measuring it needs a wattmeter or a clamp
meter in series — or the INA226 + shunt already listed as future work in
`doc/plans/2026-08-10-battery-cell-monitoring.md`. Once the pack's true `R_i` is
known from a known-load step, the sag on the balance taps becomes a free
ammeter with no added hardware.

## Consequence for the HUD alarm

Full throttle costs **~0.3–0.36 V/cell** of *indicated* voltage. This follows
from the measured sag alone and does not depend on the current estimate above:

| Resting | SOC | Reads at full throttle |
|---|---|---|
| 3.70 V/cell | ~50 % | ~3.35–3.41 V/cell |
| 3.50 V/cell | ~25 % | ~3.15–3.21 V/cell |

The HUD's `CELL_FLOOR_V = 3.2` (`cockpitview/hud.py`) therefore **blinks with
roughly a quarter of the pack still in it**, if read at full throttle.

Rule: **judge state of charge at low throttle or in a glide**, never under
power. The firmware has no throttle signal today (the command channels are
neutral, no servos yet); when throttle control lands, the alarm should be gated
on low throttle. The plan's note about a min-hold window alongside the IIR
filter is the other half of this — the interesting value under power is the
*sag*, which a plain low-pass hides.

## Protection — there is almost none

The cells are unprotected and the firmware's `BAT` telemetry is an indicator,
not a protection. **The only automatic protection in the airplane is the ESC's
adjustable voltage cutoff** ([RAY G2 R-20B](esc-ray-g2-r-20b.md)) — set it.

Trap worth knowing: below 2.5 V/cell the firmware's plausibility band
(`kCellMinMv` in `esp32/src/air/main.cpp`) declares the battery *absent*,
`TELEM_FLAG_BATT_OK` drops and the HUD `BAT` line **disappears instead of
alarming** — indistinguishable from an unplugged balance lead. You should never
get there, but a vanished `BAT` line is not automatically a wiring fault.

## Charging, storage, temperature

- Standard charge 1.5 A (0.5C), 4 A maximum.
- Charge to **4.20 V — use the charger's LiPo mode.** Some chargers' "Li-ion"
  mode targets 4.10 V, which permanently undercharges the pack.
- **Never charge below 0 °C** — lithium plates on the anode, irreversibly.
- Storage beyond a few days: **3.7–3.8 V/cell**. Neither full nor empty.
- The 30Q runs noticeably warm near its 15 A rating, and a foam fuselage has no
  airflow. Leave a gap around the pack; discharging a hot cell (>60 °C)
  degrades capacity fast.

## ADC calibration status

`esp32/src/air/battery.cpp` carries a **one-point scale calibration** against a
multimeter on a full pack (2026-08-23) with zero offsets.

A repeat measurement on 2026-08-29 (log `2026-08-29_105101.cvlog`, DMM
4.17 / 8.33 / 12.48 V at the cumulative nodes) confirmed it to within
0.08–0.16 % per node. It **cannot** produce the offsets, though: both points sit
at a full pack, ~1 % apart in voltage, so the two-point fit is ill-conditioned
and returns nonsense (node 3: slope 6.02, offset +996 mV). The residual signs
across the three channels are `+, −, +`; a genuine ADC offset would be the same
sign on all three, so what is left is the multimeter's own resolution, not a
systematic error.

**Next measurement: after the first real flight, on the discharged pack.** What
it needs:

- a voltage span of **≥ 20 %** between the two points — a pack rested at
  ~3.4 V/cell against the full-pack point already measured;
- the multimeter in its **highest-resolution range** per node (the C1 node on a
  6 V range gives 1 mV instead of 10 mV — DMM resolution is the dominant error,
  and going from 10 mV to 1 mV improves the fitted offset from ~±20 mV to
  ~±2 mV);
- the wall-clock time of each reading noted, and the firmware value taken as the
  **mean of the raw `cellMv` over ±15 s** from the log — not the rounded HUD;
- no changes to `kCalScale` in between, so the pin voltages stay recoverable.

Alternative that needs no deep discharge at all: the full-throttle sag plateau
is itself a low-voltage operating point, and the ADC does not care whether the
low voltage comes from depth of discharge or from load. A full pack gives
12.5 → 11.0 V (12 % span); a half-empty pack at full throttle gives roughly
11.4 → 10.3 V, which together with the full-pack point covers ~18 %.

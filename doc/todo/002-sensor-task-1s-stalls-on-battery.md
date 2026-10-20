# 002 — ~1 s sensor-task stalls, only with the balance lead connected

Status: **open** (2026-10-12). Analysis done, cause unknown. Since
2026-10-20 (protocol v7) the experiments below no longer need USB: the air
diagnostics reach `cocpitView/logs/<stem>.air.log` over the radio.

## Symptom

The sensor task stops publishing for exactly 0.98–1.00 s, repeatedly. The
snapshot goes stale after 200 ms, so `loop()` drops *all* OK flags
(IMU/baro/pitot/battery together) for ~0.8 s per stall: cocpitView hides the
horizon and the readouts, the HUD "stutters". No sensor dies in lifecycle
terms — after the stall the same values continue, nothing goes NAN, so the
stalled transaction eventually succeeds (or belongs to a sensor whose
lifecycle survives 1 s). Rotation during a stall is lost (dt is capped at
0.5 s), which is why a plane turned over during stalls takes seconds to
level on the HUD afterwards.

Signature in a log: `roll`, `pitch`, `yaw` bit-identical for 40–60
consecutive frames (the ~1 s freeze), the OK flags clear from the 11th frame
on. Counted per boot (`time_boot_ms` reset = new boot) over the attitude
columns of `flightlog csv`.

## What the logs say (all 29 `.cvlog` files, per boot)

| Boot | Balance lead | Length | ~1 s stalls | Notes |
|---|---|---|---|---|
| `2026-10-11_164858` | yes | 308 s | **107** | plane motionless, no motor (cells flat at 4.06 V), firmware *before* the IMU config check |
| `2026-10-11_181117` b0 | yes | 175 s | 50 | the baro-death boot (todo 001) |
| `2026-10-12_185154` | yes | 117 s | 33 | |
| `2026-10-12_190526` b1 | yes | 41 s | 21 | the inverted-boot test |
| `2026-10-12_194036` b4 | yes | 44 s | 8 | v6 firmware; all 8 in the last 13 s, 93–105 s into the log, the first five while the plane was being rotated at 70–85 °/s |
| `2026-10-12_194258` | no | 87 s | 0 | v6, same handling test, USB |
| every other USB boot of 2026-10-11/12 (8) | no | 11–428 s | 0–1 | |
| every battery boot of 2026-08-27 … 2026-09-25 (≈20) | yes | up to 269 s | 0–2 | same battery code |

So:

- **Only boots with the balance lead** (= pack powering the plane via the
  BEC; "balance lead present" is also the only condition under which
  `battery::read()` runs its 192 `adc1_get_raw()` calls at 10 Hz).
- **Only since 2026-10-11**, the day of the ground motor run and the baro
  death. The battery boots of August/September ran the same battery code
  and had none — something in the airframe/wiring changed, not the firmware.
- Not the motor (cells flat during the stalls), not the per-second IMU
  config check (`164858` predates it), not orientation (`190526` stalled
  inverted and upright alike), not motion per se (`164858` stalled
  motionless; `194036` stalled while handled, `194258` was handled the same
  way on USB and did not).
- Inter-stall spacing clusters at 1.04–1.2 s (stall + a few samples) with
  multiples at ~2.1 and ~3.2 s — stalls come in bursts, i.e. whatever holds
  the bus tends to hold it again right after the driver gives up.

## Hypotheses

1. **Hardware, BEC/ESC related** (preferred): a floating chip-select on the
   GY-91 (NCS of the MPU-6500, CSB of the BMP280 — see todo 001) picking up
   switching noise that is present only on battery/BEC power. A low NCS puts
   the MPU-6500 into SPI mode for the duration, which would hold the bus
   until the IDF driver's ~1 s timeout and recover by itself; the same
   mechanism on the BMP280's CSB latches permanently (the 2026-10-11 baro
   death). Fits "only on battery", "only since the airframe was assembled
   for the motor run", and the bursts.
2. **Software, the ADC path**: `adc1_get_raw()` × 192 per 100 ms inside the
   same task as the I2C transactions (critical sections / RTC peripheral
   locks interfering with the I2C ISR). Weak — the same code ran clean on
   the battery in August/September — but it is the only software difference
   between a battery boot and a USB boot, and it is cheap to rule out.

## Why the stall is exactly 1.0 s

Likely the IDF 4.4 I2C driver, not the bus speed: `i2c_master_cmd_begin()`
waits for the transaction-done event at least `I2C_CMD_ALIVE_INTERVAL_TICK`
(1000 ms) regardless of the caller's timeout. Exactly 1.0 s therefore means
the completion interrupt never came - the I2C peripheral got stuck on a bus
event (a glitch on SCL/SDA), which points at electrical noise rather than
at a slow code path (192 ADC reads take milliseconds and would not produce
a fixed 1.0 s).

## Experiments since v7 (battery only, airframe closed)

1. **Which transaction:** an ordinary battery session. Every stall leaves
   `slow I2C: <sensor> took N ms` in the `.air.log` (the battery ADC read is
   timed too, as `battery ADC read`).
2. **ADC or not:** flash with
   `PLATFORMIO_BUILD_FLAGS=-DAIR_NO_BATTERY pio run -e air -t upload`
   (battery never read, `BAT` disappears from the HUD) and count stalls in
   a battery session of a few minutes. Stalls stay → hypothesis 1, vanish →
   hypothesis 2. Reflash without the flag afterwards.

## Original experiment (needs the airframe + USB)

Power the board over **USB with only the balance lead plugged** (no main
lead, so no BEC): the battery lifecycle reports present and the ADC path
runs, while the I2C bus sees USB power.

- Stalls appear → hypothesis 2; `# slow I2C: <sensor> took N ms` on the USB
  serial names the stalled transaction.
- No stalls over a few minutes (and handling the plane) → hypothesis 1; do
  todo 001 (tie CSB/NCS to 3V3) and repeat the battery test.

Either way the `# slow I2C` diagnostic is the tool that names the
transaction; it has never yet run during a stall because stalls need the
battery and the battery excludes USB.

## Possible firmware mitigation (after the cause is known)

Per-sensor snapshot timestamps instead of one `snapshotUs`, so a stalled
baro read does not blank a healthy IMU; and/or move the battery ADC reads to
`loop()` (they are not I2C and do not belong in the bus task).

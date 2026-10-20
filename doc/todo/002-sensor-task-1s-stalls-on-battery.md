# 002 — ~1 s sensor-task stalls, only with the balance lead connected

Status: **resolved** 2026-10-20. Opened 2026-10-12.

- **Cause:** a bad (cold) ground joint in the airframe wiring. With the
  battery in and the airframe closed, it made the MS4525DO pitot glitch on
  the I2C bus (address NACKs, SDA conflicts, once SDA held low). Resoldered
  on 2026-10-20; `2026-10-20_220345` (284 s) and `220935` (97 s) on battery
  are entirely clean.
- **Why a glitch cost a second:** the Arduino/IDF I2C driver turned each
  glitched pitot read into a ~1 s wait (1006-1008 ms, regardless of
  `Wire.setTimeOut(20)`), which froze the whole sensor snapshot. Where in
  the driver stack that second comes from is not known (see below). Fixed
  independently of the joint, twice: first by a bit-banged bus
  (`git show e65f783`, `doc/plans/2026-10-20-soft-i2c.md` - its failure
  classes are what found the joint), then by the move to arduino-esp32 3.x,
  whose native IDF 5 driver fails the same NACK in 0.26 ms instead of
  1007 ms (`pio run -e i2ctest`, `doc/plans/2026-10-20-arduino3-native-i2c.md`).
  A slave holding SDA is freed right after the transaction, so a glitch
  costs one sample. Glitches inside a transaction with a real sensor could
  not be reproduced on the bench; that is left to operation - the
  `I2C <sensor> 1s:` lines in the `.air.log` will show it.

The rest of this file is the investigation, kept because the method is
reusable.

## Symptom

The sensor task stopped publishing for exactly 0.98–1.00 s, repeatedly. The
snapshot went stale after 200 ms, so `loop()` dropped *all* OK flags
(IMU/baro/pitot/battery together) for ~0.8 s per stall: cocpitView hid the
horizon and the readouts, the HUD "stuttered". No sensor died in lifecycle
terms — the stalled transaction eventually succeeded. Rotation during a
stall was lost (dt is capped at 0.5 s).

Signature in a log: `roll`, `pitch`, `yaw` bit-identical for 40–60
consecutive frames, the OK flags clear from the 11th frame on.

## What the old logs said (all 29 `.cvlog` files, per boot)

| Boot | Balance lead | Length | ~1 s stalls | Notes |
|---|---|---|---|---|
| `2026-10-11_164858` | yes | 308 s | **107** | plane motionless, no motor |
| `2026-10-11_181117` b0 | yes | 175 s | 50 | the baro-death boot (todo 001) |
| `2026-10-12_185154` | yes | 117 s | 33 | |
| `2026-10-12_190526` b1 | yes | 41 s | 21 | the inverted-boot test |
| `2026-10-12_194036` b4 | yes | 44 s | 8 | v6 firmware |
| `2026-10-12_194258` | no | 87 s | 0 | same handling test, USB |
| every other USB boot of 2026-10-11/12 (8) | no | 11–428 s | 0–1 | |
| every battery boot of 2026-08-27 … 2026-09-25 (≈20) | yes | up to 269 s | 0–2 | same battery code |

Only on battery (the balance lead goes with the pack), and only since
2026-10-11 — the day the airframe was assembled for the ground motor run.

## How it was narrowed down (2026-10-20)

Each step needed the air diagnostics on battery, which is why they now
travel over the radio into `<stem>.air.log` (protocol v7,
`doc/plans/2026-10-20-air-diag-log.md`).

| Log | Setup | Result |
|---|---|---|
| `175900` | battery, Wire driver, per-transaction timing | 22 freezes = 22 `MS4525DO read took 1006-1007 ms`, nothing else slow; the reads *succeeded* |
| `180823` | VTX + camera unplugged | same rate → not the video chain |
| `181656` | bit-banged bus | no 1 s stall; pitot ~0.5 errors/s (`addr-nack`, `sda-conflict`, no clock stretch), one SDA-held-low episode (2 s rescue → 2–3 s freezes, hence the immediate bus clear) |
| `204503` | ADC never read (temporary build flag) | 0.34 errors/s, same within noise → not the battery ADC; error times uncorrelated with the MPU config check and the telemetry send → not our periodic work |
| `210343`, `210559` | airframe opened | **0** errors in 196 s |
| `211657` | partly reassembled | 0.13 errors/s, in bursts → mechanical / wiring |
| `220345`, `220935` | ground joint resoldered, closed | **0** errors in 381 s |

Ruled out along the way: the motor and ESC (motor disconnected the whole
time), the original RC receiver (disconnected), orientation, handling.

## The 1 s: what is known

Measured: 1006-1008 ms per glitched transaction with `Wire.setTimeOut(20)`,
and arduino-esp32 2.0.17 does pass that 20 ms down to
`i2c_master_cmd_begin()`. An earlier note here blamed the IDF 4.4
`I2C_CMD_ALIVE_INTERVAL_TICK` (1000 ms) as a minimum wait — wrong: the IDF
v4.4 source waits `MIN(remaining timeout, alive interval)`, i.e. 20 ms
here. Not a hardware limit either (the ESP32 I2C timeout register tops out
at ~13 ms). So the second comes from somewhere else in the Wire →
esp32-hal-i2c → IDF driver → FSM reset path; not pursued, since the
bit-banged bus makes the question moot.

Reproduced on a bare devkit (`pio run -e i2ctest`, 2026-10-20): a *plain
read* from an address nobody answers takes 1006.7-1007.0 ms and returns
`ESP_ERR_TIMEOUT`; the same NACK in a register (write-then-read) read
returns `ESP_FAIL` in 7-15 ms. The pitot is the only sensor read with a
plain read - which is why only the pitot ever stalled. Details:
`doc/plans/2026-10-20-arduino3-native-i2c.md`, phase 0.

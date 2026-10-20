# 001 — Tie the GY-91 CSB and NCS pins high (BMP280 latched into SPI mode)

Status: **open** (2026-10-11). Hardware change, needs the airframe open.

Note 2026-10-20: the baro death happened in the same period as the bus
glitches of `doc/todo/002`, whose trigger turned out to be a cold ground
joint (resoldered). A ground bouncing on battery may well have been what
pulled the floating CSB low, so this may not recur - but tying CSB/NCS high
is still the cheap, permanent fix.

## Problem

On 2026-10-11 the barometer died 10 s into a boot and stayed dead for the
rest of that boot (7 min, logs `2026-10-11_180628.cvlog` seg3,
`2026-10-11_180819.cvlog`, `2026-10-11_181117.cvlog` seg0 — one boot, the
`time_boot_ms` is continuous across the files). It had passed the boot probe
and the reference calibration and reported sane altitude for 7 s; then its
reads failed instantly (no bus stall) and for good, while the MPU-6500 on the
same GY-91 board, the same SDA/SCL and the same LDO stayed healthy throughout.
A power glitch is ruled out (a reset BMP280 returns adc 0x80000, which
compensates to ~820 hPa — inside the plausibility band, so it would have
shown as a bogus altitude, not a dead sensor). Only the next battery power
cycle revived it; every other boot that day was fine. The airframe was being
turned over at the moment of death.

The symptom matches the BMP280 datasheet behaviour on its CSB pin: the first
low on CSB disables the I2C interface until the next power-on reset. On our
GY-91, **CSB and NCS are unconnected** — floating header pins held high only
by the on-board pull-up, two pins from the GND-tied SDO/SAO lead
(`esp32/src/air/main.cpp` header, `doc/parts/README.md`,
`doc/parts/gy91-mpu9250-bmp280.md`). No firmware re-init can reach a chip
that no longer ACKs its address, so the fix is in the wiring.

## Change

1. Wire **CSB → 3V3** and **NCS → 3V3** on the GY-91 header (short jumper or
   solder bridge to the board's 3V3 pin). 3.3 V, never the 5 V VIN — the
   sensors' VDDIO is 3.3 V.
2. Inspect the SDO/SAO → GND lead for stray strands near CSB/NCS; re-tin or
   sleeve it.
3. Update the wiring notes: the header comment in `esp32/src/air/main.cpp`
   (`NCS + CSB unconnected`), the wiring table in `doc/parts/README.md` and
   the wiring block in `doc/parts/gy91-mpu9250-bmp280.md`.

## Verification

The firmware prints `BMP280 no ACK - off the bus (CSB latched into SPI?
...)` when the baro dies without answering its address, and an
`I2C BMP280 1s: ...` report for any bus trouble; since protocol v7 both
reach `cocpitView/logs/<stem>.air.log` on battery too. After the change,
run several battery sessions, handle the airframe (roll it over, lay it on
its back) and confirm neither line appears and `TELEM_FLAG_BARO_OK` stays
up for the session.

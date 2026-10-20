# Arduino-ESP32 3.x and back to a native I2C driver

Status: **phase 1 done** 2026-10-20 (pioarduino 55.03.312-1 = arduino-esp32
3.3.12 / IDF 5.5.5); phase 2 not started.

## Why

The bit-banged bus (`doc/plans/2026-10-20-soft-i2c.md`) works and found the
cold ground joint of `doc/todo/002`, but it is our own driver and costs
~3 ms of busy-waiting per 20 ms tick on core 0. The native driver of
arduino-esp32 2.0.17 is unusable: a single glitched transaction costs ~1 s
(cause in the driver stack unknown). arduino-esp32 2.x is also no longer
developed; the servo work will need the 3.x LEDC API anyway.

What the sources say (checked 2026-10-20):

- arduino-esp32 3.x builds `Wire` on `esp32-hal-i2c-ng.c` from IDF 5.4 on,
  i.e. on the new `driver/i2c_master.h`; the timeout is passed through
  unchanged.
- IDF 5.5 `i2c_master.c` has no fixed wait: NACK → STOP + FSM reset +
  `ESP_ERR_INVALID_STATE`; a timeout or busy bus is reset *with* a 9-clock
  bus clear at the start of the next transaction. NACK and other failures
  return the same error, so the `addr-nack` / `sda-conflict` / `bus-busy`
  split of the bit-banged bus is lost.
- The official PlatformIO platform does not ship Arduino 3.x; the
  community fork **pioarduino** does (55.03.x = Arduino 3.3.x / IDF 5.5).

Migration and I2C driver are separate decisions: the bit-banged bus runs on
3.x unchanged, so phase 2 is optional.

## Phase 0 — a reproducible test (on 2.0.17, bare devkit)

`pio run -e i2ctest -t upload` (ground board by default): times reads from
an address nobody answers (0x29) through Wire - plain read (pitot-like),
register read (MPU-like), empty-write probe - and through `i2cbus` as the
reference; prints min/avg/max per case on USB serial. Expectation on
2.0.17: ~1 s per Wire case (the old firmware comment said failed
transactions cost ~1 s each). If confirmed, the same test is the
acceptance test for any driver on 3.x.

### Result on 2.0.17 (ground devkit, nothing on the bus, 2026-10-20)

```
arduino-esp32 2.0.17, IDF v4.4.7, Wire timeout 20 ms, 100 kHz, address 0x29 x5
Wire plain read (pitot)      min  1006.71  avg  1006.90  max  1007.00 ms   i2cRead -> 263 (ESP_ERR_TIMEOUT)
Wire register read (MPU)     min     7.19  avg     8.66  max    14.55 ms   i2cWriteReadNonStop -> -1 (ESP_FAIL)
Wire probe (empty write)     min     0.14  avg     0.19  max     0.30 ms
i2cbus plain read            min     0.15  avg     0.17  max     0.22 ms
i2cbus register read         min     0.15  avg     0.16  max     0.17 ms
```

The 1 s is reproduced exactly (1006-1007 ms, the stall durations of
todo 002) - and only for the **plain read** (`i2c_master_read_from_device`):
an address NACK there ends in `ESP_ERR_TIMEOUT` after a second, while the
same NACK in a write-then-read returns `ESP_FAIL` in 7-15 ms. That is why
only the pitot stalled: it is the one sensor read with a plain read; the
MPU and BMP280 use register reads, whose glitches were cheap. The test
needs no sensors and no disturbance, so it is the acceptance test for
phase 2.

### Result on 3.3.12 (same devkit, pioarduino 55.03.312-1, IDF v5.5.5)

```
                              with core logging         CORE_DEBUG_LEVEL=0
Wire plain read (pitot)       15.30 / 16.64 / 21.99 ms   0.16 / 0.26 / 0.68 ms   (min / avg / max)
Wire register read (MPU)      18.05 / 19.51 / 25.33 ms   0.16 / 0.19 / 0.30 ms
Wire probe (empty write)       0.15 /  0.17 /  0.26 ms   0.15 / 0.17 / 0.23 ms
i2cbus plain read              0.15 /  0.16 /  0.21 ms   0.15 / 0.16 / 0.21 ms
```

The native driver of 3.x passes: a NACK costs what it costs the bit-banged
bus. The catch is the left column - every failure prints two `[E]` lines
on UART0, ~9 ms of blocking at 115200 baud, battery or not. A native-I2C
air firmware must build with `-DCORE_DEBUG_LEVEL=0` (or silence the
`i2c`/hal log tags).

## Phase 1 — arduino-esp32 3.x, bit-banged bus unchanged

- `platform =` pioarduino, pinned to an exact release; both environments.
- ESP-NOW: `recv_cb(const esp_now_recv_info_t *, ...)` — RSSI from
  `rx_ctrl`, drop the promiscuous-mode sniffer; `send_cb(const
  esp_now_send_info_t *, status)`.
- Battery ADC: `driver/adc.h` + `esp_adc_cal` → `adc_oneshot` +
  `adc_cali`; **re-verify the calibration against the DMM** (the eFuse
  characterization scheme differs).
- GPS UART2: default pins changed in 3.x — pass GPIO16/17 explicitly.
- Verify: link (LR, encryption, failsafe, loss, RSSI against the old
  values), battery calibration, all sensors on battery, `i2ctest`.

### Phase 1 result (2026-10-20)

- Code changes were exactly the predicted ones: the two ESP-NOW callback
  signatures (RSSI now from `rx_ctrl`, promiscuous sniffer removed) and the
  battery ADC (`adc_oneshot` + `adc_cali` line fitting). GPS already passed
  its UART2 pins. Flash 61 % → 72 %.
- Surprises: pioarduino turns core logging on by default (now
  `-DCORE_DEBUG_LEVEL=0` in both envs); PlatformIO ≥ 6.2 runs pre-scripts
  with the interpreter pio was installed into, which lacks esptool 5's
  dependencies, so `select_port.py` now probes with `~/.platformio/penv`'s
  Python (and `board_id.py` reports *why* a port was unreadable).
- Verified: bench link (`225857`, 59 s: 0 % tx fail, 0.4 % command loss,
  RSSI −47 dBm), outdoor battery session (`232229`, 100 s: GPS cold-start
  fix at 70 s, 12 sats, hacc 0.85 m, home + baro locked, all sensors OK, 0 %
  loss down to −95 dBm).
- Battery vs DMM (`232951`): N1 3.897 vs 3.90 V, N2 7.756 vs 7.81 V, N3
  11.619 vs 11.70 V - N2/N3 −0.69 % (on 2.0.17 they were within 0.16 %), so
  the new calibration code shifts those two channels; `kCalScale` N2/N3
  rescaled by 7.81/7.756.

## Phase 2 — native I2C behind `i2cbus.h`

- A second implementation of the `i2cbus.h` API on Wire (or directly on
  `i2c_master`), selected at build time; the sensor drivers and
  `BusStats` stay as they are, the bit-banged one stays as the fallback.
- Status mapping: OK / NACK-or-other (`ESP_ERR_INVALID_STATE`) / timeout.
- Acceptance: `i2ctest` failures cost single-digit ms, never seconds; a
  held SDA is freed within one sample; battery sessions with the airframe
  closed stay clean.

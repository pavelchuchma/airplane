# Arduino-ESP32 3.x and back to a native I2C driver

Status: **planned** 2026-10-20. Phase 0 done (below).

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

## Phase 2 — native I2C behind `i2cbus.h`

- A second implementation of the `i2cbus.h` API on Wire (or directly on
  `i2c_master`), selected at build time; the sensor drivers and
  `BusStats` stay as they are, the bit-banged one stays as the fallback.
- Status mapping: OK / NACK-or-other (`ESP_ERR_INVALID_STATE`) / timeout.
- Acceptance: `i2ctest` failures cost single-digit ms, never seconds; a
  held SDA is freed within one sample; battery sessions with the airframe
  closed stay clean.

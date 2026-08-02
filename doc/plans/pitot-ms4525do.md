# Plan: MS4525DO pitot (airspeed) sensor integration

## Context

The MS4525DO-DS5AI001DP differential-pressure sensor (pitot) is already wired to
the air ESP32's I2C bus (GPIO21/22, address 0x28, 5 V supply with 3.3 V
pull-ups — see `doc/parts/README.md` and `doc/parts/pitot-ms4525do.md`,
which contains the full decode math). Nothing reads it yet: the HUD airspeed
tape (`hud.py`) exists but shows a constant 0 because
`espnow_source.py` hardcodes `VFR_HUD.airspeed = 0.0`. Goal: read the
sensor on the air side, compute IAS, carry it in telemetry, and make the HUD
airspeed tape + QGC live. Air-side IAS is also a prerequisite for the future
autopilot.

## Design decisions

- **Driver copies the `bmp280` pattern**: `namespace ms4525do`, `begin()` /
  `read()`, register-level Wire calls, no library dependency. No chip-id
  register exists → presence check = ACK at 0x28 + sanity check that zero-ΔP
  counts are near 8192 (per `doc/parts/pitot-ms4525do.md`).
- **Zero-offset calibration on the air side at boot** (mandatory — the sensor
  has tens-of-Pa zero drift): blocking average like `calibrateBaroReference()`
  (`main.cpp:207-224`), ~50 samples / 1 s, runs during the existing ~2 s
  blocking calibration window. Failure → pitot marked dead, fields NAN.
- **Read at 50 Hz in the main loop** (conversion time is 0.5 ms, read is a
  4-byte burst ≈ 0.1 ms at 400 kHz — negligible). Filter ΔP with the existing
  dt-aware IIR idiom `alpha = dt/(tau+dt)` (`main.cpp:243`), tau ≈ 0.3 s.
- **IAS computed on the air side**: `ias = sqrt(2·max(ΔP,0)/1.225)` (ρ₀ fixed
  → that *is* IAS; TAS correction is a later PC-side concern). Negative ΔP
  clamps to 0.
- **Telemetry carries both `diffPressPa` (zero-corrected, filtered) and
  `iasMps`** — ΔP enables PC-side diagnostics/re-zero later, costs 4 B.
  76 → 84 B, still ≪ 250 B ESP-NOW limit. NAN = pitot dead, plus a
  `TELEM_FLAG_PITOT_OK` status flag (bit 4, next free). `PROTOCOL_VERSION`
  1 → 2.
- Sensor temperature is read (it's in the same 4-byte transfer) but only kept
  driver-internal for now; not put on the wire.

## Changes

### 1. New driver — `esp32/src/air/ms4525do.{h,cpp}`

Auto-compiled via `build_src_filter = +<air/>`; mirror `bmp280.h` shape:

```cpp
namespace ms4525do {
bool begin(uint8_t address = 0x28);       // ACK probe + raw counts sanity (8192 ± ~1500)
bool read(float &diffPressPa, float &temperatureC);  // raw, NOT zero-corrected
}
```

Decode per `doc/parts/pitot-ms4525do.md`: status bits `b0>>6` (0 = fresh;
2 = stale → return false and skip the sample; 3 = fault → false),
`counts = ((b0 & 0x3F) << 8) | b1`,
`dP_Pa = (counts − 8192) · 2·6894.76 / (0.8·16383)`,
`T = (((b2<<8)|b3) >> 5) · 200/2047 − 50`.

### 2. Air firmware — `esp32/src/air/main.cpp`

- Constants (~line 64): `kPitotZeroSamples`, `kAirspeedTimeConstant = 0.3f`.
- `setup()` after the BMP280 block (~line 281): `pitotOk = ms4525do::begin();`
  on failure print `# MS4525DO not found - airspeed will be nan`; then
  `calibratePitotZero()` next to `calibrateBaroReference()` (~line 285) —
  average ~50 raw ΔP samples with `delay(20)`, discard the first few; zero
  successful samples → `pitotOk = false`.
- New `updatePitot(now)` modeled on `updateBaro()` (`main.cpp:228-246`):
  read, subtract zero offset, IIR-filter ΔP, derive `iasMps`; called from
  `loop()` next to `updateBaro` (~line 316). Track consecutive read failures
  (e.g. >25 → drop `pitotOk`, fields go NAN) so an in-flight sensor death is
  visible.
- Telemetry assembly (~line 374, after `t.climbMps`): `t.diffPressPa`,
  `t.iasMps`; add `TELEM_FLAG_PITOT_OK` to the flags expression
  (lines 363-364).

### 3. Wire protocol — `esp32/lib/protocol/messages.h`

- `PROTOCOL_VERSION` 1 → 2.
- `TELEM_FLAG_PITOT_OK = 1 << 4`.
- `float diffPressPa; float iasMps;` after `climbMps`; update the
  `static_assert(sizeof == 76)` → 84 and the field-offset comment.

### 4. Python mirror — `cocpitView/cockpitview/protocol.py`

Atomically with #3 (CLAUDE.md rule): `PROTOCOL_VERSION = 2`, new flag
constant, format string `"<BBBBI8f2i6fbBBB"` → `"<BBBBI10f2i6fbBBB"`,
size assert 76 → 84, namedtuple fields `diff_press_pa`, `ias_mps` after
`climb_mps`.

### 5. MAVLink out — `cocpitView/cockpitview/espnow_source.py`

In `_publish_telemetry`: first `vfr_hud_encode` argument becomes
`0.0 if math.isnan(t.ias_mps) else t.ias_mps`; update the stale
"No airspeed sensors yet" comment. The surrounding
`if not math.isnan(t.baro_alt_m):` guard stays — the plane always has the
baro; pitot-without-baro is a hypothetical failure mode not worth a branch.

### 6. HUD — no code change

`hud.py` already renders `snap.airspeed_kmh` from VFR_HUD; it goes live
automatically. Pitot dead → 0 on the tape (same as today).

### 7. Docs

- `doc/architecture.md` — current-state paragraph: pitot read, IAS in
  telemetry and VFR_HUD.
- `CLAUDE.md` "Current state" section: same, per its own update rule.

## Verification

1. Build both sides: `cd esp32 && pio run -e air` (ground unchanged —
   protocol-agnostic bridge). Python sanity:
   `cd cocpitView && python -c "from cockpitview import protocol"` (runs the
   size asserts).
2. Bench test: upload air fw (`pio run -e air -t upload`), start
   `cocpitView/run.sh` with the ground bridge plugged in. Expect: HUD
   airspeed tape at ~0 when still; blow into the pitot tube → tape and QGC
   VFR_HUD respond, decays back to ~0.
3. Failure paths: with the sensor disconnected, air serial prints
   `# MS4525DO ...` diag, telemetry floats are NAN, flag bit 4 clear, HUD
   shows 0. Flash only one side with the new protocol → the existing
   version-mismatch warning fires (validates the version bump).
# VTX control over IRC Tramp (protocol v8)

Status: **proposed** 2026-10-25, to be implemented in a later session —
entry point `doc/todo/003-vtx-tramp-control.md`. Hardware is in place: the
ULTRA 1600's `IRC` pin is wired to GPIO26 (RX, via 10 kΩ) and GPIO27 (TX,
via 1 kΩ), measured and documented in `doc/parts/fpv-58g-uvc-kit.md` §9.

## Why

The VTX is configured with one button and three LEDs, on the airframe,
with the pack plugged. Tramp puts the same three settings (frequency,
power, pit mode) plus status (actual power, temperature, race lock) on a
UART, so:

- the plane can sit on the ground in pit mode or at 25 mW and go to a higher
  step only on a deliberate keypress in cocpitView — the EU-legal story of
  §7 becomes an operating procedure instead of a wish;
- power and channel changes no longer need hands on the model;
- VTX temperature lands in telemetry and the flight log. The module has
  thermal protection; this shows how close to it the 800/1600 mW steps run
  inside a closed EPP fuselage.

## Tramp protocol (what the firmware must speak)

Single wire, half duplex, 9600 8N1. Every frame is exactly 16 bytes:

| byte | content |
|---|---|
| 0 | `0x0F` start |
| 1 | command (ASCII) |
| 2..13 | payload, little-endian, unused bytes 0 |
| 14 | checksum = sum of bytes 1..13, low 8 bits |
| 15 | `0x00` |

| cmd | request payload | reply payload |
|---|---|---|
| `r` (0x72) | — | [2-3] freq min MHz, [4-5] freq max MHz, [6-7] max power mW |
| `v` (0x76) | — | [2-3] freq MHz, [4-5] configured power mW, [6] control mode (≠ 0 = race lock), [7] pit mode, [8-9] actual power mW |
| `s` (0x73) | — | [6-7] temperature °C (int16) |
| `F` (0x46) | [2-3] freq MHz | none |
| `P` (0x50) | [2-3] power mW | none |
| `I` (0x49) | [2] pit mode | none |

Set commands are not acknowledged; the follow-up `v` is the confirmation.
This is the layout Betaflight's `vtx_tramp.c` uses and it is written down
here from memory of that driver, so **every offset is verified against the
readback at bring-up** before anything depends on it. Two known traps: the
`I` byte polarity (Betaflight sends 0 to *enter* pit mode and 1 to leave
it — check which way the ULTRA reads it), and `v` reporting the configured
power while the actual one differs in pit mode and under thermal
throttling.

With the 1 kΩ series resistor on TX, RX hears every byte the ESP32 sends.
The parser drops the first received frame that is byte-identical to the
request just sent; a `v` reply has the same command byte but a different
payload, so the comparison is safe.

## Air side — `src/air/vtx.{h,cpp}`

`vtx::begin()` opens `Serial1` at 9600 8N1 on RX 26 / TX 27 (UART1's
default pins are the flash, it has to be remapped anyway). `vtx::poll()`
runs from `loop()` every pass like `gps::poll()`: it drains RX into a
16-byte frame assembler (resync on `0x0F` at position 0, checksum and
trailing `0x00` checked) and steps a small state machine. Nothing blocks:
a request is written into the UART FIFO (16 B = 17 ms on the wire) and the
reply is collected on later passes.

**One request in flight at a time**, 100 ms reply timeout. Cadence:

1. `r` every 1 s until it is answered → `present`, range and max power
   stored, one diag line `VTX present: 5645-5945 MHz, max 1600 mW`. While
   unanswered the VTX is `absent` (the bench on USB: the VTX is unpowered)
   and keeps being probed — unlike the I2C sensors it is not "decided at
   boot"; the probe is cheap and silent.
2. `v` every 500 ms, `s` every 2 s. `alive` = a valid reply within the
   last 2 s; losing it is one diag line, regaining it another.
3. A set command goes out between polls when the desired state differs
   from the readback (below), followed by a `v` 100 ms later.

**Desired-state reconciliation, no acks.** The 50 Hz `CommandMessage`
carries what the operator wants; the air side compares it with the last
`v` readback and sends the matching set when they differ:

- `vtxFreqMhz` ≠ 0 and ≠ readback freq → `F`
- `vtxPowerMw` = 0 (pit) and readback pit mode off → `I` pit on
- `vtxPowerMw` ∉ {0, 0xFFFF}: readback pit mode on → `I` pit off; then
  readback configured power ≠ desired → `P`
- `vtxPowerMw` = 0xFFFF / `vtxFreqMhz` = 0 → leave that setting alone

This is idempotent and survives packet loss, link loss and an air reboot
without any handshake: when commands return, the next frame says what the
state should be. Rules that keep it from misbehaving:

- at most one set per 500 ms, and at most 5 attempts for a given desired
  value; after that `VTX refused power 1600 mW (reads 800)` once and no
  more tries until the desired value changes;
- requests outside the `r` range (frequency) or above max power are
  rejected with a diag line and never sent;
- race lock (`v` control mode ≠ 0) → no sets at all, one diag line;
- **failsafe takes no VTX action** — the video keeps whatever it had;
- until cocpitView asks for something the firmware changes nothing; the
  VTX boots with its button-stored configuration.

**Diagnostics** (`diag::print`, so they reach the `.air.log`): present /
absent / alive transitions, every readback change as one line
`VTX 5800 MHz 25 mW (actual 25) pit=0 42 C`, every set sent and every
rejection. A clean session with no changes is three lines.

**The radio log is the only debugger.** The VTX runs off the pack and the
pack and USB are mutually exclusive by construction, so whenever the VTX
is powered the air board's USB console is unreachable. Everything the
bring-up needs to see must therefore be a `diag::print` line that reaches
the `.air.log`, and the firmware dumps raw frames for exactly that:

- the first valid reply of each command type (`r`, `v`, `s`) once per boot
  as hex, `VTX rx r: 0F 72 0D 16 39 17 40 06 ...` — this is how the payload
  offsets above get verified without a scope or a terminal;
- every frame that fails the checksum or arrives malformed, as hex, rate
  limited to one line per 5 s (a wiring fault must not flood the 32-line
  buffer);
- echo frames are dropped silently.

Commands only flow while cocpitView is connected, and the diag lines wait
in the buffer until then — so the boot lines of a battery-only session are
not lost, they arrive when the ground side comes up.

## Wire protocol v8

`CommandMessage` (20 → 24 B):

```c
uint16_t vtxFreqMhz;   // desired VTX frequency, 0 = leave as is
uint16_t vtxPowerMw;   // desired VTX power, 0 = pit mode, 0xFFFF = leave as is
```

`TelemetryMessage` (90 → 96 B):

```c
uint16_t vtxFreqMhz;   // from the last 'v' readback, 0 = none yet
uint16_t vtxPowerMw;   // configured power from 'v', 0 = none yet
int8_t vtxTempC;       // from 's', -128 = unknown
uint8_t vtxStatus;     // VTX_STATUS_* bits
```

```c
constexpr uint8_t VTX_STATUS_ALIVE = 1 << 0;      // a valid reply within 2 s
constexpr uint8_t VTX_STATUS_PIT = 1 << 1;        // pit mode on (readback)
constexpr uint8_t VTX_STATUS_RACE_LOCK = 1 << 2;  // control mode != 0, sets disabled
constexpr uint8_t VTX_STATUS_REFUSED = 1 << 3;    // last requested change was not taken
```

`flags` has all eight bits taken, hence a separate status byte. Fields are
only added, names never change (the pandas column rule), `msgId` and
`version` stay first. `protocol.py` mirrors both structs, `PROTOCOL_VERSION`
goes to 8; old logs decode by their header as before and `flightlog.upgrade()`
stays identity (no semantics changed).

## cocpitView

- **Command state.** `espnow_source._send_commands` currently encodes
  neutral channels. It gets a small thread-safe `CommandState` (desired
  VTX freq/power now; the joystick channels will land in the same object
  later) that the UI thread writes and the sender thread reads each frame.
- **Operator control.** Frequency comes from configuration only — a
  channel change means retuning the receiver as well, so it is not a
  flight-time action: `[vtx] freq_mhz = 5800` in `esp32/local.ini`
  (read by `localconf.py`, optional `--vtx-freq` override), absent = never
  requested. Accepted values are the channels of bands A/B/E/F/R (the
  receiver covers 5645–5945 MHz; D, Low Race and X are outside it) and
  must lie inside the range the `r` reply announces; a band+channel form
  (`R4` = 5806) translated with the FPV doc §9 table is optional. Power is
  keyboard-driven: `V` one step **down**, `Shift+V` one
  step **up**, `P` toggles pit mode (leaving pit keeps the configured
  power). Letters, because `[`/`]` need Alt on a Czech layout and pygame
  reports letters the same on US and CZ layouts; the safe direction is on
  the bare key, the hot one needs the modifier. The steps are a table in
  cocpitView taken from the ULTRA 1600 manual — **pit, 25, 200, 400, 800,
  1600 mW** — Tramp carries power as a plain mW number and `r` reports only
  the maximum, not the list; the maximum is used to drop steps the module
  cannot do. Only listed values are ever sent (what the VTX does with an
  unlisted one is undocumented). The first keypress starts from the current
  readback, so the operator steps *from what the VTX is actually doing*,
  never from a stale default, and nothing is requested at all until a key
  is pressed. Stepping, not cycling: 1600 mW cannot be reached by one
  extra press from pit. Only the 25 mW step is EU-legal (§7 of the FPV
  doc, and with the 2.8 dBi antenna even that is ~48 mW EIRP).
- **HUD.** A `VTX 5800 25mW 42°C` line bottom-right above `BAT` (LNK
  moves up one more). `VTX PIT 42°C` in pit mode, `VTX ---` while not
  alive. The line blinks while the requested state differs from the
  readback for more than 2 s (a set is pending or was refused — the
  REFUSED bit makes it say `VTX REFUSED`). Race lock shows as `VTX LOCKED`.
- **Emulator** gains a fake VTX (5800 MHz, power follows the request
  after 0.5 s, temperature 40 °C + a little per power step) so the HUD and
  the key handling can be exercised without the airframe.
- **Replay** needs nothing: the new fields decode from the log header and
  the HUD shows the logged VTX state; the `UP` records carry what was
  requested, so a flight's power changes are reconstructible.
- **QGC**: no MAVLink message fits; a `STATUSTEXT` on each power change is
  a nice-to-have, not part of this.

## Bring-up (bench, pack powered, receiver on)

Procedure for every step: flash on USB (VTX unpowered — `VTX absent` is the
expected bench state), unplug USB, plug the pack, start cocpitView and read
`tail -f logs/<stem>.air.log`. The air board reboots on the power change,
so the probe-forever behaviour of step 1 under *Air side* is what makes the
VTX show up at all.

1. Pack in; expect `VTX present: ...` within a second and the `VTX rx r/v/s`
   hex dumps, then a readback line matching the LED-set state. Check the
   hex against the offset table above first — a wrong offset would make
   every later step lie.
2. Request 25 mW from cocpitView: red LED blink count, receiver RSSI and
   the readback must agree. Repeat for 200 mW.
3. Pit mode on and off: fixes the `I` polarity; note it in the code.
4. Frequency change to a neighbouring channel; retune the receiver;
   change back.
5. Temperature: plausible at rest, rising at 800 mW.
6. Power-cycle the pack: does the ULTRA keep a Tramp-set configuration or
   revert to the button-stored one? Either is fine for the design (the
   desired state re-applies), but the answer decides what the button
   setting should be left at — 25 mW or pit, so every boot starts legal.
7. A full cocpitView session into the `.cvlog`; check the new columns in
   `flightlog csv` and the HUD line in replay.

## Not in scope

Automatic power-up on takeoff (GPS speed or IAS gate), audio/OSD features
of the VTX, a MAVLink VTX message, in-flight frequency hopping.

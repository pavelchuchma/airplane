# 003 — Control the ULTRA 1600 VTX from cocpitView over IRC Tramp

Status: **open** (2026-10-26; plan reviewed against Betaflight's
`vtx_tramp.c` and marked ready 2026-11-05). Hardware done, firmware and
cocpitView not started. Design: `doc/plans/2026-10-25-vtx-tramp-control.md`
(read it first; this file is the entry point, the plan is the
specification).

## What exists

- The ULTRA 1600 (`doc/parts/fpv-58g-uvc-kit.md` §9) replaced the dead
  TS5823 and works on the bench since 2026-10-25.
- Its `IRC` pin (Tramp, single-wire half-duplex UART 9600 8N1) is soldered
  to the air ESP32: **GPIO26 = UART1 RX via 10 kΩ, GPIO27 = UART1 TX via
  1 kΩ**, ground common through the pack. Measured before wiring: 3.59 V
  idle, 0.31 V under 1 kΩ to GND → input with a ~10.6 kΩ pull-up to a 3.6 V
  rail. Why those two resistors: §9 of the FPV doc.
- Nothing in the firmware touches UART1 or those pins yet.

## What to build (summary of the plan)

1. **Air: `src/air/vtx.{h,cpp}`** — non-blocking Tramp state machine on
   `Serial1` (9600, RX 26, TX 27), polled from `loop()` like `gps::poll()`.
   `r` until answered (present), `v` every 500 ms, `s` every 2 s, one
   request in flight, 100 ms timeout, ≥ 200 ms between requests, own echo
   dropped by comparing with the request (plus the non-zero key-field
   check). Desired-state reconciliation from `CommandMessage`: send
   `F`/`P`/`I` only when the readback differs and only after the first
   valid `v`; one set per 500 ms, 10 answered-but-disagreeing tries then
   `VTX refused`, range checks from `r`, race lock = no sets, failsafe =
   no VTX action, nothing changed until cocpitView asks.
2. **Protocol v9** — `CommandMessage` + `vtxFreqMhz` (0 = leave) and
   `vtxPowerMw` (0 = pit, 0xFFFF = leave), 20 → 24 B. `TelemetryMessage` +
   `vtxFreqMhz`, `vtxPowerMw`, `vtxTempC`, `vtxStatus` (ALIVE / PIT /
   RACE_LOCK / REFUSED bits), 90 → 96 B. Mirror in `protocol.py`, bump
   `PROTOCOL_VERSION`, `flightlog.upgrade()` stays identity.
3. **cocpitView** — a thread-safe `CommandState` between the UI and the
   50 Hz sender (the joystick will use it later). Keys: `V` power one step
   down, `Shift+V` one step up, `P` pit toggle; steps are the table
   pit / 25 / 200 / 400 / 800 / 1600 mW from the manual, first keypress
   starts from the readback. Frequency from `esp32/local.ini` `[vtx]
   freq_mhz = …` (+ `--vtx-freq`), never a key — the receiver has to be
   retuned too. HUD line `VTX 5800 25mW 42°C` above `BAT`, blinking while
   requested ≠ readback, `VTX PIT` / `VTX ---` / `VTX REFUSED` / `VTX
   LOCKED`. Emulator gets a fake VTX.
4. **Diag** — present/absent/alive transitions, every readback change,
   every set and rejection; plus hex dumps of the first `r`/`v`/`s` reply
   per boot and of malformed frames (1 per 5 s).

## Constraints to keep in mind

- **Bench: logic jumper open, USB and pack together.** USB powers the
  ESP32, the pack powers the VTX, the console is live (jumper story in
  `CLAUDE.md`). USB alone leaves the VTX unpowered — `VTX absent` is
  normal then. In the field the same `#` lines arrive in
  `logs/<stem>.air.log`.
- The Tramp payload offsets in the plan match Betaflight's `vtx_tramp.c`
  source (checked 2026-11-05) — still confirm them against the ULTRA's hex
  dumps at bring-up. Pit mode is inverted between the `I` command (0 =
  enter) and the `v` readback (1 = on) in Betaflight; step 3 of the plan
  confirms the ULTRA does the same.
- Only the 25 mW step is EU-legal (FPV doc §7); with the 2.8 dBi antenna
  even that is ~48 mW EIRP. The point of the control line is to make low
  power on the ground the default and higher power a deliberate act.
- Wire-protocol rule: both `messages.h` and `protocol.py` in one commit,
  version bumped, fields only added, `msgId`/`version` stay first.

## Open decisions (defaults if nobody objects)

- Pit as `vtxPowerMw = 0` rather than a flag bit in `CommandMessage.flags`
  — default: value 0.
- Frequency notation in `local.ini`: plain MHz, optionally also band +
  channel (`R4` = 5806) translated with the table in FPV doc §9 — default:
  MHz only, band+channel if cheap.
- One protocol bump (v9) for command and telemetry together — default: yes.
- Does the ULTRA persist a Tramp-set configuration across power cycles?
  Unknown; bring-up step 6. Either way the desired state re-applies, the
  answer only decides what the button setting is left at (25 mW or pit).

## Done when

- A battery session shows `VTX present`, matching readback, and power
  steps changed from the keyboard confirmed by the red LED blink count and
  the receiver's RSSI; pit mode on/off works; a frequency change from
  `local.ini` is taken and the receiver re-finds the picture after
  auto-search.
- The new columns appear in `flightlog csv`, the HUD line shows in replay,
  the `.air.log` has the hex dumps and no `malformed` lines on a clean
  session.
- `CLAUDE.md` current-state and FPV doc §9 updated; the plan's status
  flipped to implemented with the verified byte offsets and the pit
  polarity written in.

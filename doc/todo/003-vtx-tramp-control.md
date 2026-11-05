# 003 — Control the ULTRA 1600 VTX from cocpitView over IRC Tramp

Status: **done — implemented and brought up, all 7 steps passed**
(2026-11-06; firmware `esp32/src/air/vtx.cpp`, cocpitView `command.py` +
`vtx.py`, protocol v9/v10; the VTX flies on R2 = 5695 MHz from
`local.ini`; frequency and power both persist across a power cycle). Design: `doc/plans/2026-10-25-vtx-tramp-control.md`;
results below under *Bring-up log*.

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

## Bring-up log

- **2026-11-06, step 1 passed** (USB + pack, jumper open, air board in the
  airframe). First boot with the v9 firmware, verbatim from the console:

  ```
  # VTX rx r: 0F 72 7E 13 37 18 C4 09 00 00 00 00 00 00 1F 00
  # VTX present: 4990-6199 MHz, max 2500 mW
  # VTX rx v: 0F 76 A8 16 40 06 00 00 40 06 00 00 00 00 C0 00
  # VTX 5800 MHz 1600 mW (actual 1600) pit=0 lock=0 -128 C
  # VTX rx s: 0F 73 00 00 00 00 58 00 00 00 00 00 00 00 CB 00
  ```

  Offsets confirmed against the plan's table: `r` min/max/maxpower at
  [2-3]/[4-5]/[6-7] (the module advertises 4990–6199 MHz and 2500 mW —
  more than the manual's 1600 mW; the cocpitView table stays at the
  manual), `v` freq 5800 at [2-3], configured 1600 mW at [4-5], control 0,
  pit 0, actual 1600 mW at [8-9]; `s` temperature at [6-7] = 0x58 =
  **88 °C** with the VTX button-set to 1600 mW. No echo leaked through
  (no `bad` lines), checksums verified by hand.
- **Same evening, steps 2, 3, 6 and 7 passed** (`logs/2026-11-06_001342.air.log`
  / `.cvlog`). Every `P` set was taken on the first try, the follow-up `v`
  confirmed it ~250 ms after the set: 1600 → 800 → 1600 → 800 → 400 → 200
  → 25 mW from the keys. `VTX set pit on` → readback `pit=1`, `set pit
  off` → `pit=0`: **`I` 0 = enter, `v` byte 7 = 1 in pit — Betaflight's
  polarity holds on the ULTRA**. Power-cycling the pack (and USB) brought
  the VTX back at **25 mW: a Tramp-set configuration persists**, so the
  button setting is irrelevant once set once — but the module *was*
  delivered at 1600 mW, which is what any new unit will boot at. The
  `.cvlog` has the `vtx_*` columns in `flightlog csv`, and they carry a
  real temperature trail: 88 °C at 1600 mW, cooling to 64 °C within
  ~50 s of dropping to 25 mW. The first `s` right after the VTX's own
  power-up reads 0 (= "no reading yet", filtered like an echo); the
  firmware now logs `VTX temp N C at P mW` per 5 °C change.
- **Step 4 passed** (`logs/2026-11-06_002256.air.log`): `[vtx] freq = R2`
  in `local.ini`, cocpitView restarted → `VTX set freq 5695` 1.6 s after
  the air boot, readback `VTX 5695 MHz 25 mW (actual 25)` 243 ms later,
  the receiver re-found the picture after auto-search. Idle temperature
  at 25 mW in the open airframe on the bench: 56–62 °C.
- **Step 5 passed** (`logs/2026-11-06_003626.air.log` / `.cvlog`, module
  loose on the bench, still air — the VTX will be mounted outside the
  airframe in the slipstream, so this is the worst case, not the closed
  fuselage the plan assumed): 800 mW settles at **~75 °C** (τ ≈ 2.2 min),
  1600 mW at **~88 °C**, no self-throttling up to there; the HUD alarm
  (85 °C) blinked as designed and cleared ~30 s after stepping down;
  cooling 87 → 78 °C in 100 s at 25 mW. Pit mode range ≈ 30 cm (weak RF
  on the same channel). Numbers and conclusions in
  `doc/parts/fpv-58g-uvc-kit.md` §9 *Thermal behaviour*. Also seen in
  that session: one garbled `v` echo (a bus collision, dropped as `bad`,
  harmless), and two ~0.8 s stalls of the **ground-side** path at
  boot 274–286 s (PC telemetry loss 33–62 %, air `cmd_loss` 64 %, RSSI
  steady −75 dBm, `txFail` 0 % — radio ACKs fine, so the ground bridge /
  USB / PC hiccupped, not RF and not the VTX; unexplained — a burst of
  five more 0.1–2.7 s gaps followed at boot 1248–1264 s while the
  airframe was being handled for the power cycle).
- **Step 6 repeated with the frequency set**: after the pack (and USB)
  power cycle the VTX came back on **5695 MHz / 25 mW** — both Tramp-set
  values persist. Bring-up complete.

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

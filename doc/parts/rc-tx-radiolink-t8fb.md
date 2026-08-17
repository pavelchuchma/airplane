# RadioLink T8FB (BT) — 2.4 GHz FHSS 8-Channel Transmitter

![RadioLink T8FB BT transmitter](img/rc-tx-radiolink-t8fb.jpg)

- **Source:** comes with the [Beta V2 1400 donor airframe](airframe-beta-v2-1400.md)
  (RTF kit; our unit is the **BT variant, Mode 1**), paired with the
  [R8EF receiver](rc-rx-radiolink-r8ef.md)
- **What it is:** the stock RC transmitter of the donor model — **replaced by
  the ESP-NOW link** in this project, kept as reference/fallback gear
- **Manual:** [T8FB instruction manual (PDF)](https://d3if9wubzr0anm.cloudfront.net/manuals/RLKT081000-1.pdf)
  ([BT-variant manual](https://modellflybutikken.no/Userfiles/Upload/files/RL%20T8FB%20BT%20manual.pdf))

## Specs

| Parameter | Value |
|---|---|
| Channels | 8 |
| RF | 2.4 GHz ISM (2400.0–2483.5 MHz), **FHSS over 67 channels**, pseudo-random hopping |
| Control range | ~2 km air (R8EF product page says 500 m ground / 1000 m air — believe the smaller numbers) |
| Power | JST connector: 4× AA or **2S–4S LiPo**; low-voltage alarm auto-detects pack (3S default 11.0 V) |
| Operating current | < 80 mA |
| Compatible receivers | R8EF, R8FM (mini) |
| Controls | 2 sticks, 1 two-way switch (SwA), 1 three-way switch (SwB), 2 pots (VrA/VrB), 4 trims |
| Default aux mapping | SwB → CH5, VrB → CH6, SwA → CH7, VrA → CH8 |
| Mode 1 (our unit) | left stick = rudder + elevator, right stick = aileron + throttle |

## Configuration — no onboard UI

The T8FB has no screen; all setup happens over its data port (or Bluetooth on
the BT variant) using the **RadiolinkT8FB Android app** (OTG cable; no
iPhone) or **Windows software** (enumerates as a COM port). Parameters:

- **BASIC** (per channel): REV, SUB-trim (−120…+120), EPA-L/EPA-R endpoints
  (default 96), **F/S failsafe**, DELAY
- **ADVANCED:** D/R on a switch, ATTITUDE (6 flight-mode presets on one
  channel — for Radiolink PIX FCs), ELEVON and V-TAIL mixes
- **SYSTEM:** aux channel ↔ switch assignment, TX low-voltage alarm,
  stick mode, 2 programmable mixes
- The app's SERVO screen shows **live CH1–CH8 positions** (−100…+100) and TX
  battery voltage while connected

### Failsafe is configured here, not in the receiver

Per-channel F/S in the BASIC menu defines what the R8EF outputs on signal
loss (or low RX voltage): `NOR` = hold last commanded position, `F/S` = move
to a preset percentage (0 = throttle low, 50 = center). Throttle F/S also
triggers on low RX battery. This is the authoritative answer to "what do the
servos do when the TX disappears" — set it deliberately (throttle F/S = 0
at minimum) and verify on the bench.

## The data port (back of the case)

One port, labeled **"trainer cable / simulator / firmware update"** in the
manual (micro-USB-shaped on our unit; the supplied "Android USB cable" is a
serial adapter, not USB data):

- **Firmware upgrade:** Windows tool, connect as COM port, flash APROM hex.
- **Parameter setup:** the phone app talks to the same port through an OTG
  serial link ("Serial port settings successfully" — it is a UART stream).
- **Simulator/trainer:** officially via Radiolink dongle or an R16F/R12F
  receiver with built-in USB-HID simulator mode.

**Project-relevant observation:** since the setup app renders live stick
positions, the T8FB *does* stream channel data out of this port. The protocol
is undocumented — using the T8FB as a direct-wire input device (sticks →
serial → ground station, no RF) would require sniffing the app ↔ TX traffic
first. The documented, zero-reverse-engineering way to read the sticks
remains over the air: T8FB → R8EF in SBUS mode (see the
[R8EF card](rc-rx-radiolink-r8ef.md)).

## Calibration (sticks)

With the TX off: sticks centered, hold **left rudder trim + power button** →
red+green LEDs flash. Move both sticks to all extremes and back to center,
then press **right rudder trim**; both LEDs solid = calibrated.

## Gotchas

1. **2.4 GHz coexistence with the ESP-NOW link:** FHSS hops across the whole
   band, so it will periodically land on ESP-NOW channel 1. Expect a small
   extra frame-loss rate when both links run simultaneously — watch the
   `U%/D%` HUD stats; both links tolerate isolated losses by design.
2. Factory default EPA 96 % and possible REV settings live in the TX — when
   comparing R8EF SBUS/PWM readings against cocpitView commands, remember the
   TX applies trim/EPA/REV/mixes *before* transmission; raw stick ≠ channel
   output.
3. No iPhone support for the setup app; Android 4.0+ with OTG (some phones
   need OTG enabled in settings), or the Windows COM tool.
4. TX battery: the low-voltage alarm threshold auto-detects the pack type but
   is worth checking after changing packs (2S default 7.3 V, 3S 11.0 V,
   4×AA 5.0 V).

## Sources

- [T8FB instruction manual (PDF)](https://d3if9wubzr0anm.cloudfront.net/manuals/RLKT081000-1.pdf)
- [T8FB BT manual (PDF, modellflybutikken.no mirror)](https://modellflybutikken.no/Userfiles/Upload/files/RL%20T8FB%20BT%20manual.pdf)
- [radiolink.com — T8FB manual page (specs)](https://www.radiolink.com/t8fb_manual)
- [radiolink.com — T8S FAQ (simulator dongle / R16F-R12F USB-sim note)](https://www.radiolink.com/t8s_faq)

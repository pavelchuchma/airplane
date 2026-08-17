# RadioLink R8EF — 2.4 GHz FHSS 8-Channel Receiver (SBUS/PPM/PWM)

![RadioLink R8EF receiver](img/rc-rx-radiolink-r8ef.jpg)

- **Source:** comes installed in the [Beta V2 1400 donor airframe](airframe-beta-v2-1400.md)
  (RTF kit, paired with the T8FB BT transmitter); sold separately as
  [Přijímač R8EF](https://www.rcprofi.cz/prijimac-r8ef) — rcprofi.cz
- **What it is:** the stock RC receiver of the donor model — **replaced by the
  air-side ESP32** in this project, kept as reference/fallback gear while the
  ESP-NOW link is being proven
- **Manual:** [R8EF user manual (PDF)](https://d3if9wubzr0anm.cloudfront.net/manuals/RLKR081005-1.pdf)

## Specs (from the manual)

| Parameter | Value |
|---|---|
| Channels | 8 |
| RF | 2.4 GHz FHSS |
| Working voltage | **4.8–10 V** (case print: 4.8–10V/DC) |
| Working current | 30 mA @ 5 V (without servos) |
| Servo resolution | 4096 steps, 0.5 µs per step |
| Range | ~2000 m air per manual; rcprofi lists 500 m ground / 1000 m air |
| Size / weight | 48.5 × 21 × 11 mm / 7 g |
| Compatible transmitters | T8FB, T8S, RC4GS V2, RC6GS V2 (newer listings add T12D, T16D, RC8X…) |

## Two working modes (LED color = current mode)

| Mode | LED | Outputs |
|---|---|---|
| PWM | **red** | 8× PWM: CH1 aileron, CH2 elevator, CH3 throttle, CH4 rudder, CH5–8 aux |
| S-BUS&PPM | **blue/purple** | **CH1 = SBUS, CH2 = PPM, CH3–CH8 = PWM** — all simultaneously |

- **Mode switch:** quick-press the bind (ID SET) button **twice within 1 s**.
- SBUS mode costs nothing but the CH1/CH2 PWM outputs — this model uses 4
  servo functions, which still fit on CH3–CH8 if PWM is ever needed alongside
  SBUS.

## Pinout

Standard UNI/JR servo headers, one 3-pin column per channel: signal (top),
VCC (+, middle), GND (−, bottom) — marked `⎍ + −` on the case. Any channel's
+/− pair powers the whole receiver. Anti-polarity protection: a reversed servo
cable stops the receiver from working but doesn't damage it (a reversed
*battery* damages connected servos, not the receiver).

## Binding

1. Transmitter and receiver within 1 m.
2. Turn on the transmitter, then power the R8EF.
3. Hold the black bind button ~1 s until the LED blinks, release; after ~8
   blinks the LED goes solid = bound.

## Connection — ESP32 (reading the receiver as a command source)

Prefer **SBUS over per-channel PWM**: one pin instead of four, all 8 channels
in every frame, and the flag byte carries the receiver's own **frame-lost and
failsafe bits** — the link-health signal the per-channel PWM outputs can't
provide. SBUS is an inverted UART: 100 000 baud, 8E2, 25-byte frames
(16 channels + flags), one frame every ~14 ms.

Two hardware details:

1. **Power the R8EF from the 5 V servo BEC** (4.8–10 V range; 30 mA — noise
   in the ESP32's own supply is not a concern at that draw). Common ground
   with the ESP32.
2. **Signal level is unspecified in the manual** and the ESP32 is not 5 V
   tolerant. Don't rely on a multimeter reading of an idle line — put a
   one-transistor inverter in the signal path; it both level-shifts safely
   and un-inverts SBUS:

```
R8EF CH1 (SBUS) ──10k──┤B  NPN (BC547/2N3904)
                         E ── GND
                         C ──┬── ESP32 UART1 RX pin
                             └──10k── 3V3
```

```cpp
// after the transistor the signal is non-inverted 3.3 V:
Serial1.begin(100000, SERIAL_8E2, rxPin, -1, false);
// direct wire (only if the output is verified 3.3 V on a scope):
// Serial1.begin(100000, SERIAL_8E2, rxPin, -1, true);  // HW invert
```

UART budget on the air-side ESP32: UART0 = USB diagnostics, UART2 = GPS →
**UART1 is free** for SBUS (any spare GPIO via the pin matrix).

## Gotchas

1. **Failsafe on TX signal loss is configured from the transmitter**, not in
   the receiver: the [T8FB's](rc-tx-radiolink-t8fb.md) BASIC menu sets each
   channel to `NOR` (hold last position) or `F/S` (move to a preset
   percentage; throttle F/S also triggers on low RX voltage). What the SBUS
   *failsafe/frame-lost bits* do is still undocumented — bench-test (power
   the RX, kill the TX, watch the frames) before any override/arbitration
   logic trusts them.
2. Mode is toggled by the same button used for binding — a sloppy bind
   attempt can flip the output mode. Check the LED color after any button
   press: red = PWM, blue/purple = SBUS.
3. Working voltage floor is **4.8 V** — it cannot run from the ESP32's 3.3 V
   rail (and shouldn't share it anyway; it belongs on the servo BEC).
4. Antenna: keep the active tip straight, ≥ 0.5″ from metal/carbon, away from
   motor/ESC; the coax part may be routed freely but not bent tightly.

## Sources

- [R8EF user manual (PDF)](https://d3if9wubzr0anm.cloudfront.net/manuals/RLKR081005-1.pdf)
- [RadioLink R8EF product page](https://www.radiolink.com/r8ef)
- [RadioLink R8EF FAQ](https://www.radiolink.com/r8ef_faq)
- [rcprofi.cz — Přijímač R8EF](https://www.rcprofi.cz/prijimac-r8ef)

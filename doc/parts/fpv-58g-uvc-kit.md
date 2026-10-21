# 5.8 GHz Analog FPV Kit — UVC Receiver + VTX + Camera

![5.8G FPV UVC kit](img/fpv-58g-uvc-kit.jpg)

- **Ordered:** 1× variant **"200mW version"** (AliExpress, Mar 2023)
- **Product:** [Ready to Use 5.8G FPV UVC Receiver + 200/600mW Transmitter + CMOS 1200TVL Camera](https://www.aliexpress.com/item/1005004766486947.html) — RC Market Store
- **What it is:** complete analog FPV chain: mini CVBS camera → 5.8 GHz video transmitter (TS58xx family) → UVC receiver dongle (ROTG01 class) that plugs into an Android phone via OTG
- **Brand / models (seller sheet, 2026-10-21):** the whole kit is **EWRF**-branded — receiver "EWRF 5.8G OTG RECEIVER", VTX **EWRF TS5823** (board silkscreen `TS5823Pro_V10`), camera "1200TVL Mini Camera". Box contents: receiver + 5 dBi whip, VTX + 2 dBi whip, camera with tilt bracket, lens cap, screws and hex key, two 3-pin camera leads (JST↔JST and JST↔servo-style Dupont), the VTX's 6-pin JST pigtail, and two OTG cables (micro-USB↔micro-USB, micro-USB↔USB-C).

## 1. UVC receiver dongle (ROTG01 class)

| Parameter | Value |
|---|---|
| RF | 5.8 GHz, 5645–5945 MHz, **150 channels** (bands A/B/E/F/R + low), sensitivity −90 dBm |
| Output | UVC (standard webcam device); digitized **640×480 @ 25–30 fps** |
| Power | from phone, ~200 mA @ 5 V |
| Android | OTG plug-and-play; apps: **Go FPV**, UVC FPV, RotG (DVR recording, VR split-screen) |
| PC | works as a normal UVC webcam (VLC, OBS) |
| iOS | **not supported** |
| Antenna | RP-SMA female, ~5 dBi whip included |
| Button | short press = next channel; **long press = auto-search** (locks strongest signal) |

## 2. Video transmitter (TS5823-family, 200 mW variant)

| Parameter | Value |
|---|---|
| RF | 5.8 GHz, **40 CH** (5 bands × 8: A/B/E/F/R), FM analog |
| Output power | **200 mW (~23 dBm)** — this ordered variant; 600 mW version exists |
| Supply | **DC 7–24 V** (2S–4S LiPo direct; 3S recommended) — NOT the 5 V whoop type. The 24 V is the input buck's own rating with no margin (abs. max 26 V, see §8) |
| Current | seller sheet at 12 V: **150 mA @ 200 mW, 230 mA @ 600 mW** — measured with camera: see §5 |
| Video in | CVBS 0.8–1.2 Vp-p / 75 Ω, PAL & NTSC transparent |
| Antenna | RP-SMA female, ~2 dBi whip |
| Channel select | one push-button + a single 7-segment digit showing band letter / channel number (table below) |
| Connector | 6-pin JST pigtail — pinout not traced yet (pack in, camera power out, video in, audio by convention) |
| Size/weight | 30×20×8 mm without the antenna mount, 5 g (seller sheet) |

![TS5823Pro_V10 board](img/fpv-58g-vtx-board.jpg)

Frequency table as printed by the seller ([image](img/fpv-58g-vtx-spec.jpg)); the
board's digit shows the letter in the first column, which maps onto the usual
band names as noted:

| Digit | Band | CH1 | CH2 | CH3 | CH4 | CH5 | CH6 | CH7 | CH8 |
|---|---|---|---|---|---|---|---|---|---|
| A | A (Boscam A) | 5865 | 5845 | 5825 | 5805 | 5785 | 5765 | 5745 | 5725 |
| b | B (Boscam B) | 5733 | 5752 | 5771 | 5790 | 5809 | 5828 | 5847 | 5866 |
| C | E (Boscam E) | 5705 | 5685 | 5665 | 5645 | 5885 | 5905 | 5925 | 5945 |
| d | F (FatShark / IRC) | 5740 | 5760 | 5780 | 5800 | 5820 | 5840 | 5860 | 5880 |
| E | R (Raceband) | 5658 | 5695 | 5732 | 5769 | 5806 | 5843 | 5880 | 5917 |

All in MHz. The receiver covers 5645–5945 MHz, so every row is reachable; its
auto-search finds the channel, the table only matters when setting the VTX
by hand.

⚠️ **Never power up without the antenna attached** — can destroy the PA stage. The 600 mW boards overheat without airflow; at 200 mW heating is milder but still mount with airflow (confirmed by buyer reviews).

## 3. Camera (generic "1200TVL" mini CMOS)

![1200TVL mini camera with bracket and leads](img/fpv-58g-camera.jpg)

Seller's sheet (2026-10-21) unless noted:

| Parameter | Value |
|---|---|
| Sensor | 1/3" color CMOS; PAL 976H×582V, NTSC 976H×494V effective pixels |
| Real resolution | the sheet itself says **1000TVL** horizontal; "1200TVL" is the listing name. Analog CVBS tops out ~500–600 TVL and the UVC dongle resamples to 640×480 anyway |
| Output | CVBS 1.0 Vp-p / 75 Ω, PAL (NTSC optional per SKU), internal sync |
| Image processing | AGC 4 levels (0.25/0.50/0.75/1.00, max 55 dB), WB on/off, electronic shutter 1/50 (1/60) – 1/100000 s, gamma 0.45/1.0 — none of it user-accessible (no OSD/menu) |
| Lens / FOV | sheet offers **2.5 mm / 130°** or **2.8 mm / 120°**; which one shipped is not recorded (an earlier note here said 2.1 mm / 125°, not from the sheet) — read the lens barrel |
| Supply | **DC 5–12 V** (seller: measured to 13 V), 70 mA — fed from the VTX's camera header, not from the pack directly. Measured 66 mA at 4.3 V (2026-10-21). A full 3S pack (12.6 V) sits at the top of its range |
| Operating temp. | −20 … +60 °C |
| Connector | 3-pin JST (VCC, GND, video); the kit's JST↔JST lead goes to the VTX camera header, the JST↔Dupont lead is for a separate supply / recorder |
| Size/weight | 22×22 mm, ~12 g with bracket |

## 4. Wiring

```
3S pack (9–12.6 V) ── power distribution board ──┬─ DC/DC 5 V → ESP32 & co.
                                                 └─ VTX VCC (7–24 V) + GND
VTX camera header (6-pin ribbon) → camera VCC + GND + video-in
VTX antenna: RP-SMA whip — ALWAYS attach before power-up
```

The camera hangs on the VTX's camera header, so it shares whatever rail the
VTX gives it there (pack pass-through or the internal buck output — not yet
traced; it saw the pack voltage when the buck failed, see §8).

**One connection point only.** The VTX stays permanently wired to the
distribution board and the pack connector is the only thing that gets
plugged in or out. Never connect or disconnect the VTX while the pack is
live — that killed one (§8). If a separately pluggable VTX is ever needed,
put a 220 µF/25 V electrolytic (ordinary ESR — it is the damper) plus an
SMBJ15A TVS right at the VTX input.

Ground side: receiver dongle → OTG cable → Android phone (Go FPV app), or USB → PC as webcam. No pairing — power the VTX, long-press the RX button to auto-search.

## 5. Power draw — measured (Aug 2026)

- Whole chain (camera + 200 mW VTX) straight from a 3S pack: **312 mA @ 12 V
  ≈ 3.7 W**. Against the seller sheet (150 mA @ 200 mW, 230 mA @ 600 mW, plus
  70 mA camera) that is 90 mA above the 200 mW figure and 12 mA above the
  600 mW one — either the sheet is optimistic or the unit runs at 600 mW
  despite the "200mW version" order. Not resolved; a replacement unit should
  be measured alone (no camera) before it is trusted.
- The chain is a **constant-power load** — the VTX derives its internal rails
  with switching regulators, not a linear one. Verified by feeding it through
  a Mini-360 buck at 7.5 V: battery draw *rose* to 350 mA @ 12 V (the ~0.45 W
  difference is the converter's own loss, and the module runs warm).
  **Do not put a step-down converter in front of it** — power camera and VTX
  directly from the pack.
- The VTX heatsink heat is RF power-amplifier dissipation (~25–35 % PA
  efficiency → roughly 1–1.5 W of heat at 200 mW out) and does not depend on
  supply voltage. Mount with airflow; a lower rail changes nothing.
- Motor at full throttle produced no visible interference in the video,
  with or without the buck (bench test — recheck once installed in the
  airframe).

## 6. Latency

- Pure analog chain (camera → VTX → RX): < 40 ms
- UVC digitization + USB + Android rendering adds ~50–100 ms → **~100 ms glass-to-glass** total
- Fine for line-of-sight/camera-plane use; not for proximity/racing flying

## 7. Legal note (CZ/EU)

License-free 5.8 GHz analog video is limited to **25 mW EIRP** (ERC Rec 70-03). Both 200 mW and 600 mW settings **exceed the limit** — transmitting legally requires a HAM (amateur radio) license, and these generic VTXs have no valid CE/RED certification. Use accordingly (low power indoors/test bench, or get the license).

## 8. Failure 2026-10-20 — input buck destroyed by hot-plug

The VTX died the first time it was plugged **onto an already live pack**
(the DC/DC for the ESP32 was running). Every earlier session had connected
the pack to the distribution board with the VTX already wired, and that
worked for over a year. Same wiring, no reverse polarity; the chip
vaporized on contact.

**What burned:** the SOT-23-6 controller of the first of two identical
step-down cells on the board (`TS5823Pro_V10`; each cell = SOT-23-6 IC +
SS34/SK34 Schottky + shielded inductor + input tantalum — an asynchronous
buck). The intact unit carries a Richtek-style top mark (`D0S=93`, the `=`
is Richtek's format); the exact code could not be resolved (Richtek does
not publish marking tables), but the circuit matches the Richtek
RT8258/RT8259 class: 4.5–24 V in, 1.2 A, **absolute maximum VIN 26 V**,
pinout 1 BOOT, 2 GND, 3 FB (0.8 V ref), 4 EN, 5 VIN, 6 PHASE. Both parts
are EOL.

**Why it died:** a stiff source (pack node held low-impedance by the ESP
buck's charged input capacitors) plus the VTX's short lead and ceramic
input caps make an undamped LC edge that rings toward 2 × V_in — 24–25 V
from a full pack — against a 26 V absolute maximum with no TVS on the
board. Plugging the pack into the whole harness instead puts the ESP
buck's electrolytic on the same node and damps the ring. The "7–24 V" on
the box is the chip's rating, not a margin.

**Collateral:** the camera survived (66 mA at 4.3 V, spec 70 mA), which is
consistent with it seeing the pack voltage through the shorted high-side
switch and tolerating 13 V. Whether anything behind the dead buck (second
buck, MCU, RF) survived is unknown until tested.

**Repair options, in order of preference:**

1. Replace the SOT-23-6 (new RT8259 or a pin-compatible part with a higher
   rating — verify pinout against the board: pin 5 to the input cap, pin 6
   to the Schottky cathode + inductor, pin 1 via a small ceramic to PHASE).
   Check the Schottky and inductor of that cell first.
2. Bypass the cell: remove the dead IC, read the rail voltage off the FB
   divider (`Vout = 0.8 × (1 + R_top/R_bottom)`), ohm-check the output node
   to GND, then feed it from a current-limited bench supply to prove the
   rest of the board lives. If it does, a small MP1584/Mini-360 module next
   to the VTX, set to that rail and fed from the pack, replaces the cell
   (it replaces, not cascades, so the constant-power argument in §5 does
   not apply). Sharing the ESP32's 5 V rail is a bench test only — video
   would share digital supply and ground.
3. New module, protected by the single-connection-point rule above.

## Sources

- [Eachine ROTG01 receiver](https://www.eachine.com/Eachine-ROTG01-UVC-OTG-5.8G-150CH-Full-Channel-FPV-Receiver-For-Android-Mobile-Phone-Smartphone-p-843.html)
- [USB OTG FPV receivers overview (BrushlessWhoop)](https://brushlesswhoop.com/usb-otg-fpv-receivers/)
- [TS5823 200 mW VTX review (Oscar Liang)](https://oscarliang.com/ts5823-200mw-video-transmitter/)
- [Foxtech TS5828 spec](https://dev.foxtechfpv.com/58g-600mw-48channel-vtx.html)
- [VTX power limits by country](https://blog.uavmodel.com/vtx-power-limits-by-country-fcc-ce-and-international-fpv-compliance-2026-guide/)

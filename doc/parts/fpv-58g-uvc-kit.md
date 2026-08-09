# 5.8 GHz Analog FPV Kit — UVC Receiver + VTX + Camera

![5.8G FPV UVC kit](img/fpv-58g-uvc-kit.jpg)

- **Ordered:** 1× variant **"200mW version"**, AliExpress (Mar 20, 2023, completed)
- **Product:** [Ready to Use 5.8G FPV UVC Receiver + 200/600mW Transmitter + CMOS 1200TVL Camera](https://www.aliexpress.com/item/1005004766486947.html) — RC Market Store
- **What it is:** complete analog FPV chain: mini CVBS camera → 5.8 GHz video transmitter (TS58xx family) → UVC receiver dongle (ROTG01 class) that plugs into an Android phone via OTG

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
| Supply | **DC 7–24 V** (2S–4S LiPo direct; 3S recommended) — NOT the 5 V whoop type |
| Current | ~150 mA @ 200 mW (spec) — measured with camera: see §5 |
| Video in | CVBS 0.8–1.2 Vp-p / 75 Ω, PAL & NTSC transparent |
| Antenna | RP-SMA female, ~2 dBi whip |
| Channel select | push-button + LEDs (frequency table printed on board) |
| Size/weight | ~30×20×8 mm, 5–7 g without antenna |

⚠️ **Never power up without the antenna attached** — can destroy the PA stage. The 600 mW boards overheat without airflow; at 200 mW heating is milder but still mount with airflow (confirmed by buyer reviews).

## 3. Camera (generic "1200TVL" mini CMOS)

| Parameter | Value |
|---|---|
| Sensor | 1/3" color CMOS |
| Real resolution | "1200TVL" is marketing — analog CVBS tops out ~500–600 TVL, and the UVC dongle resamples to 640×480 anyway |
| Output | CVBS 1.0 Vp-p / 75 Ω, PAL or NTSC (fixed per SKU) |
| Lens / FOV | 2.1 mm, ~125° horizontal |
| Supply | wide input **DC 5–40 V** (~100–135 mA) — fed from the same battery lead as the VTX |
| Size/weight | ~22×22 mm, ~12 g |

## 4. Wiring

```
LiPo 2S–4S (7–16.8 V)
  ├─ red   → VTX VCC (7–24 V)
  ├─ red   → camera VCC (wide 5–40 V input, parallel to VTX)
  └─ black → VTX GND + camera GND (common ground)

camera video-out (yellow) → VTX video-in
VTX antenna: RP-SMA whip — ALWAYS attach before power-up
```

Ground side: receiver dongle → OTG cable → Android phone (Go FPV app), or USB → PC as webcam. No pairing — power the VTX, long-press the RX button to auto-search.

## 5. Power draw — measured (Aug 2026)

- Whole chain (camera + 200 mW VTX) straight from a 3S pack: **312 mA @ 12 V
  ≈ 3.7 W**.
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

## Sources

- [Eachine ROTG01 receiver](https://www.eachine.com/Eachine-ROTG01-UVC-OTG-5.8G-150CH-Full-Channel-FPV-Receiver-For-Android-Mobile-Phone-Smartphone-p-843.html)
- [USB OTG FPV receivers overview (BrushlessWhoop)](https://brushlesswhoop.com/usb-otg-fpv-receivers/)
- [TS5823 200 mW VTX review (Oscar Liang)](https://oscarliang.com/ts5823-200mw-video-transmitter/)
- [Foxtech TS5828 spec](https://dev.foxtechfpv.com/58g-600mw-48channel-vtx.html)
- [VTX power limits by country](https://blog.uavmodel.com/vtx-power-limits-by-country-fcc-ce-and-international-fpv-compliance-2026-guide/)

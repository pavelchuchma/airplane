# ESP32 Development Board — WROOM-32U (external antenna)

![ESP32-WROOM-32U DevKit board](img/esp32-wroom-32u.png)

- **Ordered:** 2× "type 1set" (board + 2.4G antenna with IPEX→SMA pigtail), $7.40/pc (AliExpress, Jul 19, 2026)
- **Product:** [MICRO-WROOM-32U WROVER Module WIFI Module with 2.4G Antenna](https://www.aliexpress.com/item/1005007329234480.html) — Good Luck Electronic Products Store
- **What it actually is:** generic **ESP32-DevKitC-style board with an ESP32-WROOM-32U module** ("WROVER" in the title is seller keyword spam). Equivalent to Espressif ESP32-DevKitC-32U.

## Module specs (ESP32-WROOM-32U, Espressif datasheet v2.6)

| Parameter | Value |
|---|---|
| SoC | ESP32-D0WD-V3, dual-core Xtensa LX6 @ up to 240 MHz |
| SRAM / ROM | 520 KB SRAM (+2×8 KB RTC) / 448 KB ROM |
| Flash | 4 MB SPI (standard on these boards) |
| Wi-Fi | 802.11 b/g/n, 2.4 GHz only, up to 150 Mbps |
| Bluetooth | v4.2 BR/EDR + BLE |
| Antenna | **U.FL/IPEX (MHF-I) connector only — no PCB antenna** |
| Operating voltage | 3.0–3.6 V (3.3 V nominal) |
| ADC / DAC | 2× 12-bit SAR ADC (18 ch) / 2× 8-bit DAC (GPIO25/26) |
| Temp range | −40…+85 °C |

Note: WROOM-32U is marked NRND by Espressif (successor WROOM-32UE); clones still ship it.

## Board (38-pin DevKitC clone)

- USB-UART: CP2102 (some batches CH340C — check chip marking); auto-reset circuit for flashing
- Regulator: AMS1117-3.3 (practical 3.3 V budget for external loads ~400–600 mA)
- Buttons: EN (reset), BOOT (GPIO0); on-board LED on GPIO2

## Pinout — key groups

| Group | Pins | Notes |
|---|---|---|
| Input-only | GPIO34, 35, 36 (VP), 39 (VN) | no pull-ups, no output; ADC1 inputs |
| Strapping pins | GPIO0, 2, 5, 12 (MTDI!), 15 | mind boot-time levels; GPIO12 must be LOW at boot |
| **Never use** | GPIO6–11 | wired to internal SPI flash |
| I2C (default) | **GPIO21 = SDA, GPIO22 = SCL** | remappable |
| UART0 | GPIO1/3 | USB console — avoid |
| UART2 | **GPIO17 = TX2, GPIO16 = RX2** | free (→ GPS module) |
| VSPI | GPIO18 SCK, 19 MISO, 23 MOSI, 5 CS | HSPI alt: 14/12/13/15 |
| ADC | ADC1 = GPIO32–39 (works with WiFi); ADC2 unusable while WiFi active | 12-bit |
| DAC | GPIO25, GPIO26 | 8-bit |

## Power

- Via USB (5 V → AMS1117), or via VIN/5V pin (regulated 5 V; don't feed both at once)
- WiFi TX peaks ~250 mA (transients to ~500 mA) — budget the supply for 500 mA min
- Deep sleep: chip ~10 µA, but **whole board ~9–20 mA** (CP2102 + LDO quiescent) — not a true low-power board

## Programming

- Arduino IDE: board **"ESP32 Dev Module"**; PlatformIO: `board = esp32dev`; ESP-IDF target `esp32`
- Flash at 921600 baud (CP2102); CH340 batches may need 460800/115200
- Manual boot mode fallback: hold BOOT, tap EN, release BOOT

## Antenna — important

- **WiFi/BT does not work without the external antenna** attached to the U.FL/IPEX connector; don't TX into no load
- Bundled whip antenna: realistically 2–3 dBi (seller claims of 5–8 dBi are marketing)
- U.FL connector rated ~30 mating cycles — press on straight, carefully

## Sources

- [Espressif ESP32-WROOM-32D/32U datasheet (PDF v2.6)](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf)
- [ESP32 DevKitC V4 board reference](https://done.land/components/microcontroller/families/esp/esp32/developmentboards/esp32s/esp32devkitcv4/)
- [ESP32 pinout guide](https://www.studiopieters.nl/esp32-pinout/)
- [espboards.dev — WROOM-32U variants](https://www.espboards.dev/esp32/microcontroller/esp32/esp32-wroom-32u/)

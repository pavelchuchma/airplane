# Parts Overview

Hardware components for the airplane project, ordered on AliExpress (Jul 19, 2026).
One document per part with full specs, pinout and wiring notes.

| Part | Doc | What it does | Interface | Key facts |
|---|---|---|---|---|
| <img src="img/pitot-ms4525do.jpg" width="120"> | [MS4525DO airspeed sensor + pitot tube](pitot-ms4525do.md) | Differential pressure → airspeed | I2C @ 0x28 (max 400 kHz), 5 V supply / 3.3 V logic, JST-GH 4-pin | ±1 psi, 14-bit, ~0–100 m/s; zero-offset cal mandatory |
| <img src="img/esp32-wroom-32u.png" width="120"> | [ESP32 DevKitC WROOM-32U](esp32-wroom-32u.md) | MCU board — WiFi/BT, external antenna | USB (CP2102), 3.3 V logic; I2C GPIO21/22, UART2 GPIO16/17 | Dual-core LX6 @ 240 MHz, 4 MB flash; **no PCB antenna — U.FL antenna required**; 2 pcs ordered |
| <img src="img/gy91-mpu9250-bmp280.jpg" width="120"> | [GY-91 10DOF IMU (MPU-9250 + BMP280)](gy91-mpu9250-bmp280.md) | Gyro + accel + compass + barometer | I2C @ 0x68 (MPU), 0x76 (BMP280), 0x0C (mag, via bypass); VIN 3–5 V | 16-bit IMU, baro ±1 m; check WHO_AM_I = 0x71 (0x70 = MPU-6500 clone without compass) |
| <img src="img/gps-quescan-g10a-f30.jpg" width="120"> | [Quescan G10A-F30 M10 GPS](gps-quescan-g10a-f30.md) | GNSS position (GPS+GLONASS+Galileo+BeiDou) | UART (3.3 V logic), NMEA/UBX | u-blox UBX-M10050-KB, ~1.5 m CEP, up to 10 Hz |
| <img src="img/pca9685-servo-driver.jpg" width="120"> | [PCA9685 16-ch servo driver](pca9685-servo-driver.md) | Drives up to 16 servos over I2C | I2C @ 0x40 (default), VCC 3.3/5 V + separate V+ servo rail 5–6 V | 12-bit PWM, 50 Hz for servos; servo power from BEC, never from MCU |
| <img src="img/fpv-58g-uvc-kit.jpg" width="120"> | [5.8G FPV kit (UVC RX + VTX + camera)](fpv-58g-uvc-kit.md) | Analog video downlink to Android phone | camera CVBS → VTX 5.8 GHz → UVC dongle (OTG) | 200 mW variant, 40CH, VTX 7–24 V; ~100 ms latency; EU limit 25 mW — HAM license needed |
| <img src="img/mpu6050-gy521.jpg" width="120"> | [MPU-6050 6DOF IMU (GY-521)](mpu6050-gy521.md) — **currently wired in** | Gyro + accelerometer (no compass) | I2C @ 0x68 (AD0 → 0x69), VCC 3.3–5 V | 16-bit, onboard DMP quaternions; ⚠️ same 0x68 default as GY-91's MPU-9250 |

## Orders

| Order | Date | Item | Qty |
|---|---|---|---|
| — | Jul 19, 2026 | MS4525DO pitot kit ("For Pixhawk 6C") | 1 |
| — | Jul 19, 2026 | ESP32 WROOM-32U board + antenna ("type 1set") | 2 |
| — | Jul 19, 2026 | GY-91 module | 1 |
| — | Jul 19, 2026 | Quescan G10A-F30 GPS | 1 |
| — | Mar 20, 2023 | 5.8G FPV kit, "200mW version" | 1 |
| — | Jan 7, 2026 | MPU-6050 module (diymore) | 1 |

## Wiring quick reference (all sensors on one ESP32)

```
ESP32 GPIO21 (SDA) ──┬── MS4525DO SDA        (0x28)
ESP32 GPIO22 (SCL) ──┴── GY-91 SDA/SCL       (0x68 + 0x76)
ESP32 GPIO16 (RX2) ───── GPS TX
ESP32 GPIO17 (TX2) ───── GPS RX
5V  ── MS4525DO VCC, GY-91 VIN, GPS VCC (check module rating)
GND ── common ground
```

All I2C devices share one bus — no address conflicts (0x28 / 0x68 / 0x76 / 0x0C / 0x40 PCA9685).

⚠️ Exception: the **MPU-6050 (currently wired in)** and the GY-91's MPU-9250 both default to **0x68** — when both are connected, pull AD0/SAO high on one of them (→ 0x69).

## Power distribution (3S Li-ion pack)

Pack: 3S Li-ion, 9.0–12.6 V (nominal ~10.8 V). Three branches, buck (switching) converters only — no linear regulators from pack voltage (a linear reg from 11 V to 5 V wastes ~55% as heat; a buck is 90%+ efficient).

```
3S Li-ion (9–12.6 V)
 │
 ├── BUCK A — PSM module (e.g. iFlight BLITZ PSM BXN, 2–8S in, 5V/2A + 12V/3A out)
 │     ├─ 5 V/2 A  → ESP32 (VIN pin), MS4525DO, GY-91, MPU-6050, GPS, PCA9685 VCC
 │     │             (electronics peak ~0.6 A → plenty of headroom)
 │     └─ 12 V/3 A → FPV VTX + camera (regulated & filtered — cleaner video than
 │                   direct pack, no brightness sag as the battery drains)
 │
 └── BUCK B — servo UBEC 5–6 V / ≥5 A (e.g. Hobbywing/Matek 5A class)
       └─ PCA9685 V+ screw terminal (+ 470–1000 µF electrolytic on V+/GND)
```

### Sizing & rules

| Branch | Load | Peak current | Converter |
|---|---|---|---|
| Electronics 5 V | ESP32 + all sensors + PCA9685 logic | ~0.6 A | PSM 5 V/2 A |
| Video | VTX 200 mW + camera | ~0.3 A | PSM 12 V/3 A |
| Servos | PCA9685 V+ | **~1 A per loaded 9g servo**, more at stall (4 servos → budget 4–5 A) | dedicated UBEC ≥5 A |

- **Servos must have their own converter** — a servo stall transient on a shared rail brown-outs the ESP32.
- **Never feed pack voltage (11 V) into ESP32 VIN** — the onboard AMS1117 would dissipate >1 W and thermally shut down. Always go through a 5 V buck first.
- Common ground for all branches, star topology from the pack.
- UVC FPV receiver is ground-side (powered by the phone) — nothing needed onboard.
- Verify whether the PSM's 12 V rail is buck-only: with 3S input (mostly <12 V) it will track below 12 V. VTX (7–24 V) and camera (5–40 V) don't care, but don't expect a flat 12 V.
- If video shows switching-noise bands, add an LC filter (~100 µH + 470 µF) in front of the VTX.

### Battery monitoring

Voltage divider from pack to an ESP32 ADC1 pin, e.g. 47 kΩ / 15 kΩ → 12.6 V max ≈ 3.05 V at GPIO34. Li-ion discharge curve is flat — watch the hard floor of ~3.0 V/cell (9.0 V pack) rather than percentage estimates.

### Shopping list (not yet ordered)

- 1× iFlight BLITZ PSM BXN 2–8S, 5V/2A + 12V/3A ([AliExpress 1005006108885165](https://www.aliexpress.com/item/1005006108885165.html))
- 1× servo UBEC 5–6 V / 5 A (2–6S input)

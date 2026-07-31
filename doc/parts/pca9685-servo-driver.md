# PCA9685 — 16-Channel 12-bit PWM / Servo Driver (I2C)

![PCA9685 servo driver module](img/pca9685-servo-driver.jpg)

- **Product:** [PCA9685PW 16 Channel 12-Bit PWM Servo Shield Driver I2C Interface](https://www.aliexpress.com/item/32466332558.html) — TENSTAR ROBOT GREAT WALL Store
- **What it is:** breakout of the NXP **PCA9685** LED/PWM controller, the classic "Adafruit 16-channel servo driver" clone — drives up to 16 servos (or LEDs) from two I2C pins

## Chip specs (NXP PCA9685)

| Parameter | Value |
|---|---|
| Channels | 16 independent PWM outputs |
| Resolution | 12-bit (4096 steps per period) |
| PWM frequency | ~24 Hz – 1526 Hz, one common frequency for all channels (internal 25 MHz oscillator, `PRE_SCALE` register); **servos: set 50 Hz** |
| Logic supply (VCC) | 2.3–5.5 V; I2C works with 3.3 V logic (ESP32/RPi) directly |
| Servo supply (V+) | separate rail, screw terminal + pin; up to 5.5 V through the board — feed **5–6 V from a BEC/UBEC**, never from the MCU's 5 V pin |
| Output current | 25 mA source / sink per channel (signal only — servo power comes from V+) |
| Outputs | push-pull (default) or open-drain; also usable for LEDs |
| I2C address | **0x40 default**; A0–A5 solder jumpers → 0x40–0x7F (62 boards stackable) |
| Reserved addresses | **0x70 = All Call** (all PCA9685s respond — don't put another device there); 0x03 SWRST |
| I2C speed | up to 1 MHz (Fm+); standard 100/400 kHz fine |
| OE pin | active-low output enable — pull high to disable all outputs (safe-state), has internal pull-down (outputs enabled by default) |
| Sleep | low-power sleep mode via MODE1 register |

## Pinout

| Pin | Function |
|---|---|
| VCC | logic supply 3.3/5 V (from MCU) |
| V+ | servo power 5–6 V (from BEC — high current path, also on the green screw terminal) |
| GND | common ground (logic + servo) |
| SDA / SCL | I2C (onboard 10 kΩ pull-ups to VCC) |
| OE | output enable, active low (optional) |
| CH0–CH15 | 3-pin servo headers: GND / V+ / PWM signal (standard servo plug orientation) |

## Wiring to ESP32

```
PCA9685 VCC → ESP32 3V3        (logic reference = 3.3 V)
PCA9685 GND → ESP32 GND        (+ BEC ground)
PCA9685 SDA → GPIO21
PCA9685 SCL → GPIO22
PCA9685 V+  → 5–6 V BEC/UBEC   (servo current: budget ~1 A per loaded 9g servo, stall peaks more)
```

- Shares the I2C bus with MS4525DO (0x28) and GY-91 (0x68/0x76) — **0x40 does not conflict**
- Big electrolytic cap (470–1000 µF) across V+/GND recommended when driving several servos — servo stall transients brown out the rail

## Servo control

- Set PWM to 50 Hz: `pwm.setPWMFreq(50)`
- At 50 Hz one tick = 20 ms / 4096 ≈ 4.88 µs; typical servo range 1000–2000 µs ≈ ticks **205–410**, center 1500 µs ≈ **307** (calibrate per servo, many take 600–2400 µs)
- Libraries: **Adafruit PWM Servo Driver** (Arduino/ESP32), `adafruit-circuitpython-servokit` (RPi/CircuitPython)

```cpp
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm(0x40);
pwm.begin();
pwm.setPWMFreq(50);
pwm.setPWM(0, 0, 307);            // channel 0 → ~1.5 ms center
// or microseconds directly:
pwm.writeMicroseconds(0, 1500);
```

## Gotchas

1. **One PWM frequency for all 16 channels** — you can't mix 50 Hz servos and high-frequency LED dimming on one board
2. **V+ is not connected to VCC** — servos won't move with logic power only; common symptom of "dead" board
3. The internal oscillator is nominally 25 MHz but varies unit-to-unit — actual output frequency can be off by several %; calibrate `setOscillatorFrequency()` (measure with scope/logic analyzer) if pulse widths must be exact
4. Don't power servos from USB/ESP32 5 V — even one micro servo's stall current causes resets
5. Address jumpers are solder bridges (A0–A5) — for a second board bridge A0 → 0x41

## Sources

- [NXP PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf)
- [Adafruit 16-channel PWM/Servo driver guide](https://learn.adafruit.com/16-channel-pwm-servo-driver)
- [Adafruit PWM Servo Driver library](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)

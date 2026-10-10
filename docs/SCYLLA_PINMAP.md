# BTT Scylla V1 — grblHAL Pin Map and Hardware Audit

[Back to main README](../README.md)

This document records the BTT Scylla V1 pin mapping used by this fork and cross-checks it against BIGTREETECH's official Scylla documentation and the current grblHAL STM32H7xx source.

Primary firmware source of truth:

```text
boards/btt_scylla_map.h
platformio.ini
Src/main.c
```

> **Validation state:** source mapping and clean firmware build are verified. Runtime hardware validation is still in progress.

---

## 1. Important clock clarification: 550 MHz hardware capability vs 480 MHz grblHAL runtime

BIGTREETECH specifies the Scylla V1 MCU as:

```text
STM32H723VGT6
550 MHz
```

STMicroelectronics also specifies the STM32H723 family as capable of running **up to 550 MHz**.

However, the current grblHAL STM32H7xx driver intentionally configures the H723 system clock to:

```text
480 MHz
```

For a 25 MHz crystal, the current H723 PLL configuration is:

```text
HSE   = 25 MHz
PLLM  = 5
PLLN  = 96
PLLP  = 1
SYSCLK = (25 / 5) × 96 / 1 = 480 MHz
```

The same clock setup also produces an exact 48 MHz peripheral clock:

```text
PLL1Q = 10
480 / 10 = 48 MHz
```

That 48 MHz clock is used by peripherals including USB and SDMMC in the current driver.

### Why not simply change it to 550 MHz?

550 MHz is the MCU's maximum rated capability, not a requirement that every firmware must run at that frequency.

ST documents 550 MHz operation as requiring the appropriate highest-performance voltage/boost operating condition. The current grblHAL clock code is built around a shared 480 MHz system clock and an exact 48 MHz PLL-derived peripheral clock.

Changing only the CPU multiplier to reach 550 MHz would also change PLL-derived peripheral clocks. For example:

```text
550 / 10 = 55 MHz
```

which would no longer be the required 48 MHz USB/SDMMC clock.

Therefore, moving grblHAL from 480 MHz to 550 MHz is a **clock-tree redesign**, not a one-line performance tweak. It should only be attempted after recalculating PLL outputs, Flash latency, voltage scaling/boost mode and every affected peripheral clock.

### Documentation convention used in this repository

Use both values explicitly:

```text
MCU rated maximum:      550 MHz
Current grblHAL SYSCLK: 480 MHz
```

This avoids confusing the board's hardware specification with the firmware's configured runtime clock.

---

## 2. Board identity

| Item | Value |
|---|---|
| Board | BIGTREETECH Scylla V1.0 |
| MCU | STM32H723VGT6 |
| MCU architecture | Arm Cortex-M7 |
| MCU rated maximum frequency | 550 MHz |
| Current grblHAL runtime SYSCLK | 480 MHz |
| External crystal used by grblHAL map | 25 MHz |
| Physical stepper driver ICs | 4 × TMC2160 |
| grblHAL Trinamic compatibility selection | `TRINAMIC_ENABLE=5160` |
| Verified PlatformIO environment | `btt_scylla_h723_tmc5160` |
| Baseline memory layout | non-bootloader / full flash |

### TMC2160 vs `TRINAMIC_ENABLE=5160`

The Scylla hardware uses **TMC2160** driver ICs.

The grblHAL PlatformIO target currently enables:

```text
TRINAMIC_ENABLE=5160
```

This naming does not mean that TMC5160 ICs are physically fitted to the Scylla.

TMC2160 and TMC5160 share the same motor-driver functionality/register model for Step/Dir use; the TMC5160 adds an internal motion controller. The current grblHAL target therefore uses the 5160-compatible driver support to configure the physical TMC2160 devices.

For documentation in this fork:

```text
Physical hardware: TMC2160
grblHAL compatibility profile: 5160
```

---

## 3. Cross-check sources

The mapping in this document has been checked against multiple independent references:

1. Current grblHAL board map:
   `boards/btt_scylla_map.h`
2. Current grblHAL build configuration:
   `platformio.ini`
3. Current grblHAL H723 clock configuration:
   `Src/main.c`
4. BIGTREETECH official Scylla V1.0 User Manual
5. BIGTREETECH official Scylla GitHub hardware package, which includes:
   - `Hardware/BIGTREETECH Scylla V1.0-Pin.png`
   - `Hardware/Scylla V1.0-Connector.png`
6. Community hardware reports from actual Scylla/Milo installations, used only as a secondary sanity check.

The firmware source remains authoritative for what this grblHAL build will actually use.

---

## 4. Axis step / direction / enable

### X axis

| Signal | MCU pin | Physical role | Planned project role |
|---|---|---|---|
| X STEP | `PA0` | X-OUT STEP | X axis step |
| X DIR | `PA1` | X-OUT DIR | X axis direction |
| X ENABLE | `PC0` | X-OUT EN | X driver enable |
| X LIMIT | `PD11` | X-MIN physical connector | Logical **X MAX** home/limit planned |

### Y axis

| Signal | MCU pin | Physical role | Planned project role |
|---|---|---|---|
| Y STEP | `PC13` | Y-OUT STEP | Y axis step |
| Y DIR | `PE6` | Y-OUT DIR | Y axis direction |
| Y ENABLE | `PC2` | Y-OUT EN | Y driver enable |
| Y LIMIT | `PA8` | Y-MIN physical connector | Logical **Y MIN** home/limit planned |

### Z axis

| Signal | MCU pin | Physical role | Planned project role |
|---|---|---|---|
| Z STEP | `PB8` | Z-OUT STEP | Z axis step |
| Z DIR | `PB9` | Z-OUT DIR | Z axis direction |
| Z ENABLE | `PE5` | Z-OUT EN | Z driver enable |
| Z LIMIT | `PC7` | Z-MIN physical connector | Logical **Z MAX** home/limit planned |

### Motor 4 / M3 / A

| Signal | MCU pin | Physical role |
|---|---|---|
| M3 STEP | `PD3` | A-OUT STEP |
| M3 DIR | `PD4` | A-OUT DIR |
| M3 ENABLE | `PE1` | A-OUT EN |
| M3 LIMIT | `PD15` | A-MIN |

The board map supports a maximum of four motors.

---

## 5. Limit and homing inputs

The official BIGTREETECH manual states that Scylla has eight optoisolated limit inputs and explicitly notes that the printed MIN/MAX labels are suggestions only; firmware may assign them differently.

Current grblHAL mapping:

| grblHAL signal | MCU pin | Physical connector/comment | Intended project use |
|---|---|---|---|
| X limit | `PD11` | X-MIN | Logical X MAX |
| Y limit | `PA8` | Y-MIN | Logical Y MIN |
| Z limit | `PC7` | Z-MIN | Logical Z MAX |
| M3 limit | `PD15` | A-MIN | Optional |

Planned machine homing direction:

```text
X -> MAX
Y -> MIN
Z -> MAX
```

The physical connector name does not determine homing direction; grblHAL configuration does.

---

## 6. User control inputs

The current Scylla grblHAL map assigns three physical MAX inputs as control signals:

| Function | MCU pin | Physical connector | grblHAL role |
|---|---|---|---|
| Reset / Halt | `PC6` | X-MAX | `RESET` |
| Feed Hold | `PD14` | Y-MAX | `FEED_HOLD` |
| Cycle Start | `PD12` | Z-MAX | `CYCLE_START` |
| Safety Door | `PD13` | A-MAX | `SAFETY_DOOR` |

This is why the project can keep X/Y/Z limit switches on the physical MIN connectors while using software configuration to choose MAX/MIN homing direction.

---

## 7. Probe, toolsetter and IO-IN

| Function | MCU pin | Board-map role | Electrical note |
|---|---|---|---|
| Probe | `PE15` | Probe / AUX input 1 | Optoisolated probe circuit |
| Toolsetter | `PE7` | Tool / AUX input 2 | Optoisolated toolsetter circuit |
| **IO-IN** | **`PB7`** | **IO-IN / AUX input 6** | **Dedicated 5 V digital input** |

### IO-IN verification

The current grblHAL board map explicitly defines:

```c
#define AUXINPUT6_PORT GPIOB   // IO-IN
#define AUXINPUT6_PIN  7
```

Therefore:

```text
IO-IN = PB7
```

BIGTREETECH's official manual independently confirms that the Scylla contains **one dedicated 5 V digital input**.

Electrical restriction from the official manual:

```text
Maximum intended input level: 5 V
Input leakage current: < 350 nA
Do not apply more than 5 V
```

This input is not one of the optoisolated 24 V-capable limit inputs. Treat it as a 5 V logic input.

---

## 8. Auxiliary outputs

| AUX | MCU pin | Upstream role |
|---|---|---|
| AUX0 | `PA4` | General AUX0 |
| AUX1 | `PA5` | General AUX1 |
| AUX2 | `PA6` | General AUX2 |
| AUX3 | `PC5` | Spindle Enable |
| AUX4 | `PB1` | Spindle PWM |
| AUX5 | `PB0` | Spindle Direction |
| AUX6 | `PC4` | Coolant Flood |
| AUX7 | `PA7` | Coolant Mist |
| AUX8 | `PE9` | ESP32 IO0 / BOOT0 |
| AUX9 | `PE10` | ESP32 Reset |
| AUX10 | `PD5` | Mechanical Relay |

---

## 9. AUX10 relay verification

Current grblHAL board map:

```text
AUX10 = PD5
```

BIGTREETECH's official manual independently confirms the relay control pin is `PD5`.

Official relay logic:

```text
PD5 LOW  -> COM connected to NC
PD5 HIGH -> COM connected to NO
```

The manual rates the exposed relay contacts for up to 10 A at the documented AC/DC contact ratings.

### Project-specific intended use

AUX10 is intended to participate in contactor/machine-power control.

Final logic must ensure:

- no unexpected contactor enable after reset,
- E-Stop release alone does not re-enable machine power,
- a deliberate recovery/reset action is required,
- hardware E-Stop remains fail-safe independently of firmware.

---

## 10. Spindle outputs

| Function | MCU pin | AUX |
|---|---|---|
| Spindle Enable | `PC5` | AUX3 |
| Spindle PWM | `PB1` | AUX4 |
| Spindle Direction | `PB0` | AUX5 |

The official Scylla manual documents dedicated isolated VFD enable, direction and analogue/digital speed circuitry.

---

## 11. Coolant outputs

| Function | MCU pin |
|---|---|
| Flood / COOL | `PC4` |
| Mist | `PA7` |

The official manual documents five optically isolated FET outputs: AUX0-AUX2 plus Mist and Cool.

---

## 12. Trinamic SPI interface

The Scylla hardware physically contains TMC2160 drivers.

The current grblHAL configuration accesses them through the 5160-compatible Trinamic driver support.

### Software SPI

| Signal | MCU pin |
|---|---|
| MOSI | `PB5` |
| MISO | `PB4` |
| SCK | `PB3` |

### Chip select

| Axis | CS pin |
|---|---|
| X | `PC15` |
| Y | `PC14` |
| Z | `PE2` |
| M3 / A | `PE4` |

Board-map value:

```text
TRINAMIC_R_SENSE = 50
```

Motor current configuration must be validated against the actual Scylla TMC2160 power stage before changing current limits.

---

## 13. Serial and communication interfaces

| Interface | MCU pins | Board role |
|---|---|---|
| ESP32 UART | TX `PA2`, RX `PA3` | ESP32 |
| RS485 / Modbus | TX `PA9`, RX `PA10` | VFD / RS485 |
| Pi UART | TX `PD8`, RX `PD9` | Pi-CON |
| I2C | SCL `PB10`, SDA `PB11` | I2C |
| Pi SPI | SCK `PB13`, MISO `PB14`, MOSI `PB15` | Pi-CON |
| CAN | RX `PD0`, TX `PD1` | CAN |
| NeoPixel | `PD6` | RGB when enabled |

The board map explicitly selects the RS485 UART as the Modbus RTU stream.

---

## 14. SD card

When SD support is enabled:

```text
SDCARD_SDIO = 1
```

The verified PlatformIO environment includes SD-card support.

---

## 15. Verified PlatformIO target

Current baseline:

```ini
[env:btt_scylla_h723_tmc5160]
board = generic_stm32h723vg
board_build.ldscript = STM32H723VGTX_FLASH.ld
-D BOARD_BTT_SCYLLA
-D HSE_VALUE=25000000
-D TRINAMIC_ENABLE=5160
```

Also included:

- H723 USB CDC support
- SD-card support
- motors plugin
- Trinamic library

A separate bootloader target exists:

```text
btt_scylla_h723_tmc5160_bl128
```

The current baseline build is the **non-bootloader** target.

---

## 16. Quick pin reference

```text
MCU / CLOCK
STM32H723VGT6
Rated max: 550 MHz
Current grblHAL SYSCLK: 480 MHz
HSE: 25 MHz

MOTION
X STEP      PA0
X DIR       PA1
X EN        PC0
X LIMIT     PD11

Y STEP      PC13
Y DIR       PE6
Y EN        PC2
Y LIMIT     PA8

Z STEP      PB8
Z DIR       PB9
Z EN        PE5
Z LIMIT     PC7

A/M3 STEP   PD3
A/M3 DIR    PD4
A/M3 EN     PE1
A/M3 LIMIT  PD15

CONTROLS
Reset       PC6
Feed Hold   PD14
Cycle Start PD12
Safety Door PD13

PROBING / INPUT
Probe       PE15
Toolsetter  PE7
IO-IN       PB7   (5 V digital input)

AUX / SPINDLE
AUX0        PA4
AUX1        PA5
AUX2        PA6
Spindle EN  PC5
Spindle PWM PB1
Spindle DIR PB0
Flood       PC4
Mist        PA7
Relay       PD5

TMC2160 PHYSICAL / 5160-COMPATIBLE GRBLHAL PROFILE
MOSI        PB5
MISO        PB4
SCK         PB3
X CS        PC15
Y CS        PC14
Z CS        PE2
A CS        PE4

COMMUNICATION
ESP UART    PA2 / PA3
RS485       PA9 / PA10
Pi UART     PD8 / PD9
I2C         PB10 / PB11
Pi SPI      PB13 / PB14 / PB15
CAN         PD0 / PD1
RGB         PD6
```

---

## 17. Persian engineering notes / نکات مهندسی فارسی

### ۵۵۰ مگاهرتز در برابر ۴۸۰ مگاهرتز

عدد `550 MHz` مشخصه حداکثر سرعت MCU است.

Firmware فعلی grblHAL پردازنده را عمداً روی:

```text
480 MHz
```

راه‌اندازی می‌کند و هم‌زمان از PLL همان کلاک دقیق `48 MHz` مورد نیاز USB/SDMMC را می‌سازد.

بنابراین نوشتن «Scylla فقط 480MHz است» اشتباه است و نوشتن «Firmware فعلی 550MHz اجرا می‌شود» هم اشتباه است.

عبارت صحیح:

```text
STM32H723VGT6 rated up to 550 MHz
Current grblHAL firmware clocked at 480 MHz
```

### IO-IN

ورودی اختصاصی IO روی Scylla وجود دارد و در grblHAL به این پین متصل است:

```text
IO-IN = PB7
```

این ورودی **۵ ولتی** است و نباید مثل ورودی‌های optoisolated لیمیت به آن 12V یا 24V داده شود.

### درایور موتور

روی خود برد:

```text
TMC2160
```

نصب شده است.

ولی Environment فعلی grblHAL از:

```text
TRINAMIC_ENABLE=5160
```

استفاده می‌کند؛ این یک انتخاب سازگاری نرم‌افزاری است و به معنی وجود TMC5160 روی برد نیست.

---

## 18. Validation status

| Area | Status |
|---|---|
| Current grblHAL pin definitions | Verified from source |
| BIGTREETECH MCU specification | Verified |
| Current grblHAL 480 MHz clock | Verified from source |
| Physical TMC2160 driver type | Verified from official manual |
| IO-IN physical feature | Verified from official manual |
| IO-IN = PB7 in grblHAL | Verified from board map |
| Relay = PD5 | Cross-verified |
| X/Y/Z limit pins | Cross-checked |
| Clean firmware build | Verified |
| Runtime motion test | Pending |
| Homing directions | Pending |
| Control buttons | Pending |
| Probe / Toolsetter | Pending |
| IO-IN runtime test | Pending |
| Relay safety sequence | Pending |
| TMC2160 runtime SPI | Pending |
| RS485 / Modbus | Pending |
| VFD control | Pending |
| E-Stop recovery behavior | Pending |

---

## 19. Source-of-truth policy

If documentation disagrees with firmware, check these files first:

```text
boards/btt_scylla_map.h
platformio.ini
Src/main.c
```

For physical board specifications, check BIGTREETECH's official Scylla V1.0 manual and official hardware pin diagram.

Every upstream merge that changes any of these files should trigger a review of this document.

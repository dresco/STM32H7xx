# BTT Scylla V1 — Control Inputs, E-Stop and Operator Panel

[Back to main README](../README.md) | [Scylla Pin Map](SCYLLA_PINMAP.md)

This document explains the operator-control inputs used by this Scylla/grblHAL fork.

It exists because three different names can otherwise be confused:

1. the **physical connector label printed on the Scylla PCB**,
2. the **MCU GPIO pin** connected to that connector,
3. the **logical grblHAL function** assigned to that input.

These are not necessarily the same thing.

> **Important:** The physical label `X-MAX`, `A-MAX`, etc. identifies a connector on the Scylla board. It does **not** permanently define the software function of that connector.

---

# 1. Project target mapping

The intended operator-control mapping for this fork is:

| Physical Scylla connector | MCU pin | Project function | Status |
|---|---:|---|---|
| X-MAX | `PC6` | **Emergency Stop input** | Existing upstream E-Stop path; hardware test pending |
| A-MAX | `PD13` | **Separate Reset input** | Implemented in STM32H7xx driver; build verified; hardware validation pending |
| Y-MAX | `PD14` | **Feed Hold** | Existing upstream mapping |
| Z-MAX | `PD12` | **Cycle Start** | Existing upstream mapping |
| IO-IN | `PB7` | General-purpose digital input | Remains free |
| Safety Door | — | Reserved / currently unassigned | Future use |

The axis limit connectors remain:

```text
X-MIN / PD11 -> X axis limit input
Y-MIN / PA8  -> Y axis limit input
Z-MIN / PC7  -> Z axis limit input
```

with the planned logical homing directions:

```text
X -> MAX
Y -> MIN
Z -> MAX
```

The words `MIN` and `MAX` on the PCB are connector names. grblHAL configuration determines the actual logical homing direction.

---

# 2. Why X-MAX / PC6 is used for E-Stop

The upstream Scylla board map currently declares:

```text
AUXINPUT3 = PC6   // Reset - X-MAX
```

and maps that input through the traditional `RESET_PIN` interface.

Modern grblHAL can treat that traditional HALT/RESET input as either:

```text
Reset
```

or:

```text
Emergency Stop
```

depending on whether `ESTOP_ENABLE` is enabled.

In the current Core:

```c
#if ESTOP_ENABLE
#define CONTROL_HALT CONTROL_ESTOP
#else
#define CONTROL_HALT CONTROL_RESET
#endif
```

and the traditional `RESET_PIN` is registered as `Input_EStop` when E-Stop support is enabled.

Therefore using:

```text
X-MAX / PC6 -> E-STOP
```

follows the current grblHAL E-Stop architecture and also matches common Scylla installations.

---

# 3. Why a separate Reset input is needed

Emergency Stop and Reset are different operator actions.

They should not be treated as the same physical button in this machine.

Our target behavior is:

```text
PC6  -> Emergency Stop status
PD13 -> Reset / recovery command
```

The current generic grblHAL mapping does not automatically expose both functions at the same time through separate pins. When E-Stop is enabled, the traditional `RESET_PIN` becomes the E-Stop input.

Therefore this fork will add a Scylla-specific **separate Reset input** on:

```text
A-MAX / PD13
```

without modifying the upstream grblHAL Core submodule.

This keeps the Core updateable while making the Scylla driver implement the required machine behavior.

---

# 4. Emergency Stop — what it means

## Software meaning

The grblHAL Emergency Stop input tells the controller that the machine is in an emergency-stop condition.

The intended controller sequence is:

```text
E-Stop pressed
      ↓
grblHAL detects E-Stop
      ↓
machine enters E-Stop/alarm state
      ↓
E-Stop is physically released
      ↓
machine must NOT automatically resume
      ↓
operator performs deliberate recovery/reset/unlock
```

The current grblHAL Core includes setting:

```text
$484
```

for:

```text
Unlock required after E-stop cleared
```

The default behavior is intended to prevent automatic return to an operable state simply because the E-Stop input was released.

## Safety meaning

The grblHAL input is **not the primary safety circuit**.

A real CNC E-Stop must directly act on a hardware safety chain, for example:

```text
E-Stop mushroom
   ├──> safety relay / contactor chain
   │       └──> removes hazardous machine energy
   │
   └──> auxiliary monitoring contact
           └──> Scylla PC6 / E-Stop input
```

Firmware should know that E-Stop happened, but software must not be the only mechanism preventing hazardous motion or spindle operation.

### Recommended concept

Use the E-Stop's safety-rated contact(s) for the hardwired safety circuit.

Use an electrically compatible auxiliary contact for the Scylla E-Stop status input.

Where practical, a normally-closed monitoring circuit is preferred because a broken wire can be detected as an unsafe state. Final wiring must match the Scylla input circuit and the configured grblHAL inversion logic.

---

# 5. Reset — what it means

Project mapping:

```text
A-MAX / PD13 -> Reset
```

This is a **grblHAL control Reset**, not the STM32 hardware `NRST` pin.

Its purpose is operator recovery/control handling, not physically power-cycling the MCU.

A normal panel Reset button is expected to be a momentary operator control.

In this project it is especially important because Reset will be part of the recovery sequence after an E-Stop.

Target concept:

```text
1. E-Stop is pressed
2. dangerous power is removed by hardware
3. PC6 reports E-Stop to grblHAL
4. E-Stop is physically released
5. machine remains inhibited
6. operator presses Reset on PD13
7. required unlock/recovery procedure is completed
8. machine power/enable may only then be restored
```

The exact interaction with AUX10 and the machine contactor will be validated on the physical machine before being marked as final.

---

# 6. Feed Hold — what it means

Project mapping:

```text
Y-MAX / PD14 -> Feed Hold
```

Feed Hold is a **controlled pause**, not an emergency stop.

Typical use:

```text
running CNC program
      ↓
Feed Hold pressed
      ↓
controller performs controlled motion stop
      ↓
program remains paused
      ↓
Cycle Start resumes execution
```

Feed Hold should be used when the operator wants to pause normal machining without invoking the emergency safety chain.

It must **not** be used as a substitute for E-Stop.

### Typical panel button

A momentary push button is normally appropriate.

---

# 7. Cycle Start — what it means

Project mapping:

```text
Z-MAX / PD12 -> Cycle Start
```

Cycle Start is used to start or resume permitted program execution.

Common sequence:

```text
Feed Hold
   ↓
machine reaches controlled hold
   ↓
operator checks condition
   ↓
Cycle Start
   ↓
program continues
```

Cycle Start cannot override an active E-Stop or other controller interlocks.

### Typical panel button

A momentary push button is normally appropriate.

---

# 8. Safety Door — what it means

Safety Door is a distinct grblHAL function.

It is **not equivalent to E-Stop**.

Its intended use is for an enclosure or guard interlock where the controller should enter a controlled safety-door state.

For example:

```text
enclosure door opens
      ↓
Safety Door input changes state
      ↓
grblHAL performs configured controlled response
```

This can be useful on enclosed routers or mills, but it must not be confused with a hardwired emergency-stop circuit.

### This project's current decision

The original Scylla map uses:

```text
A-MAX / PD13 -> Safety Door
```

when Safety Door support is enabled.

This fork plans to reuse `PD13` for the **separate Reset button**.

Therefore:

```text
Safety Door = currently unassigned / reserved
```

If Safety Door is required later, another suitable available input must be selected and documented.

---

# 9. IO-IN — general digital input

The Scylla provides a dedicated input:

```text
IO-IN = PB7
```

In this project it remains free.

It is intentionally **not consumed by Reset or E-Stop**, so it remains available for future functions such as:

- external machine status,
- accessory interlock,
- pressure/flow switch,
- user-defined input,
- future plugin function.

The Scylla manual describes IO-IN as a dedicated **5 V digital input**.

Do not treat IO-IN as a 12/24 V optoisolated limit input.

---

# 10. Probe and Toolsetter are not panel controls

These inputs are separate from the operator buttons:

```text
Probe      -> PE15
Toolsetter -> PE7
```

Their purpose is measurement/probing logic.

They must not be repurposed as Reset, E-Stop, Feed Hold or Cycle Start in this project.

---

# 11. Control input inversion and pull-up settings

grblHAL allows control input electrical logic to be configured.

Relevant Core settings include:

```text
$14 = Control input invert mask
$17 = Control input pull-up disable mask
```

This means the firmware should not assume that every button is always active-high or active-low.

The correct settings depend on:

- the Scylla input circuit,
- NO vs NC contact wiring,
- external voltage/interface,
- fail-safe requirements,
- the desired trigger state.

Do not copy `$14` or `$17` values from another machine without checking the actual wiring.

---

# 12. E-Stop recovery setting

Current grblHAL Core includes:

```text
$484 = Unlock required after E-stop cleared
```

This setting exists specifically to control whether an unlock operation is required after the E-Stop condition disappears.

For this CNC project, the intended safety behavior is:

```text
E-Stop release alone MUST NOT restore normal machine operation.
```

The exact `$484` behavior will be tested on the target firmware and documented with the final machine settings.

---

# 13. Physical connector name vs software function

This is the most important rule when reading Scylla documentation.

Example:

```text
Physical label: X-MAX
MCU GPIO:       PC6
Project role:   Emergency Stop
```

Another example:

```text
Physical label: A-MAX
MCU GPIO:       PD13
Upstream role:  Safety Door
Project role:   Separate Reset
```

Therefore never infer a function only from the text printed next to a Scylla connector.

Always check:

```text
Physical connector
      ↓
MCU GPIO
      ↓
boards/btt_scylla_map.h
      ↓
grblHAL function
      ↓
project-specific override
```

---

# 14. Operator-panel quick reference

```text
+------------------------------------------------+
|              CNC OPERATOR PANEL                |
+------------------------------------------------+
|                                                |
|  [ E-STOP ]    X-MAX / PC6                    |
|      |                                         |
|      +---- Hardware safety chain               |
|      +---- grblHAL E-Stop monitoring           |
|                                                |
|  [ RESET ]     A-MAX / PD13                    |
|                                                |
|  [ FEED HOLD ] Y-MAX / PD14                    |
|                                                |
|  [ CYCLE START ] Z-MAX / PD12                  |
|                                                |
+------------------------------------------------+

IO-IN / PB7 -> remains available
```

---

# 15. Function comparison

| Function | Emergency? | Controlled pause? | Resume command? | Safety-chain replacement? |
|---|---:|---:|---:|---:|
| E-Stop | **Yes** | No | No | **No — hardware chain still required** |
| Reset | No | No | No | No |
| Feed Hold | No | **Yes** | No | No |
| Cycle Start | No | No | **Yes** | No |
| Safety Door | Safety interlock | Controlled behavior | Depends on state/recovery | No |

---

# 16. Current implementation status

| Function | Target pin | Firmware state |
|---|---|---|
| E-Stop | `PC6` | Supported via current grblHAL HALT/E-Stop path; build verified; hardware validation pending |
| Reset | `PD13` | **Implemented as independent `Input_Reset`; build verified; hardware validation pending** |
| Feed Hold | `PD14` | Existing mapping |
| Cycle Start | `PD12` | Existing mapping |
| Safety Door | unassigned | Intentionally reserved |
| IO-IN | `PB7` | Available |
| Probe | `PE15` | Existing mapping |
| Toolsetter | `PE7` | Existing mapping |

The separate Reset implementation now compiles successfully in the `btt_scylla_h723_tmc5160` environment. It must still be validated on the physical Scylla board before being described as hardware-tested.

---

# 17. Implemented firmware architecture

This fork now implements:

```text
PC6  / X-MAX -> standard grblHAL E-Stop/HALT path
PD13 / A-MAX -> independent Input_Reset
PD14 / Y-MAX -> Feed Hold
PD12 / Z-MAX -> Cycle Start
PB7  / IO-IN -> general-purpose input
```

Implementation details:

- `PC6` remains on the standard grblHAL HALT/E-Stop path.
- `PD13` is claimed from the Scylla auxiliary-input pool as a separate `Input_Reset`.
- `systemGetState()` reports Reset and E-Stop independently.
- the Reset input uses the grblHAL control-input inversion and pull-up settings.
- the Reset input has its own interrupt registration.
- enabling Safety Door while `PD13` is reserved for Reset is blocked at compile time to prevent a pin-function conflict.
- the grblHAL Core submodule remains unmodified.

Build validation:

```text
Environment: btt_scylla_h723_tmc5160
Result:      SUCCESS
RAM:         31,484 / 577,536 bytes (5.5%)
Flash:       322,028 / 1,048,576 bytes (30.7%)
```

Hardware validation is still pending.

---

# 18. Persian summary / خلاصه فارسی

در Scylla اسم نوشته‌شده کنار کانکتور الزاماً همان کاری نیست که Firmware از آن انجام می‌دهد.

چیدمان هدف این پروژه:

```text
X-MAX / PC6  = E-STOP
A-MAX / PD13 = RESET
Y-MAX / PD14 = FEED HOLD
Z-MAX / PD12 = CYCLE START
IO-IN / PB7  = آزاد
```

### E-STOP

ای‌استاپ برای وضعیت اضطراری است.

همزمان باید:

1. مدار سخت‌افزاری ایمنی، توان خطرناک را قطع کند.
2. برد Scylla نیز از طریق `PC6` متوجه فعال شدن E-Stop شود.

**فقط Firmware برای E-Stop کافی نیست.**

### RESET

Reset روی `PD13` یک فرمان کنترلی grblHAL است و با پایه سخت‌افزاری Reset میکروکنترلر فرق دارد.

این دکمه برای Recovery بعد از E-Stop و Reset کنترلر استفاده خواهد شد.

### FEED HOLD

`PD14` برای توقف کنترل‌شده برنامه است.

Feed Hold ای‌استاپ نیست.

### CYCLE START

`PD12` برای شروع یا ادامه برنامه بعد از Hold استفاده می‌شود.

### SAFETY DOOR

Safety Door یک ورودی مستقل برای در یا حفاظ دستگاه است و با E-Stop فرق دارد.

چون در این پروژه `PD13` را برای Reset اختصاص می‌دهیم، فعلاً Safety Door پین اختصاصی ندارد و برای آینده رزرو می‌شود.

### IO-IN

```text
PB7
```

به‌عنوان ورودی دیجیتال عمومی آزاد باقی می‌ماند.

---

# 19. Source-of-truth policy

For Core behavior, consult the current grblHAL Core.

For physical GPIO assignment, consult:

```text
boards/btt_scylla_map.h
```

For this fork's final machine behavior, consult this document together with:

```text
docs/SCYLLA_PINMAP.md
```

If implementation and documentation disagree, implementation must be audited and this document updated before release.

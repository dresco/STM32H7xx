# grblHAL STM32H7xx — BTT Scylla Compatibility Fork

[English](README.md) | [فارسی](docs/README.fa.md)

This repository is a public fork of [`dresco/STM32H7xx`](https://github.com/dresco/STM32H7xx) focused on keeping the **BTT Scylla V1 / STM32H723** target buildable with the current grblHAL Core and current plugin revisions.

The immediate goal of this fork is to provide a clean, reproducible, documented baseline for BTT Scylla V1 before machine-specific configuration is added.

> **Current validation status:** build verified from a clean PlatformIO build. Full on-board hardware validation is still pending.

---

## Why this fork exists

The upstream STM32H7xx driver currently tracks an older grblHAL Core API. The current grblHAL Core removed the deprecated `hal.settings_changed` callback and moved settings-change handling to `grbl.on_settings_changed`.

After updating the Core and plugin submodules to their current upstream revisions, the BTT Scylla PlatformIO build failed in the STM32H7xx driver because the driver still referenced the removed API.

This fork updates the STM32H7xx driver to the current callback API and verifies that the Scylla target builds successfully from a clean tree.

### Compatibility change applied

The following driver files were updated:

- `Src/driver.c`
- `Src/driver_spindles.c`
- `Src/encoders.c`
- `Src/neopixel_spi.c`

The deprecated callback usage:

```c
hal.settings_changed
```

was migrated to:

```c
grbl.on_settings_changed
```

This matches the current grblHAL Core event model.

---

## Current status

| Item | Status |
|---|---|
| BTT Scylla V1 / STM32H723 target | Build verified |
| PlatformIO environment | Verified |
| USB CDC build support | Enabled by target configuration |
| SD card build support | Enabled by target configuration |
| TMC5160 build support | Enabled by target configuration |
| Clean build from scratch | Verified |
| Runtime hardware validation | Pending |
| Machine-specific CNC configuration | Planned |

The verified PlatformIO environment is:

```text
btt_scylla_h723_tmc5160
```

This is the **non-bootloader** environment intended for direct firmware flashing. The `_bl128` environment is a separate bootloader build and is not the baseline used here.

---

## Verified upstream baseline

At the time this compatibility baseline was created:

| Component | Revision |
|---|---|
| STM32H7xx upstream base | `bfe5d4f6ec271cc9929336244c3b1c8e55d1d040` |
| grblHAL Core | `516e5ad80757bd2eba86bff18feb613ca121dc16` |
| Submodule update commit in this fork | `3279baa` |
| Compatibility commit in this fork | `15dd0e3` |
| PlatformIO / VS Code setup commit | `7eeb637` |

The local project was created from the latest available STM32H7xx upstream branch and then its configured submodules were updated before building.

> Note: grblHAL Core evolves faster than some MCU driver repositories. This fork intentionally documents the exact revision set used for a successful build.

---

## Verified build environment

The clean build was verified with:

```text
PlatformIO Core: 6.2.0
Platform: ST STM32 20.0.0
Framework: STM32CubeH7 1.12.1
Target MCU: STM32H723VGT6
PlatformIO environment: btt_scylla_h723_tmc5160
```

Successful clean-build memory usage:

```text
RAM:   31,484 / 577,536 bytes     (5.5%)
Flash: 321,700 / 1,048,576 bytes  (30.7%)
```

---

## Supported boards inherited from upstream

The upstream STM32H7xx driver includes support for multiple STM32H7-based boards, including:

- Nucleo-H743ZI
- Nucleo-H723ZG
- WeAct MiniSTM32H7xx
- BTT SKR 3 EZ
- BTT Octopus Max EZ
- BTT Scylla
- other targets already present in the upstream driver

This fork currently focuses validation and documentation on **BTT Scylla V1**.

---

## Prerequisites

For the workflow used to verify this repository on Windows:

- Git
- Python
- Visual Studio Code — optional but recommended
- PlatformIO IDE extension — when using VS Code
- PlatformIO Core CLI

VS Code is not required for building. The project can be built entirely from the PlatformIO command line.

---

## Clone the repository

Because this project uses Git submodules, clone it recursively:

```bash
git clone --recursive https://github.com/coolboysalam/STM32H7xx.git
cd STM32H7xx
git checkout scylla-grblhal-latest
```

If the repository was cloned without `--recursive`, initialize submodules with:

```bash
git submodule update --init --recursive
```

---

## Build BTT Scylla V1

Clean the previous build output:

```bash
pio run -e btt_scylla_h723_tmc5160 -t clean
```

Build the firmware:

```bash
pio run -e btt_scylla_h723_tmc5160
```

A successful build should end with:

```text
SUCCESS
```

Generated firmware files are located at:

```text
.pio/build/btt_scylla_h723_tmc5160/firmware.bin
.pio/build/btt_scylla_h723_tmc5160/firmware.elf
```

---

## Bootloader note

The BTT Scylla PlatformIO configuration includes both normal and bootloader-oriented environments.

This fork's current verified baseline uses:

```text
btt_scylla_h723_tmc5160
```

It does **not** use:

```text
btt_scylla_h723_tmc5160_bl128
```

Use the bootloader environment only when the board actually has the matching bootloader and memory layout.

---

## Planned Scylla documentation

The next project stages will document the Scylla V1 board and CNC-specific configuration in separate files, including:

- complete Scylla pin map
- limit and homing configuration
- Reset / Feed Hold / Cycle Start inputs
- Probe and Toolsetter inputs
- AUX outputs, including AUX10
- TMC5160 configuration
- spindle control
- Modbus / VFD configuration
- E-Stop and safety behavior
- direct flashing procedure
- hardware validation checklist

These will be added under `docs/` as the hardware validation progresses.

---

## Keeping this fork synchronized with upstream

Recommended Git remote layout:

```text
origin   -> https://github.com/coolboysalam/STM32H7xx.git
upstream -> https://github.com/dresco/STM32H7xx.git
```

Fetch future upstream changes with:

```bash
git fetch upstream
```

For submodules:

```bash
git submodule sync --recursive
git submodule update --remote --recursive
```

Because grblHAL Core APIs can change ahead of MCU drivers, always perform a clean build after updating submodules.

---

## Development policy for this fork

The intended workflow is:

1. Start from a known successful baseline.
2. Update upstream components intentionally.
3. Keep compatibility patches isolated in clear commits.
4. Run a clean Scylla build after every compatibility change.
5. Perform hardware validation before marking a feature as tested.
6. Document configuration and behavior as the project evolves.

This keeps the firmware reproducible and makes regressions easier to diagnose.

---

## Upstream projects and credits

This repository is based on the work of the grblHAL community and the STM32H7xx driver maintainers.

Primary upstream projects:

- [grblHAL Core](https://github.com/grblHAL/core)
- [STM32H7xx driver by dresco](https://github.com/dresco/STM32H7xx)

Please refer to the original repositories for upstream authorship and project history.

---

## License

This fork preserves the upstream licensing and copyright notices.

grblHAL and this STM32H7xx driver are distributed under the GNU General Public License terms included in this repository. See `COPYING` and the license headers in the source files for details.

---

## Persian documentation

A full Persian version of this documentation is available here:

**[فارسی — مستندات کامل پروژه](docs/README.fa.md)**

---

## Project stage

**Stage 1 — Clean latest-Core compatibility baseline:** complete.

**Stage 2 — BTT Scylla V1 hardware and CNC configuration:** in progress.

Hardware validation results will be added after the firmware is tested on the actual board.

# Changelog

All notable changes to this fork will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project follows a practical revision-based workflow rather than semantic versioning while the Scylla-specific firmware is still under active development.

## [Unreleased]

### Planned
- Add complete BTT Scylla V1 pin map documentation.
- Add hardware validation results for the STM32H723 Scylla target.
- Document direct flashing procedure for non-bootloader builds.
- Document limit and homing configuration.
- Document Reset / Feed Hold / Cycle Start inputs.
- Document Probe and Toolsetter inputs.
- Document AUX outputs, including AUX10.
- Document TMC5160 configuration.
- Document Modbus / VFD integration.
- Document E-Stop and safety behavior.
- Add automated GitHub Actions build validation for the Scylla target.
- Add release packaging for `firmware.bin`.

---

## [2026-09-09] - Latest-Core compatibility baseline

### Summary
Created a clean, reproducible BTT Scylla V1 baseline using the current STM32H7xx driver, current grblHAL Core, and updated project submodules.

The main purpose of this baseline is to provide a known-good compile state before applying machine-specific CNC configuration.

### Upstream baseline
- STM32H7xx upstream base:
  - `bfe5d4f6ec271cc9929336244c3b1c8e55d1d040`
  - `Enable SD card support for Manta M8P v2`
- grblHAL Core:
  - `516e5ad80757bd2eba86bff18feb613ca121dc16`
  - `Updated changelog`

### Added
- New development branch:
  - `scylla-grblhal-latest`
- PlatformIO / VS Code workspace support files.
- English project documentation in `README.md`.
- Persian project documentation in `docs/README.fa.md`.
- Clear upstream/fork Git remote structure:
  - `origin` -> personal fork
  - `upstream` -> `dresco/STM32H7xx`

### Updated
Updated configured Git submodules to their current upstream revisions, including:
- grblHAL Core
- EEPROM plugin
- networking plugin
- SD card plugin
- Bluetooth plugin
- laser plugin
- spindle plugins
- motor plugins
- plasma plugin
- miscellaneous plugins
- keypad plugin
- WebUI plugin

The submodule update was recorded in:

```text
3279baa Update grblHAL core and plugins to latest upstream
```

### Fixed
The updated grblHAL Core removed the deprecated:

```c
hal.settings_changed
```

callback from the HAL API.

The STM32H7xx driver still referenced this removed callback, causing compilation to fail after updating Core.

The following files were migrated to the current callback API:

- `Src/driver.c`
- `Src/driver_spindles.c`
- `Src/encoders.c`
- `Src/neopixel_spi.c`

The old callback usage:

```c
hal.settings_changed
```

was replaced/chained using:

```c
grbl.on_settings_changed
```

The compatibility fix was recorded in:

```text
15dd0e3 Adapt STM32H7xx settings callbacks to latest grblHAL core
```

### Build verification
A full clean PlatformIO build was completed successfully with:

```text
Environment: btt_scylla_h723_tmc5160
PlatformIO Core: 6.2.0
Framework: STM32CubeH7 1.12.1
Target MCU: STM32H723VGT6
```

Build result:

```text
RAM:   31,484 / 577,536 bytes     (5.5%)
Flash: 321,700 / 1,048,576 bytes  (30.7%)
Result: SUCCESS
```

Generated output files:

```text
.pio/build/btt_scylla_h723_tmc5160/firmware.bin
.pio/build/btt_scylla_h723_tmc5160/firmware.elf
```

### Build target note
The verified baseline uses the non-bootloader target:

```text
btt_scylla_h723_tmc5160
```

The bootloader-oriented environment:

```text
btt_scylla_h723_tmc5160_bl128
```

was not used for this baseline.

### Development tooling
Workspace setup was recorded in:

```text
7eeb637 Add PlatformIO and VS Code workspace setup
```

### Validation status
- Clean compile: **Verified**
- Firmware binary generation: **Verified**
- Firmware ELF generation: **Verified**
- On-board runtime validation: **Pending**
- Machine-specific configuration validation: **Pending**

No claim of complete hardware validation is made at this stage.

---

## Upstream history

This changelog documents changes specific to this fork.

For upstream grblHAL Core history, see:

```text
grbl/changelog.md
```

and the upstream repository:

https://github.com/grblHAL/core

For STM32H7xx upstream history, see:

https://github.com/dresco/STM32H7xx

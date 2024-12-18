# grblHAL driver for STM32H7xx processors

This is a port of [grblHAL](https://www.github.com/grblhAL) for the STM32H7xx series of processors, originally based on the F7 driver from the official repository.

## Supported boards
- [Nucleo-H743ZI](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html)
- [Nucleo-H723ZG](https://www.st.com/en/evaluation-tools/nucleo-h723zg.html)
- [WeAct MiniSTM32H7xx  (H743 & H723)](https://github.com/WeActTC/MiniSTM32H7xx)
- [BTT SKR 3 EZ (H743 & H723)](https://www.biqu.equipment/products/bigtreetech-btt-skr-3-ez-control-board-mainboard-for-3d-printer)
- [BTT Octopus Max EZ](https://biqu.equipment/products/bigtreetech-btt-octopusmax-ez-for-3d-printer)
- [BTT Scylla](https://biqu.equipment/products/bigtreetech-scylla-v1-0)

## Building

### Web App

Builds for the above boards are supported through the upstream [web app](http://svn.io-engineering.com:8080/).

### Local builds

Local builds are supported from both STMCubeIDE and PlatformIO. Board specific settings have been included in the IDE configurations where possible - to avoid the need for code changes when building for different boards. (Please see the upstream Wiki-page [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL) for further instructions for how to import the project, configure the driver and compile).

Note that the PlatformIO builds are currently using a custom download url for the **framework-stm32cubeh7** library files. This will be removed once v1.10 is available in the upstream PlatformIO package registry.

### GitHub CI builds

PlatformIO builds for a number of configurations are run automatically as a GitHub Action on each push to the master branch. For convenience, the resulting firmwares are archived in the artifacts for each run, and can be found under the Actions tab on the GitHub repo (these files remain available for 90 days).

---
2024-12-19
## grblHAL driver for STM32H7xx processors

This is a port of [grblHAL](https://www.github.com/grblhAL) for the STM32H7xx series of processors, originally based on the F7 driver from the official repository.

__NOTE:__ This is an initial version, currently only lightly tested with [BTT SKR 3 (EZ)](https://www.biqu.equipment/products/bigtreetech-btt-skr-3-ez-control-board-mainboard-for-3d-printer), [Nucleo-H743ZI2](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html), and [WeAct MiniSTM32H743](https://github.com/WeActTC/MiniSTM32H7xx) boards.

Builds should be supported from both STMCubeIDE and PlatformIO. Board specific settings have been included in the IDE configurations where possible - to avoid the need for code changes when building for different boards. (Please see the upstream Wiki-page [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL) for further instructions for how to import the project, configure the driver and compile).

The current build configurations include;
- WeAct MiniSTM32H743 with SDCard and USB serial support.
- BTT SKR 3 (including EZ) with SDCard, USB serial, and TMC 2209/5160 driver support. (Note that settings are currently stored in FLASH, as the onboard EEPROM will need a software I2C implementation).
- Nucleo H743ZI with Ethernet support. (WebUI support has also been added to the Nucleo build, however this requires an SDCard breakout).

PlatformIO builds are currently missing Ethernet support, pending an update to the **framework-stm32cubeh7** version included in platformio.

Available driver options can be found [here](Inc/my_machine.h).

---
2022-08-05

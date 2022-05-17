## grblHAL driver for STM32H7xx processors (Nucleo-144, WeAct MiniSTM32H7xx)

See the Wiki-page for [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

__NOTE:__ This is an initial version, currently only lightly tested with [Nucleo-H743ZI2](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html) and [WeAct MiniSTM32H7xx](https://github.com/WeActTC/MiniSTM32H7xx) development boards.

- The STMCubeIDE builds enable Ethernet support for the Nucleo board, and SDCard support for the WeAct board.
- PlatformIO builds are currently broken, pending an updated **framework-stm32cubeh7** release.

Available driver options can be found [here](Inc/my_machine.h).

---
2022-05-17

/*
  weact_mini_h743.c - driver code for STM32H7xx processors

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe

  Some software serial code is ported from Arduino.  Credit belongs to the many
  authors that contributed to that project.

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"
#if defined(BOARD_WEACT_MINI)

void board_init (void)
{
#if TRINAMIC_SPI_ENABLE
  extern void if_init (uint8_t motors, axes_signals_t enabled);
  trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};
  trinamic_if_init(&driver_if);
#elif TRINAMIC_UART_ENABLE
    extern void tmc_uart_init (void);
    tmc_uart_init();
#endif
}

#endif //BOARD_WEACT_MINI

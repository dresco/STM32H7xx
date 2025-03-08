/*
  i2c.h - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for STM32H7xx

  Copyright (c) 2018-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "driver.h"
#include "grbl/plugins.h"

#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C

#include "motors/trinamic.h"
#include "trinamic/tmc_i2c_interface.h"

#define I2C_ADR_I2CBRIDGE 0x47

#endif

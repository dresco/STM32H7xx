/*

  sdmmc.c - driver code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Terje Io

  This code sets up the SDMMC peripheral for SD card access

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


#if SDCARD_ENABLE

#include "sdmmc.h"

SD_HandleTypeDef hsd1;

void sdmmc_init()
{
	// SDMMC1 init
    //
    // Start simple.. polling with clock division to prevent underrun
    //
	hsd1.Instance = SDMMC1;
	hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
	hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd1.Init.ClockDiv = 2;

	// FatFS init
	MX_FATFS_Init();

}

#endif

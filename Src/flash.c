/*

  flash.c - driver code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

  This code reads/writes the whole RAM-based emulated EPROM contents from/to flash

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
#include "flash.h"

#if FLASH_ENABLE

#include <string.h>

extern void *_EEPROM_Emul_Start;
extern uint8_t _EEPROM_Emul_Sector;

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, &_EEPROM_Emul_Start, hal.nvs.size);

    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    // The STM32H7xx HAL_FLASH_Program implementation differs to other STM32 families.
    //
    // Writes are either 16 or 32 bytes long (dependent on processor model), instead of the byte,
    // halfword, word or double word writes in other processor families. Erase operations are
    // performed on 128-Kbyte sectors.
    //
    // Rather than using two sectors for boot & eeprom emulation, have chosen to store the NVS data
    // at the end of the user accessible flash (in the last 128KB sector).
	//
	// Note that devices may have either one or two banks of flash memory, depending on flash size.

    if (!memcmp(source, &_EEPROM_Emul_Start, hal.nvs.size))
        return true;

    HAL_StatusTypeDef status;

    if((status = HAL_FLASH_Unlock()) == HAL_OK) {

        static FLASH_EraseInitTypeDef erase = {
            .Sector = FLASH_SECTOR_TOTAL - 1,
#ifdef FLASH_BANK_2
            .Banks = FLASH_BANK_2,
#else
            .Banks = FLASH_BANK_1,
#endif
            .TypeErase = FLASH_TYPEERASE_SECTORS,
            .NbSectors = 1,
            .VoltageRange = FLASH_VOLTAGE_RANGE_3
        };

        uint32_t error;
        status = HAL_FLASHEx_Erase(&erase, &error);

        uint8_t buffer[FLASH_WRITE_SIZE];
        uint8_t *data = source;
        uint32_t address = (uint32_t)&_EEPROM_Emul_Start, remaining = (uint32_t)hal.nvs.size;

        while(remaining && status == HAL_OK) {

            //copy a buffers worth of nvs data, zero padded if less than the minimum flash write size...
            memset(&buffer, 0, FLASH_WRITE_SIZE);
            memcpy(&buffer, data, min(remaining, FLASH_WRITE_SIZE));

            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)buffer);
            address += FLASH_WRITE_SIZE;
            data += FLASH_WRITE_SIZE;
            remaining -= min(remaining, FLASH_WRITE_SIZE);
        }

        HAL_FLASH_Lock();
    }

    return status == HAL_OK;
}

#endif


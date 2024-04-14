/*

  spiflash.h - driver code for STM32H7xx ARM processors

  Part of grblHAL

  Copyright (c) 2024 Jon Escombe

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

#ifndef _SPIFLASH_H_
#define _SPIFLASH_H_

#include "driver.h"

// Return codes
#define SPIFLASH_OK             0
#define SPIFLASH_ERROR          1

// Public function prototypes
void spiflash_init(void);
int  spiflash_read(uint32_t offset, uint8_t *buffer, size_t buffer_size);
int  spiflash_program(uint32_t offset, const uint8_t *buffer, size_t buffer_size);
int  spiflash_sector_erase(uint32_t address);

// SPI flash settings
#ifndef SPIFLASH_BASE_ADDRESS
#define SPIFLASH_BASE_ADDRESS   0x90000000
#endif

#ifndef SPIFLASH_MEMORY_MAPPED
#define SPIFLASH_MEMORY_MAPPED  1
#endif

#ifndef SPIFLASH_SIZE
#define SPIFLASH_SIZE           32        // 32 Mb = 4 MB
#endif

#define SPIFLASH_PAGE_SIZE      256
#define SPIFLASH_SECTOR_SIZE    4096

// H743/H723 specific defines, used to keep the code as generic as possible..
#if defined(STM32H743xx)

#define xSPI_(name)              QSPI_ ## name
#define HAL_xSPI(name)           HAL_QSPI_ ## name

#define xSPI_CmdTypeDef          QSPI_CommandTypeDef
#define xSPI_MemoryMappedTypeDef QSPI_MemoryMappedTypeDef

#define xSPI_AF9                 GPIO_AF9_QUADSPI
#define xSPI_AF10                GPIO_AF10_QUADSPI

#define xSPI_TIMEOUT_DEFAULT     HAL_QSPI_TIMEOUT_DEFAULT_VALUE

#define xSPI_INSTRUCTION_NONE    QSPI_INSTRUCTION_NONE
#define xSPI_INSTRUCTION_1_LINE  QSPI_INSTRUCTION_1_LINE
#define xSPI_INSTRUCTION_4_LINES QSPI_INSTRUCTION_4_LINES

#define xSPI_ADDRESS_NONE        QSPI_ADDRESS_NONE
#define xSPI_ADDRESS_1_LINE      QSPI_ADDRESS_1_LINE
#define xSPI_ADDRESS_4_LINES     QSPI_ADDRESS_4_LINES

#define xSPI_ADDRESS_8_BITS      QSPI_ADDRESS_8_BITS
#define xSPI_ADDRESS_16_BITS     QSPI_ADDRESS_16_BITS
#define xSPI_ADDRESS_24_BITS     QSPI_ADDRESS_24_BITS
#define xSPI_ADDRESS_32_BITS     QSPI_ADDRESS_32_BITS

#define xSPI_DATA_NONE           QSPI_DATA_NONE
#define xSPI_DATA_1_LINE         QSPI_DATA_1_LINE
#define xSPI_DATA_4_LINES        QSPI_DATA_4_LINES

#elif defined(STM32H723xx)

#define xSPI_(name)              OSPI_ ## name
#define HAL_xSPI(name)           HAL_OSPI_ ## name

#define xSPI_CmdTypeDef          OSPI_RegularCmdTypeDef

#define xSPI_AF9                 GPIO_AF9_OCTOSPIM_P1
#define xSPI_AF10                GPIO_AF10_OCTOSPIM_P1

#define xSPI_TIMEOUT_DEFAULT     HAL_OSPI_TIMEOUT_DEFAULT_VALUE

#define xSPI_INSTRUCTION_NONE    HAL_OSPI_INSTRUCTION_NONE
#define xSPI_INSTRUCTION_1_LINE  HAL_OSPI_INSTRUCTION_1_LINE
#define xSPI_INSTRUCTION_4_LINES HAL_OSPI_INSTRUCTION_4_LINES

#define xSPI_ADDRESS_NONE        HAL_OSPI_ADDRESS_NONE
#define xSPI_ADDRESS_1_LINE      HAL_OSPI_ADDRESS_1_LINE
#define xSPI_ADDRESS_4_LINES     HAL_OSPI_ADDRESS_4_LINES

#define xSPI_ADDRESS_8_BITS      HAL_OSPI_ADDRESS_8_BITS
#define xSPI_ADDRESS_16_BITS     HAL_OSPI_ADDRESS_16_BITS
#define xSPI_ADDRESS_24_BITS     HAL_OSPI_ADDRESS_24_BITS
#define xSPI_ADDRESS_32_BITS     HAL_OSPI_ADDRESS_32_BITS

#define xSPI_DATA_NONE           HAL_OSPI_DATA_NONE
#define xSPI_DATA_1_LINE         HAL_OSPI_DATA_1_LINE
#define xSPI_DATA_4_LINES        HAL_OSPI_DATA_4_LINES

#endif

// WB (Winbond) status register definitions
#define WB_SR1_PROTECT_MASK 0xFC     // 11111100
#define WB_SR2_PROTECT_MASK 0x41     // 01000001 (excluding OTP security register lock bits)
#define WB_SR3_PROTECT_MASK 0x04     // 00000100

#define WB_SR2_QE_MASK      0x02     // 00000010
#define WB_SR2_QE_MATCH     0x02     // 00000010

#define WB_SR3_DRV_MASK     0x60     // 01100000
#define WB_SR3_DRV_MATCH    0x40     // 01000000 (50% drive strength)

#define WB_SR1_WEL_MASK     0x02     // 00000010
#define WB_SR1_WEL_MATCH    0x02     // 00000010

#define WB_SR1_BUSY_MASK    0x01     // 00000001
#define WB_SR1_BUSY_MATCH   0x00     // 00000000

// Convenience macro to access the struct for a command
#define CMD(cmd_name) &wb_cmds[CMD_##cmd_name]

// Convenience macro to initialize a flash_cmd_t struct
#define CMD_DEF(_cmd, _instr_lines, _addr_lines, _addr_size, _data_lines, _dummy) \
{                                                                                 \
    .cmd         = (_cmd),                                                        \
    .instr_lines = (_instr_lines),                                                \
    .addr_lines  = (_addr_lines),                                                 \
    .addr_size   = (_addr_size),                                                  \
    .data_lines  = (_data_lines),                                                 \
    .dummy       = (_dummy)                                                       \
}

typedef enum {
    LINES_0,        // Mapped to HAL_OSPI_*_NONE
    LINES_1,        // Mapped to HAL_OSPI_*_1_LINE
    LINES_4,        // Mapped to HAL_OSPI_*_4_LINES
} lines_t;

typedef enum {
    ADDR_SIZE_8B,   // Mapped to HAL_OSPI_ADDRESS_8_BITS
    ADDR_SIZE_16B,  // Mapped to HAL_OSPI_ADDRESS_16_BITS
    ADDR_SIZE_24B,  // Mapped to HAL_OSPI_ADDRESS_24_BITS
    ADDR_SIZE_32B,  // Mapped to HAL_OSPI_ADDRESS_32_BITS
} addr_size_t;

typedef struct {
    uint8_t            cmd;             // Command / Instruction
    uint8_t            instr_lines : 2; // Instruction Lines
    lines_t            addr_lines  : 2; // Address Lines
    addr_size_t        addr_size   : 2; // Address Size
    uint8_t            data_lines  : 2; // Data Lines
    uint8_t            dummy;           // Dummy Cycles
} flash_cmd_t;

typedef union {
    uint32_t u32;
    uint8_t  u8[4];
} jedec_id_t;

enum {
    CMD_RDSR1,      // Read Status Register 1
    CMD_RDSR2,      // Read Status Register 2
    CMD_RDSR3,      // Read Status Register 3

    CMD_WRSR1,      // Write Status Register 1
    CMD_WRSR2,      // Write Status Register 2
    CMD_WRSR3,      // Write Status Register 3

    CMD_RSTEN,      // Reset Enable
    CMD_RST,        // Reset
    CMD_RDID,       // Read Identification
    CMD_WREN,       // Write Enable

    CMD_SE,         // Sector Erase (4kB)
    CMD_BE,         // Block Erase (64kB)
    CMD_CE,         // Chip Erase

    CMD_PP,         // Page Program
    CMD_READ,       // Read Data Bytes

    CMD_COUNT,
};

#endif // _SPIFLASH_H_

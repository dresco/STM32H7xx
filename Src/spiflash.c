/*

  spiflash.c - driver code for STM32H7xx ARM processors

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

//
// Initial support for Winbond WS25Qxx NOR flash IC's, with capacity
// up to 128Mb (using 24bit addressing mode).
//
// STM32H743xx devices using QUADSPI peripheral in QSPI mode.
// STM32H723xx devices using OCTOSPI peripheral in QSPI mode.
//
// Code derived from various sources, including WeAct examples, GnWManager (Apache),
// AdaFruit UF2 bootloader (MIT), and ST ExternalLoader drivers (BSD).
//

#include "driver.h"
#include <stdio.h>

#if SPIFLASH_ENABLE

#include "../grbl/protocol.h"
#include "spiflash.h"

#if defined (STM32H743xx)
QSPI_HandleTypeDef hxspi;
static int  QSPI_Init(void);
static void QSPI_SetCmd(QSPI_CommandTypeDef *qspi_cmd, const flash_cmd_t *cmd, uint32_t address, uint8_t *data, size_t len);
#if SPIFLASH_MEMORY_MAPPED
static int  QSPI_EnableMemoryMappedMode(void);
#endif // SPIFLASH_MEMORY_MAPPED
#elif defined(STM32H723xx)
OSPI_HandleTypeDef hxspi;
static int  OSPI_Init(void);
static void OSPI_SetCmd(OSPI_RegularCmdTypeDef *ospi_cmd, const flash_cmd_t *cmd, uint32_t address, uint8_t *data, size_t len);
#if SPIFLASH_MEMORY_MAPPED
static int  OSPI_EnableMemoryMappedMode(void);
#endif // SPIFLASH_MEMORY_MAPPED
#endif // STM32H723xx

const uint32_t instruction_line_map[] = {
    [LINES_0] = xSPI_INSTRUCTION_NONE,
    [LINES_1] = xSPI_INSTRUCTION_1_LINE,
    [LINES_4] = xSPI_INSTRUCTION_4_LINES,
};

const uint32_t address_line_map[] = {
    [LINES_0] = xSPI_ADDRESS_NONE,
    [LINES_1] = xSPI_ADDRESS_1_LINE,
    [LINES_4] = xSPI_ADDRESS_4_LINES,
};

const uint32_t address_size_map[] = {
    [ADDR_SIZE_8B]  = xSPI_ADDRESS_8_BITS,
    [ADDR_SIZE_16B] = xSPI_ADDRESS_16_BITS,
    [ADDR_SIZE_24B] = xSPI_ADDRESS_24_BITS,
    [ADDR_SIZE_32B] = xSPI_ADDRESS_32_BITS,
};

const uint32_t data_line_map[] = {
    [LINES_0] = xSPI_DATA_NONE,
    [LINES_1] = xSPI_DATA_1_LINE,
    [LINES_4] = xSPI_DATA_4_LINES,
};

const flash_cmd_t wb_cmds[CMD_COUNT] = {
    //     cmd              cmd  i_lines  a_lines         a_size  d_lines dummy
    [CMD_WRSR1]  = CMD_DEF(0x01, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_WRSR2]  = CMD_DEF(0x31, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_WRSR3]  = CMD_DEF(0x11, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_RDSR1]  = CMD_DEF(0x05, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_RDSR2]  = CMD_DEF(0x35, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_RDSR3]  = CMD_DEF(0x15, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_RSTEN]  = CMD_DEF(0x66, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_0,    0),
    [CMD_RST]    = CMD_DEF(0x99, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_0,    0),
    [CMD_RDID]   = CMD_DEF(0x9F, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_1,    0),
    [CMD_WREN]   = CMD_DEF(0x06, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_0,    0),
    [CMD_CE]     = CMD_DEF(0x60, LINES_1, LINES_0, ADDR_SIZE_24B, LINES_0,    0), // Chip Erase
    [CMD_SE]     = CMD_DEF(0x20, LINES_1, LINES_1, ADDR_SIZE_24B, LINES_0,    0), // Sector Erase 4KB
    [CMD_BE]     = CMD_DEF(0xD8, LINES_1, LINES_1, ADDR_SIZE_24B, LINES_0,    0), // Block Erase 64KB
    [CMD_PP]     = CMD_DEF(0x32, LINES_1, LINES_1, ADDR_SIZE_24B, LINES_4,    0), // Quad Input Page Program
    [CMD_READ]   = CMD_DEF(0xEB, LINES_1, LINES_4, ADDR_SIZE_24B, LINES_4,    6), // Fast Read Quad I/O
};

#if SPIFLASH_MEMORY_MAPPED
static int xSPI_DisableMemoryMappedMode(void)
{
    // This will *ONLY* work if you don't look at the memory mapped address.
    // https://community.st.com/s/question/0D50X00009XkaJuSAJ/stm32f7-qspi-exit-memory-mapped-mode
    if (HAL_xSPI(Abort)(&hxspi) != HAL_OK) {
        return SPIFLASH_ERROR;
    }
    return SPIFLASH_OK;
}
#endif

static int xSPI_ReadBytes(const flash_cmd_t *cmd, uint32_t offset, uint8_t *data, size_t len)
{
    xSPI_CmdTypeDef xspi_cmd;

    //debug_printf("RB %x 0x%08lx %p %d\n", cmd->cmd, offset, data, len);

    xSPI_(SetCmd)(&xspi_cmd, cmd, offset, (uint8_t *) data, len);

    HAL_StatusTypeDef res;
    res = HAL_xSPI(Command)(&hxspi, &xspi_cmd, xSPI_TIMEOUT_DEFAULT);
    if (res != HAL_OK) {
        return SPIFLASH_ERROR;
    }

    if (HAL_xSPI(Receive)(&hxspi, data, xSPI_TIMEOUT_DEFAULT) != HAL_OK) {
        return SPIFLASH_ERROR;
    }

    return SPIFLASH_OK;
}

static int xSPI_Read(uint32_t offset, uint8_t *buffer, size_t buffer_size)
{
#if SPIFLASH_MEMORY_MAPPED
    unsigned char *address = (unsigned char *)SPIFLASH_BASE_ADDRESS + offset;
    memcpy(buffer, address, buffer_size);
#else
    if (xSPI_ReadBytes(CMD(READ), offset, (uint8_t *) buffer, buffer_size) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
#endif

    return SPIFLASH_OK;
}

/*
 * Wait for memory to be ready.
 *
 * Note: not using the HAL_xSPI_AutoPolling() functions, due to differences
 * between the QUADSPI and OCOTOSPI HAL implementations. (Could use the same
 * approach as EnableMemoryMappedMode, with separate QSPI/OSPI functions).
 *
 */
static int xSPI_WaitForStatus(uint32_t Match, uint32_t Mask, uint32_t Timeout)
{
    uint8_t sr1 = 0;
    uint32_t start_time = HAL_GetTick();
    uint32_t prev_time = start_time;
    bool startup = true;

    do {
        // read SR1 status register
        xSPI_ReadBytes(CMD(RDSR1), 0, &sr1, 1);

        // exit if timeout exceeded
        if ((Timeout > 0) && (HAL_GetTick() > start_time + Timeout)) {
            return SPIFLASH_ERROR;
        }

        // wait 10ms between subsequent read attempts
        if (!startup) {
            while (HAL_GetTick() < prev_time + 10);
            prev_time = HAL_GetTick();
        }
        startup = false;
    } while ((sr1 & Mask) != Match);

    return SPIFLASH_OK;
}

static int xSPI_WriteBytes(const flash_cmd_t *cmd, uint32_t address, const uint8_t *data, size_t len)
{
    xSPI_CmdTypeDef xspi_cmd;

    //debug_printf("WB %x 0x%08lx %p %d\n", cmd->cmd, address, data, len);

    xSPI_(SetCmd)(&xspi_cmd, cmd, address, (uint8_t *) data, len);

    if (HAL_xSPI(Command)(&hxspi, &xspi_cmd, xSPI_TIMEOUT_DEFAULT) != HAL_OK) {
        return SPIFLASH_ERROR;
    }

    if (len > 0) {
        if (HAL_xSPI(Transmit)(&hxspi, (uint8_t *) data, xSPI_TIMEOUT_DEFAULT) != HAL_OK) {
            return SPIFLASH_ERROR;
        }
    }
    return SPIFLASH_OK;
}

static int xSPI_WriteEnable(void)
{
    if (xSPI_WriteBytes(CMD(WREN), 0, NULL, 0) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

    // Wait for Write Enable Latch to be set
    if (xSPI_WaitForStatus(WB_SR1_WEL_MATCH, WB_SR1_WEL_MASK, xSPI_TIMEOUT_DEFAULT) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

    return SPIFLASH_OK;
}

static int xSPI_SectorErase(uint32_t address)
{
#if SPIFLASH_MEMORY_MAPPED
    if (xSPI_DisableMemoryMappedMode() != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
#endif

    if (xSPI_WriteEnable() != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

    if (xSPI_WriteBytes(CMD(SE), address, NULL, 0) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

    // Wait for BUSY bit to be cleared..
    if (xSPI_WaitForStatus(WB_SR1_BUSY_MATCH, WB_SR1_BUSY_MASK, xSPI_TIMEOUT_DEFAULT) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

#if SPIFLASH_MEMORY_MAPPED
    if (xSPI_(EnableMemoryMappedMode)() != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
#endif

    return SPIFLASH_OK;
}

static int xSPI_PageProgram(uint32_t address, const uint8_t *buffer, size_t buffer_size)
{
    assert(buffer_size <= 256);

    //debug_printf("PP cmd=%02X addr=0x%lx buf=%p len=%d\n", (*CMD(PP)).cmd, address, buffer, buffer_size);

    if (xSPI_WriteBytes(CMD(PP), address, buffer, buffer_size) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

    // Wait for BUSY bit to be cleared..
    if (xSPI_WaitForStatus(WB_SR1_BUSY_MATCH, WB_SR1_BUSY_MASK, xSPI_TIMEOUT_DEFAULT) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }

    return SPIFLASH_OK;
}

static int xSPI_Program(uint32_t address, const uint8_t *buffer, size_t buffer_size)
{
    unsigned iterations = (buffer_size + 255) / 256;
    unsigned dest_page = address / 256;

    assert((address & 0xff) == 0);

#if SPIFLASH_MEMORY_MAPPED
    if (xSPI_DisableMemoryMappedMode() != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
#endif

    for (int i = 0; i < iterations; i++) {
        if (xSPI_WriteEnable() != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if (xSPI_PageProgram((i + dest_page) * 256,
                         buffer + (i * 256),
                         buffer_size > 256 ? 256 : buffer_size) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        buffer_size -= 256;
    }

#if SPIFLASH_MEMORY_MAPPED
    if (xSPI_(EnableMemoryMappedMode)() != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
#endif

    return SPIFLASH_OK;
}

/*
 * Init the GPIOs for the defined pinout
 */
static void xSPI_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

#if SPIFLASH_PINOUT == 1 // SPI Flash pinout for WeAct Mini boards

    /**
    PB6      ------> NCS
    PB2      ------> CLK
    PD11     ------> IO0
    PD12     ------> IO1
    PE2      ------> IO2
    PD13     ------> IO3
    */

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF9;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#elif SPIFLASH_PINOUT == 2 // SPI Flash pinout for dresco 8 axis board

    /** GPIO Configuration
    PB6     ------> NCS
    PF10    ------> CLK
    PF8     ------> IO0
    PF9     ------> IO1
    PF7     ------> IO2
    PF6     ------> IO3
    */

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF9;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = xSPI_AF10;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

#else
    #error "SPIFLASH_PINOUT must be defined in board map."
#endif
}

/*
 * Reset the GPIOs for the defined pinout
 */
static void xSPI_GPIO_DeInit(void)
{
#if SPIFLASH_PINOUT == 1 // SPI Flash pinout for WeAct Mini boards

    /**
    PB6      ------> NCS
    PB2      ------> CLK
    PD11     ------> IO0
    PD12     ------> IO1
    PE2      ------> IO2
    PD13     ------> IO3
    */

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2|GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13);

#elif SPIFLASH_PINOUT == 2 // SPI Flash pinout for dresco 8 axis board

    /** GPIO Configuration
    PB6     ------> NCS
    PF10    ------> CLK
    PF8     ------> IO0
    PF9     ------> IO1
    PF7     ------> IO2
    PF6     ------> IO3
    */

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10);

#endif
}

/*
 * Configure the Winbond NOR Flash memory.
 *
 * Initial support for WS25Qxx NOR flash IC's, capacity up to 128Mb (using 24bit addressing mode)
 *
 */
static int W25Qxx_Init()
{
    jedec_id_t jedec_id;
    uint8_t sr1, sr2, sr3;

    // * reset chip (RESET ENABLE & RESET commands)
    // * read JEDEC id bytes
    // * read STATUS register 1
    //   - clear WP bit if set
    //   - re-read to confirm (if changed)
    // * read STATUS register 2
    //   - clear WP bit if set
    //   - set QE bit if not set
    //   - re-read to confirm (if changed)
    // * read STATUS register 3
    //   - clear WP bit if set
    //   - set DRVn bits if needed
    //   - re-read to confirm (if changed)
    //

    if (xSPI_WriteBytes(CMD(RSTEN), 0, NULL, 0) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
    if (xSPI_WriteBytes(CMD(RST), 0, NULL, 0) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
    HAL_Delay(1);

    if (xSPI_ReadBytes(CMD(RDID), 0, &jedec_id.u8[0], 3) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
    debug_printf("JEDEC_ID: %02X %02X %02X\n", jedec_id.u8[0], jedec_id.u8[1], jedec_id.u8[2]);

    if (xSPI_ReadBytes(CMD(RDSR1), 0, &sr1, 1) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
    if (xSPI_ReadBytes(CMD(RDSR2), 0, &sr2, 1) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
    if (xSPI_ReadBytes(CMD(RDSR3), 0, &sr3, 1) != SPIFLASH_OK) {
        return SPIFLASH_ERROR;
    }
    debug_printf("Winbond SR1: %02X SR2: %02X SR3: %02X\n", sr1, sr2, sr3);

    // Check/clear SR1 write protect bits
    if (sr1 & WB_SR1_PROTECT_MASK) {
        //debug_printf("Clearing SR1 write protect bits\n");

        // Clear SR1 register, BUSY & WEL are status only bits (not changeable)
        sr1 = 0;
        if (xSPI_WriteEnable() != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if (xSPI_WriteBytes(CMD(WRSR1), 0, &sr1, 1) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }

        // Wait for BUSY bit to be cleared..
        if (xSPI_WaitForStatus(WB_SR1_BUSY_MATCH, WB_SR1_BUSY_MASK, xSPI_TIMEOUT_DEFAULT) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }

        if (xSPI_ReadBytes(CMD(RDSR1), 0, &sr1, 1) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if (sr1 & WB_SR1_PROTECT_MASK) {
            //debug_printf("SR1: %02X, change failed\n", sr1);
            return SPIFLASH_ERROR;
        }

    }

    // Check/clear SR2 write protect bits, also set quad enable bit if necessary
    if ((sr2 & WB_SR2_PROTECT_MASK) || !(sr2 & WB_SR2_QE_MASK)) {

        if (sr2 & WB_SR2_PROTECT_MASK) {
            //debug_printf("Clearing SR2 write protect bits\n");
        }

        if (!(sr2 & WB_SR2_QE_MASK)) {
            //debug_printf("Setting SR2 Quad Enable bit\n");
        }

           // Clear SR2 register, set just Quad Enable bit
        sr2 = WB_SR2_QE_MASK;

        if (xSPI_WriteEnable() != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if (xSPI_WriteBytes(CMD(WRSR2), 0, &sr2, 1) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }

        // Wait for BUSY bit to be cleared..
        if (xSPI_WaitForStatus(WB_SR1_BUSY_MATCH, WB_SR1_BUSY_MASK, xSPI_TIMEOUT_DEFAULT) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }

        if (xSPI_ReadBytes(CMD(RDSR2), 0, &sr2, 1) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if ((sr2 & WB_SR2_PROTECT_MASK) || !(sr2 & WB_SR2_QE_MASK)) {
            //debug_printf("SR2: %02X, change failed\n", sr2);
            return SPIFLASH_ERROR;
        }
    }

    // Check/clear SR2 write protect bits, also set drive stength if necessary
    if ((sr3 & WB_SR3_PROTECT_MASK) || ((sr3 & WB_SR3_DRV_MASK) != WB_SR3_DRV_MATCH)) {

        if (sr3 & WB_SR3_PROTECT_MASK) {
            //debug_printf("Clearing SR3 write protect bits\n");
        }

        if ((sr3 & WB_SR3_DRV_MASK) != WB_SR3_DRV_MATCH) {
            //debug_printf("Setting SR3 Drive Strength bits\n");
        }

           // Clear SR3 register, set just Drive Strength bits
        sr3 = WB_SR3_DRV_MATCH;

        if (xSPI_WriteEnable() != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if (xSPI_WriteBytes(CMD(WRSR3), 0, &sr3, 1) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }

        // Wait for BUSY bit to be cleared..
        if (xSPI_WaitForStatus(WB_SR1_BUSY_MATCH, WB_SR1_BUSY_MASK, xSPI_TIMEOUT_DEFAULT) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }

        if (xSPI_ReadBytes(CMD(RDSR3), 0, &sr3, 1) != SPIFLASH_OK) {
            return SPIFLASH_ERROR;
        }
        if ((sr3 & WB_SR3_PROTECT_MASK) || ((sr3 & WB_SR3_DRV_MASK) != WB_SR3_DRV_MATCH)) {
            //debug_printf("SR3: %02X, change failed\n", sr3);
            return SPIFLASH_ERROR;
        }
    }

    return SPIFLASH_OK;
}

/*
 * Enable the SPI interface.
 *
 * Initial support as follows;
 * - STM32H743xx devices using QUADSPI peripheral in QSPI mode
 * - STM32H723xx devices using OCTOSPI peripheral in QSPI mode
 *
 * The HAL MSP functions are called from within the ST HAL_xSPI_Init() code.
 *
 * Peripheral clocks are already configured elsewhere, so just need to start
 * the QSPI/OSPI clocks, and initialise the GPIOs.
 *
 */
#if defined(STM32H743xx)

static int QSPI_Init(void)
{
    hxspi.Instance = QUADSPI;
    hxspi.Init.ClockPrescaler = 1-1;
    hxspi.Init.FifoThreshold = 1;
    hxspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hxspi.Init.FlashSize = 24-1;
    hxspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
    hxspi.Init.ClockMode = QSPI_CLOCK_MODE_3;
    hxspi.Init.FlashID = QSPI_FLASH_ID_1;
    hxspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
    if (HAL_QSPI_Init(&hxspi) != HAL_OK)
    {
        return SPIFLASH_ERROR;
    }
    return SPIFLASH_OK;
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{
    if(qspiHandle->Instance==QUADSPI)
    {
        /* QUADSPI clock enable */
        __HAL_RCC_QSPI_CLK_ENABLE();

        /* QUADSPI GPIO Initialisation */
        xSPI_GPIO_Init();
    }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{
    if(qspiHandle->Instance==QUADSPI)
    {
        /* QUADSPI clock disable */
        __HAL_RCC_QSPI_CLK_DISABLE();

        /* QUADSPI GPIO Configuration */
        xSPI_GPIO_DeInit();
    }
}

static void QSPI_SetCmd(QSPI_CommandTypeDef *qspi_cmd, const flash_cmd_t *cmd, uint32_t address, uint8_t *data, size_t len)
{
    memset(qspi_cmd, 0x0, sizeof(*qspi_cmd));

    qspi_cmd->Instruction = cmd->cmd;
    qspi_cmd->InstructionMode = instruction_line_map[cmd->instr_lines];

    qspi_cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    qspi_cmd->DummyCycles = cmd->dummy;

    qspi_cmd->DdrMode = QSPI_DDR_MODE_DISABLE;
    qspi_cmd->DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;

    qspi_cmd->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    qspi_cmd->Address = address;
    qspi_cmd->AddressSize = address_size_map[cmd->addr_size];
    qspi_cmd->AddressMode = address_line_map[cmd->addr_lines];

    qspi_cmd->NbData = len;
    qspi_cmd->DataMode = data_line_map[cmd->data_lines];
}

#if SPIFLASH_MEMORY_MAPPED
static int QSPI_EnableMemoryMappedMode(void)
{
    QSPI_CommandTypeDef      qspi_cmd = { 0 };
    QSPI_MemoryMappedTypeDef mem_mapped_cfg = { 0 };
    const flash_cmd_t       *cmd = CMD(READ);

    xSPI_(SetCmd)(&qspi_cmd, cmd, 0, NULL, 0);

    // Disable timeout counter for memory mapped mode
    mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    mem_mapped_cfg.TimeOutPeriod     = 0;

    // Enable memory mapped mode
    if (HAL_QSPI_MemoryMapped(&hxspi, &qspi_cmd, &mem_mapped_cfg) != HAL_OK)
    {
        return SPIFLASH_ERROR;
    }
    return SPIFLASH_OK;
}
#endif // SPIFLASH_MEMORY_MAPPED

#elif defined(STM32H723xx)

static int OSPI_Init(void)
{
    OSPIM_CfgTypeDef sOspiManagerCfg = {0};

    hxspi.Instance = OCTOSPI1;
    hxspi.Init.FifoThreshold = 1;
    hxspi.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    hxspi.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
    hxspi.Init.DeviceSize = 24;
    hxspi.Init.ChipSelectHighTime = 2;
    hxspi.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    hxspi.Init.ClockMode = HAL_OSPI_CLOCK_MODE_3;
    hxspi.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    hxspi.Init.ClockPrescaler = 1;
    hxspi.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    hxspi.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
    hxspi.Init.ChipSelectBoundary = 0;
    hxspi.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
    hxspi.Init.MaxTran = 0;
    hxspi.Init.Refresh = 0;
    if (HAL_OSPI_Init(&hxspi) != HAL_OK)
    {
        return SPIFLASH_ERROR;
    }

    sOspiManagerCfg.ClkPort = 1;
    sOspiManagerCfg.NCSPort = 1;
    sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
    if (HAL_OSPIM_Config(&hxspi, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        return SPIFLASH_ERROR;
    }

    return SPIFLASH_OK;
}

void HAL_OSPI_MspInit(OSPI_HandleTypeDef* ospiHandle)
{
    if(ospiHandle->Instance==OCTOSPI1)
    {
        /* OCTOSPI1 clock enable */
        __HAL_RCC_OCTOSPIM_CLK_ENABLE();
        __HAL_RCC_OSPI1_CLK_ENABLE();

        /* OCTOSPI GPIO Configuration */
        xSPI_GPIO_Init();
    }
}

void HAL_OSPI_MspDeInit(OSPI_HandleTypeDef* ospiHandle)
{
    if(ospiHandle->Instance==OCTOSPI1)
    {
      /* OCTOSPI clock disable */
      __HAL_RCC_OCTOSPIM_CLK_DISABLE();
      __HAL_RCC_OSPI1_CLK_DISABLE();

      /* OCTOSPI GPIO Configuration */
      xSPI_GPIO_DeInit();
    }
}

static void OSPI_SetCmd(OSPI_RegularCmdTypeDef *ospi_cmd, const flash_cmd_t *cmd, uint32_t address, uint8_t *data, size_t len)
{
    memset(ospi_cmd, 0x0, sizeof(*ospi_cmd));

    ospi_cmd->OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    ospi_cmd->FlashId = 0;
    ospi_cmd->Instruction = cmd->cmd;
    ospi_cmd->InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    ospi_cmd->InstructionMode = instruction_line_map[cmd->instr_lines];

    ospi_cmd->AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    ospi_cmd->DummyCycles = cmd->dummy;
    ospi_cmd->DQSMode = HAL_OSPI_DQS_DISABLE;
    ospi_cmd->SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    ospi_cmd->InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;

    ospi_cmd->Address = address;
    ospi_cmd->AddressSize = address_size_map[cmd->addr_size];
    ospi_cmd->AddressMode = address_line_map[cmd->addr_lines];

    ospi_cmd->NbData = len;
    ospi_cmd->DataMode = data_line_map[cmd->data_lines];
}

#if SPIFLASH_MEMORY_MAPPED
static int OSPI_EnableMemoryMappedMode(void)
{
    OSPI_RegularCmdTypeDef   ospi_cmd = { 0 };
    OSPI_MemoryMappedTypeDef mem_mapped_cfg = { 0 };
    const flash_cmd_t       *cmd = CMD(READ);

    xSPI_(SetCmd)(&ospi_cmd, cmd, 0, NULL, 0);

    // Memory-mapped mode configuration for linear burst read operations
    ospi_cmd.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
    if (HAL_OSPI_Command(&hxspi, &ospi_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return SPIFLASH_ERROR;
    }

    // Also use read instruction for write configuration (in order to not alter the flash by accident)
    ospi_cmd.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
    if (HAL_OSPI_Command(&hxspi, &ospi_cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return SPIFLASH_ERROR;
    }

    // Disable timeout counter for memory mapped mode
    mem_mapped_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
    mem_mapped_cfg.TimeOutPeriod = 0;

    // Enable memory mapped mode
    if (HAL_OSPI_MemoryMapped(&hxspi, &mem_mapped_cfg) != HAL_OK) {
        return SPIFLASH_ERROR;
    }

    return SPIFLASH_OK;
}
#endif // SPIFLASH_MEMORY_MAPPED

#endif // STM32H743xx / STM32H723xx specific code

void spiflash_init(void)
{
    // Initialise the QSPI/OPSI peripheral
    if (xSPI_(Init)() != SPIFLASH_OK) {
        protocol_enqueue_foreground_task(report_warning, "SPI Flash initialisation failed!");
        return;
    }

    // Initialise the Winbond memory
    if (W25Qxx_Init() != SPIFLASH_OK) {
        protocol_enqueue_foreground_task(report_warning, "SPI Flash initialisation failed!");
        return;
    }

#if SPIFLASH_MEMORY_MAPPED
    if (xSPI_(EnableMemoryMappedMode)() != SPIFLASH_OK) {
        protocol_enqueue_foreground_task(report_warning, "SPI Flash initialisation failed!");
        return;
    }
#endif
}

int spiflash_read(uint32_t offset, uint8_t *buffer, size_t buffer_size)
{
    return (xSPI_Read(offset, buffer, buffer_size));
}

int spiflash_program(uint32_t offset, const uint8_t *buffer, size_t buffer_size)
{
    return (xSPI_Program(offset, buffer, buffer_size));
}

int spiflash_sector_erase(uint32_t address)
{
    return (xSPI_SectorErase(address));
}

#endif //SPIFLASH_ENABLE


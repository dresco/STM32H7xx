/*
  spi.c - SPI support for Trinamic & networking (WizNet) plugins

  Part of grblHAL driver for STM32H7xx

  Copyright (c) 2020-2024 Terje Io
  Copyright (c) 2022-2024 Jon Escombe

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

#include "main.h"
#include "driver.h"
#include "cache.h"

#ifndef SPI_DMA_THRESHOLD
#define SPI_DMA_THRESHOLD 32
#endif

#ifndef SPI_SCRATCH_BUFFER_SIZE
#define SPI_SCRATCH_BUFFER_SIZE 128
#endif

// Used for debugging info only
static uint32_t tx_count_dma, tx_count_poll;
static uint32_t rx_count_dma, rx_count_poll;

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI ## p

#define DMAirq(d, p) DMAirqI(d, p)
#define DMAirqI(d, p) DMA ## d ## _Stream ## p ## _IRQn

#define DMAhandler(d, p) DMAhandlerI(d, p)
#define DMAhandlerI(d, p) DMA ## d ## _Stream ## p ## _IRQHandler

#define DMAStream(d, p) DMAStreamI(d, p)
#define DMAStreamI(d, p) DMA ## d ## _Stream ## p

#define DMARequestTX(p) DMARequestTXI(p)
#define DMARequestTXI(p) DMA_REQUEST_SPI ## p ## _TX

#define DMARequestRX(p) DMARequestRXI(p)
#define DMARequestRXI(p) DMA_REQUEST_SPI ## p ## _RX

#define SPIirq(d) SPIirqI(d)
#define SPIirqI(d) SPI ## d ## _IRQn

#define SPIHandler(d) SPIHandlerI(d)
#define SPIHandlerI(d) SPI ## d ## _IRQHandler

#if SPI_PORT == 11 || SPI_PORT == 12
#define SPIPORT SPIport(1)
#define SPI_IRQ SPIirq(1)
#define SPI_IRQ_HANDLER SPIHandler(1)
#define DMA_REQUEST_TX DMARequestTX(1)
#define DMA_REQUEST_RX DMARequestRX(1)
#else
#define SPIPORT SPIport(SPI_PORT)
#define SPI_IRQ SPIirq(SPI_PORT)
#define SPI_IRQ_HANDLER SPIHandler(SPI_PORT)
#define DMA_REQUEST_TX DMARequestTX(SPI_PORT)
#define DMA_REQUEST_RX DMARequestRX(SPI_PORT)
#endif

// H7 has a DMAMUX, so using the same DMA streams regardless of SPI port
// TX - DMA1_Stream5
// RX - DMA1_Stream6
#define DMA_STREAM_TX DMAStream(1,5)
#define DMA_TX_IRQ DMAirq(1, 5)
#define DMA_TX_IRQ_HANDLER DMAhandler(1, 5)

#define DMA_STREAM_RX DMAStream(1,6)
#define DMA_RX_IRQ DMAirq(1, 6)
#define DMA_RX_IRQ_HANDLER DMAhandler(1, 6)

static SPI_HandleTypeDef spi_port = {
    .Instance               = SPIPORT,
    .Init.Mode              = SPI_MODE_MASTER,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
    .Init.Direction         = SPI_DIRECTION_2LINES,
    .Init.CLKPhase          = SPI_PHASE_1EDGE,
    .Init.CLKPolarity       = SPI_POLARITY_LOW,
    .Init.DataSize          = SPI_DATASIZE_8BIT,
    .Init.FirstBit          = SPI_FIRSTBIT_MSB,
    .Init.TIMode            = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial     = 7,
    .Init.CRCLength         = SPI_CRC_LENGTH_8BIT,
    .Init.NSS               = SPI_NSS_SOFT,
    .Init.NSSPMode          = SPI_NSS_PULSE_DISABLE,
    .Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE,  /* Recommended setting to avoid glitches */
};

static DMA_HandleTypeDef spi_dma_tx = {
    .Instance                 = DMA_STREAM_TX,
    .Init.Request             = DMA_REQUEST_TX,
    .Init.FIFOMode            = DMA_FIFOMODE_DISABLE,
    .Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
    .Init.MemBurst            = DMA_MBURST_INC4,
    .Init.PeriphBurst         = DMA_PBURST_INC4,
    .Init.Direction           = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc           = DMA_PINC_DISABLE,
    .Init.MemInc              = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
    .Init.Mode                = DMA_NORMAL,
    .Init.Priority            = DMA_PRIORITY_LOW,
};

static DMA_HandleTypeDef spi_dma_rx = {

    .Instance                 = DMA_STREAM_RX,
    .Init.Request             = DMA_REQUEST_RX,
    .Init.FIFOMode            = DMA_FIFOMODE_DISABLE,
    .Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL,
    .Init.MemBurst            = DMA_MBURST_INC4,
    .Init.PeriphBurst         = DMA_PBURST_INC4,
    .Init.Direction           = DMA_PERIPH_TO_MEMORY,
    .Init.PeriphInc           = DMA_PINC_DISABLE,
    .Init.MemInc              = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE,
    .Init.Mode                = DMA_NORMAL,
    .Init.Priority            = DMA_PRIORITY_LOW,
};

#if USE_SPI_DMA && L1_CACHE_ENABLE
/* Scratch buffer must start and end on cache line boundaries for safe cache maintenance */
static uint8_t scratch_buffer[align_up(SPI_SCRATCH_BUFFER_SIZE, __SCB_DCACHE_LINE_SIZE)] __ALIGNED(__SCB_DCACHE_LINE_SIZE);
#endif

void spi_init (void)
{
    static bool init = false;

    // Pin order: SCK|MISO|MOSI

    if(!init) {

#if SPI_PORT == 1

        __HAL_RCC_SPI1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 5,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 6,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 7,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 12

        __HAL_RCC_SPI1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin =  GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 3,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 4,
            .mode = { .mask = PINMODE_NONE }
        };
        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 2

        __HAL_RCC_SPI2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI2,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 14,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 15,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 3

        __HAL_RCC_SPI3_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF6_SPI3
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 10,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 11,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 4

        __HAL_RCC_SPI4_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = GPIO_AF5_SPI4
        };
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 12,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 13,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 14,
            .mode = { .mask = PINMODE_NONE }
        };

#endif

#if USE_SPI_DMA
        __HAL_RCC_DMA1_CLK_ENABLE();
        HAL_DMA_Init(&spi_dma_rx);
        __HAL_LINKDMA(&spi_port, hdmarx, spi_dma_rx);
        HAL_DMA_Init(&spi_dma_tx);
        __HAL_LINKDMA(&spi_port, hdmatx, spi_dma_tx);

        HAL_NVIC_SetPriority(DMA_RX_IRQ, 1, 1);
        HAL_NVIC_EnableIRQ(DMA_RX_IRQ);
        HAL_NVIC_SetPriority(DMA_TX_IRQ, 1, 1);
        HAL_NVIC_EnableIRQ(DMA_TX_IRQ);
#endif

        HAL_NVIC_SetPriority(SPI_IRQ, 1, 0);
        HAL_NVIC_EnableIRQ(SPI_IRQ);

        HAL_SPI_Init(&spi_port);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);

        init = true;
    }
}

// set the SPI speed to the max setting
void spi_set_max_speed (void)
{
    __HAL_SPI_DISABLE(&spi_port);
    MODIFY_REG(spi_port.Instance->CFG1, SPI_CFG1_MBR, SPI_BAUDRATEPRESCALER_2); // should be able to go to 24Mhz...
    __HAL_SPI_ENABLE(&spi_port);
}

uint32_t spi_set_speed (uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&spi_port);
    MODIFY_REG(spi_port.Instance->CFG1, SPI_CFG1_MBR, prescaler);
    __HAL_SPI_ENABLE(&spi_port);
    return prescaler;
}

uint8_t spi_get_byte (void)
{
    uint8_t byte;
    HAL_SPI_Receive(&spi_port, &byte, 1, 1000);
    return byte;
}

uint8_t spi_put_byte (uint8_t tx_byte)
{
    uint8_t rx_byte;

    // using TransmitReceive as TMC_SPI code uses the RX byte in some cases.
    HAL_SPI_TransmitReceive(&spi_port, &tx_byte, &rx_byte, 1, 1000);

    return rx_byte;
}

void spi_write (uint8_t *data, uint16_t len)
{
    bool use_dma = false;

#if USE_SPI_DMA
    if (len >= SPI_DMA_THRESHOLD)
        use_dma = true;
#endif

    if (use_dma) {

        tx_count_dma++;

        /*
         * Cache aligned buffers not strictly required for TX only.
         * Clean cache before DMA start, flushing cached data to RAM for DMA to access.
         * See notes in cache.c for context.
         */
#if L1_CACHE_ENABLE
        SCB_CleanDCache_by_Addr((uint32_t*)data, len);
#endif

        if(HAL_SPI_Transmit_DMA(&spi_port, data, len) == HAL_OK)
            while(spi_port.State != HAL_SPI_STATE_READY);

        __HAL_DMA_DISABLE(&spi_dma_tx);
    } else {
        tx_count_poll++;
        HAL_SPI_Transmit(&spi_port, data, len, 1000);
    }
}

void spi_read (uint8_t *data, uint16_t len)
{
    bool use_dma = false;
    bool use_scratch_buffer = false;

#if USE_SPI_DMA
    if (len >= SPI_DMA_THRESHOLD)
        use_dma = true;
#if L1_CACHE_ENABLE
    // If using data cache, the RX buffer must be aligned to cache line boundaries
    if (!is_cache_aligned(data, len))
        use_scratch_buffer = true;
#endif // L1_CACHE_ENABLE
#endif // USE_SPI_DMA

    if (use_dma) {

        rx_count_dma++;

        /*
         * Cache aligned buffers are required for RX.
         * Invalidate cache before and after DMA operation.
         * Use scratch buffer if not correctly aligned,
         * See notes in cache.c for context.
         */

        if(!use_scratch_buffer) {

#if L1_CACHE_ENABLE
            SCB_InvalidateDCache_by_Addr((uint32_t*)data, len);
#endif

            if(HAL_SPI_Receive_DMA(&spi_port, data, len) == HAL_OK)
                while(spi_port.State != HAL_SPI_STATE_READY);

#if L1_CACHE_ENABLE
            SCB_InvalidateDCache_by_Addr((uint32_t*)data, len);
#endif

        } else {
#if L1_CACHE_ENABLE
            // buffer is not aligned for safe cache maintenance, using scratch buffer for DMA access
            int remain = len;
            while (remain > 0) {
                int bytes = remain > SPI_SCRATCH_BUFFER_SIZE ? SPI_SCRATCH_BUFFER_SIZE : remain;

                SCB_InvalidateDCache_by_Addr((uint32_t*)scratch_buffer, bytes);

                if(HAL_SPI_Receive_DMA(&spi_port, scratch_buffer, bytes) == HAL_OK)
                    while(spi_port.State != HAL_SPI_STATE_READY);

                SCB_InvalidateDCache_by_Addr((uint32_t*)scratch_buffer, bytes);

                memcpy(data, scratch_buffer, bytes);

                data += bytes;
                remain -= bytes;
            }
#endif
        }
        __HAL_DMA_DISABLE(&spi_dma_rx);
        __HAL_DMA_DISABLE(&spi_dma_tx);

    } else {
        rx_count_poll++;
        HAL_SPI_Receive(&spi_port, data, len, 1000);
    }
}

void DMA_RX_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&spi_dma_rx);
}

void DMA_TX_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&spi_dma_tx);
}

void SPI_IRQ_HANDLER(void)
{
    HAL_SPI_IRQHandler(&spi_port);
}
